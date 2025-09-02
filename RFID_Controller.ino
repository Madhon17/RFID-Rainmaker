/*
 * ESP32 RFID + 8 Relay + ESP RainMaker (v2.0.0)
 * Fitur:
 * - Device "RFID_Controller":
 *   Status (text, RO), AddMode (bool), RemoveMode (bool),
 *   Mark1..Mark8 (bool), ListCards (text, RO), LastAccess (text, RO)
 * - Device "Relay_Controller":
 *   Relay1..Relay8 (bool)
 * - AddMode/RemoveMode auto-off setelah sukses, atau timeout tanpa tap -> kembali Normal + buzzer 3x
 * - Saat Normal: tap kartu -> cek izin (mask), ON relay-relay terkait 5 detik (auto-off)
 * - Buzzer pola: add, remove, granted, denied, timeout(3x)
 * - Data kartu & mask disimpan di NVS (Preferences)
 * - Provisioning Wi-Fi via RainMaker App (BLE)
 */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Preferences.h>

#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <time.h>

// =================== Pin Mapping ===================
#define SS_PIN         5    // MFRC522 SS (SDA)
#define RST_PIN        22   // MFRC522 RST
#define BUZZER_PIN     15    // Buzzer aktif HIGH

// 8 pin relay
const int RELAY_PINS[8] = {21, 12, 25, 26, 13, 14, 33, 32};
#define RELAY_ACTIVE_HIGH  false  // ubah ke false jika relay board aktif LOW

// =================== RFID ===================
MFRC522 mfrc522(SS_PIN, RST_PIN);

// =================== NVS ===================
Preferences prefs;
const char *NVS_NS = "rfid";
const char *NVS_KEY_COUNT = "count";  // uint16_t
#define MAX_CARDS 64

struct CardEntry {
  String uid;     
  uint8_t mask;   
};
#include <vector>
std::vector<CardEntry> cards;

// =================== RainMaker ===================
static Node my_node;
static Device rfid_dev("RFID_Controller");   // Device utama
static Device relay_dev("Relay_Controller"); // Device baru
static uint8_t wifiLed = 2;  //D2



// Nama parameter
const char *P_STATUS      = "Status";
const char *P_ADD_MODE    = "AddMode";
const char *P_REMOVE_MODE = "RemoveMode";
const char *P_LISTCARDS   = "ListCards";
const char *P_LASTACCESS  = "LastAccess";

String statusStr = "NORMAL";
bool addMode = false;
bool removeMode = false;
bool mark[8] = {false,false,false,false,false,false,false,false};
bool relayState[8] = {false,false,false,false,false,false,false,false};
unsigned long relayOffAt[8] = {0,0,0,0,0,0,0,0};
const char *P_MARK[8]  = {"LT1","LT2","LT3","LT4","LT5","LT6","LT7","LT8"};
const char *P_RELAY[8] = {"LT1","LT2","LT3","LT4","LT5","LT6","LT7","LT8"};

// Mode timeout
const uint32_t MODE_TIMEOUT_MS = 15000;
uint32_t modeStartMs = 0;

// =================== Provisioning ===================
const char *service_name = "Prov_ESP32";
const char *pop = "abcd1234";

// =================== Util: Buzzer ===================
void buzzOnce(uint16_t onMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(onMs);
  digitalWrite(BUZZER_PIN, LOW);
}
void buzzer_add()      { buzzOnce(120); delay(60); buzzOnce(60); }
void buzzer_remove()   { buzzOnce(60);  delay(60); buzzOnce(60); }
void buzzer_granted()  { buzzOnce(200); }
void buzzer_denied()   { buzzOnce(60); }
void buzzer_timeout3() { buzzOnce(60); delay(80); buzzOnce(60); delay(80); buzzOnce(60); }

// =================== Util: Waktu ===================
String formatTimeNow() {
  time_t now = time(nullptr);
  if (now < 1600000000) {
    return String("NTP_SYNCING");
  }
  struct tm tm_info;
  localtime_r(&now, &tm_info);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_info);
  return String(buf);
}

// =================== Util: Relay ===================
void setRelay(int idx, bool on) {
  if (idx < 0 || idx >= 8) return;
  relayState[idx] = on;
  digitalWrite(RELAY_PINS[idx], (RELAY_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH)));
  // pantulkan ke app (device relay)
  relay_dev.updateAndReportParam(P_RELAY[idx], (bool)on);
}

void pulseRelayAutoOff(int idx, uint32_t ms = 5000) {
  setRelay(idx, true);
  relayOffAt[idx] = millis() + ms;
}

// =================== Util: UID & Daftar ===================
String uidToString(const MFRC522::Uid &uid) {
  String s;
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) s += "0";
    s += String(uid.uidByte[i], HEX);
  }
  s.toUpperCase();
  return s;
}

int findCardIndex(const String &uid) {
  for (size_t i = 0; i < cards.size(); i++) {
    if (cards[i].uid == uid) return (int)i;
  }
  return -1;
}

uint8_t currentMarkMask() {
  uint8_t m = 0;
  for (int i=0;i<8;i++) if (mark[i]) m |= (1 << i);
  return m;
}

String maskToList(uint8_t m) {
  String s;
  bool first = true;
  for (int i=0;i<8;i++) if (m & (1<<i)) {
    if (!first) s += ",";
    s += "R"; s += (i+1);
    first = false;
  }
  if (first) s = "-";
  return s;
}

String buildListCardsText() {
  String s;
  for (size_t i=0;i<cards.size(); i++) {
    s += cards[i].uid;
    s += "  [";
    s += maskToList(cards[i].mask);
    s += "]";
    if (i != cards.size()-1) s += "\n";
  }
  return s;
}

void updateStatus(const char *st) {
  statusStr = st;
  rfid_dev.updateAndReportParam(P_STATUS, (char*)statusStr.c_str());
}

void updateListCardsParam() {
  String text = buildListCardsText();
  rfid_dev.updateAndReportParam(P_LISTCARDS, (char*)text.c_str());
}

void updateLastAccessParam(const String &uid, bool granted, uint8_t mask) {
  String s = uid + " | " + formatTimeNow() + " | " + (granted ? "GRANTED " : "DENIED ");
  if (granted) { s += "["; s += maskToList(mask); s += "]"; }
  rfid_dev.updateAndReportParam(P_LASTACCESS, (char*)s.c_str());
}

// =================== NVS Save/Load ===================
void saveCardsToNVS() {
  prefs.begin(NVS_NS, false);
  uint16_t prev = prefs.getUShort(NVS_KEY_COUNT, 0);
  uint16_t cnt = (cards.size() > MAX_CARDS) ? MAX_CARDS : (uint16_t)cards.size();
  prefs.putUShort(NVS_KEY_COUNT, cnt);
  for (uint16_t i=0;i<cnt;i++) {
    char ku[8], km[8];
    snprintf(ku, sizeof(ku), "u%02u", i);
    snprintf(km, sizeof(km), "m%02u", i);
    prefs.putString(ku, cards[i].uid);
    prefs.putUChar(km, cards[i].mask);
  }
  for (uint16_t i=cnt;i<prev;i++) {
    char ku[8], km[8];
    snprintf(ku, sizeof(ku), "u%02u", i);
    snprintf(km, sizeof(km), "m%02u", i);
    prefs.remove(ku);
    prefs.remove(km);
  }
  prefs.end();
}

void loadCardsFromNVS() {
  cards.clear();
  prefs.begin(NVS_NS, true);
  uint16_t cnt = prefs.getUShort(NVS_KEY_COUNT, 0);
  cnt = min<uint16_t>(cnt, MAX_CARDS);
  for (uint16_t i=0;i<cnt;i++) {
    char ku[8], km[8];
    snprintf(ku, sizeof(ku), "u%02u", i);
    snprintf(km, sizeof(km), "m%02u", i);
    String uid = prefs.getString(ku, "");
    uint8_t mask = prefs.getUChar(km, 0);
    if (uid.length()) {
      cards.push_back({uid, mask});
    }
  }
  prefs.end();
}

// =================== Mode helpers ===================
void enterAddMode() {
  addMode = true; removeMode = false;
  modeStartMs = millis();
  updateStatus("ADD_MODE");
}

void enterRemoveMode() {
  removeMode = true; addMode = false;
  modeStartMs = millis();
  updateStatus("REMOVE_MODE");
}

//void exitToNormal(bool fromTimeout=false) {
  //addMode = false; removeMode = false;
  //updateStatus("NORMAL");
  //rfid_dev.updateAndReportParam(P_ADD_MODE, false);
  //rfid_dev.updateAndReportParam(P_REMOVE_MODE, false);
  //if (fromTimeout) buzzer_timeout3();
//}

void exitToNormal(bool fromTimeout=false) {
  addMode = false; 
  removeMode = false;
  updateStatus("NORMAL");
  rfid_dev.updateAndReportParam(P_ADD_MODE, false);
  rfid_dev.updateAndReportParam(P_REMOVE_MODE, false);

  // 🔹 Reset semua mark ke false
  for (int i = 0; i < 8; i++) {
    mark[i] = false;
    rfid_dev.updateAndReportParam(P_MARK[i], false);
  }

  if (fromTimeout) buzzer_timeout3();
}


// =================== RainMaker Callbacks ===================
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
      printQR(service_name, pop, "ble");
#else
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
#endif
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("\nConnected to Wi-Fi!");
      digitalWrite(wifiLed, HIGH);  // LED ON
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("\nDisconnected from Wi-Fi!");
      digitalWrite(wifiLed, LOW);   // LED OFF
      break;
  }
}


void write_callback(Device *device, Param *param, const param_val_t val, void *priv, write_ctx_t *ctx)
{
  const char *pname = param->getParamName();

  // Add / Remove mode
  if (!strcmp(pname, P_ADD_MODE)) {
    bool v = val.val.b;
    if (v) enterAddMode(); else exitToNormal(false);
    param->updateAndReport(val);
    return;
  }
  if (!strcmp(pname, P_REMOVE_MODE)) {
    bool v = val.val.b;
    if (v) enterRemoveMode(); else exitToNormal(false);
    param->updateAndReport(val);
    return;
  }

  // Mark toggles
  for (int i=0;i<8;i++) {
    if (!strcmp(pname, P_MARK[i])) {
      mark[i] = val.val.b;
      param->updateAndReport(val);
      return;
    }
  }

  // Manual Relay buttons
  for (int i=0;i<8;i++) {
    if (!strcmp(pname, P_RELAY[i])) {
      bool v = val.val.b;
      if (v) {
        pulseRelayAutoOff(i, 5000);
      } else {
        setRelay(i, false);
        relayOffAt[i] = 0;
      }
      param->updateAndReport(val);
      return;
    }
  }

  // RO param reflect back
  param_val_t cur;
  if (!strcmp(pname, P_STATUS)) { cur = value((char*)statusStr.c_str()); param->updateAndReport(cur); }
  if (!strcmp(pname, P_LISTCARDS)) { String t = buildListCardsText(); cur = value((char*)t.c_str()); param->updateAndReport(cur); }
}

// =================== Setup ===================
void setup() {
  Serial.begin(115200);

  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, LOW);  // awalnya OFF

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  for (int i=0;i<8;i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    setRelay(i, false);
  }

  SPI.begin();
  mfrc522.PCD_Init();

  loadCardsFromNVS();

  my_node = RMaker.initNode("ESP32_RFID_Controller");

  // Device 1: RFID_Controller
  {
    Param p(P_STATUS, NULL, value((char*)"NORMAL"), PROP_FLAG_READ);
    p.addUIType(ESP_RMAKER_UI_TEXT);
    rfid_dev.addParam(p);
  }
  {
    Param p1(P_ADD_MODE, NULL, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    p1.addUIType(ESP_RMAKER_UI_TOGGLE);
    rfid_dev.addParam(p1);
    Param p2(P_REMOVE_MODE, NULL, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    p2.addUIType(ESP_RMAKER_UI_TOGGLE);
    rfid_dev.addParam(p2);
  }
  for (int i=0;i<8;i++) {
    Param pm(P_MARK[i], NULL, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE | PROP_FLAG_PERSIST);
    pm.addUIType(ESP_RMAKER_UI_TOGGLE);
    rfid_dev.addParam(pm);
  }
  {
    Param p3(P_LISTCARDS, NULL, value((char*)buildListCardsText().c_str()), PROP_FLAG_READ);
    p3.addUIType(ESP_RMAKER_UI_TEXT);
    rfid_dev.addParam(p3);
    Param p4(P_LASTACCESS, NULL, value((char*)"-"), PROP_FLAG_READ);
    p4.addUIType(ESP_RMAKER_UI_TEXT);
    rfid_dev.addParam(p4);
  }
  rfid_dev.addCb(write_callback);
  my_node.addDevice(rfid_dev);

  // Device 2: Relay_Controller
  for (int i=0;i<8;i++) {
    Param pr(P_RELAY[i], NULL, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
    pr.addUIType(ESP_RMAKER_UI_TOGGLE);
    relay_dev.addParam(pr);
  }
  relay_dev.addCb(write_callback);
  my_node.addDevice(relay_dev);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.setTimeZone("Asia/Jakarta");
  RMaker.enableSchedule();

  RMaker.start();

  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

  updateListCardsParam();
}

// =================== Loop ===================
void loop() {
  uint32_t now = millis();
  for (int i=0;i<8;i++) {
    if (relayOffAt[i] && (int32_t)(now - relayOffAt[i]) >= 0) {
      setRelay(i, false);
      relayOffAt[i] = 0;
    }
  }

  if ((addMode || removeMode) && (now - modeStartMs >= MODE_TIMEOUT_MS)) {
    exitToNormal(true);
  }

  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    String uid = uidToString(mfrc522.uid);
    mfrc522.PICC_HaltA();
    int idx = findCardIndex(uid);

    if (addMode) {
      uint8_t m = currentMarkMask();
      if (idx >= 0) {
        cards[idx].mask = m;
      } else if ((int)cards.size() < MAX_CARDS) {
        cards.push_back({uid, m});
      }
      saveCardsToNVS();
      updateListCardsParam();
      updateLastAccessParam(uid, true, m);
      buzzer_add();
      exitToNormal(false);
    }
    else if (removeMode) {
      if (idx >= 0) {
        cards.erase(cards.begin() + idx);
        saveCardsToNVS();
        updateListCardsParam();
        updateLastAccessParam(uid, true, 0);
        buzzer_remove();
      } else {
        updateLastAccessParam(uid, false, 0);
        buzzer_denied();
      }
      exitToNormal(false);
    }
    else {
      if (idx >= 0 && cards[idx].mask != 0) {
        uint8_t m = cards[idx].mask;
        for (int i=0;i<8;i++) if (m & (1<<i)) {
          pulseRelayAutoOff(i, 5000);
        }
        updateLastAccessParam(uid, true, m);
        buzzer_granted();
      } else {
        updateLastAccessParam(uid, false, 0);
        buzzer_denied();
      }
    }
  }

  delay(10);
}
