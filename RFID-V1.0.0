#include <EEPROM.h>
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <SPI.h>
#include <MFRC522.h>
#include <time.h>

// ================= CONFIG =================
#define ENABLE_EEPROM true
#define EEPROM_SIZE 2048   // increased to store card DB + logs

const char *service_name = "PROV_12345";
const char *pop = "1234567";

// Device Names (8 relays)
char deviceName_1[] = "Switch1";
char deviceName_2[] = "Switch2";
char deviceName_3[] = "Switch3";
char deviceName_4[] = "Switch4";
char deviceName_5[] = "Switch5";
char deviceName_6[] = "Switch6";
char deviceName_7[] = "Switch7";
char deviceName_8[] = "Switch8";

// RFID-related device names
char rfid_add_name[]    = "RFID_Add_Mode";
char rfid_remove_name[] = "RFID_Remove_Mode";
char rfid_list_name[]   = "RFID_List";
char rfid_lastaccess_name[] = "RFID_LastAccess"; // virtual display

// ====================== HARDWARE PINS ======================
uint8_t RelayPin[] = { 21, 12, 13, 14, 25, 26, 27, 32, 33 }; // 8 relays (active LOW)
uint8_t RELAY9_PIN = 33; // new relay9 controlled by RFID (active LOW)

const uint8_t MFRC522_SS_PIN = 5;   // SDA/SS for MFRC522
const uint8_t MFRC522_RST_PIN = 22; // RST for MFRC522

uint8_t wifiLed    = 2;   // LED indikator koneksi WiFi
uint8_t gpio_reset = 0;   // Tombol reset (WiFi / Factory)
uint8_t buzzerPin  = 15;  // Buzzer

// ================= EEPROM layout =================
const int CARD_START_ADDR = 1;
const int CARD_SLOT_BYTES = 16;
const int MAX_CARDS = 80;
const int LOG_START_ADDR = CARD_START_ADDR + (CARD_SLOT_BYTES * MAX_CARDS);
const int LOG_ENTRY_BYTES = 12;
const int MAX_LOGS = 40;

// ================= GLOBALS =================
bool toggleState[8] = { LOW };
Switch *my_switch[8];

Switch *rfidAddSwitch = nullptr;
Switch *rfidRemoveSwitch = nullptr;
Switch *rfidListSwitch = nullptr;
Switch *rfidLastAccessSwitch = nullptr;

bool rfidAddMode = false;
bool rfidRemoveMode = false;
unsigned long rfidModeExpireAt = 0;
const unsigned long RFID_MODE_TIMEOUT_MS = 30000;

bool relay9_state = false;
unsigned long relay9_on_at = 0;
const unsigned long RELAY9_AUTO_OFF_MS = 5000;

struct LogEntry {
  uint32_t ts;
  uint8_t event;
  uint8_t uidIndex;
  char shortUid[6];
};
LogEntry logs[MAX_LOGS];
int logCount = 0;
int logPos = 0;

MFRC522 mfrc(MFRC522_SS_PIN, MFRC522_RST_PIN);
Node my_node;

// ================= EEPROM HELPERS =================
void eepromInit() {
  if (ENABLE_EEPROM) EEPROM.begin(EEPROM_SIZE);
}

void eepromCommit() { if (ENABLE_EEPROM) EEPROM.commit(); }
uint8_t eepromReadByte(int addr) { return ENABLE_EEPROM ? EEPROM.read(addr) : 0; }
void eepromWriteByte(int addr, uint8_t v) { if (ENABLE_EEPROM) EEPROM.write(addr, v); }

// Card functions
void storeCardSlot(int idx, const String &uid) {
  int base = CARD_START_ADDR + idx * CARD_SLOT_BYTES;
  for (int i = 0; i < CARD_SLOT_BYTES; i++)
    eepromWriteByte(base + i, i < uid.length() ? uid[i] : 0);
  eepromCommit();
}

String readCardSlot(int idx) {
  int base = CARD_START_ADDR + idx * CARD_SLOT_BYTES;
  String s = "";
  for (int i = 0; i < CARD_SLOT_BYTES; i++) {
    uint8_t b = eepromReadByte(base + i);
    if (!b) break;
    s += (char)b;
  }
  return s;
}

int getCardCount() { return eepromReadByte(0); }
void setCardCount(int c) { eepromWriteByte(0, (uint8_t)c); eepromCommit(); }

int findCardIndex(const String &uid) {
  int cnt = getCardCount();
  for (int i = 0; i < cnt; i++) if (readCardSlot(i).equals(uid)) return i;
  return -1;
}

int addCard(const String &uid) {
  if (uid.length() == 0) return -1;
  int idx = findCardIndex(uid);
  if (idx >= 0) return idx;
  int cnt = getCardCount();
  if (cnt >= MAX_CARDS) return -1;
  storeCardSlot(cnt, uid);
  setCardCount(cnt + 1);
  Serial.printf("[RFID] Card added idx=%d uid=%s\n", cnt, uid.c_str());
  return cnt;
}

bool removeCard(const String &uid) {
  int idx = findCardIndex(uid);
  if (idx < 0) return false;
  int cnt = getCardCount();
  for (int i = idx; i < cnt - 1; i++) storeCardSlot(i, readCardSlot(i + 1));
  int lastBase = CARD_START_ADDR + (cnt - 1) * CARD_SLOT_BYTES;
  for (int i = 0; i < CARD_SLOT_BYTES; i++) eepromWriteByte(lastBase + i, 0);
  setCardCount(cnt - 1);
  eepromCommit();
  Serial.printf("[RFID] Card removed idx=%d\n", idx);
  return true;
}

// ================= LOG helpers =================
void pushLog(uint8_t event, int uidIndex, const String &uid) {
  uint32_t ts = (uint32_t)(millis() / 1000);
  LogEntry le;
  le.ts = ts;
  le.event = event;
  le.uidIndex = (uidIndex < 0) ? 255 : uidIndex;
  memset(le.shortUid, 0, sizeof(le.shortUid));
  for (int i = 0; i < 5 && i < uid.length(); i++) le.shortUid[i] = uid[i];
  logs[logPos] = le;
  logPos = (logPos + 1) % MAX_LOGS;
  if (logCount < MAX_LOGS) logCount++;

  int slot = (logPos == 0) ? (MAX_LOGS - 1) : (logPos - 1);
  int eaddr = LOG_START_ADDR + slot * LOG_ENTRY_BYTES;
  if (ENABLE_EEPROM) {
    EEPROM.write(eaddr + 0, (ts >> 24) & 0xFF);
    EEPROM.write(eaddr + 1, (ts >> 16) & 0xFF);
    EEPROM.write(eaddr + 2, (ts >> 8) & 0xFF);
    EEPROM.write(eaddr + 3, (ts >> 0) & 0xFF);
    EEPROM.write(eaddr + 4, event);
    EEPROM.write(eaddr + 5, (uidIndex < 0) ? 255 : uidIndex);
    for (int i = 0; i < 6; i++) EEPROM.write(eaddr + 6 + i, (i < 5) ? le.shortUid[i] : 0);
    EEPROM.commit();
  }
}

// ================= EEPROM init helpers =================
void ensureEepromInitialized() {
  eepromInit();
  if (getCardCount() > MAX_CARDS) setCardCount(0);
}

// ================= Relay control =================
void writeEEPROMRelayState(int addr, bool state) { eepromWriteByte(100 + addr, state ? 1 : 0); eepromCommit(); }
bool readEEPROMRelayState(int addr) { return eepromReadByte(100 + addr); }

void setRelay(uint8_t pin, int addr, bool state) {
  digitalWrite(pin, !state);
  writeEEPROMRelayState(addr, state);
}

void setRelay9(bool state) {
  relay9_state = state;
  digitalWrite(RELAY9_PIN, !state);
  writeEEPROMRelayState(8, state);
}

// ================= Buzzer helper =================
void buzz(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(duration);
    digitalWrite(buzzerPin, LOW);
    delay(duration);
  }
}

// ================= MFRC522 helpers =================
String uidToString(MFRC522::Uid uid) {
  String s = "";
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) s += "0";
    s += String(uid.uidByte[i], HEX);
  }
  s.toUpperCase();
  return s;
}

void processUID(const String &uid) {
  if (uid.length() == 0) return;
  Serial.printf("[RFID] UID=%s\n", uid.c_str());

  if (rfidAddMode) {
    int idx = addCard(uid);
    if (idx >= 0) { pushLog(3, idx, uid); buzz(2, 100); } else { buzz(2, 300); }
    rfidAddMode = false;
    return;
  } 
  else if (rfidRemoveMode) {
    bool ok = removeCard(uid);
    if (ok) { pushLog(4, -1, uid); buzz(2, 100); } else { buzz(2, 300); }
    rfidRemoveMode = false;
    return;
  } 
  else {
    int idx = findCardIndex(uid);
    if (idx >= 0) { pushLog(1, idx, uid); setRelay9(true); relay9_on_at = millis(); buzz(3, 80); }
    else { pushLog(2, -1, uid); buzz(2, 300); }
  }
}

// ================= RainMaker write callback =================
void write_callback(Device *device, Param *param, const param_val_t val,
                    void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if (strcmp(param_name, "Power") == 0) {
    bool newState = val.val.b;
    for (int i = 0; i < 8; i++) {
      if (strcmp(device_name, my_switch[i]->getDeviceName()) == 0) {
        setRelay(RelayPin[i], i, newState);
        toggleState[i] = newState;
        my_switch[i]->updateAndReportParam(param_name, newState);
        return;
      }
    }
    if (rfidAddSwitch && strcmp(device_name, rfidAddSwitch->getDeviceName()) == 0) {
      rfidAddMode = true; rfidRemoveMode = false; rfidModeExpireAt = millis() + RFID_MODE_TIMEOUT_MS;
      rfidAddSwitch->updateAndReportParam(param_name, false);
      return;
    }
    if (rfidRemoveSwitch && strcmp(device_name, rfidRemoveSwitch->getDeviceName()) == 0) {
      rfidRemoveMode = true; rfidAddMode = false; rfidModeExpireAt = millis() + RFID_MODE_TIMEOUT_MS;
      rfidRemoveSwitch->updateAndReportParam(param_name, false);
      return;
    }
    if (rfidListSwitch && strcmp(device_name, rfidListSwitch->getDeviceName()) == 0) {
      rfidListSwitch->updateAndReportParam(param_name, false);
      return;
    }
    if (rfidLastAccessSwitch && strcmp(device_name, rfidLastAccessSwitch->getDeviceName()) == 0) {
      rfidLastAccessSwitch->updateAndReportParam(param_name, false);
      return;
    }
  }
}

// ================= Provisioning / events =================
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START: printQR(service_name, pop, "ble"); break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED: digitalWrite(wifiLed, true); break;
  }
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  delay(50);

  ensureEepromInitialized();

  for (int i = 0; i < 8; i++) {
    pinMode(RelayPin[i], OUTPUT);
    toggleState[i] = readEEPROMRelayState(i);
    setRelay(RelayPin[i], i, toggleState[i]);
  }
  pinMode(RELAY9_PIN, OUTPUT);
  setRelay9(readEEPROMRelayState(8));

  pinMode(wifiLed, OUTPUT);
  pinMode(gpio_reset, INPUT);
  pinMode(buzzerPin, OUTPUT);

  my_node = RMaker.initNode("Madhonnnnn");

  char *deviceNames[] = { deviceName_1, deviceName_2, deviceName_3, deviceName_4,
                          deviceName_5, deviceName_6, deviceName_7, deviceName_8 };
  for (int i = 0; i < 8; i++) {
    my_switch[i] = new Switch(deviceNames[i], &RelayPin[i]);
    my_switch[i]->addCb(write_callback);
    my_node.addDevice(*my_switch[i]);
    my_switch[i]->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState[i]);
  }

  rfidAddSwitch = new Switch(rfid_add_name, NULL); rfidAddSwitch->addCb(write_callback); my_node.addDevice(*rfidAddSwitch);
  rfidRemoveSwitch = new Switch(rfid_remove_name, NULL); rfidRemoveSwitch->addCb(write_callback); my_node.addDevice(*rfidRemoveSwitch);
  rfidListSwitch = new Switch(rfid_list_name, NULL); rfidListSwitch->addCb(write_callback); my_node.addDevice(*rfidListSwitch);
  rfidLastAccessSwitch = new Switch(rfid_lastaccess_name, NULL); rfidLastAccessSwitch->addCb(write_callback); my_node.addDevice(*rfidLastAccessSwitch);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  RMaker.start();

  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);

  SPI.begin();
  mfrc.PCD_Init();
}

// ================= Loop =================
void loop() {
  if (digitalRead(gpio_reset) == LOW) {
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int duration = millis() - startTime;
    if (duration > 10000) RMakerFactoryReset(2);
    else if (duration > 3000) RMakerWiFiReset(2);
  }

  digitalWrite(wifiLed, WiFi.status() == WL_CONNECTED);

  static unsigned long lastReport = 0;
  if (millis() - lastReport > 60000) {
    for (int i = 0; i < 8; i++) my_switch[i]->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState[i]);
    if (rfidLastAccessSwitch) rfidLastAccessSwitch->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, false);
    lastReport = millis();
  }

  if (mfrc.PICC_IsNewCardPresent() && mfrc.PICC_ReadCardSerial()) {
    processUID(uidToString(mfrc.uid));
    mfrc.PICC_HaltA();
    mfrc.PCD_StopCrypto1();
    delay(150);
  }

  if ((rfidAddMode || rfidRemoveMode) && (millis() > rfidModeExpireAt)) {
    rfidAddMode = false;
    rfidRemoveMode = false;
  }

  if (relay9_state && (millis() - relay9_on_at > RELAY9_AUTO_OFF_MS)) setRelay9(false);

  delay(10);
}
