/*
 * ======================================================================================
 * BẢN GHI CHÚ CẬP NHẬT (RELEASE NOTES) - SMART SOLAR METER
 * ======================================================================================
 * 1. Sửa lỗi nghiêm trọng (Critical Fixes - Chống Crash & Treo Mạch)
 * - Chống Panic Reboot do Watchdog Timer (WDT) khi nạp OTA: Thêm cờ `isSystemUpdating` 
 * để ngừng giao tiếp phần cứng nhưng vẫn duy trì `esp_task_wdt_reset()` để nuôi WDT.
 * - Chống đụng độ cổng UART (Hardware Race Condition): Phân quyền lại để Core 1 chỉ bật 
 * cờ hiệu (`flag_reset_ac`), nhường Core 0 an toàn xuất lệnh reset PZEM ra UART.
 *
 * 2. Bảo vệ tính toàn vẹn dữ liệu (Data Integrity)
 * - Khắc phục sai lệch số điện (Word Tearing trên biến 64-bit): Chuyển sang lấy dữ liệu 
 * từ `currentData.ac_energy` (đã được khóa an toàn bằng Mutex) khi lưu `saveEnergyData()`.
 * - Chống rớt gói tin MQTT (JSON Payload Truncation): Thêm lệnh `mqttClient.setBufferSize(512)` 
 * để mở rộng khoang chứa gói tin, chống tràn bộ đệm mặc định 256 byte của PubSubClient.
 *
 * 3. Tối ưu hiệu năng & Tuổi thọ thiết bị (Optimization & Longevity)
 * - Bảo vệ Flash ESP32 (NVS Wear Leveling): Nâng ngưỡng lệch lưu trữ Auto-save lên 0.1 kWh 
 * (100Wh) cho AC và 100.0 Wh cho DC, giúp giảm tần suất ghi và kéo dài tuổi thọ chip.
 * - Sửa lỗi ngưng đọc cảm biến sau 49.7 ngày: Thay điều kiện `millis() < 3000` bằng cờ 
 * `static bool isBooting = true;` để tránh lỗi tràn biến `millis()` của hệ thống.
 * - Giảm nghẽn cổ chai CPU ở Core 1: Đổi `delay(1);` thành `vTaskDelay(pdMS_TO_TICKS(1));` 
 * chuẩn FreeRTOS để hệ điều hành phân bổ tài nguyên mượt mà hơn.
 * ======================================================================================
 */

// --- KHAI BÁO CÁC THƯ VIỆN ---
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusMaster.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <time.h>
#include <WiFiUdp.h>
// --- THƯ VIỆN & CẤU HÌNH MQTT ---
#include <PubSubClient.h>

char mqtt_server[40] = "broker.emqx.io"; // Mặc định dùng máy chủ Test miễn phí
int mqtt_port = 1883;
char mqtt_user[32] = "admin";            // Sau này tạo VPS sẽ dùng
char mqtt_pass[32] = "123456";           // Sau này tạo VPS sẽ dùng
char device_id[16] = "";     // Mã ID 6 số của thiết bị này
bool enableMQTT = false; // Mặc định tắt để tránh treo mạch khi chưa có VPS
WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long t_mqtt_reconnect = 0;
unsigned long t_mqtt_push = 0;
// --------------------------------
// --- CẤU HÌNH HỆ THỐNG ---
WiFiUDP udp;
#define UDP_PORT 4210
#define WDT_TIMEOUT 60 // Watchdog 60s
#define FW_VERSION "V11"

// --- ĐỊNH NGHĨA CHÂN KẾT NỐI ---
#define ONE_WIRE_BUS      14
#define PZEM_AC_RX_PIN    16
#define PZEM_AC_TX_PIN    17
#define PZEM_DC_RX_PIN    26
#define PZEM_DC_TX_PIN    25
#define RELAY_PIN         32 // Relay 1 (Main)
#define RELAY2_PIN        33 // Relay 2 (AC Protection)
#define RELAY3_PIN        21 // Relay 3 (DC Window Control)
#define RELAY4_PIN        22 // Dự phòng Relay 4 (Chưa dùng)
#define GRID_DETECT_PIN   13
#define TFT_BL_PIN        27 // Chân điều khiển đèn nền màn hình

// NÚT BẤM (INPUT_PULLUP)
#define BTN_UP_PIN        36
#define BTN_MENU_PIN      35
#define BTN_DOWN_PIN      34

// --- MÀU SẮC TFT ---
#define C_BLACK       TFT_BLACK
#define C_VOLT        0x07FF
#define C_POWER       TFT_WHITE
#define C_AMP         0xFD20
#define C_TEMP        0x07E0
#define C_LABEL       TFT_SILVER
#define C_WHITE       TFT_WHITE
#define C_BOX_T1      0x03EF
#define C_BOX_T2      0xD340
#define C_BOX_T3      0x9000
#define C_RELAY_ON    TFT_GREEN
#define C_RELAY_OFF   TFT_RED
#define C_RELAY_STOP  TFT_ORANGE
#define C_BG_AUTO     0x03EF
#define C_BG_MANU     0x9000
#define C_BG_BACKUP   0xD340
#define C_BG_WAIT     0xE8E4
#define C_BG_IDLE     0x4208
#define C_MENU_BG     0x10A2
#define C_MENU_SEL    0xD69A
#define C_KEY_BG      0x2124
#define C_KEY_SEL     TFT_ORANGE

// --- CHẾ ĐỘ HOẠT ĐỘNG ---
#define MODE_AUTO     0
#define MODE_MANUAL   1
#define MODE_BACKUP   2

// --- CÁC MÀN HÌNH ---
#define SCREEN_MAIN             0
#define SCREEN_MENU             1
#define SCREEN_WIFI_SCAN        2  
#define SCREEN_WIFI_PASS        3  
#define SCREEN_WIFI_CONNECTING  4  
#define SCREEN_WIFI_SCANNING    5  
#define SCREEN_ABOUT            6
#define SCREEN_OTA              7
#define SCREEN_SENSOR_CHECK     8  // <-- MÀN HÌNH CHẨN ĐOÁN (Số 8)
#define SCREEN_VPS_INPUT        9  
char inputVPS[41] = "";
// --- DÁN 2 DÒNG NÀY VÀO ĐÂY ĐỂ MỌI HÀM ĐỀU THẤY ---
volatile int ac_error_count = 10; 
volatile int dc_error_count = 10;
// -------------------------------------------------

// [STRUCT] DỮ LIỆU ĐO
struct DataBox {
  float dc_volt; float dc_amp; float dc_power; float dc_energy;
  float ac_volt; float ac_amp; float ac_power; float ac_energy;
  float ac_freq; float ac_pf;
  float t1; float t2; float t3;
};

// [STRUCT] NÚT BẤM
struct Button {
  uint8_t pin;              
  bool stableState;        
  bool lastReading;        
  unsigned long lastDebounceTime;
  unsigned long pressTime;
  unsigned long repeatTimer;
  bool eventHandled;        
};

// --- ĐỐI TƯỢNG TOÀN CỤC ---
TFT_eSPI tft = TFT_eSPI();
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
ModbusMaster nodeDC; ModbusMaster nodeAC;
HardwareSerial SerialDC(1); HardwareSerial SerialAC(2);
WebServer server(80); Preferences preferences;
SemaphoreHandle_t dataMutex; WidgetTerminal terminal(V50);

// --- BIẾN GIAO TIẾP ---
volatile bool flag_sync_relay = false;
volatile bool flag_sync_settings = false;
volatile bool flag_req_setShunt = false;
volatile uint16_t val_req_shunt = 0x0000;
volatile int flag_shunt_result = 0;
volatile bool flag_reset_ac = false;
volatile bool flag_reset_dc = false;
volatile bool isSystemUpdating = false; // [FIX 1] Cờ báo hiệu đang nạp OTA
// --- BIẾN QUẢN LÝ NĂNG LƯỢNG ĐỘC LẬP (ESP32 TỰ TÍNH TRÊN RAM) ---
volatile double esp_total_ac_energy = 0;  // Lưu tổng điện AC (kWh)
volatile double esp_total_dc_energy = 0;  // Lưu tổng điện DC (Wh)
double last_saved_ac_energy = -1;         // Nhớ mốc đã lưu Flash
double last_saved_dc_energy = -1;
volatile bool update_screen_now = false;;

DataBox sharedData = {0}; DataBox currentData = {0}; DataBox oldData = { -9999 };
const DataBox emptyData = { -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999 };

// --- BIẾN LOGIC ---
// --- BIẾN HIỆU CHỈNH (CALIBRATION) ---
float calib_v_ac = 1.000;
float calib_a_ac = 1.000;
float calib_v_dc = 1.000;
float calib_a_dc = 1.000;
float latch_ac_day = 0; float latch_ac_month = 0;
float latch_dc_day = 0; float latch_dc_month = 0;
// THÊM 2 DÒNG NÀY ĐỂ LƯU DỮ LIỆU NGÀY TRƯỚC / THÁNG TRƯỚC
float prev_ac_day = 0; float prev_ac_month = 0;
float prev_dc_day = 0; float prev_dc_month = 0;
int current_day = -1; int current_month = -1;
int energyResetHour = 5; int energyBillingDate = 1; int blynkInterval = 1000;

int systemMode = MODE_AUTO; int oldSystemMode = -1;
bool relayState = false; bool oldRelayState = !relayState;
bool manualRelayCmd = false;
float highThreshold = 53.0; float lowThreshold = 50.0;
int delayHigh = 5; int delayLow = 5;
unsigned long relayTimerStart = 0;

volatile float load_core0 = 0; volatile float load_core1 = 0;
bool forceOff = false; bool oldForceOff = !forceOff;
char debugStatus[20] = ""; char oldDebugStatus[20] = "xx";
bool relay2State = false;
bool acProtectionActive = false;
unsigned long acProtectionTimerStart = 0;

// BIẾN BẢO VỆ QUÁ TẢI (RELAY 2)
float overloadLimitW = 10000.0;
int overloadRecoverySec = 60;  
bool isOverloadTripped = false;
unsigned long overloadTripTimer = 0;

// BIẾN RELAY 3 VÀ 4
bool enableRelay3 = true;
int r3_mode = 0; // 0: Theo DC Volt, 1: Theo AC Power
bool relay3State = false; bool oldRelay3State = !relay3State;
float r3_min_volt = 48.0;
float r3_max_volt = 58.0;
unsigned long relay3_last_switch_time = 0;

bool enableRelay4 = false;
int r4_mode = 0; // 0: Theo DC Volt, 1: Theo AC Power
bool relay4State = false; bool oldRelay4State = !relay4State;
float r4_min_volt = 48.0;
float r4_max_volt = 58.0;
unsigned long relay4_last_switch_time = 0;

unsigned long relay1_last_switch_time = 0;

// BIẾN THEO DÕI THAO TÁC NGƯỜI DÙNG
unsigned long t_last_user_action = 0;

char blynk_token[34] = ""; char blynk_server[40] = "blynkvn.ddns.net";
char custom_ssid[33] = ""; 
char custom_pass[65] = "";
bool shouldSaveConfig = false;
unsigned long t_blynk_update = 0; int oldWifiBars = -2;
bool isFirstConnect = true;

// --- BIẾN WIFI SCAN & KEYBOARD ---
int wifiScanCursor = 0;
int wifiNetworksFound = 0;
char selectedSSID[33] = ""; 
char inputPassword[65] = ""; 
bool isWebOTA = false;
bool isWifiScanning = false;
unsigned long t_wifi_connect_start = 0;
// Biến phục vụ tối ưu hóa Ping mồi không chặn DNS
IPAddress blynk_cached_ip;
bool is_blynk_ip_resolved = false;
unsigned long t_last_dns_resolve = 0;

// Biến phục vụ chuyển màn hình không dùng hàm delay()
unsigned long t_state_delay = 0;
int next_screen_target = -1;
// MAPPING BÀN PHÍM
const char* kb_map_base = "1234567890qwertyuiopasdfghjklzxcvbnm.@-_";
int kb_cursor = 0; 
int old_kb_cursor = -1; 
bool kb_caps = false; 
#define KB_IDX_CAPS 40
#define KB_IDX_SPACE 41
#define KB_IDX_DEL   42
#define KB_IDX_OK    43
#define KB_TOTAL_KEYS 44

// --- BIẾN MENU ---
int currentScreen = SCREEN_MAIN;
int menuCursor = 0;
bool isEditing = false;
bool menuRedraw = true;
bool menuNeedsUpdate = true;
int timeEditIndex = 0;

struct MenuConfig {
  int mode; float highT; float lowT; int dHigh; int dLow; float ovLoadW; int ovRecS;    
  bool r3En; int r3Mode; float r3Min; float r3Max;
  bool r4En; int r4Mode; float r4Min; float r4Max;
  bool mqttEn;
} tempConfig;

const char* menuItems[] = {
  "Che do (Mode)",      // 0
  "Ap Cao (High V)",    // 1
  "Ap Thap (Low V)",    // 2
  "Delay Bat (s)",      // 3
  "Delay Tat (s)",      // 4
  "Qua Tai (W)",        // 5
  "Tre Qua Tai (s)",    // 6
  "R3 Kich Hoat",       // 7
  "R3 Theo (DC/AC)",    // 8
  "R3 Nguong Min",      // 9
  "R3 Nguong Max",      // 10
  "R4 Kich Hoat",       // 11
  "R4 Theo (DC/AC)",    // 12
  "R4 Nguong Min",      // 13
  "R4 Nguong Max",      // 14
  
  // --- MENU TRANG 4: KET NOI (4/5) ---
  "Ket noi WiFi",       // 15
  "Reset WiFi",         // 16
  "Ket noi Web app",     // 17
  "Dia chi VPS (MQTT)", // 18
  
  // --- MENU TRANG 5: HE THONG (5/5) ---
  "Thong tin (About)",  // 19
  "Test Cam Bien",      // 20
  "Luu & Thoat",        // 21
  "Huy bo & Thoat"      // 22
};
#define MENU_ITEM_COUNT 23      // Giữ nguyên tổng số 23 mục

Button btns[3] = {
  {BTN_UP_PIN, HIGH, HIGH, 0, 0, 0, false},
  {BTN_MENU_PIN, HIGH, HIGH, 0, 0, 0, false},
  {BTN_DOWN_PIN, HIGH, HIGH, 0, 0, 0, false}
};
const unsigned long DEBOUNCE_DELAY = 25;
const unsigned long HOLD_TIME = 2000;

TaskHandle_t Task0;
TaskHandle_t TaskNetwork;

// ======================================================================================
// FORWARD DECLARATIONS
// ======================================================================================
void drawInterface();
void drawAboutScreen();
void updateScreenSmart();
void drawMenu();
void drawWifiScan();
void drawWifiKeyboard();
void startScanWifiNetworks();
void drawWifiConnecting();
void initMenuConfig();
void handleMenuInput(int btnType, bool isHold);
void handleWifiScanInput(int btnType);
void handleWifiKeyboardInput(int btnType);
void handleWifiConnectingInput(int btnType);
void setRelay(bool state);
void syncBlynkRelayState(bool state);
void updateACRelayProtection();
void updateRelay();
void updateRelay3Logic();
void updateBlynk();
void configModeCallback(WiFiManager *wm);
void setupWebOTA();
void saveSettings();
void saveConfigCallback();
void executeAction(int btnIndex, int actionType);
void syncSettingsToBlynk();
void processSerialCommand(String cmd);

// ======================================================================================
// MODULE TIỆN ÍCH
// ======================================================================================
const char* getMacSuffix() {
  static char macBuf[5];
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  String suffix = mac.length() >= 4 ? mac.substring(mac.length() - 4) : "XXXX";
  suffix.toCharArray(macBuf, 5);
  return macBuf;
}

void setupTime() { configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov"); }
void saveConfigCallback () { shouldSaveConfig = true; }

// ======================================================================================
// MODULE LƯU TRỮ (STORAGE)
// ======================================================================================
void loadSettings() {
  preferences.begin("solar_cfg", false);
  systemMode = preferences.getInt("mode", MODE_AUTO);
  highThreshold = preferences.getFloat("highT", 53.0);
  lowThreshold = preferences.getFloat("lowT", 50.0);
  delayHigh = preferences.getInt("dHigh", 5); delayLow = preferences.getInt("dLow", 5);
 
  overloadLimitW = preferences.getFloat("ovW", 10000.0);
  overloadRecoverySec = preferences.getInt("ovRec", 60);

  // --- THÊM 4 DÒNG NÀY ĐỂ LOAD HỆ SỐ CALIBRATION KHI KHỞI ĐỘNG ---
  calib_v_ac = preferences.getFloat("c_vac", 1.000);
  calib_a_ac = preferences.getFloat("c_aac", 1.000);
  calib_v_dc = preferences.getFloat("c_vdc", 1.000);
  calib_a_dc = preferences.getFloat("c_adc", 1.000);
  // -------------------------------------------------------------

  enableRelay3 = preferences.getBool("r3_en", true);
  r3_mode = preferences.getInt("r3_mode", 0);
  r3_min_volt = preferences.getFloat("r3min", 48.0);
  r3_max_volt = preferences.getFloat("r3max", 58.0);

  enableRelay4 = preferences.getBool("r4_en", false);
  r4_mode = preferences.getInt("r4_mode", 0);
  r4_min_volt = preferences.getFloat("r4min", 48.0);
  r4_max_volt = preferences.getFloat("r4max", 58.0);
  energyResetHour = preferences.getInt("rHour", 5); energyBillingDate = preferences.getInt("bDate", 1);
  
  String s = preferences.getString("cssid", ""); s.toCharArray(custom_ssid, 33);
  String p = preferences.getString("cpass", ""); p.toCharArray(custom_pass, 65);
 
  forceOff = preferences.getBool("forceOff", false);

  String t = preferences.getString("token", "");
  if (t.length() > 0) t.toCharArray(blynk_token, 34);
  String sv = preferences.getString("server", "blynkvn.ddns.net");
  if (sv.length() > 0) sv.toCharArray(blynk_server, 40);

  // ...
  String mqtt_sv = preferences.getString("mqtt_srv", "broker.emqx.io");
  mqtt_sv.toCharArray(mqtt_server, 40);
  
  // --- THÊM DÒNG NÀY ĐỂ LOAD TRẠNG THÁI BẬT/TẮT ---
  enableMQTT = preferences.getBool("mqtt_en", false); 
  // -----------------------------------------------

  preferences.end();
}
void saveSettings() {
  preferences.begin("solar_cfg", false);
  preferences.putInt("mode", systemMode); preferences.putFloat("highT", highThreshold);
  preferences.putFloat("lowT", lowThreshold); preferences.putInt("dHigh", delayHigh); preferences.putInt("dLow", delayLow);
 
  preferences.putFloat("ovW", overloadLimitW);
  preferences.putInt("ovRec", overloadRecoverySec);

  preferences.putBool("r3_en", enableRelay3);
  preferences.putInt("r3_mode", r3_mode);
  preferences.putFloat("r3min", r3_min_volt);
  preferences.putFloat("r3max", r3_max_volt);

  preferences.putBool("r4_en", enableRelay4);
  preferences.putInt("r4_mode", r4_mode);
  preferences.putFloat("r4min", r4_min_volt);
  preferences.putFloat("r4max", r4_max_volt);

  preferences.end();
}
// ==========================================
struct __attribute__((packed)) EnergyStorage {
    uint16_t magic_code;
    double ac_total;
    double dc_total;
    float ac_day;
    float ac_month;
    float dc_day;
    float dc_month;
};
const uint16_t VALID_DATA_CODE = 0x55AA;

void saveEnergyData() {
    EnergyStorage saveData;
    saveData.magic_code = VALID_DATA_CODE;
    // [FIX 2] Lấy dữ liệu an toàn từ biến currentData (đã bọc Mutex)
    saveData.ac_total = currentData.ac_energy;
    saveData.dc_total = currentData.dc_energy;
    saveData.ac_day = latch_ac_day;
    saveData.ac_month = latch_ac_month;
    saveData.dc_day = latch_dc_day;
    saveData.dc_month = latch_dc_month;

    preferences.begin("energy", false);
    preferences.putBytes("energy_blk", &saveData, sizeof(EnergyStorage));
    preferences.end();
}
void loadEnergyLatch() {
  preferences.begin("energy", false);
  
  EnergyStorage loadData;
  // Dọn dẹp rác bộ đệm RAM trước khi đưa dữ liệu vào
  memset(&loadData, 0, sizeof(EnergyStorage)); 
  
  size_t len = preferences.getBytes("energy_blk", &loadData, sizeof(EnergyStorage));
  
  // FIX CHÍNH: Chỉ cần đọc được dữ liệu (len > 0) và đúng Mã bảo mật thì nạp.
  // Không ép cứng len == sizeof(EnergyStorage) nữa để tương thích ngược khi OTA code mới.
  if (len > 0 && loadData.magic_code == VALID_DATA_CODE) {
      esp_total_ac_energy = loadData.ac_total;
      esp_total_dc_energy = loadData.dc_total;
      latch_ac_day = loadData.ac_day;
      latch_ac_month = loadData.ac_month;
      latch_dc_day = loadData.dc_day;
      latch_dc_month = loadData.dc_month;
  } else {
      // Nếu chưa có hộp Struct -> Set mặc định 0
      esp_total_ac_energy = 0;
      esp_total_dc_energy = 0;
      latch_ac_day = 0; latch_ac_month = 0;
      latch_dc_day = 0; latch_dc_month = 0;
  }
  
  current_day = preferences.getInt("day", -1);
  current_month = preferences.getInt("mon", -1);
  
  prev_ac_day = preferences.getFloat("p_ac_d", 0); 
  prev_ac_month = preferences.getFloat("p_ac_m", 0);
  prev_dc_day = preferences.getFloat("p_dc_d", 0); 
  prev_dc_month = preferences.getFloat("p_dc_m", 0);
  
  preferences.end();
  
  last_saved_ac_energy = esp_total_ac_energy;
  last_saved_dc_energy = esp_total_dc_energy;
}
void checkEnergyReset() {
  if (millis() < 10000) return;
  if (currentData.ac_energy == 0 && currentData.dc_energy == 0) return;

  struct tm timeinfo; 
  if (!getLocalTime(&timeinfo, 10)) return; 
  if (timeinfo.tm_year < 120) return; 

  int now_day = timeinfo.tm_mday; 
  int now_month = timeinfo.tm_mon + 1; 
  int now_hour = timeinfo.tm_hour;

  if (current_day == -1) {
      preferences.begin("energy", false);
      current_day = now_day; current_month = now_month;
      
      latch_ac_day = currentData.ac_energy; latch_dc_day = currentData.dc_energy;
      latch_ac_month = currentData.ac_energy; latch_dc_month = currentData.dc_energy;
      
      preferences.putInt("day", current_day); preferences.putInt("mon", current_month);
      preferences.putFloat("ac_d", latch_ac_day); preferences.putFloat("dc_d", latch_dc_day);
      preferences.putFloat("ac_m", latch_ac_month); preferences.putFloat("dc_m", latch_dc_month);
      preferences.end();
      return; 
  }

  if ((now_day != current_day || now_month != current_month) && now_hour >= energyResetHour) {
      preferences.begin("energy", false);
      
      prev_ac_day = currentData.ac_energy - latch_ac_day; if(prev_ac_day < 0) prev_ac_day = 0;
      prev_dc_day = currentData.dc_energy - latch_dc_day; if(prev_dc_day < 0) prev_dc_day = 0;
      preferences.putFloat("p_ac_d", prev_ac_day);
      preferences.putFloat("p_dc_d", prev_dc_day);

      latch_ac_day = currentData.ac_energy; latch_dc_day = currentData.dc_energy;
      
      bool isBillingTime = false;
      int monthsPassed = (now_month + 12 - current_month) % 12;
      
      if (now_day == energyBillingDate) {
          isBillingTime = true;
      } 
      else if (monthsPassed > 1) {
          isBillingTime = true; 
      }
      else if (monthsPassed == 1) {
          if (current_day < energyBillingDate || now_day > energyBillingDate) {
              isBillingTime = true;
          }
      } 
      else if (now_day > energyBillingDate && current_day < energyBillingDate) {
          isBillingTime = true; 
      }

      if (isBillingTime) {
          prev_ac_month = currentData.ac_energy - latch_ac_month; if(prev_ac_month < 0) prev_ac_month = 0;
          prev_dc_month = currentData.dc_energy - latch_dc_month; if(prev_dc_month < 0) prev_dc_month = 0;
          preferences.putFloat("p_ac_m", prev_ac_month);
          preferences.putFloat("p_dc_m", prev_dc_month);

          latch_ac_month = currentData.ac_energy; latch_dc_month = currentData.dc_energy;
          preferences.putFloat("ac_m", latch_ac_month); preferences.putFloat("dc_m", latch_dc_month); 
      }
      
      current_day = now_day; 
      current_month = now_month; 
      
      preferences.putInt("day", current_day);
      preferences.putInt("mon", current_month);
      preferences.end();
      
      // GỌI HÀM NÀY ĐỂ ĐỒNG BỘ TOÀN BỘ STRUCT VÀO FLASH (Bao gồm cả latch_ac_day)
      saveEnergyData();
  }
}
// ======================================================================================
// MODULE HIỂN THỊ (UI)
// ======================================================================================

void drawDigitalNum(int x, int y, const char* val, uint16_t color, int width_to_clear) {
  tft.setTextFont(7);
  tft.setTextColor(color, C_BLACK); 
  tft.setTextPadding(width_to_clear); 
  tft.setTextDatum(TR_DATUM);
  tft.drawString(val, x, y);
  tft.setTextPadding(0); 
}

void drawTextNum(int x, int y, const char* val, uint8_t f, uint16_t color, int width_to_clear) {
  tft.setTextFont(f);
  tft.setTextColor(color, C_BLACK); 
  tft.setTextPadding(width_to_clear); 
  tft.setTextDatum(TR_DATUM);
  tft.drawString(val, x, y);
  tft.setTextPadding(0); 
}

void drawUnit(int x, int y, const char* unit, uint16_t color) {
  int w = (x < 200) ? 18 : 40;
  tft.setTextFont(2);
  tft.setTextColor(color, C_BLACK);
  tft.setTextPadding(w); 
  tft.setTextDatum(TL_DATUM);
  tft.drawString(unit, x, y);
  tft.setTextPadding(0);
}
void drawWifiArc(int cx, int cy, int r, uint16_t color) {
  for (int i = 0; i < 3; i++) {
    int currentR = r - i;
    for (int a = 220; a <= 320; a++) {
        float rad = a * 0.0174532925;
        tft.drawPixel(cx + cos(rad) * currentR, cy + sin(rad) * currentR, color);
    }
  }
}
void updateWifiIcon(int x, int y) {
  int bars = 0;
  if (WiFi.status() == WL_CONNECTED) {
      long rssi = WiFi.RSSI();
      if (rssi > -60) bars = 3; else if (rssi > -70) bars = 2; else if (rssi > -80) bars = 1; else bars = 0;
  } else { bars = -1; }
 
  if (bars == oldWifiBars) return; oldWifiBars = bars;
 
  tft.fillRect(x - 18, y - 22, 36, 24, C_BLACK);
  if (bars == -1) { tft.drawLine(x-6,y-6,x+6,y+6,TFT_RED); tft.drawLine(x+6,y-6,x-6,y+6,TFT_RED); return; }
 
  uint16_t color = (bars == 0) ? TFT_RED : TFT_GREEN;
  tft.fillCircle(x, y - 2, 3, (bars >= 0) ? color : TFT_DARKGREY);
  if (bars >= 1) drawWifiArc(x, y, 9, color); else drawWifiArc(x, y, 9, TFT_DARKGREY);
  if (bars >= 2) drawWifiArc(x, y, 15, color); else drawWifiArc(x, y, 15, TFT_DARKGREY);
  if (bars >= 3) drawWifiArc(x, y, 21, color); else drawWifiArc(x, y, 21, TFT_DARKGREY);
}
void drawTempCard(int x, int y, const char* label, float temp, uint16_t boxColor) {
  tft.fillRoundRect(x, y, 75, 22, 4, boxColor);
  tft.setTextFont(2); tft.setTextDatum(TL_DATUM); tft.setTextColor(TFT_SILVER, boxColor); tft.drawString(label, x + 5, y + 3);
  tft.setTextFont(2); tft.setTextDatum(TR_DATUM); tft.setTextColor(TFT_WHITE, boxColor);
  char buf[8]; if (temp > 0) snprintf(buf, sizeof(buf), "%.0fC", temp); else strcpy(buf, "--");
  tft.drawString(buf, x + 70, y + 3);
}
void drawModeBox(int x, int y) {
  if (systemMode == oldSystemMode) return;
  const char* mStr = "AUTO"; uint16_t bg = C_BG_AUTO;
  if (systemMode == MODE_MANUAL) { mStr = "MANUAL"; bg = C_BG_MANU; } if (systemMode == MODE_BACKUP) { mStr = "BACKUP"; bg = C_BG_BACKUP; }
  tft.fillRoundRect(x, y, 62, 20, 4, bg); tft.setTextFont(2); tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_WHITE, bg);
  tft.drawString(mStr, x + 31, y + 10); oldSystemMode = systemMode;
}
void drawTimerBox(int x, int y) {
  if (strcmp(debugStatus, oldDebugStatus) == 0) return;
  uint16_t bg = C_BG_IDLE; uint16_t txtColor = TFT_SILVER;
  if (strstr(debugStatus, "Wait") != NULL) { bg = C_BG_WAIT; txtColor = TFT_BLACK; }
  else if (strstr(debugStatus, "STOP") != NULL) { bg = TFT_RED; txtColor = TFT_WHITE; }
  else if (strstr(debugStatus, "OvLoad") != NULL) { bg = TFT_RED; txtColor = TFT_YELLOW; }
 
  tft.fillRoundRect(x, y, 75, 20, 4, bg); tft.setTextFont(2); tft.setTextDatum(MC_DATUM); tft.setTextColor(txtColor, bg);
  tft.drawString(debugStatus, x + 37, y + 10); strcpy(oldDebugStatus, debugStatus);
}
void drawRelayStatus(int x, int y) {
  bool showStop = (systemMode != MODE_MANUAL && forceOff);
  if (relayState != oldRelayState || forceOff != oldForceOff) {
    uint16_t c = relayState ? C_RELAY_ON : C_RELAY_OFF; const char* txt = relayState ? "ON" : "OFF";
    if (showStop) { txt = "OFF"; c = C_RELAY_STOP; }
    tft.fillRoundRect(x, y, 50, 20, 4, c); tft.setTextFont(2); tft.setTextDatum(MC_DATUM); tft.setTextColor(C_BLACK, c);
    tft.drawString(txt, x + 25, y + 10); oldRelayState = relayState; oldForceOff = forceOff;
  }
}
void drawExtraRelaysStatus(int x, int y) {
  static bool last_r3_en = true;
  static bool last_r4_en = true;

  // Xử lý đồ hoạ Relay 3
  if (enableRelay3) {
    if (relay3State != oldRelay3State || !last_r3_en) {
      uint16_t color = relay3State ? TFT_GREEN : TFT_DARKGREY;
      tft.fillRoundRect(x, y, 55, 20, 4, color);
      tft.setTextFont(2); tft.setTextDatum(MC_DATUM); tft.setTextColor(relay3State ? TFT_BLACK : TFT_WHITE, color);
      char buf[10]; snprintf(buf, sizeof(buf), "R3:%s", relay3State ? "ON" : "OFF");
      tft.drawString(buf, x + 27, y + 10);
      oldRelay3State = relay3State;
    }
  } else if (last_r3_en) {
    tft.fillRoundRect(x, y, 55, 20, 4, C_BLACK); // Chỉ lấy màu đen xóa 1 lần duy nhất khi vừa tắt
  }
  last_r3_en = enableRelay3;

  // Xử lý đồ hoạ Relay 4
  if (enableRelay4) {
    if (relay4State != oldRelay4State || !last_r4_en) {
      uint16_t color = relay4State ? TFT_GREEN : TFT_DARKGREY;
      tft.fillRoundRect(x + 60, y, 55, 20, 4, color);
      tft.setTextFont(2); tft.setTextDatum(MC_DATUM); tft.setTextColor(relay4State ? TFT_BLACK : TFT_WHITE, color);
      char buf[10]; snprintf(buf, sizeof(buf), "R4:%s", relay4State ? "ON" : "OFF");
      tft.drawString(buf, x + 60 + 27, y + 10);
      oldRelay4State = relay4State;
    }
  } else if (last_r4_en) {
    tft.fillRoundRect(x + 60, y, 55, 20, 4, C_BLACK); // Chỉ lấy màu đen xóa 1 lần duy nhất khi vừa tắt
  }
  last_r4_en = enableRelay4;
}
void drawInterface() {
    tft.fillScreen(C_BLACK); tft.drawFastHLine(0, 110, 320, 0x52AA); tft.drawFastVLine(160, 0, 215, 0x52AA);
    tft.setTextFont(2); tft.setTextDatum(TL_DATUM); tft.setTextColor(C_WHITE, C_BLACK); tft.drawString("AC INVERTER", 5, 5);
    oldData = emptyData; oldRelayState = !relayState; oldForceOff = !forceOff; oldSystemMode = -1; strcpy(oldDebugStatus, "xx"); oldWifiBars = -2;
    oldRelay3State = !relay3State;
    oldRelay4State = !relay4State;

    drawModeBox(108, 2); drawTimerBox(178, 2); drawRelayStatus(260, 2);
    drawUnit(140, 45, "V", C_VOLT); drawUnit(140, 85, "A", C_AMP); drawUnit(295, 85, "kWh", C_WHITE);
    
    tft.setTextColor(C_WHITE, C_BLACK); tft.drawString("DC SOLAR", 5, 115);
    
    drawExtraRelaysStatus(105, 112);

    drawUnit(140, 155, "V", C_VOLT); drawUnit(140, 195, "A", C_AMP); drawUnit(295, 195, "kWh", C_WHITE);
    updateWifiIcon(35, 238);
}
void updateScreenSmart() {
    if (currentScreen != SCREEN_MAIN) return;
    drawModeBox(108, 2); drawTimerBox(178, 2); drawRelayStatus(260, 2);
    
    drawExtraRelaysStatus(105, 112);
    
    char buf[16];
    
    if (fabs(currentData.ac_volt - oldData.ac_volt) > 0.5) {
        if (currentData.ac_volt >= 40.0 && currentData.ac_volt <= 600.0) {
            snprintf(buf, sizeof(buf), "%.0f", currentData.ac_volt);
            drawDigitalNum(135, 25, buf, C_VOLT, 130);
        } else {
            drawDigitalNum(135, 25, "0", C_VOLT, 130);
        }
        oldData.ac_volt = currentData.ac_volt;
    }

    if (fabs(currentData.ac_power - oldData.ac_power) > 1.0) {
        if (currentData.ac_power > 30000.0) {
             strcpy(buf, "---");
             drawDigitalNum(290, 25, buf, C_POWER, 128);
             drawUnit(295, 30, "W ", C_POWER);
        }
        else {
             if (currentData.ac_power >= 10000) { snprintf(buf, sizeof(buf), "%.1f", currentData.ac_power/1000.0); drawDigitalNum(290, 25, buf, C_POWER, 128); drawUnit(295, 30, "kW", C_POWER); }
             else if (currentData.ac_power >= 1000) { snprintf(buf, sizeof(buf), "%.2f", currentData.ac_power/1000.0); drawDigitalNum(290, 25, buf, C_POWER, 128); drawUnit(295, 30, "kW", C_POWER); }
             else { snprintf(buf, sizeof(buf), "%.0f", currentData.ac_power); drawDigitalNum(290, 25, buf, C_POWER, 128); drawUnit(295, 30, "W ", C_POWER); }
        }
        oldData.ac_power = currentData.ac_power;
    }
    
    if (fabs(currentData.ac_amp - oldData.ac_amp) > 0.01) {
        if (currentData.ac_amp > 120.0) strcpy(buf, "---");
        else snprintf(buf, sizeof(buf), "%.2f", currentData.ac_amp);
        drawTextNum(135, 80, buf, 4, C_AMP, 100);
        oldData.ac_amp = currentData.ac_amp;
    }
    
    if (currentData.ac_energy >= 0 && fabs(currentData.ac_energy - oldData.ac_energy) >= 0.001) {
       float daily_ac = currentData.ac_energy - latch_ac_day;
       if (daily_ac < 0) daily_ac = 0;
       if (daily_ac > 500.0) strcpy(buf, "---");
       else snprintf(buf, sizeof(buf), "%.2f", daily_ac);
       drawTextNum(290, 80, buf, 4, C_WHITE, 128);
       oldData.ac_energy = currentData.ac_energy;
    }

    if (fabs(currentData.dc_volt - oldData.dc_volt) > 0.1) { snprintf(buf, sizeof(buf), "%.1f", currentData.dc_volt); drawDigitalNum(135, 135, buf, C_VOLT, 130); oldData.dc_volt = currentData.dc_volt; }
    
    if (fabs(currentData.dc_power - oldData.dc_power) > 1.0) {
        if (currentData.dc_power > 50000.0) {
             strcpy(buf, "---");
             drawDigitalNum(290, 135, buf, C_POWER, 128);
             drawUnit(295, 140, "W ", C_POWER);
        } else {
            if (currentData.dc_power >= 10000) { snprintf(buf, sizeof(buf), "%.1f", currentData.dc_power/1000.0); drawDigitalNum(290, 135, buf, C_POWER, 128); drawUnit(295, 140, "kW", C_POWER); }
            else if (currentData.dc_power >= 1000) { snprintf(buf, sizeof(buf), "%.2f", currentData.dc_power/1000.0); drawDigitalNum(290, 135, buf, C_POWER, 128); drawUnit(295, 140, "kW", C_POWER); }
            else { snprintf(buf, sizeof(buf), "%.0f", currentData.dc_power); drawDigitalNum(290, 135, buf, C_POWER, 128); drawUnit(295, 140, "W ", C_POWER); }
        }
        oldData.dc_power = currentData.dc_power;
    }
    
    if (fabs(currentData.dc_amp - oldData.dc_amp) > 0.01) {
        if (currentData.dc_amp > 500.0) strcpy(buf, "---");
        else snprintf(buf, sizeof(buf), "%.2f", currentData.dc_amp);
        drawTextNum(135, 190, buf, 4, C_AMP, 100);
        oldData.dc_amp = currentData.dc_amp;
    }
    
    if (currentData.dc_energy >= 0 && fabs(currentData.dc_energy - oldData.dc_energy) >= 1.0) {
       float daily_dc_wh = currentData.dc_energy - latch_dc_day; if (daily_dc_wh < 0) daily_dc_wh = 0;
       if (daily_dc_wh > 500000.0) strcpy(buf, "---");
       else snprintf(buf, sizeof(buf), "%.2f", daily_dc_wh / 1000.0);
       drawTextNum(290, 190, buf, 4, C_WHITE, 128);
       oldData.dc_energy = currentData.dc_energy;
    }

    static unsigned long t_temp_ui = 0;
    if (millis() - t_temp_ui >= 3000) {
        if (fabs(currentData.t1 - oldData.t1) > 0.5) { drawTempCard(70,  217, "T1", currentData.t1, C_BOX_T1); oldData.t1 = currentData.t1; }
        if (fabs(currentData.t2 - oldData.t2) > 0.5) { drawTempCard(155, 217, "T2", currentData.t2, C_BOX_T2); oldData.t2 = currentData.t2; }
        if (fabs(currentData.t3 - oldData.t3) > 0.5) { drawTempCard(240, 217, "T3", currentData.t3, C_BOX_T3); oldData.t3 = currentData.t3; }
        t_temp_ui = millis();
    }
    
    updateWifiIcon(35, 238);
}
// ======================================================================================
// MODULE LOGIC CONTROL & BLYNK
// ======================================================================================
void syncBlynkRelayState(bool state) {
    if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
        Blynk.virtualWrite(V40, state ? 1 : 0); Blynk.virtualWrite(V49, state ? 255 : 0);
    }
}
void syncSettingsToBlynk() {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    int modeVal = 1;
    if (systemMode == MODE_MANUAL) modeVal = 2;
    else if (systemMode == MODE_BACKUP) modeVal = 3;
   
    Blynk.virtualWrite(V41, modeVal);
    Blynk.virtualWrite(V42, highThreshold);
    Blynk.virtualWrite(V43, lowThreshold);
    Blynk.virtualWrite(V44, delayLow);
    Blynk.virtualWrite(V45, delayHigh);
  }
}

void setRelay(bool state) {
  if (millis() - relay1_last_switch_time < 1000) return;

  if (relayState != state) {
      relayState = state;
      digitalWrite(RELAY_PIN, state ? HIGH : LOW);
     
      relay1_last_switch_time = millis();
      if (currentScreen == SCREEN_MAIN) {
          drawRelayStatus(260, 2);
      }

      preferences.begin("solar_cfg", false); preferences.putBool("lastRelay", state); preferences.end();
      flag_sync_relay = true;
  }
}
void syncModeState() {
  if (systemMode == MODE_MANUAL) { 
      manualRelayCmd = relayState; 
      syncBlynkRelayState(relayState); 
  } 
}
void getUptimeStr(char* buf, size_t len) {
  unsigned long sec = millis() / 1000;
  int days = sec / 86400; int hours = (sec % 86400) / 3600; int mins = (sec % 3600) / 60; int s = sec % 60;
  snprintf(buf, len, "%dd %02d:%02d:%02d", days, hours, mins, s);
}
int getWifiPercent() {
  if (WiFi.status() != WL_CONNECTED) return 0; long rssi = WiFi.RSSI();
  if (rssi <= -100) return 0; if (rssi >= -50) return 100; return 2 * (rssi + 100);
}

void updateACRelayProtection() {
  float acV = currentData.ac_volt;
  float acP = currentData.ac_power;

  if (acP >= overloadLimitW) {
      if (relay2State == true) {
          digitalWrite(RELAY2_PIN, LOW);
          relay2State = false;
      }
      isOverloadTripped = true;
      overloadTripTimer = millis();
      strcpy(debugStatus, "OvLoad");
  }

  if (isOverloadTripped) {
      unsigned long timePassed = millis() - overloadTripTimer;
      if (timePassed < (unsigned long)overloadRecoverySec * 1000) {
          if (relay2State == true) { digitalWrite(RELAY2_PIN, LOW); relay2State = false; }
          return;
      } else {
          isOverloadTripped = false;
      }
  }

  if (acV < 180.0 || acV > 240.0) {
    if (relay2State == true) { digitalWrite(RELAY2_PIN, LOW); relay2State = false; }
    acProtectionActive = true; acProtectionTimerStart = millis();
  }
  else if (acProtectionActive) {
    if (millis() - acProtectionTimerStart >= 5000) {      
        if (acV >= 200.0 && acV <= 240.0) { digitalWrite(RELAY2_PIN, HIGH); relay2State = true; acProtectionActive = false; }
    }
  }
  else {
      if (acV >= 200.0 && acV <= 240.0) {
          if (relay2State == false) { digitalWrite(RELAY2_PIN, HIGH); relay2State = true; }
      }
  }
}
void updateRelay3Logic() {
  // Khai báo biến lưu thời gian tắt để tính trễ 10s cho chế độ điện áp DC
  static unsigned long r3_off_time = 0;
  static unsigned long r4_off_time = 0;

  // ==========================================
  // --- XỬ LÝ RELAY 3 ---
  // ==========================================
  if (!enableRelay3) {
      if (relay3State == true) { digitalWrite(RELAY3_PIN, LOW); relay3State = false; }
      r3_off_time = 0; 
  } else {
      float val3 = (r3_mode == 0) ? currentData.dc_volt : currentData.ac_power;
      bool target3 = relay3State; // Giữ nguyên trạng thái hiện tại làm mặc định

      if (r3_mode == 0) { 
          // 1. CHẠY THEO ĐIỆN ÁP DC
          if (val3 >= r3_min_volt && val3 <= r3_max_volt) {
              // Nằm trong ngưỡng -> Cho phép BẬT
              if (!relay3State) {
                  // Phải chờ đủ 10 giây sau lần TẮT gần nhất mới được BẬT lại
                  if (millis() - r3_off_time >= 10000) {
                      target3 = true;
                  }
              }
          } else {
              // Nằm ngoài ngưỡng -> TẮT NGAY LẬP TỨC
              if (relay3State) {
                  target3 = false;
                  r3_off_time = millis(); // Ghi nhớ mốc thời gian vừa tắt
              }
          }
      } 
      else { 
          // 2. CHẠY THEO CÔNG SUẤT AC (WATT)
          // (Sử dụng r3_max_volt làm NGƯỠNG CÀI ĐẶT chính)
          if (val3 >= r3_max_volt) {
              target3 = true;  // Trên ngưỡng cài đặt -> BẬT
          } 
          else if (val3 <= (r3_max_volt - 501.0)) {
              target3 = false; // Thấp hơn ngưỡng cài đặt 501W -> TẮT NGAY LẬP TỨC
          }
          // TRƯỜNG HỢP CÒN LẠI: Nằm trong khoảng (Ngưỡng - 500W) đến (Ngưỡng)
          // Hệ thống sẽ không thay đổi target3, nghĩa là: Đang ON thì vẫn giữ ON, Đang OFF thì vẫn giữ OFF.
      }

      // Xuất tín hiệu ra Relay nếu có thay đổi
      if (relay3State != target3) {
          relay3State = target3;
          digitalWrite(RELAY3_PIN, relay3State ? HIGH : LOW);
      }
  }

  // ==========================================
  // --- XỬ LÝ RELAY 4 ---
  // ==========================================
  if (!enableRelay4) {
      if (relay4State == true) { digitalWrite(RELAY4_PIN, LOW); relay4State = false; }
      r4_off_time = 0; 
  } else {
      float val4 = (r4_mode == 0) ? currentData.dc_volt : currentData.ac_power;
      bool target4 = relay4State;

      if (r4_mode == 0) { 
          // 1. CHẠY THEO ĐIỆN ÁP DC
          if (val4 >= r4_min_volt && val4 <= r4_max_volt) {
              if (!relay4State) {
                  if (millis() - r4_off_time >= 10000) {
                      target4 = true;
                  }
              }
          } else {
              if (relay4State) {
                  target4 = false;
                  r4_off_time = millis(); 
              }
          }
      } 
      else { 
          // 2. CHẠY THEO CÔNG SUẤT AC (WATT)
          if (val4 >= r4_max_volt) {
              target4 = true;
          } 
          else if (val4 <= (r4_max_volt - 501.0)) {
              target4 = false;
          }
      }

      if (relay4State != target4) {
          relay4State = target4;
          digitalWrite(RELAY4_PIN, relay4State ? HIGH : LOW);
      }
  }
}
void updateRelay() {
  switch (systemMode) {
    case MODE_AUTO:
      if (forceOff) { 
        if (relayState == true) { setRelay(false); relayTimerStart = 0; } 
        strcpy(debugStatus, "STOP"); 
        return; 
      }
      if (currentData.dc_volt >= highThreshold) {
          if (relayState == false) {
             if (relayTimerStart == 0) relayTimerStart = millis();
             if (millis() - relayTimerStart >= delayHigh * 1000) { setRelay(true); relayTimerStart = 0; strcpy(debugStatus, "Stable"); }
             else { snprintf(debugStatus, sizeof(debugStatus), "WaitH %lds", (delayHigh * 1000 - (millis() - relayTimerStart))/1000); }
          } else { relayTimerStart = 0; strcpy(debugStatus, "Stable"); }
      }
      else if (currentData.dc_volt <= lowThreshold) {
          if (relayState == true) {
             if (relayTimerStart == 0) relayTimerStart = millis();
             if (millis() - relayTimerStart >= delayLow * 1000) { setRelay(false); relayTimerStart = 0; strcpy(debugStatus, "Stable"); }
             else { snprintf(debugStatus, sizeof(debugStatus), "WaitL %lds", (delayLow * 1000 - (millis() - relayTimerStart))/1000); }
          } else { relayTimerStart = 0; strcpy(debugStatus, "Stable"); }
      }
      else { relayTimerStart = 0; strcpy(debugStatus, "Mid-V"); }
      break;
    case MODE_MANUAL:
      if (relayState != manualRelayCmd) { setRelay(manualRelayCmd); } strcpy(debugStatus, "Manual");
      break;
    case MODE_BACKUP:
      if (forceOff) { if (relayState == true) { setRelay(false); } strcpy(debugStatus, "STOP"); return; }
      bool gridSignal = digitalRead(GRID_DETECT_PIN);
      if (gridSignal == LOW) { if (relayState == true) { setRelay(false); } strcpy(debugStatus, "GridOK"); }
      else { if (relayState == false) { setRelay(true); } strcpy(debugStatus, "NoGrid"); }
      break;
  }
}
BLYNK_CONNECTED() {
  int v40State = 0;
  if (systemMode == MODE_AUTO) {
     v40State = forceOff ? 0 : 1;
  } else {
     v40State = relayState ? 1 : 0;
  }
  Blynk.virtualWrite(V40, v40State);
  Blynk.virtualWrite(V49, relayState ? 255 : 0);
  Blynk.virtualWrite(V41, systemMode == MODE_AUTO ? 1 : (systemMode == MODE_MANUAL ? 2 : 3));
  Blynk.syncVirtual(V41, V42, V43, V44, V45);

  if (isFirstConnect) {
      terminal.clear();
      terminal.flush(); delay(200);
      terminal.println(F("================================="));
      terminal.print(F("WIFI: ")); terminal.print(WiFi.SSID());
      terminal.print(F(" | ")); terminal.println(WiFi.localIP());
      String deviceName = String("SolarMeter_") + getMacSuffix();
      terminal.print(F("AP NAME: ")); terminal.println(deviceName);
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 10)) {
        char timeBuff[30];
        strftime(timeBuff, sizeof(timeBuff), "%H:%M %d/%m/%Y", &timeinfo);
        terminal.print(F("INVERTER ONLINE ")); terminal.println(timeBuff);
      }
      terminal.println(F("================================="));
      terminal.flush();
      isFirstConnect = false;
  }
}
BLYNK_WRITE(V40) {
    int action = param.asInt();
    if (systemMode == MODE_MANUAL) manualRelayCmd = action;
    else {
        if (action == 1) forceOff = false;
        else { forceOff = true; setRelay(false); }
        preferences.begin("solar_cfg", false); preferences.putBool("forceOff", forceOff); preferences.end();
    }
    update_screen_now = true; // THÊM DÒNG NÀY: Báo cờ cho loop() tự vẽ an toàn trên Core 1
}
BLYNK_WRITE(V41) { 
    int m = param.asInt(); 
    if (m == 1) systemMode = MODE_AUTO; 
    else if (m == 2) systemMode = MODE_MANUAL; 
    else if (m == 3) systemMode = MODE_BACKUP; 
    saveSettings(); 
    syncModeState(); 
    // XÓA 2 DÒNG NÀY: drawModeBox(108, 2); drawRelayStatus(260, 2); 
    update_screen_now = true; // THÊM DÒNG NÀY
}
BLYNK_WRITE(V42) { highThreshold = param.asFloat(); saveSettings(); }
BLYNK_WRITE(V43) { lowThreshold = param.asFloat(); saveSettings(); }
BLYNK_WRITE(V44) { delayLow = param.asInt(); saveSettings(); }
BLYNK_WRITE(V45) { delayHigh = param.asInt(); saveSettings(); }

BLYNK_WRITE(V50) { String cmd = param.asStr(); processSerialCommand(cmd); }

void updateBlynk() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  // --- KHỐI COPY DỮ LIỆU AN TOÀN TUYỆT ĐỐI ---
  DataBox blynkData;
  if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) { 
      blynkData = sharedData; 
      xSemaphoreGive(dataMutex); 
  } else {
      blynkData = currentData; // Dự phòng nếu Mutex đang kẹt
  }
  // --------------------------------------------------------

  float wh_dc_day = blynkData.dc_energy - latch_dc_day; if (wh_dc_day < 0) wh_dc_day = 0;
  float wh_dc_mon = blynkData.dc_energy - latch_dc_month; if (wh_dc_mon < 0) wh_dc_mon = 0;
  float kwh_ac_day = blynkData.ac_energy - latch_ac_day; if (kwh_ac_day < 0) kwh_ac_day = 0;
  float kwh_ac_mon = blynkData.ac_energy - latch_ac_month; if (kwh_ac_mon < 0) kwh_ac_mon = 0;
  float val_V6 = wh_dc_day / 1000.0f; float val_V16 = kwh_ac_day; float ket_qua_v26 = (val_V6 * 0.82) - val_V16;
  
  if (Blynk.connected()) {
      static DataBox sent = { -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999, -9999 };
      static float s_v6=-99, s_v7=-99, s_v16=-99, s_v17=-99, s_v26=-99;
      static float s_p_ac_d=-99, s_p_ac_m=-99, s_p_dc_d=-99, s_p_dc_m=-99;
      static unsigned long t_last_sys = 0;

      // ĐÃ SỬA: Thay toàn bộ currentData thành blynkData bên dưới
      if (fabs(blynkData.dc_volt - sent.dc_volt) >= 0.01) { Blynk.virtualWrite(V0, blynkData.dc_volt); sent.dc_volt = blynkData.dc_volt; }
      if (fabs(blynkData.dc_amp - sent.dc_amp) >= 0.001) { Blynk.virtualWrite(V1, blynkData.dc_amp); sent.dc_amp = blynkData.dc_amp; }
      if (fabs(blynkData.dc_power - sent.dc_power) >= 0.1) { Blynk.virtualWrite(V2, blynkData.dc_power); sent.dc_power = blynkData.dc_power; }
      float dc_kwh = blynkData.dc_energy / 1000.0f;
      if (fabs(dc_kwh - sent.dc_energy) >= 0.001) { Blynk.virtualWrite(V3, dc_kwh); sent.dc_energy = dc_kwh; }
      if (fabs(blynkData.ac_volt - sent.ac_volt) >= 0.1) { Blynk.virtualWrite(V10, blynkData.ac_volt); sent.ac_volt = blynkData.ac_volt; }
      if (fabs(blynkData.ac_amp - sent.ac_amp) >= 0.001) { Blynk.virtualWrite(V11, blynkData.ac_amp); sent.ac_amp = blynkData.ac_amp; }
      if (fabs(blynkData.ac_power - sent.ac_power) >= 0.1) { Blynk.virtualWrite(V12, blynkData.ac_power); sent.ac_power = blynkData.ac_power; }
      if (fabs(blynkData.ac_freq - sent.ac_freq) >= 0.01) { Blynk.virtualWrite(V14, blynkData.ac_freq); sent.ac_freq = blynkData.ac_freq; }
      if (fabs(blynkData.ac_pf - sent.ac_pf) >= 0.001) { Blynk.virtualWrite(V15, blynkData.ac_pf); sent.ac_pf = blynkData.ac_pf; }
      if (fabs(blynkData.ac_energy - sent.ac_energy) >= 0.01) { Blynk.virtualWrite(V13, blynkData.ac_energy); sent.ac_energy = blynkData.ac_energy; }
      
      if (fabs(blynkData.t1 - sent.t1) >= 0.5) { Blynk.virtualWrite(V21, blynkData.t1); sent.t1 = blynkData.t1; }
      if (fabs(blynkData.t2 - sent.t2) >= 0.5) { Blynk.virtualWrite(V22, blynkData.t2); sent.t2 = blynkData.t2; }
      if (fabs(blynkData.t3 - sent.t3) >= 0.5) { Blynk.virtualWrite(V23, blynkData.t3); sent.t3 = blynkData.t3; }
      
      if (fabs(val_V6 - s_v6) >= 0.005) { Blynk.virtualWrite(V6, val_V6); s_v6 = val_V6; }
      float dc_mon_kwh = wh_dc_mon / 1000.0f;
      if (fabs(dc_mon_kwh - s_v7) >= 0.01) { Blynk.virtualWrite(V7, dc_mon_kwh); s_v7 = dc_mon_kwh; }
      if (fabs(kwh_ac_day - s_v16) >= 0.005) { Blynk.virtualWrite(V16, kwh_ac_day); s_v16 = kwh_ac_day; }
      if (fabs(kwh_ac_mon - s_v17) >= 0.01) { Blynk.virtualWrite(V17, kwh_ac_mon); s_v17 = kwh_ac_mon; }
      if (fabs(ket_qua_v26 - s_v26) >= 0.01) { Blynk.virtualWrite(V26, ket_qua_v26); s_v26 = ket_qua_v26; }
      
      float p_dc_d_kwh = prev_dc_day / 1000.0f;
      float p_dc_m_kwh = prev_dc_month / 1000.0f;
      if (fabs(p_dc_d_kwh - s_p_dc_d) >= 0.005) { Blynk.virtualWrite(V8, p_dc_d_kwh); s_p_dc_d = p_dc_d_kwh; }
      if (fabs(p_dc_m_kwh - s_p_dc_m) >= 0.01) { Blynk.virtualWrite(V9, p_dc_m_kwh); s_p_dc_m = p_dc_m_kwh; }
      if (fabs(prev_ac_day - s_p_ac_d) >= 0.005) { Blynk.virtualWrite(V18, prev_ac_day); s_p_ac_d = prev_ac_day; }
      if (fabs(prev_ac_month - s_p_ac_m) >= 0.01) { Blynk.virtualWrite(V19, prev_ac_month); s_p_ac_m = prev_ac_month; }
      
      if (millis() - t_last_sys >= 5000) {
          char buffer[64]; snprintf(buffer, sizeof(buffer), "C0:%.0f%% C1:%.0f%%", load_core0, load_core1); Blynk.virtualWrite(V60, buffer);
          snprintf(buffer, sizeof(buffer), "%d KB", ESP.getFreeHeap() / 1024); Blynk.virtualWrite(V63, buffer);
          getUptimeStr(buffer, sizeof(buffer)); Blynk.virtualWrite(V61, buffer); Blynk.virtualWrite(V62, getWifiPercent());
          t_last_sys = millis();
      }
  }
 
  char msg[256];
  snprintf(msg, sizeof(msg), "%.1f,%.2f,%.0f,%.3f,%.0f,%.2f,%.0f,%.3f,%.0f,%.0f,%.0f,%d,%d,%d,%s",            
            blynkData.dc_volt, blynkData.dc_amp, blynkData.dc_power, val_V6,
            blynkData.ac_volt, blynkData.ac_amp, blynkData.ac_power, val_V16,
            blynkData.t1, blynkData.t2, blynkData.t3, systemMode, relayState, forceOff, debugStatus);
  udp.beginPacket(WiFi.broadcastIP(), UDP_PORT); udp.print(msg); udp.endPacket();
}
void configModeCallback (WiFiManager *wm) {
  tft.fillScreen(C_BLACK); tft.setTextFont(4); tft.setTextColor(C_AMP); tft.setTextDatum(MC_DATUM);
  tft.drawString("CHE DO CAI DAT WIFI", 160, 40);
  tft.setTextFont(2); tft.setTextColor(TFT_WHITE);
  String apName = String("SolarMeter_") + getMacSuffix();
  tft.drawString("WiFi: " + apName + " / IP: 192.168.4.1", 160, 80);
  tft.setTextColor(TFT_ORANGE);
  tft.drawString("1. Ket noi WiFi tren de cai dat", 160, 120);
  tft.drawString("2. Nhan MENU de thoat -> Offline", 160, 150);
  tft.setTextColor(TFT_RED);
  tft.drawString("Tu dong Offline sau 120s...", 160, 190);
}
// --- KHAI BÁO GIAO DIỆN TĨNH TRÊN FLASH (PROGMEM) ---
const char HTML_MAIN[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Control Panel | Smart Solar</title>
  <style>
    :root { --bg: #0f172a; --card-bg: #1e293b; --primary: #3b82f6; --primary-hover: #2563eb; --danger: #ef4444; --danger-hover: #dc2626; --text: #f8fafc; --text-muted: #94a3b8; --border: #334155; --success: #10b981; --input-bg: #0f172a; }
    body { font-family: 'Segoe UI', Tahoma, sans-serif; background-color: var(--bg); color: var(--text); margin: 0; padding: 20px; display: flex; justify-content: center; }
    .container { max-width: 480px; width: 100%; }
    .header { text-align: center; margin-bottom: 24px; }
    .header h1 { margin: 0; color: var(--primary); font-size: 24px; letter-spacing: 1px; }
    .header p { margin: 4px 0 0; color: var(--text-muted); font-size: 14px; }
    .card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 12px; padding: 24px; margin-bottom: 20px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1); }
    .card-title { margin-top: 0; font-size: 16px; font-weight: 600; border-bottom: 1px solid var(--border); padding-bottom: 12px; margin-bottom: 20px; text-transform: uppercase; letter-spacing: 0.5px; color: var(--text-muted); }
    .label { display: block; margin-bottom: 8px; color: var(--text-muted); font-size: 13px; font-weight: 500; }
    input[type="text"], input[type="file"] { width: 100%; padding: 12px; background: var(--input-bg); border: 1px solid var(--border); color: var(--text); border-radius: 8px; box-sizing: border-box; font-size: 14px; transition: border-color 0.2s; }
    input[type="text"] { font-family: monospace; letter-spacing: 1px; }
    input:focus { outline: none; border-color: var(--primary); }
    .btn-group { display: flex; gap: 12px; margin-top: 20px; }
    .btn { flex: 1; padding: 12px; border: none; border-radius: 8px; font-weight: 600; cursor: pointer; font-size: 14px; transition: all 0.2s; text-align: center; text-decoration: none; box-sizing: border-box; display: inline-block; }
    .btn-primary { background: var(--primary); color: white; }
    .btn-primary:hover { background: var(--primary-hover); }
    .btn-danger { background: rgba(239, 68, 68, 0.1); color: var(--danger); border: 1px solid var(--danger); }
    .btn-danger:hover { background: var(--danger); color: white; }
    .token-display { background: rgba(16, 185, 129, 0.1); color: var(--success); padding: 12px; border-radius: 8px; font-family: monospace; word-break: break-all; border: 1px solid rgba(16,185,129,0.2); text-align: center; font-size: 15px; margin-bottom: 20px; }
    .progress-container { display: none; margin-top: 20px; }
    .progress-bg { background: var(--input-bg); border: 1px solid var(--border); border-radius: 999px; height: 16px; overflow: hidden; position: relative; }
    .progress-fill { background: var(--primary); height: 100%; width: 0%; transition: width 0.2s ease; }
    .progress-text { text-align: center; margin-top: 10px; font-size: 14px; font-weight: 600; color: var(--primary); }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>SMART SOLAR METER</h1>
      <p>He thong Quan ly & Cap nhat</p>
    </div>

    <div class="card">
      <h2 class="card-title">Cap Nhat Firmware (OTA)</h2>
      <form method='POST' action='/update' enctype='multipart/form-data' id='upload_form'>
        <input type='file' name='update' id='file' accept='.bin'>
        <button type='button' class='btn btn-primary' id='upload-btn' style='width: 100%; margin-top: 15px;' onclick='uploadFile()'>Bat Dau Cap Nhat</button>
      </form>
      <div class="progress-container" id="progress-wrapper">
        <div class="progress-bg"><div class="progress-fill" id="progress-fill"></div></div>
        <div class="progress-text" id="progress-text">0%</div>
      </div>
    </div>
)rawliteral";
void setupWebOTA() {
  // [CỐT LÕI FIX 50%]: Báo cho ESP32 biết cần thu thập header "X-File-Size" từ trình duyệt
  const char* headerKeys[] = {"X-File-Size"};
  server.collectHeaders(headerKeys, 1);

  // --- TRANG CHÍNH: GIAO DIỆN CHUYÊN NGHIỆP ---
  // --- TRANG CHÍNH: GIAO DIỆN CHUYÊN NGHIỆP ---
  server.on("/", HTTP_GET, []() { 
      // Đọc các giá trị cài đặt hiện tại từ Preferences để hiển thị lên Form web
      preferences.begin("solar_cfg", true);
      String webPass = preferences.getString("webpass", "254125");
      String currentToken = preferences.getString("token", String(blynk_token));
      String currentServer = preferences.getString("server", String(blynk_server));
      
      float c_vac = preferences.getFloat("c_vac", 1.000);
      float c_aac = preferences.getFloat("c_aac", 1.000);
      float c_vdc = preferences.getFloat("c_vdc", 1.000);
      float c_adc = preferences.getFloat("c_adc", 1.000);
      preferences.end();

      // Xác thực quyền truy cập trang quản trị admin
      if (!server.authenticate("admin", webPass.c_str())) return server.requestAuthentication();

      // [BƯỚC CHỐT] Kích hoạt chế độ gửi dữ liệu theo mảnh (Chunked Streaming)
      server.setContentLength(CONTENT_LENGTH_UNKNOWN);
      server.send(200, "text/html", "");

      // 1. Gửi phần giao diện HTML tĩnh trực tiếp từ bộ nhớ FLASH (0 byte RAM bị chiếm dụng)
      server.sendContent_P(HTML_MAIN);

      // 2. Tạo một vùng đệm nhỏ trên RAM chỉ dành riêng cho các thành phần Form động
      String dynamicForm = "";
      dynamicForm.reserve(6000); // Cấp phát trước 3KB, không lo phân mảnh bộ nhớ

      dynamicForm += "    <div class=\"card\">\n";
      dynamicForm += "      <h2 class=\"card-title\">Cai Dat Ket Noi Blynk</h2>\n";
      dynamicForm += "      <span class=\"label\">Server dang hoat dong:</span>\n";
      dynamicForm += "      <div class=\"token-display\" style=\"margin-bottom: 10px;\">" + currentServer + "</div>\n";
      dynamicForm += "      <span class=\"label\">Token dang hoat dong (32 ki tu):</span>\n";
      dynamicForm += "      <div class=\"token-display\">" + currentToken + "</div>\n";
      dynamicForm += "      <form method='POST' action='/blynk_config'>\n";
      dynamicForm += "        <span class=\"label\">Server moi (IP hoac Domain):</span>\n";
      dynamicForm += "        <input type='text' name='newServer' value='" + currentServer + "' style='margin-bottom: 15px;' required>\n";
      dynamicForm += "        <span class=\"label\">Token moi (32 ki tu):</span>\n";
      dynamicForm += "        <input type='text' name='newToken' value='" + currentToken + "' minlength='32' maxlength='32' required>\n";
      dynamicForm += "        <div class=\"btn-group\">\n";
      dynamicForm += "           <button type='submit' class='btn btn-primary'>Luu Cai Dat</button>\n";
      dynamicForm += "           <button type='button' class='btn btn-danger' onclick='if(confirm(\"Xac nhan khoi dong lai he thong?\")) window.location.href=\"/reboot\";'>Reboot</button>\n";
      dynamicForm += "        </div>\n";
      dynamicForm += "      </form>\n";
      dynamicForm += "    </div>\n";

      dynamicForm += "    <div class=\"card\">\n";
      dynamicForm += "      <h2 class=\"card-title\">Hieu Chinh Sai So (Calibration)</h2>\n";
      dynamicForm += "      <form method='POST' action='/calib_config'>\n";
      dynamicForm += "        <div style=\"display: grid; grid-template-columns: 1fr 1fr; gap: 12px;\">\n";
      dynamicForm += "          <div>\n";
      dynamicForm += "            <span class=\"label\">Vol AC (Dang do: <span id=\"live_vac\" style=\"color:var(--success); font-weight:bold;\">" + String(currentData.ac_volt, 1) + "</span>V):</span>\n";
      dynamicForm += "            <input type='text' name='c_vac' value='" + String(c_vac, 3) + "' required pattern=\"[0-9]+([.][0-9]+)?\">\n";
      dynamicForm += "          </div>\n";
      dynamicForm += "          <div>\n";
      dynamicForm += "            <span class=\"label\">Ampe AC (Dang do: <span id=\"live_aac\" style=\"color:var(--success); font-weight:bold;\">" + String(currentData.ac_amp, 2) + "</span>A):</span>\n";
      dynamicForm += "            <input type='text' name='c_aac' value='" + String(c_aac, 3) + "' required pattern=\"[0-9]+([.][0-9]+)?\">\n";
      dynamicForm += "          </div>\n";
      dynamicForm += "          <div>\n";
      dynamicForm += "            <span class=\"label\">Vol DC (Dang do: <span id=\"live_vdc\" style=\"color:#f59e0b; font-weight:bold;\">" + String(currentData.dc_volt, 1) + "</span>V):</span>\n";
      dynamicForm += "            <input type='text' name='c_vdc' value='" + String(c_vdc, 3) + "' required pattern=\"[0-9]+([.][0-9]+)?\">\n";
      dynamicForm += "          </div>\n";
      dynamicForm += "          <div>\n";
      dynamicForm += "            <span class=\"label\">Ampe DC (Dang do: <span id=\"live_adc\" style=\"color:#f59e0b; font-weight:bold;\">" + String(currentData.dc_amp, 2) + "</span>A):</span>\n";
      dynamicForm += "            <input type='text' name='c_adc' value='" + String(c_adc, 3) + "' required pattern=\"[0-9]+([.][0-9]+)?\">\n";
      dynamicForm += "          </div>\n";
      dynamicForm += "        </div>\n";
      dynamicForm += "        <div style=\"font-size:13px; color:var(--text-muted); margin-top:15px; background: rgba(0,0,0,0.2); padding: 12px; border-radius: 6px; border-left: 3px solid var(--primary); text-align: left; line-height: 1.6;\">\n";
      dynamicForm += "          <strong style=\"color:var(--text);\">* Hướng dẫn tính Hệ số hiệu chỉnh:</strong><br>\n";
      dynamicForm += "          Hệ số = [Số đo bằng đồng hồ thực tế] / [Số đang hiển thị trên mạch]<br>\n";
      dynamicForm += "          <i>Ví dụ: Đồng hồ đo 106V, mạch đang báo 115V -> Hệ số = 106 / 115 = <b>0.922</b></i><br>\n";
      dynamicForm += "          <span style=\"font-size: 11px; opacity: 0.8;\">(Mặc định khi chưa hiệu chỉnh là: 1.000)</span>\n";
      dynamicForm += "        </div>\n";
      dynamicForm += "        <button type='submit' class='btn btn-primary' style='width: 100%; margin-top: 15px;'>Luu He So Kiem Dinh</button>\n";
      dynamicForm += "      </form>\n";
      dynamicForm += "    </div>\n";
      dynamicForm += "  </div>\n";

      // Đóng gói mã nguồn kịch bản JavaScript điều khiển tiến trình nạp Flash & Live Update
      dynamicForm += "  <script>\n";
      dynamicForm += "    setInterval(function() {\n";
      dynamicForm += "      fetch('/data').then(r => r.json()).then(d => {\n";
      dynamicForm += "        if(document.getElementById('live_vac')) document.getElementById('live_vac').innerText = d.ac_v.toFixed(1);\n";
      dynamicForm += "        if(document.getElementById('live_aac')) document.getElementById('live_aac').innerText = d.ac_a.toFixed(2);\n";
      dynamicForm += "        if(document.getElementById('live_vdc')) document.getElementById('live_vdc').innerText = d.dc_v.toFixed(1);\n";
      dynamicForm += "        if(document.getElementById('live_adc')) document.getElementById('live_adc').innerText = d.dc_a.toFixed(2);\n";
      dynamicForm += "      }).catch(e => console.log('Err'));\n";
      dynamicForm += "    }, 2000);\n";
      dynamicForm += "    function uploadFile() {\n";
      dynamicForm += "      var file = document.getElementById('file').files[0];\n";
      dynamicForm += "      if(!file) return alert('Vui long chon file Firmware (.bin)!');\n";
      dynamicForm += "      document.getElementById('upload-btn').style.display = 'none';\n";
      dynamicForm += "      document.getElementById('progress-wrapper').style.display = 'block';\n";
      dynamicForm += "      var formData = new FormData(); formData.append('update', file);\n";
      dynamicForm += "      var xhr = new XMLHttpRequest();\n";
      dynamicForm += "      xhr.upload.addEventListener('progress', function(e) {\n";
      dynamicForm += "        if(e.lengthComputable) {\n";
      dynamicForm += "          var percent = Math.round((e.loaded / e.total) * 100);\n";
      dynamicForm += "          if (percent >= 100) percent = 99;\n";
      dynamicForm += "          document.getElementById('progress-fill').style.width = percent + '%';\n";
      dynamicForm += "          document.getElementById('progress-text').innerText = percent + '% (Dang ghi Flash...)';\n";
      dynamicForm += "        }\n";
      dynamicForm += "      });\n";
      dynamicForm += "      xhr.onload = function() {\n";
      dynamicForm += "        var pText = document.getElementById('progress-text');\n";
      dynamicForm += "        var pFill = document.getElementById('progress-fill');\n";
      dynamicForm += "        if(xhr.status === 200) {\n";
      dynamicForm += "          pFill.style.width = '100%'; pText.innerText = '100% - THANH CONG! Dang reboot...';\n";
      dynamicForm += "          pText.style.color = 'var(--success)'; pFill.style.background = 'var(--success)';\n";
      dynamicForm += "          setTimeout(() => window.location.href = '/', 6000);\n";
      dynamicForm += "        } else {\n";
      dynamicForm += "          pText.innerText = 'LOI CAP NHAT!'; pText.style.color = 'var(--danger)'; pFill.style.background = 'var(--danger)';\n";
      dynamicForm += "        }\n";
      dynamicForm += "      };\n";
      dynamicForm += "      xhr.open('POST', '/update');\n";
      dynamicForm += "      xhr.setRequestHeader('X-File-Size', file.size);\n";
      dynamicForm += "      xhr.send(formData);\n";
      dynamicForm += "    }\n";
      dynamicForm += "  </script>\n";
      dynamicForm += "</body>\n";
      dynamicForm += "</html>\n";

      // 3. Tiến hành gửi chuỗi động và phát tín hiệu kết thúc stream dữ liệu
      server.sendContent(dynamicForm);
      server.sendContent(""); 
  });

  // --- XỬ LÝ NHẬN FILE FIRMWARE ---
  server.on("/update", HTTP_POST, []() { 
      preferences.begin("solar_cfg", true);
      String webPass = preferences.getString("webpass", "254125");
      preferences.end();
      if (!server.authenticate("admin", webPass.c_str())) return server.requestAuthentication();

      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK"); 
      delay(1000); ESP.restart(); 
  }, []() { 
      preferences.begin("solar_cfg", true);
      String webPass = preferences.getString("webpass", "254125");
      preferences.end();
      if (!server.authenticate("admin", webPass.c_str())) return;

      HTTPUpload& upload = server.upload(); 
      if (upload.status == UPLOAD_FILE_START) { 
          // [FIX 1] Bật cờ ngưng đọc phần cứng thay vì Suspend Task
          isSystemUpdating = true; 

          // --- FIX: LƯU GẤP DỮ LIỆU TRƯỚC KHI UPDATE ---
          saveEnergyData(); // <-- GỌI HÀM LƯU GOM CỤM
          delay(50);
          isWebOTA = true; 
          currentScreen = SCREEN_OTA;
          tft.fillScreen(C_BLACK);
// ... (Phần vẽ tft bên dưới giữ nguyên) ...
          
          // Đồng bộ giao diện TFT giống hệt Arduino IDE
          // ... (Các đoạn code vẽ màn hình tft.drawString bên dưới bạn giữ nguyên) ...
          
          // Đồng bộ giao diện TFT giống hệt Arduino IDE
          tft.fillRect(0, 0, 320, 30, 0x18E3);
          tft.setTextFont(2); 
          tft.setTextColor(TFT_WHITE, 0x18E3);
          tft.setTextDatum(MC_DATUM);
          tft.drawString("SYSTEM FIRMWARE UPDATE", 160, 15);
          
          tft.setTextDatum(TL_DATUM);
          tft.setTextColor(TFT_CYAN, C_BLACK);
          tft.drawString("Nguon nap: Trinh Duyet Web", 15, 45); // Hiển thị nguồn nạp là Web
          tft.setTextColor(TFT_SILVER, C_BLACK);
          tft.drawString("Dia chi IP: " + WiFi.localIP().toString(), 15, 65);
          
          tft.drawRect(28, 128, 264, 24, TFT_SILVER);
          tft.fillRect(30, 130, 260, 20, 0x2124);
          
          tft.setTextColor(TFT_YELLOW, C_BLACK);
          tft.setTextDatum(MC_DATUM);
          tft.setTextPadding(280); 
          tft.drawString("DANG NHAN DATA TU WEB...", 160, 175);
          tft.setTextPadding(0);

          // [GIẢI PHÁP LỖI 50%]: Đọc dung lượng từ JS gửi qua
          size_t fileSize = server.header("X-File-Size").toInt();
          if (fileSize == 0) fileSize = UPDATE_SIZE_UNKNOWN;

          Update.begin(fileSize); 
      } else if (upload.status == UPLOAD_FILE_WRITE) { 
          Update.write(upload.buf, upload.currentSize); 
      } else if (upload.status == UPLOAD_FILE_END) { 
          // BÙ 100% CHO TFT Ở GIÂY CUỐI CÙNG
          tft.fillRect(30, 130, 260, 20, TFT_GREEN);
          tft.setTextFont(4); tft.setTextColor(TFT_WHITE, C_BLACK);
          tft.setTextDatum(MC_DATUM); tft.setTextPadding(100);
          tft.drawString("100%", 160, 105);

          tft.fillRoundRect(40, 195, 240, 30, 4, TFT_GREEN);
          tft.setTextFont(2); tft.setTextColor(TFT_BLACK, TFT_GREEN);
          tft.setTextDatum(MC_DATUM); tft.setTextPadding(0);
          tft.drawString("THANH CONG! DANG REBOOT...", 160, 210);

          Update.end(true); 
      } 
  });

  // --- HÀM VẼ TIẾN TRÌNH LÊN TFT (CHO WEB OTA) ---
  Update.onProgress([](size_t progress, size_t total) {
      if (!isWebOTA) return; // Chặn đánh nhau với Arduino IDE

      if (total == 0 || total == UPDATE_SIZE_UNKNOWN) total = 1; 
      int percent = (progress * 100) / total;
      if (percent > 100) percent = 100;
      
      static int lastPercent = -1;
      static int lastBarWidth = 0;

      if (progress == 0) { lastPercent = -1; lastBarWidth = 0; }
      
      if (percent != lastPercent) {
          lastPercent = percent;
          
          int barWidth = (percent * 260) / 100;
          if (barWidth > lastBarWidth) {
              tft.fillRect(30 + lastBarWidth, 130, barWidth - lastBarWidth, 20, TFT_GREEN);
              lastBarWidth = barWidth;
          }
          
          tft.setTextFont(4); tft.setTextColor(TFT_WHITE, C_BLACK);
          tft.setTextDatum(MC_DATUM); tft.setTextPadding(100); 
          char pctBuf[10]; snprintf(pctBuf, sizeof(pctBuf), "%d%%", percent);
          tft.drawString(pctBuf, 160, 105); 
          
          tft.setTextFont(2); tft.setTextColor(TFT_SILVER, C_BLACK);
          tft.setTextPadding(280); 
          char infoBuf[50]; snprintf(infoBuf, sizeof(infoBuf), "Da tai: %u KB / %u KB", progress / 1024, total / 1024);
          tft.drawString(infoBuf, 160, 175);
          tft.setTextPadding(0); 
      }
  });

  // --- CÁC ROUTE KHÁC GIỮ NGUYÊN NHƯ CŨ ---
server.on("/blynk_config", HTTP_POST, []() {
      preferences.begin("solar_cfg", true);
      String webPass = preferences.getString("webpass", "254125");
      preferences.end();
      if (!server.authenticate("admin", webPass.c_str())) return server.requestAuthentication();

      // Lấy cả 2 dữ liệu từ Form
      String newToken = server.arg("newToken");
      String newServer = server.arg("newServer");
      newToken.trim();
      newServer.trim();
      
      // Kiểm tra hợp lệ
      bool success = (newToken.length() == 32 && newServer.length() > 0);
      if (success) {
          preferences.begin("solar_cfg", false);
          preferences.putString("token", newToken);
          preferences.putString("server", newServer); // Lưu Server
          preferences.end();
          
          newToken.toCharArray(blynk_token, 34);
          newServer.toCharArray(blynk_server, 40); // Cập nhật biến RAM
      }

      String htmlResult = R"rawliteral(
      <!DOCTYPE html><html lang="vi"><head><meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { font-family: 'Segoe UI', sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; display: flex; justify-content: center; align-items: center; min-height: 90vh; }
        .card { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 30px; text-align: center; max-width: 400px; width: 100%; }
        .icon { font-size: 48px; margin-bottom: 10px; }
        .success { color: #10b981; } .error { color: #ef4444; }
        .btn-group { display: flex; gap: 12px; margin-top: 25px; }
        .btn { flex: 1; padding: 12px; border: none; border-radius: 8px; font-weight: 600; cursor: pointer; text-decoration: none; font-size: 14px; }
        .btn-primary { background: #3b82f6; color: white; }
        .btn-danger { background: rgba(239, 68, 68, 0.1); color: #ef4444; border: 1px solid #ef4444; }
      </style></head><body><div class="card">
      )rawliteral";

      if (success) {
          htmlResult += "<div class='icon success'>&#10004;</div><h2 style='margin:0 0 10px 0;'>Thanh Cong!</h2><p style='color:#94a3b8; font-size:14px;'>Token va Server da duoc luu.</p><p style='font-family:monospace; color:#34d399; word-break:break-all; background:#0f172a; padding:10px; border-radius:6px; margin-bottom: 5px;'>" + newServer + "</p><p style='font-family:monospace; color:#34d399; word-break:break-all; background:#0f172a; padding:10px; border-radius:6px;'>" + newToken + "</p><div class='btn-group'><a href='/' class='btn btn-primary'>Quay Lai</a><a href='/reboot' class='btn btn-danger'>Reboot Ngay</a></div>";
      } else {
          htmlResult += "<div class='icon error'>&#10006;</div><h2 style='margin:0 0 10px 0;'>Loi Cu Phap!</h2><p style='color:#94a3b8; font-size:14px;'>Server khong duoc de trong va Token phai co dung 32 ki tu.</p><div class='btn-group'><a href='/' class='btn btn-primary'>Quay Lai</a></div>";
      }
      htmlResult += "</div></body></html>";
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", htmlResult);
  });
// --- API XỬ LÝ LƯU CẤU HÌNH HIỆU CHỈNH ---
  server.on("/calib_config", HTTP_POST, []() {
      preferences.begin("solar_cfg", true);
      String webPass = preferences.getString("webpass", "254125");
      preferences.end();
      if (!server.authenticate("admin", webPass.c_str())) return server.requestAuthentication();

      // Nhận dữ liệu từ trình duyệt
      String s_vac = server.arg("c_vac"); s_vac.trim();
      String s_aac = server.arg("c_aac"); s_aac.trim();
      String s_vdc = server.arg("c_vdc"); s_vdc.trim();
      String s_adc = server.arg("c_adc"); s_adc.trim();

      // Ép kiểu sang Float và cập nhật biến RAM
      if (s_vac.length() > 0 && s_aac.length() > 0 && s_vdc.length() > 0 && s_adc.length() > 0) {
          calib_v_ac = s_vac.toFloat();
          calib_a_ac = s_aac.toFloat();
          calib_v_dc = s_vdc.toFloat();
          calib_a_dc = s_adc.toFloat();

          // Lưu vĩnh viễn vào bộ nhớ
          preferences.begin("solar_cfg", false);
          preferences.putFloat("c_vac", calib_v_ac);
          preferences.putFloat("c_aac", calib_a_ac);
          preferences.putFloat("c_vdc", calib_v_dc);
          preferences.putFloat("c_adc", calib_a_dc);
          preferences.end();
      }

      // Trả về giao diện báo thành công
      String htmlResult = R"rawliteral(
      <!DOCTYPE html><html lang="vi"><head><meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { font-family: 'Segoe UI', sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; display: flex; justify-content: center; align-items: center; min-height: 90vh; }
        .card { background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 30px; text-align: center; max-width: 400px; width: 100%; }
        .icon { font-size: 48px; margin-bottom: 10px; color: #10b981; }
        .btn { padding: 12px 24px; background: #3b82f6; color: white; border: none; border-radius: 8px; font-weight: 600; cursor: pointer; text-decoration: none; font-size: 14px; display: inline-block; margin-top: 20px;}
      </style></head><body><div class="card">
      <div class='icon'>&#10004;</div><h2 style='margin:0 0 10px 0;'>Thanh Cong!</h2>
      <p style='color:#94a3b8; font-size:14px;'>He so hieu chinh da duoc ap dung. Tu dong chuyen huong sau 3s...</p>
      <a href='/' class='btn'>Quay Lai Ngay</a>
      </div>
      <script>setTimeout(() => window.location.href = "/", 3000);</script>
      </body></html>
      )rawliteral";
      
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", htmlResult);
  });
  server.on("/reboot", HTTP_GET, []() {
      preferences.begin("solar_cfg", true);
      String webPass = preferences.getString("webpass", "254125");
      preferences.end();
      if (!server.authenticate("admin", webPass.c_str())) return server.requestAuthentication();

      String html = R"rawliteral(
      <!DOCTYPE html><html lang="vi"><head><meta name="viewport" content="width=device-width, initial-scale=1">
      <style>
        body { font-family: 'Segoe UI', sans-serif; background: #0f172a; color: #f8fafc; margin: 0; display: flex; justify-content: center; align-items: center; min-height: 100vh; text-align: center; }
        .spinner { width: 40px; height: 40px; border: 4px solid #334155; border-top: 4px solid #3b82f6; border-radius: 50%; animation: spin 1s linear infinite; margin: 0 auto 20px; }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        .text-muted { color: #94a3b8; margin-top: 10px; font-size: 14px; }
        #timer { color: #3b82f6; font-weight: bold; font-size: 18px; }
      </style></head><body>
      <div><div class="spinner"></div><h2 style="margin:0;">He thong dang khoi dong lai</h2><p class="text-muted">Tu dong quay ve trang chu sau <span id="timer">7</span> giay...</p></div>
      <script>let t = 7; setInterval(() => { t--; document.getElementById('timer').innerText = t; if (t <= 0) window.location.href = '/'; }, 1000);</script>
      </body></html>
      )rawliteral";
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", html);
      delay(1000); ESP.restart();
  });
// =========================================================
  // =========================================================
  // 1. TRANG GIÁM SÁT ĐỘC LẬP (KHÔNG CẦN PASS) - TRUY CẬP IP/monitor
  // =========================================================
  server.on("/monitor", HTTP_GET, []() {
      String html = R"rawliteral(
      <!DOCTYPE html>
      <html lang="vi">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Monitor | Smart Solar</title>
        <style>
          :root { --bg: #0f172a; --card-bg: #1e293b; --text: #f8fafc; --text-muted: #94a3b8; --border: #334155; }
          body { font-family: 'Segoe UI', sans-serif; background-color: var(--bg); color: var(--text); margin: 0; padding: 20px; display: flex; justify-content: center; }
          .container { max-width: 480px; width: 100%; }
          .card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 12px; padding: 24px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1); }
          .card-title { margin-top: 0; font-size: 16px; font-weight: 600; border-bottom: 1px solid var(--border); padding-bottom: 12px; margin-bottom: 20px; color: #38bdf8; text-align: center; }
          .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 15px; }
          .box { background: rgba(255,255,255,0.05); padding: 12px; border-radius: 8px; text-align: center; border: 1px solid var(--border); transition: all 0.3s; }
          .box-title { font-size: 12px; color: var(--text-muted); margin-bottom: 4px; display: block; text-transform: uppercase; }
          .box-val { font-size: 19px; font-weight: bold; font-family: monospace; }
          .val-ac { color: #10b981; } 
          .val-dc { color: #f59e0b; }
        </style>
      </head>
      <body>
        <div class="container">
          <div class="card">
            <h2 class="card-title">GIAM SAT TRUC TUYEN</h2>
            
            <h3 style="color:#10b981; font-size:14px; margin: 10px 0 5px 0;">⚡ LUOI DIEN (AC)</h3>
            <div class="grid">
              <div class="box"><span class="box-title">Dien Ap</span><span class="box-val val-ac" id="ac_v">-- V</span></div>
              <div class="box"><span class="box-title">Dong Dien</span><span class="box-val val-ac" id="ac_a">-- A</span></div>
              <div class="box"><span class="box-title">Cong Suat</span><span class="box-val val-ac" id="ac_p">-- W</span></div>
              <div class="box"><span class="box-title">Hom Nay</span><span class="box-val val-ac" id="ac_ed">-- kWh</span></div>
              <div class="box"><span class="box-title">Hom Qua</span><span class="box-val val-ac" id="ac_ep">-- kWh</span></div>
              <div class="box"><span class="box-title">Tong Cong</span><span class="box-val val-ac" id="ac_e">-- kWh</span></div>
            </div>

            <h3 style="color:#f59e0b; font-size:14px; margin: 15px 0 5px 0;">☀️ SOLAR (DC)</h3>
            <div class="grid">
              <div class="box"><span class="box-title">Dien Ap</span><span class="box-val val-dc" id="dc_v">-- V</span></div>
              <div class="box"><span class="box-title">Dong Dien</span><span class="box-val val-dc" id="dc_a">-- A</span></div>
              <div class="box"><span class="box-title">Cong Suat</span><span class="box-val val-dc" id="dc_p">-- W</span></div>
              <div class="box"><span class="box-title">Hom Nay</span><span class="box-val val-dc" id="dc_ed">-- kWh</span></div>
              <div class="box"><span class="box-title">Hom Qua</span><span class="box-val val-dc" id="dc_ep">-- kWh</span></div>
              <div class="box"><span class="box-title">Tong Cong</span><span class="box-val val-dc" id="dc_e">-- kWh</span></div>
            </div>

            <div style="text-align:center; margin-top:15px; font-size:13px; color:var(--text-muted); border-top: 1px solid var(--border); padding-top: 12px;">
              Trang thai Relay: <span id="relay_st" style="font-weight:bold; font-size:14px;">--</span> <br><br>
              T1: <span id="t1" style="color:white; font-family:monospace;">--</span>°C &nbsp;|&nbsp; 
              T2: <span id="t2" style="color:white; font-family:monospace;">--</span>°C &nbsp;|&nbsp; 
              T3: <span id="t3" style="color:white; font-family:monospace;">--</span>°C
            </div>
          </div>
        </div>
        <script>
          // 1. TỰ ĐỘNG CẬP NHẬT TẤT CẢ SỐ ĐO MỖI 2 GIÂY
          setInterval(function() {
            fetch('/data')
              .then(response => response.json())
              .then(data => {
                document.getElementById('ac_v').innerText = data.ac_v + ' V';
                document.getElementById('ac_a').innerText = data.ac_a + ' A';
                document.getElementById('ac_p').innerText = data.ac_p + ' W';
                document.getElementById('ac_ed').innerText = data.ac_ed + ' kWh';
                document.getElementById('ac_ep').innerText = data.ac_ep + ' kWh';
                document.getElementById('ac_e').innerText = data.ac_e + ' kWh';
                
                document.getElementById('dc_v').innerText = data.dc_v + ' V';
                document.getElementById('dc_a').innerText = data.dc_a + ' A';
                document.getElementById('dc_p').innerText = data.dc_p + ' W';
                document.getElementById('dc_ed').innerText = data.dc_ed + ' kWh';
                document.getElementById('dc_ep').innerText = data.dc_ep + ' kWh';
                document.getElementById('dc_e').innerText = data.dc_e + ' kWh';
                
                document.getElementById('t1').innerText = data.t1;
                document.getElementById('t2').innerText = data.t2;
                document.getElementById('t3').innerText = data.t3;
                
                document.getElementById('relay_st').innerHTML = data.relay ? "<span style='color:#10b981'>DANG BAT (ON)</span>" : "<span style='color:#ef4444'>DA TAT (OFF)</span>";
              })
              .catch(err => console.log('Chua lay duoc du lieu live'));
          }, 2000);
        </script>
      </body>
      </html>
      )rawliteral";
      
      server.sendHeader("Connection", "close");
      server.send(200, "text/html", html);
  });

  // =========================================================
  // 2. API XUẤT DỮ LIỆU JSON (MỞ CÔNG KHAI KHÔNG CẦN PASS)
  // =========================================================
  server.on("/data", HTTP_GET, []() { 
      char json[512];
      
      // Chuyển đổi và bảo vệ dữ liệu DC/AC kWh
      float dc_kwh = currentData.dc_energy / 1000.0;
      
      float kwh_ac_day = currentData.ac_energy - latch_ac_day; 
      if (kwh_ac_day < 0) kwh_ac_day = 0;
      
      float kwh_dc_day = (currentData.dc_energy - latch_dc_day) / 1000.0; 
      if (kwh_dc_day < 0) kwh_dc_day = 0;
      
      float kwh_dc_prev = prev_dc_day / 1000.0; // prev_dc_day lưu dạng Wh nên cần chia 1000
      
      // Đóng gói JSON kèm dữ liệu Hôm nay & Hôm qua
      snprintf(json, sizeof(json),
          "{\"ac_v\":%.1f,\"ac_a\":%.2f,\"ac_p\":%.0f,\"ac_e\":%.2f,\"ac_ed\":%.2f,\"ac_ep\":%.2f,"
          "\"dc_v\":%.1f,\"dc_a\":%.2f,\"dc_p\":%.0f,\"dc_e\":%.2f,\"dc_ed\":%.2f,\"dc_ep\":%.2f,"
          "\"t1\":%.1f,\"t2\":%.1f,\"t3\":%.1f,\"relay\":%d}",
          currentData.ac_volt, currentData.ac_amp, currentData.ac_power, currentData.ac_energy, kwh_ac_day, prev_ac_day,
          currentData.dc_volt, currentData.dc_amp, currentData.dc_power, dc_kwh, kwh_dc_day, kwh_dc_prev,
          currentData.t1, currentData.t2, currentData.t3, relayState ? 1 : 0
      );
      
      server.sendHeader("Connection", "close");
      server.send(200, "application/json", json); 
  });

  server.begin();
}
// ======================================================================================
// MODULE MENU & WIFI SCANNER UI


void startScanWifiNetworks() {
    isWifiScanning = true;
    wifiNetworksFound = 0;
    
    tft.fillScreen(C_BLACK);
    tft.setTextFont(4); tft.setTextColor(TFT_ORANGE, C_BLACK); tft.setTextDatum(MC_DATUM);
    tft.drawString("DANG QUET WIFI...", 160, 120);
    
    WiFi.disconnect(); 
    delay(10);
    WiFi.mode(WIFI_STA);
    delay(10);
    
    WiFi.scanNetworks(true); 
    currentScreen = SCREEN_WIFI_SCANNING;
}

void drawWifiScan() {
    if (!menuRedraw) return;
    tft.fillScreen(C_BLACK);
    tft.setTextFont(4); tft.setTextColor(TFT_CYAN, C_BLACK); tft.setTextDatum(MC_DATUM);
    tft.drawString("CHON WIFI", 160, 20); tft.drawFastHLine(0, 40, 320, TFT_BLUE);
   
    const int itemsPerPage = 7;
    int visualTopItem = wifiScanCursor - (itemsPerPage / 2);
    if (visualTopItem < 0) visualTopItem = 0;
    if (visualTopItem > wifiNetworksFound - itemsPerPage) visualTopItem = wifiNetworksFound - itemsPerPage;
    if (visualTopItem < 0) visualTopItem = 0;

    for (int i = 0; i < itemsPerPage; i++) {
        int idx = visualTopItem + i;
        if (idx >= wifiNetworksFound) break;
       
        int y = 50 + i * 24;
        uint16_t bgColor = (idx == wifiScanCursor) ? C_MENU_SEL : C_BLACK;
        uint16_t txtColor = (idx == wifiScanCursor) ? TFT_BLACK : TFT_WHITE;
       
        tft.fillRect(10, y, 300, 22, bgColor);
        tft.setTextFont(2); tft.setTextDatum(TL_DATUM); tft.setTextColor(txtColor, bgColor);
       
        String ssid = WiFi.SSID(idx);
        if (ssid.length() > 20) ssid = ssid.substring(0, 20) + "..";
        String signal = String(WiFi.RSSI(idx)) + "dBm";
       
        tft.drawString(ssid, 15, y + 3);
        tft.setTextDatum(TR_DATUM);
        tft.drawString(signal, 305, y + 3);
    }
    menuRedraw = false;
}

void drawWifiConnecting() {
    static unsigned long lastAnim = 0;
    static int dots = 0;
    if (millis() - lastAnim > 500) {
        lastAnim = millis();
        dots++; if (dots > 3) dots = 0;
        
        tft.setTextFont(4); tft.setTextColor(TFT_GREEN, C_BLACK); tft.setTextDatum(MC_DATUM);
        String s = "DANG KET NOI";
        for(int i=0; i<dots; i++) s += ".";
        tft.drawString(s + "   ", 160, 100); 
        tft.setTextFont(2); tft.drawString(String(selectedSSID), 160, 140);
        tft.setTextColor(TFT_SILVER, C_BLACK);
        tft.drawString("Giu MENU de huy", 160, 180);
    }
}

void drawWifiKeyboard() {
    bool fullRedraw = menuRedraw;
    static bool last_caps = false;
    bool capsChanged = (kb_caps != last_caps);
    
    if (fullRedraw) {
        tft.fillScreen(C_BLACK);
        tft.setTextFont(2); tft.setTextColor(TFT_YELLOW, C_BLACK); tft.setTextDatum(TL_DATUM);
        tft.drawString("MAT KHAU:", 10, 5);
        tft.drawRect(10, 25, 300, 26, TFT_WHITE);
        tft.setTextFont(2); tft.setTextColor(TFT_SILVER, C_BLACK); tft.setTextDatum(MC_DATUM);
        tft.drawString("UP/DOWN: Chon | MENU: Nhap", 160, 225);
        old_kb_cursor = -1; 
    }
    
    tft.setTextFont(2); tft.setTextColor(TFT_WHITE, C_BLACK); tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(290);
    tft.drawString(String(inputPassword) + "_", 15, 30);
    tft.setTextPadding(0);
   
    int startX = 2; int startY = 60;
    int keyW = 30; int keyH = 25; int gap = 2;
    tft.setTextFont(2); tft.setTextDatum(MC_DATUM);

    for (int i = 0; i < KB_TOTAL_KEYS; i++) {
        if (!fullRedraw) {
            bool isSelectionChanged = (i == kb_cursor || i == old_kb_cursor);
            bool isCapsUpdate = capsChanged && ((i >= 10 && i <= 35) || i == 40);
            if (!isSelectionChanged && !isCapsUpdate) continue; 
        }

        int row = 0; int col = 0;
        int xPos = 0; int yPos = 0;
        int w = keyW;
        
        if (i < 10) { row = 0; col = i; xPos = startX + col*(keyW+gap); } 
        else if (i < 20) { row = 1; col = i-10; xPos = startX + col*(keyW+gap); }
        else if (i < 29) { row = 2; col = i-20; xPos = startX + 15 + col*(keyW+gap); }
        else if (i < 36) { row = 3; col = i-29; xPos = startX + 30 + col*(keyW+gap); }
        else if (i < 40) { row = 3; col = i-29; xPos = startX + 30 + col*(keyW+gap); }
        else { 
           row = 4;
           if (i == KB_IDX_CAPS) { xPos = startX; w = 40; }
           else if (i == KB_IDX_SPACE) { xPos = startX + 42; w = 150; }
           else if (i == KB_IDX_DEL) { xPos = startX + 194; w = 60; }
           else if (i == KB_IDX_OK) { xPos = startX + 256; w = 60; }
        }
        yPos = startY + row*(keyH+gap);
        
        uint16_t bg = (i == kb_cursor) ? C_KEY_SEL : C_KEY_BG;
        uint16_t txtColor = (i == kb_cursor) ? TFT_BLACK : TFT_WHITE;
        if (i >= 40) txtColor = (i == kb_cursor) ? TFT_BLACK : TFT_YELLOW; 

        tft.fillRoundRect(xPos, yPos, w, keyH, 3, bg);
        tft.setTextColor(txtColor, bg);
        
        if (i < 40) {
           char c = kb_map_base[i];
           if (i >= 10 && i <= 35 && kb_caps) c = toupper(c);
           char s[2] = {c, '\0'};
           tft.drawString(s, xPos + w/2, yPos + keyH/2);
        } else {
           if (i == KB_IDX_CAPS) tft.drawString(kb_caps ? "abc" : "ABC", xPos + w/2, yPos + keyH/2);
           else if (i == KB_IDX_SPACE) tft.drawString("SPACE", xPos + w/2, yPos + keyH/2);
           else if (i == KB_IDX_DEL) tft.drawString("DEL", xPos + w/2, yPos + keyH/2);
           else if (i == KB_IDX_OK) tft.drawString("OK", xPos + w/2, yPos + keyH/2);
        }
    }
    old_kb_cursor = kb_cursor;
    last_caps = kb_caps; 
    menuRedraw = false;
}


void drawAboutScreen() {
    if (!menuRedraw) return;
    tft.fillScreen(C_BLACK);
    
    tft.setTextFont(4); 
    tft.setTextColor(TFT_YELLOW, C_BLACK); 
    tft.setTextDatum(MC_DATUM);
    tft.drawString("THONG TIN HE THONG", 160, 30); 
    tft.drawFastHLine(0, 50, 320, TFT_BLUE);
    
    tft.setTextFont(2);
    tft.setTextColor(TFT_WHITE, C_BLACK);
    tft.drawString("Du an: SOLAR METER", 160, 90);
    
    tft.setTextColor(TFT_GREEN, C_BLACK);
    tft.drawString(String("Phien ban: ") + FW_VERSION, 160, 120);
    
    tft.setTextColor(TFT_CYAN, C_BLACK);
    tft.drawString("Tac gia: Thien An", 160, 150);
    
    tft.setTextColor(TFT_ORANGE, C_BLACK);
    tft.drawString("Zalo ho tro: 034.989.7777", 160, 180);
    
    tft.setTextColor(TFT_SILVER, C_BLACK);
    tft.drawString("Bam phim bat ky de thoat...", 160, 220);
    
    menuRedraw = false;
}
void drawSensorCheckScreen() {
  // Biến lưu trữ trạng thái để chống chớp (Delta Rendering)
  static unsigned long lastSensorUpdate = 0;
  // Các trạng thái: -1: Mới vào, 0: Lỗi, 1: OK, 2: Đang kết nối
  static int lastStates[5] = {-1, -1, -1, -1, -1}; 

  // 1. PHẦN VẼ KHUNG NỀN & NHÃN CỐ ĐỊNH (Chỉ vẽ ĐÚNG 1 LẦN khi mở trang)
  if (menuRedraw) {
      tft.fillScreen(C_BLACK);
      
      // Header
      tft.fillRect(0, 0, 320, 30, 0x10A2); 
      tft.setTextFont(4); tft.setTextColor(TFT_WHITE, 0x10A2); tft.setTextDatum(MC_DATUM);
      tft.drawString("CHUYEN DOAN CAM BIEN", 160, 15);
      tft.drawFastHLine(0, 30, 320, TFT_CYAN);
    
      // Chân trang
      tft.setTextFont(2); tft.setTextColor(TFT_SILVER, C_BLACK); tft.setTextDatum(MC_DATUM);
      tft.drawString("Bam nut bat ky de quay lai...", 160, 225);
      
      // Vẽ nhãn cố định
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(TFT_SILVER, C_BLACK);
      tft.drawString("1. PZEM AC:", 20, 55);
      tft.drawString("2. PZEM DC:", 20, 83);
      tft.drawString("3. Sensor T1:", 20, 111);
      tft.drawString("4. Sensor T2:", 20, 139);
      tft.drawString("5. Sensor T3:", 20, 167);

      // Reset lại bộ nhớ đệm
      for(int i = 0; i < 5; i++) lastStates[i] = -1;
      lastSensorUpdate = 0; 
      
      menuRedraw = false;
  }

  // 2. PHẦN QUÉT DATA (0.5s / lần)
  if (millis() - lastSensorUpdate < 500) return;
  lastSensorUpdate = millis();

  tft.setTextFont(2); tft.setTextDatum(TL_DATUM);
  
  // Hàm vẽ thông minh: Xử lý 3 màu sắc dựa theo biến "state"
  auto updateStatus = [&](int index, int state, int yPos) {
      if (lastStates[index] != state) {
          lastStates[index] = state; // Chốt trạng thái mới vào bộ nhớ
          
          tft.setTextPadding(160); // Dọn dẹp khoảng trống 160px bên phải
          
          if (state == 2) {
              tft.setTextColor(TFT_ORANGE, C_BLACK);
              tft.drawString("Dang ket noi...", 140, yPos);
          } else if (state == 1) {
              tft.setTextColor(TFT_GREEN, C_BLACK);
              tft.drawString("OK (Ket noi tot)", 140, yPos);
          } else {
              tft.setTextColor(TFT_RED, C_BLACK);
              tft.drawString("Loi / Mat ket noi", 140, yPos);
          }
          
          tft.setTextPadding(0);
      }
  };

  // 3. LOGIC XỬ LÝ 10 GIÂY KHỞI ĐỘNG
  bool isBooting = (millis() < 10000);

  // Cấu trúc: Nếu đang khởi động -> Trạng thái 2. Nếu xong rồi -> Xét xem lỗi hay OK
  int acState = isBooting ? 2 : ((ac_error_count < 10) ? 1 : 0);
  int dcState = isBooting ? 2 : ((dc_error_count < 10) ? 1 : 0);
  int t1State = isBooting ? 2 : ((currentData.t1 != 0) ? 1 : 0);
  int t2State = isBooting ? 2 : ((currentData.t2 != 0) ? 1 : 0);
  int t3State = isBooting ? 2 : ((currentData.t3 != 0) ? 1 : 0);

  // Đẩy dữ liệu vào kiểm tra cập nhật màn hình
  updateStatus(0, acState, 55);
  updateStatus(1, dcState, 83);
  updateStatus(2, t1State, 111);
  updateStatus(3, t2State, 139);
  updateStatus(4, t3State, 167);
}
void initMenuConfig() {
  tempConfig.mode = systemMode; tempConfig.highT = highThreshold; tempConfig.lowT = lowThreshold; tempConfig.dHigh = delayHigh; tempConfig.dLow = delayLow;
  tempConfig.ovLoadW = overloadLimitW; tempConfig.ovRecS = overloadRecoverySec;
  tempConfig.r3En = enableRelay3; tempConfig.r3Mode = r3_mode; tempConfig.r3Min = r3_min_volt; tempConfig.r3Max = r3_max_volt;
  tempConfig.r4En = enableRelay4; tempConfig.r4Mode = r4_mode; tempConfig.r4Min = r4_min_volt; tempConfig.r4Max = r4_max_volt;
  tempConfig.mqttEn = enableMQTT;
  menuCursor = 0; isEditing = false; menuRedraw = true; menuNeedsUpdate = true;
}

bool isItemHidden(int idx) {
    // Ẩn toàn bộ cài đặt nếu không Kích hoạt Relay
    if ((idx >= 8 && idx <= 10) && !tempConfig.r3En) return true;
    if ((idx >= 12 && idx <= 14) && !tempConfig.r4En) return true;
    
    // Nếu chạy theo Công suất W (Mode == 1) -> Ẩn dòng Ngưỡng Min đi
    if (idx == 9 && tempConfig.r3Mode == 1) return true;
    if (idx == 13 && tempConfig.r4Mode == 1) return true;
    
    return false;
}

void drawMenu() {
  if (!menuNeedsUpdate) return;

  int startIdx = 0, endIdx = 6, pageNum = 1;
  const char* pageTitle = "CAI DAT CHUNG (1/5)";

  // Thuật toán chia dải quét Menu thành 5 trang riêng biệt
  if (menuCursor >= 0 && menuCursor <= 6) { startIdx = 0; endIdx = 6; pageNum = 1; pageTitle = "CAI DAT CHUNG (1/5)"; }
  else if (menuCursor >= 7 && menuCursor <= 10) { startIdx = 7; endIdx = 10; pageNum = 2; pageTitle = "CAI DAT RELAY 3 (2/5)"; }
  else if (menuCursor >= 11 && menuCursor <= 14) { startIdx = 11; endIdx = 14; pageNum = 3; pageTitle = "CAI DAT RELAY 4 (3/5)"; }
  else if (menuCursor >= 15 && menuCursor <= 18) { startIdx = 15; endIdx = 18; pageNum = 4; pageTitle = "KET NOI (4/5)"; }
  else if (menuCursor >= 19 && menuCursor <= 22) { startIdx = 19; endIdx = 22; pageNum = 5; pageTitle = "HE THONG (5/5)"; }

  static int lastPageNum = -1;
  if (menuRedraw) lastPageNum = -1; 
  if (pageNum != lastPageNum) { menuRedraw = true; lastPageNum = pageNum; }

  if (menuRedraw) {
    tft.fillScreen(C_BLACK); 
    tft.setTextFont(4); tft.setTextColor(TFT_YELLOW, C_BLACK); tft.setTextDatum(MC_DATUM);
    tft.drawString(pageTitle, 160, 20); 
    tft.drawFastHLine(0, 40, 320, TFT_BLUE); 
    
    tft.fillRect(0, 220, 320, 20, C_BLACK); tft.setTextFont(2); 
    if (WiFi.status() == WL_CONNECTED) {
        tft.setTextColor(TFT_GREEN, C_BLACK);
        tft.drawString(WiFi.SSID() + " - " + WiFi.localIP().toString(), 160, 230);
    } else {
        tft.setTextColor(TFT_DARKGREY, C_BLACK); tft.drawString("WiFi Offline", 160, 230);
    }
    menuRedraw = false;
  }

  int drawYIndex = 0; 
  for (int i = startIdx; i <= endIdx; i++) {
    if (isItemHidden(i)) continue;

    int y = 50 + drawYIndex * 24; drawYIndex++;
    uint16_t txtColor = (i == menuCursor) ? TFT_BLACK : TFT_WHITE;
    uint16_t bgColor = (i == menuCursor) ? C_MENU_SEL : C_BLACK;
   
    if (i == menuCursor && isEditing) { bgColor = TFT_ORANGE; txtColor = TFT_BLACK; }
   
    tft.fillRect(10, y, 300, 22, bgColor);
    tft.setTextFont(2); tft.setTextDatum(TL_DATUM); tft.setTextColor(txtColor, bgColor);

    const char* itemName = menuItems[i];
    if (i == 10 && tempConfig.r3Mode == 1) itemName = "R3 Nguong (W)";
    if (i == 14 && tempConfig.r4Mode == 1) itemName = "R4 Nguong (W)";
    
    tft.drawString(itemName, 15, y + 3);
    
    char valStr[20] = "";
    if (i == 0) { if (tempConfig.mode == MODE_AUTO) strcpy(valStr, "AUTO"); else if (tempConfig.mode == MODE_MANUAL) strcpy(valStr, "MANUAL"); else strcpy(valStr, "BACKUP"); }
    else if (i == 1) snprintf(valStr, sizeof(valStr), "%.1f V", tempConfig.highT);
    else if (i == 2) snprintf(valStr, sizeof(valStr), "%.1f V", tempConfig.lowT);
    else if (i == 3) snprintf(valStr, sizeof(valStr), "%d s", tempConfig.dHigh);
    else if (i == 4) snprintf(valStr, sizeof(valStr), "%d s", tempConfig.dLow);
    else if (i == 5) snprintf(valStr, sizeof(valStr), "%.0f W", tempConfig.ovLoadW);
    else if (i == 6) snprintf(valStr, sizeof(valStr), "%d s", tempConfig.ovRecS);
    else if (i == 7) strcpy(valStr, tempConfig.r3En ? "ON" : "OFF");
    else if (i == 8) strcpy(valStr, tempConfig.r3Mode == 0 ? "VOLT DC" : "WATT AC");
    else if (i == 9) snprintf(valStr, sizeof(valStr), "%.1f V", tempConfig.r3Min);
    else if (i == 10) snprintf(valStr, sizeof(valStr), tempConfig.r3Mode == 0 ? "%.1f V" : "%.0f W", tempConfig.r3Max);
    else if (i == 11) strcpy(valStr, tempConfig.r4En ? "ON" : "OFF");
    else if (i == 12) strcpy(valStr, tempConfig.r4Mode == 0 ? "VOLT DC" : "WATT AC");
    else if (i == 13) snprintf(valStr, sizeof(valStr), "%.1f V", tempConfig.r4Min);
    else if (i == 14) snprintf(valStr, sizeof(valStr), tempConfig.r4Mode == 0 ? "%.1f V" : "%.0f W", tempConfig.r4Max);
    
    // Đã chuyển biến hiển thị chữ ON/OFF về đúng ID số 17 mới sắp xếp
    else if (i == 17) strcpy(valStr, tempConfig.mqttEn ? "BAT (ON)" : "TAT (OFF)");
   
    if (strlen(valStr) > 0) { tft.setTextDatum(TR_DATUM); tft.drawString(valStr, 305, y + 3); }
  } 

  for (int i = drawYIndex; i < 7; i++) { tft.fillRect(10, 50 + i * 24, 300, 22, C_BLACK); }
  menuNeedsUpdate = false;
}
void handleMenuInput(int btnType, bool isHold) {
  bool changed = false;
  
  if (!isEditing) {
    if (btnType == 1) { 
        do { menuCursor--; if (menuCursor < 0) menuCursor = MENU_ITEM_COUNT - 1; } while(isItemHidden(menuCursor));
        changed = true;
    }
    else if (btnType == 3) { 
        do { menuCursor++; if (menuCursor >= MENU_ITEM_COUNT) menuCursor = 0; } while(isItemHidden(menuCursor));
        changed = true;
    }
    else if (btnType == 2) { 
       // --- TRANG 4: KẾT NỐI ---
       if (menuCursor == 15) { startScanWifiNetworks(); return; }
       else if (menuCursor == 16) { 
          tft.fillScreen(TFT_RED); tft.setTextColor(TFT_WHITE, TFT_RED); tft.drawString(F("RESET WIFI..."), 160, 120);
          WiFiManager wm; wm.resetSettings(); preferences.begin("solar_cfg", false); preferences.putString("cssid", ""); preferences.end();
          memset(custom_ssid, 0, sizeof(custom_ssid)); delay(1000); ESP.restart();
       } 
       // Mục 17 (Web app) đã được nhường cho chế độ Edit bên dưới
       else if (menuCursor == 18) { 
          strcpy(inputVPS, mqtt_server);
          kb_cursor = 0; kb_caps = false; menuRedraw = true;
          currentScreen = SCREEN_VPS_INPUT; return; 
       }

       // --- TRANG 5: HỆ THỐNG ---
       else if (menuCursor == 19) { currentScreen = SCREEN_ABOUT; menuRedraw = true; return; }
       else if (menuCursor == 20) { currentScreen = SCREEN_SENSOR_CHECK; menuRedraw = true; return; }
       else if (menuCursor == 21) { 
          systemMode = tempConfig.mode; highThreshold = tempConfig.highT; lowThreshold = tempConfig.lowT; delayHigh = tempConfig.dHigh; delayLow = tempConfig.dLow;
          overloadLimitW = tempConfig.ovLoadW; overloadRecoverySec = tempConfig.ovRecS;
          enableRelay3 = tempConfig.r3En; r3_mode = tempConfig.r3Mode; r3_min_volt = tempConfig.r3Min; r3_max_volt = tempConfig.r3Max;
          enableRelay4 = tempConfig.r4En; r4_mode = tempConfig.r4Mode; r4_min_volt = tempConfig.r4Min; r4_max_volt = tempConfig.r4Max;
          
          // --- CHỐT LƯU TRẠNG THÁI WEB APP ---
          if (enableMQTT != tempConfig.mqttEn) {
              enableMQTT = tempConfig.mqttEn;
              preferences.begin("solar_cfg", false); 
              preferences.putBool("mqtt_en", enableMQTT); 
              preferences.end();
              if (!enableMQTT && mqttClient.connected()) mqttClient.disconnect();
          }
          
          saveSettings(); flag_sync_settings = true; currentScreen = SCREEN_MAIN; drawInterface(); return;
       } 
       else if (menuCursor == 22) { currentScreen = SCREEN_MAIN; drawInterface(); return; }
       
       // Mọi mục còn lại (kể cả số 17) sẽ kích hoạt chế độ Edit màu Cam
       else { isEditing = true; changed = true; } 
    }
  } 
  else { 
    float step = isHold ? 1.0 : 0.1; int iStep = isHold ? 5 : 1; float wStep = isHold ? 100.0 : 10.0;
    
    if (btnType == 1) { // LÊN (UP)
       if (menuCursor == 0) { tempConfig.mode++; if (tempConfig.mode > 2) tempConfig.mode = 0; }
       else if (menuCursor == 1) tempConfig.highT += step; else if (menuCursor == 2) tempConfig.lowT += step;
       else if (menuCursor == 3) tempConfig.dHigh += iStep; else if (menuCursor == 4) tempConfig.dLow += iStep;
       else if (menuCursor == 5) { tempConfig.ovLoadW += wStep; if (tempConfig.ovLoadW > 22000.0) tempConfig.ovLoadW = 22000.0; }
       else if (menuCursor == 6) tempConfig.ovRecS += iStep; 
       else if (menuCursor == 7) tempConfig.r3En = !tempConfig.r3En;
       else if (menuCursor == 8) tempConfig.r3Mode = !tempConfig.r3Mode;
       else if (menuCursor == 9) tempConfig.r3Min += (tempConfig.r3Mode == 0 ? step : wStep);
       else if (menuCursor == 10) tempConfig.r3Max += (tempConfig.r3Mode == 0 ? step : wStep);
       else if (menuCursor == 11) tempConfig.r4En = !tempConfig.r4En;
       else if (menuCursor == 12) tempConfig.r4Mode = !tempConfig.r4Mode;
       else if (menuCursor == 13) tempConfig.r4Min += (tempConfig.r4Mode == 0 ? step : wStep);
       else if (menuCursor == 14) tempConfig.r4Max += (tempConfig.r4Mode == 0 ? step : wStep);
       else if (menuCursor == 17) tempConfig.mqttEn = !tempConfig.mqttEn; // <-- ĐỔI TRẠNG THÁI WEB APP
    } else if (btnType == 3) { // XUỐNG (DOWN)
       if (menuCursor == 0) { tempConfig.mode--; if (tempConfig.mode < 0) tempConfig.mode = 2; }
       else if (menuCursor == 1) tempConfig.highT -= step; else if (menuCursor == 2) tempConfig.lowT -= step;
       else if (menuCursor == 3) tempConfig.dHigh -= iStep; else if (menuCursor == 4) tempConfig.dLow -= iStep;
       else if (menuCursor == 5) { tempConfig.ovLoadW -= wStep; if (tempConfig.ovLoadW < 10.0) tempConfig.ovLoadW = 10.0; }
       else if (menuCursor == 6) { tempConfig.ovRecS -= iStep; if (tempConfig.ovRecS < 0) tempConfig.ovRecS = 0; } 
       else if (menuCursor == 7) tempConfig.r3En = !tempConfig.r3En;
       else if (menuCursor == 8) tempConfig.r3Mode = !tempConfig.r3Mode;
       else if (menuCursor == 9) tempConfig.r3Min -= (tempConfig.r3Mode == 0 ? step : wStep);
       else if (menuCursor == 10) tempConfig.r3Max -= (tempConfig.r3Mode == 0 ? step : wStep);
       else if (menuCursor == 11) tempConfig.r4En = !tempConfig.r4En;
       else if (menuCursor == 12) tempConfig.r4Mode = !tempConfig.r4Mode;
       else if (menuCursor == 13) tempConfig.r4Min -= (tempConfig.r4Mode == 0 ? step : wStep);
       else if (menuCursor == 14) tempConfig.r4Max -= (tempConfig.r4Mode == 0 ? step : wStep);
       else if (menuCursor == 17) tempConfig.mqttEn = !tempConfig.mqttEn; // <-- ĐỔI TRẠNG THÁI WEB APP
    } else if (btnType == 2) { isEditing = false; }
    changed = true;
  }
  if (changed) menuNeedsUpdate = true;
}
void handleWifiScanInput(int btnType) {
    if (btnType == 1) { // UP
        wifiScanCursor--;
        if (wifiScanCursor < 0) wifiScanCursor = wifiNetworksFound - 1;
        menuRedraw = true;
    } else if (btnType == 3) { // DOWN
        wifiScanCursor++;
        if (wifiScanCursor >= wifiNetworksFound) wifiScanCursor = 0;
        menuRedraw = true;
    } else if (btnType == 2) { // SELECT
        String ssid = WiFi.SSID(wifiScanCursor);
        ssid.toCharArray(selectedSSID, 33); 
        memset(inputPassword, 0, 65); 
        kb_cursor = 0;
        kb_caps = false;
        currentScreen = SCREEN_WIFI_PASS;
        menuRedraw = true;
    } else if (btnType == 4) { // CANCEL
        currentScreen = SCREEN_MENU;
        menuRedraw = true; menuNeedsUpdate = true;
    }
}
void handleWifiKeyboardInput(int btnType) {
    if (btnType == 1) { // UP
        kb_cursor--;
        if (kb_cursor < 0) kb_cursor = KB_TOTAL_KEYS - 1;
    } else if (btnType == 3) { // DOWN
        kb_cursor++;
        if (kb_cursor >= KB_TOTAL_KEYS) kb_cursor = 0;
    } else if (btnType == 2) { // SELECT
        if (kb_cursor == KB_IDX_CAPS) { 
            kb_caps = !kb_caps;
        }
        else if (kb_cursor == KB_IDX_SPACE) {
            int len = strlen(inputPassword);
            if (len < 64) { inputPassword[len] = ' '; inputPassword[len+1] = 0; }
        }
        else if (kb_cursor == KB_IDX_DEL) {
            int len = strlen(inputPassword);
            if (len > 0) inputPassword[len-1] = 0;
        }
        else if (kb_cursor == KB_IDX_OK) {
             tft.fillScreen(C_BLACK);
             preferences.begin("solar_cfg", false); 
             preferences.putString("cssid", selectedSSID); 
             preferences.putString("cpass", inputPassword);
             preferences.end();
             
             // [LOGIC MOI] Cap nhat SSID vao bo nho runtime de cho phep quet
             strcpy(custom_ssid, selectedSSID);
             strcpy(custom_pass, inputPassword);

             WiFi.disconnect();
             WiFi.begin(selectedSSID, inputPassword);
             
             t_wifi_connect_start = millis();
             currentScreen = SCREEN_WIFI_CONNECTING;
             return;
        }
        else if (kb_cursor < 40) {
            char c = kb_map_base[kb_cursor];
            if (kb_cursor >= 10 && kb_cursor <= 35 && kb_caps) c = toupper(c);
            int len = strlen(inputPassword);
            if (len < 64) { inputPassword[len] = c; inputPassword[len+1] = 0; }
        }
    } else if (btnType == 4) { // HOLD MENU
         if (strlen(inputPassword) > 0) {
             inputPassword[strlen(inputPassword)-1] = 0;
         } else {
             currentScreen = SCREEN_WIFI_SCAN;
             menuRedraw = true;
         }
    }
}
void handleWifiConnectingInput(int btnType) {
    if (btnType == 4) { 
        WiFi.disconnect();
        currentScreen = SCREEN_MENU;
        menuRedraw = true; menuNeedsUpdate = true;
    }
}

// ==========================================
// BÀN PHÍM NHẬP ĐỊA CHỈ VPS
// ==========================================
void drawVPSKeyboard() {
    bool fullRedraw = menuRedraw;
    static bool last_caps = false;
    bool capsChanged = (kb_caps != last_caps);
    
    if (fullRedraw) {
        tft.fillScreen(C_BLACK);
        tft.setTextFont(2); tft.setTextColor(TFT_YELLOW, C_BLACK); tft.setTextDatum(TL_DATUM);
        tft.drawString("DIA CHI VPS (MQTT):", 10, 5);
        tft.drawRect(10, 25, 300, 26, TFT_WHITE);
        tft.setTextFont(2); tft.setTextColor(TFT_SILVER, C_BLACK); tft.setTextDatum(MC_DATUM);
        tft.drawString("UP/DOWN: Chon | MENU: Nhap", 160, 225);
        old_kb_cursor = -1; 
    }
    
    tft.setTextFont(2); tft.setTextColor(TFT_WHITE, C_BLACK); tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(290);
    tft.drawString(String(inputVPS) + "_", 15, 30);
    tft.setTextPadding(0);
   
    int startX = 2; int startY = 60;
    int keyW = 30; int keyH = 25; int gap = 2;
    tft.setTextFont(2); tft.setTextDatum(MC_DATUM);

    for (int i = 0; i < KB_TOTAL_KEYS; i++) {
        if (!fullRedraw) {
            bool isSelectionChanged = (i == kb_cursor || i == old_kb_cursor);
            bool isCapsUpdate = capsChanged && ((i >= 10 && i <= 35) || i == 40);
            if (!isSelectionChanged && !isCapsUpdate) continue; 
        }

        int row = 0; int col = 0; int xPos = 0; int yPos = 0; int w = keyW;
        if (i < 10) { row = 0; col = i; xPos = startX + col*(keyW+gap); } 
        else if (i < 20) { row = 1; col = i-10; xPos = startX + col*(keyW+gap); }
        else if (i < 29) { row = 2; col = i-20; xPos = startX + 15 + col*(keyW+gap); }
        else if (i < 36) { row = 3; col = i-29; xPos = startX + 30 + col*(keyW+gap); }
        else if (i < 40) { row = 3; col = i-29; xPos = startX + 30 + col*(keyW+gap); }
        else { 
           row = 4;
           if (i == KB_IDX_CAPS) { xPos = startX; w = 40; }
           else if (i == KB_IDX_SPACE) { xPos = startX + 42; w = 150; }
           else if (i == KB_IDX_DEL) { xPos = startX + 194; w = 60; }
           else if (i == KB_IDX_OK) { xPos = startX + 256; w = 60; }
        }
        yPos = startY + row*(keyH+gap);
        
        uint16_t bg = (i == kb_cursor) ? C_KEY_SEL : C_KEY_BG;
        uint16_t txtColor = (i == kb_cursor) ? TFT_BLACK : TFT_WHITE;
        if (i >= 40) txtColor = (i == kb_cursor) ? TFT_BLACK : TFT_YELLOW; 

        tft.fillRoundRect(xPos, yPos, w, keyH, 3, bg);
        tft.setTextColor(txtColor, bg);
        
        if (i < 40) {
           char c = kb_map_base[i];
           if (i >= 10 && i <= 35 && kb_caps) c = toupper(c);
           char s[2] = {c, '\0'};
           tft.drawString(s, xPos + w/2, yPos + keyH/2);
        } else {
           if (i == KB_IDX_CAPS) tft.drawString(kb_caps ? "abc" : "ABC", xPos + w/2, yPos + keyH/2);
           else if (i == KB_IDX_SPACE) tft.drawString("SPACE", xPos + w/2, yPos + keyH/2);
           else if (i == KB_IDX_DEL) tft.drawString("DEL", xPos + w/2, yPos + keyH/2);
           else if (i == KB_IDX_OK) tft.drawString("OK", xPos + w/2, yPos + keyH/2);
        }
    }
    old_kb_cursor = kb_cursor; last_caps = kb_caps; menuRedraw = false;
}


void handleVPSKeyboardInput(int btnType) {
    if (btnType == 1) { kb_cursor--; if (kb_cursor < 0) kb_cursor = KB_TOTAL_KEYS - 1; } 
    else if (btnType == 3) { kb_cursor++; if (kb_cursor >= KB_TOTAL_KEYS) kb_cursor = 0; } 
    else if (btnType == 2) { 
        if (kb_cursor == KB_IDX_CAPS) { kb_caps = !kb_caps; }
        else if (kb_cursor == KB_IDX_SPACE) {
            int len = strlen(inputVPS); if (len < 40) { inputVPS[len] = ' '; inputVPS[len+1] = 0; }
        }
        else if (kb_cursor == KB_IDX_DEL) {
            int len = strlen(inputVPS); if (len > 0) inputVPS[len-1] = 0;
        }
        else if (kb_cursor == KB_IDX_OK) {
             tft.fillScreen(C_BLACK);
             preferences.begin("solar_cfg", false); 
             preferences.putString("mqtt_srv", inputVPS); 
             preferences.end();
             
             strcpy(mqtt_server, inputVPS);
             if (mqttClient.connected()) mqttClient.disconnect();
             
             currentScreen = SCREEN_MENU; menuRedraw = true; menuNeedsUpdate = true;
             return;
        }
        else if (kb_cursor < 40) {
            char c = kb_map_base[kb_cursor];
            if (kb_cursor >= 10 && kb_cursor <= 35 && kb_caps) c = toupper(c);
            int len = strlen(inputVPS);
            if (len < 40) { inputVPS[len] = c; inputVPS[len+1] = 0; }
        }
    } else if (btnType == 4) { 
         if (strlen(inputVPS) > 0) { inputVPS[strlen(inputVPS)-1] = 0; } 
         else { currentScreen = SCREEN_MENU; menuRedraw = true; menuNeedsUpdate = true; }
    }
}

// ======================================================================================
// MAIN LOOP & LOGIC
// ======================================================================================
void executeAction(int btnIndex, int actionType) {
  int btnID = btnIndex; // 0:UP, 1:MENU, 2:DOWN
  if (currentScreen == SCREEN_MAIN) {
      if (btnID == 0) { // UP
          if (systemMode == MODE_MANUAL) { manualRelayCmd = true; setRelay(true); }
          else { 
            // [CRITICAL LOGIC] Tắt Force Off -> FALSE
            forceOff = false; 
            preferences.begin("solar_cfg", false); preferences.putBool("forceOff", forceOff); preferences.end(); 
          }
      } else if (btnID == 2) { // DOWN
          if (systemMode == MODE_MANUAL) { manualRelayCmd = false; setRelay(false); }
          else { 
            // [CRITICAL LOGIC] Bật Force Off -> TRUE
            forceOff = true; setRelay(false); 
            preferences.begin("solar_cfg", false); preferences.putBool("forceOff", forceOff); preferences.end(); 
          }
      } else if (btnID == 1) { // MENU
          if (actionType == 2) { currentScreen = SCREEN_MENU; initMenuConfig(); }
      }
  } else if (currentScreen == SCREEN_MENU) {
      if (btnID == 1 && actionType == 2) { currentScreen = SCREEN_MAIN; drawInterface(); }
      else {
         int mappedBtn = (btnID == 0) ? 1 : ((btnID == 1) ? 2 : 3);
         if (btnID == 1) mappedBtn = (actionType == 2) ? 4 : 2;
         handleMenuInput(mappedBtn, actionType == 2);
      }
  } else if (currentScreen == SCREEN_WIFI_SCAN) {
      int mappedBtn = (btnID == 0) ? 1 : ((btnID == 1) ? 2 : 3);
      if (btnID == 1 && actionType == 2) mappedBtn = 4;
      handleWifiScanInput(mappedBtn);
  } else if (currentScreen == SCREEN_WIFI_PASS) {
      int mappedBtn = (btnID == 0) ? 1 : ((btnID == 1) ? 2 : 3);
      if (btnID == 1 && actionType == 2) mappedBtn = 4;
      handleWifiKeyboardInput(mappedBtn);
  } else if (currentScreen == SCREEN_WIFI_CONNECTING) {
      int mappedBtn = 0; if(btnID == 1 && actionType == 2) mappedBtn = 4;
      handleWifiConnectingInput(mappedBtn);
  } else if (currentScreen == SCREEN_ABOUT) { 
      if (actionType == 1) { 
          currentScreen = SCREEN_MENU; menuRedraw = true; menuNeedsUpdate = true;
      }
  } else if (currentScreen == SCREEN_SENSOR_CHECK) { // <-- THÊM ĐOẠN NÀY
      if (actionType == 1) { 
          currentScreen = SCREEN_MENU; menuRedraw = true; menuNeedsUpdate = true;
      }
  }
else if (currentScreen == SCREEN_VPS_INPUT) {
      int mappedBtn = (btnID == 0) ? 1 : ((btnID == 1) ? 2 : 3);
      if (btnID == 1 && actionType == 2) mappedBtn = 4;
      handleVPSKeyboardInput(mappedBtn);
  }
  // --------------------------------------------------
}

void checkButtons() {
  for (int i = 0; i < 3; i++) {
    bool reading = digitalRead(btns[i].pin);
    unsigned long now = millis();
    if (reading != btns[i].lastReading) btns[i].lastDebounceTime = now;
    btns[i].lastReading = reading;
    if ((now - btns[i].lastDebounceTime) > DEBOUNCE_DELAY) {
       if (reading != btns[i].stableState) {
          btns[i].stableState = reading;
          if (btns[i].stableState == LOW) { 
             t_last_user_action = millis();
             btns[i].pressTime = now; btns[i].eventHandled = false; btns[i].repeatTimer = now + 600;
             if (i != 1) { executeAction(i, 1); btns[i].eventHandled = true; }
          } else { 
             if (i == 1 && !btns[i].eventHandled) { if (now - btns[i].pressTime < HOLD_TIME) executeAction(i, 1); }
          }
       }
       if (btns[i].stableState == LOW) {
           if (i == 1 && !btns[i].eventHandled && (now - btns[i].pressTime > HOLD_TIME)) { executeAction(i, 2); btns[i].eventHandled = true; }
           else if (i != 1 && (currentScreen == SCREEN_MENU || currentScreen == SCREEN_WIFI_PASS || currentScreen == SCREEN_WIFI_SCAN)) {
               if (now > btns[i].repeatTimer) { 
                   executeAction(i, 2); // <--- KÍCH HOẠT CỜ "HOLD" BẰNG SỐ 2
                   btns[i].repeatTimer = now + 100; // <--- CHỈNH XUỐNG 100ms ĐỂ NHẢY SỐ TỐC ĐỘ CAO
               }
           }
       }
    }
  }
} // <--- DẤU NGOẶC CHỐT HẠ BẠN VỪA BỊ THIẾU

void TaskSensorsCode(void * parameter) {
  const TickType_t xDelay = 50 / portTICK_PERIOD_MS; esp_task_wdt_add(NULL);
  bool isTurnAC = true;
  static int loop_counter = 0;
  float temp1 = 0, temp2 = 0, temp3 = 0;
  static bool isBooting = true; // [FIX 3] Cờ chống tràn millis() 49.7 ngày

  for(;;) {
    esp_task_wdt_reset(); // Luôn cho WDT ăn để không bị Crash

    // [FIX 1] Ngưng tương tác phần cứng khi đang nạp OTA, nhưng giữ Task chạy
    if (isSystemUpdating) {
        vTaskDelay(xDelay);
        continue;
    }

    // [FIX 3] Đợi 3 giây ổn định lúc khởi động
    if (isBooting) {
        if (millis() > 3000) {
            isBooting = false;
        } else {
            vTaskDelay(xDelay);
            continue;
        }
    }
    unsigned long t_start_core0 = micros();
    DataBox localRead = {0};
    
    // Lấy dữ liệu cũ để cập nhật dần
    if (xSemaphoreTake(dataMutex, (TickType_t) 100) == pdTRUE) { 
        localRead = sharedData; 
        xSemaphoreGive(dataMutex); 
    } else { 
        vTaskDelay(xDelay); 
        continue; 
    }
    
    // Xử lý cài đặt Shunt
    if (flag_req_setShunt) {
        while(SerialDC.available()) SerialDC.read();
        bool success = false;
        for(int i = 1; i <= 3; i++) {
           if (nodeDC.writeSingleRegister(0x0003, val_req_shunt) == nodeDC.ku8MBSuccess) { success = true; break; }
           delay(150); while(SerialDC.available()) SerialDC.read();
        }
        flag_shunt_result = success ? 1 : 2; flag_req_setShunt = false; vTaskDelay(xDelay); continue;
    }

    // =======================================================
    // ĐOẠN CODE XỬ LÝ CỜ RESET
    // =======================================================
    if (flag_reset_ac) {
        // Core 0 trực tiếp gửi lệnh UART an toàn
        uint8_t resetCmd[] = {0x01, 0x42, 0x80, 0x11};
        SerialAC.write(resetCmd, 4);
        vTaskDelay(150 / portTICK_PERIOD_MS); // Chờ PZEM xử lý lệnh
        while(SerialAC.available()) SerialAC.read(); // Xóa sạch rác bộ đệm
        
        esp_total_ac_energy = 0;
        latch_ac_day = 0; latch_ac_month = 0;
        prev_ac_day = 0; prev_ac_month = 0;

        // BẢO VỆ LÕI: Sinh ra biến bộ nhớ độc lập cho Core 0
        //Preferences localPref; 
        //localPref.begin("energy", false);
        //localPref.putFloat("esp_tot_ac", 0);
       // localPref.putFloat("ac_d", 0);
       // localPref.putFloat("ac_m", 0);
        //localPref.putFloat("p_ac_d", 0);
        //localPref.putFloat("p_ac_m", 0);
       // localPref.end();
        
        flag_reset_ac = false;
    }

    if (flag_reset_dc) {
        // Core 0 trực tiếp gửi lệnh UART an toàn để reset chip phần cứng PZEM DC
        uint8_t resetCmd[] = {0x01, 0x42, 0x80, 0x11};
        SerialDC.write(resetCmd, 4);
        vTaskDelay(150 / portTICK_PERIOD_MS); // Chờ PZEM xử lý lệnh
        while(SerialDC.available()) SerialDC.read(); // Xóa sạch rác bộ đệm

        esp_total_dc_energy = 0;
        latch_dc_day = 0; latch_dc_month = 0;
        prev_dc_day = 0; prev_dc_month = 0;
        
        flag_reset_dc = false;
    }
    // =======================================================

    // Xóa bộ đệm Serial trước khi đọc Modbus
    if (isTurnAC) { while(SerialAC.available()) SerialAC.read(); } 
    else { while(SerialDC.available()) SerialDC.read(); }

    // --- BẮT ĐẦU ĐỌC DỮ LIỆU ---
    if (isTurnAC) {
       if (nodeAC.readInputRegisters(0x0000, 10) == nodeAC.ku8MBSuccess) {
           ac_error_count = 0;
           float v = (nodeAC.getResponseBuffer(0) / 10.0f) * calib_v_ac; 
           if (v < 40.0 || v > 500.0) v = 0;
           localRead.ac_volt = v;
           
           if (v == 0) {
              // Bắt buộc đẩy tất cả về 0 khi mất nguồn đo để không cộng dồn nhiễu
              localRead.ac_amp = 0; 
              localRead.ac_power = 0; 
              localRead.ac_freq = 0; 
              localRead.ac_pf = 0;
           } else {
              uint32_t tempAmp = ((uint32_t)nodeAC.getResponseBuffer(2) << 16) | nodeAC.getResponseBuffer(1); 
              localRead.ac_amp = (tempAmp / 1000.0f) * calib_a_ac; 
              
              uint32_t tempPower = ((uint32_t)nodeAC.getResponseBuffer(4) << 16) | nodeAC.getResponseBuffer(3); 
              localRead.ac_power = (tempPower / 10.0f) * (calib_v_ac * calib_a_ac); 
              
              localRead.ac_freq = nodeAC.getResponseBuffer(7) / 10.0f; 
              localRead.ac_pf = nodeAC.getResponseBuffer(8) / 100.0f;
           }
           
           // --- THUẬT TOÁN HYBRID DELTA AC (CHUẨN ĐỒNG HỒ) ---
// Thanh ghi 5 (LSB) và 6 (MSB) chứa giá trị Energy (Wh)
uint32_t pzem_ac_energy_wh = ((uint32_t)nodeAC.getResponseBuffer(6) << 16) | nodeAC.getResponseBuffer(5);

static uint32_t last_pzem_ac_energy = 0;
static bool first_ac_read = true;

if (first_ac_read) {
    last_pzem_ac_energy = pzem_ac_energy_wh;
    first_ac_read = false;
} else {
    if (pzem_ac_energy_wh >= last_pzem_ac_energy) {
        // Cộng phần chênh lệch (Delta)
        uint32_t delta_wh = pzem_ac_energy_wh - last_pzem_ac_energy;
        esp_total_ac_energy += (delta_wh / 1000.0); // Chuyển Wh sang kWh và cộng vào RAM ESP
    } else {
        // Bắt lỗi: Nếu PZEM bị reset về 0 (do người dùng gửi lệnh reset)
        // Ta lấy thẳng số mới đếm được cộng tiếp vào tổng
        esp_total_ac_energy += (pzem_ac_energy_wh / 1000.0);
    }
    last_pzem_ac_energy = pzem_ac_energy_wh; // Cập nhật mốc cho chu kỳ sau
}
       } else {
           ac_error_count++; 
           if (ac_error_count >= 10) { 
               localRead.ac_volt = 0; 
               localRead.ac_amp = 0; 
               localRead.ac_power = 0;
               localRead.ac_freq = 0;
               localRead.ac_pf = 0; 
           }
       }
    } else {
       if (nodeDC.readInputRegisters(0x0000, 6) == nodeDC.ku8MBSuccess) {
          dc_error_count = 0;
          float v_dc = (nodeDC.getResponseBuffer(0) / 100.0f) * calib_v_dc; 
          if (v_dc > 400.0) v_dc = 0;
          localRead.dc_volt = v_dc;
          
          if (v_dc == 0) {
              // Bắt buộc đẩy tất cả về 0 khi mất nguồn đo để không cộng dồn nhiễu
              localRead.dc_amp = 0;
              localRead.dc_power = 0;
          } else {
              localRead.dc_amp = (nodeDC.getResponseBuffer(1) / 100.0f) * calib_a_dc; 
              uint32_t tempDCPower = ((uint32_t)nodeDC.getResponseBuffer(3) << 16) | nodeDC.getResponseBuffer(2);
              localRead.dc_power = (tempDCPower / 10.0f) * (calib_v_dc * calib_a_dc);
          }
          
          // --- THUẬT TOÁN HYBRID DELTA DC (CHUẨN ĐỒNG HỒ) ---
// Thanh ghi 4 (LSB) và 5 (MSB) của PZEM-017 chứa giá trị Energy (Wh)
uint32_t pzem_dc_energy_wh = ((uint32_t)nodeDC.getResponseBuffer(5) << 16) | nodeDC.getResponseBuffer(4);

static uint32_t last_pzem_dc_energy = 0;
static bool first_dc_read = true;

if (first_dc_read) {
    last_pzem_dc_energy = pzem_dc_energy_wh;
    first_dc_read = false;
} else {
    if (pzem_dc_energy_wh >= last_pzem_dc_energy) {
        // Cộng phần chênh lệch (Delta)
        uint32_t delta_wh = pzem_dc_energy_wh - last_pzem_dc_energy;
        esp_total_dc_energy += delta_wh; // Với DC, ESP đang lưu biến tổng dưới dạng Wh
    } else {
        // Bắt lỗi: Nếu PZEM DC bị reset về 0
        esp_total_dc_energy += pzem_dc_energy_wh;
    }
    last_pzem_dc_energy = pzem_dc_energy_wh; // Cập nhật mốc cho chu kỳ sau
}
       } else {
          dc_error_count++; 
          if (dc_error_count >= 10) { 
              localRead.dc_volt = 0; 
              localRead.dc_amp = 0; 
              localRead.dc_power = 0; 
          }
       }
    }
    
    // --- XỬ LÝ NHIỆT ĐỘ BẤT ĐỒNG BỘ ---
    isTurnAC = !isTurnAC; 
    loop_counter++;
    if (loop_counter == 1) { 
        sensors.requestTemperatures(); 
    } else if (loop_counter == 16) {
        float t;
        
        // --- XỬ LÝ SENSOR 1 ---
        t = sensors.getTempCByIndex(0); 
        if (t == -127 || t == 85) temp1 = 0; 
        else temp1 = t; 

        // --- XỬ LÝ SENSOR 2 ---
        t = sensors.getTempCByIndex(1); 
        if (t == -127 || t == 85) temp2 = 0; 
        else temp2 = t; 

        // --- XỬ LÝ SENSOR 3 ---
        t = sensors.getTempCByIndex(2); 
        if (t == -127 || t == 85) temp3 = 0; 
        else temp3 = t; 
    } else if (loop_counter >= 60) {
        // --- THÊM 2 DÒNG NÀY ĐỂ HỖ TRỢ CẮM NÓNG (HOT-PLUG) ---
        // Quét lại bus 1-Wire mỗi 3 giây để tìm cảm biến mới cắm vào
        sensors.begin(); 
        sensors.setWaitForConversion(false);
        // ----------------------------------------------------
        loop_counter = 0; 
    }
    
    localRead.t1 = temp1; localRead.t2 = temp2; localRead.t3 = temp3;
    
    // =========================================================================
    // CHỐT CỐ ĐỊNH KWH: Đảm bảo không bao giờ bị đè số 0 khi mất kết nối PZEM
    // Bất chấp PZEM lỗi, hỏng, hay cúp điện, biến RAM của ESP32 là chân lý tuyệt đối.
    // =========================================================================
    localRead.ac_energy = esp_total_ac_energy;
    localRead.dc_energy = esp_total_dc_energy;
    
    // --- ĐÓNG GÓI VÀ TÍNH TẢI CPU ---
    if (xSemaphoreTake(dataMutex, (TickType_t) 100) == pdTRUE) { 
        sharedData = localRead; 
        xSemaphoreGive(dataMutex); 
    }
    
    long t_work = micros() - t_start_core0; 
    load_core0 = (t_work / 100000.0) * 100.0; 
    if (load_core0 > 100) load_core0 = 100;
    
    vTaskDelay(xDelay);
  }
}



// ==========================================
// HÀM RESET CHỈ SỐ PZEM VỀ 0
// ==========================================
void resetEnergyAC() {
    uint8_t resetCmd[] = {0x01, 0x42, 0x80, 0x11};
    SerialAC.write(resetCmd, 4);
}

void resetEnergyDC() {
    uint8_t resetCmd[] = {0x01, 0x42, 0x80, 0x11};
    SerialDC.write(resetCmd, 4);
}
// ==========================================

void processSerialCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  Serial.print(F("CMD >> ")); Serial.println(cmd);
  if (Blynk.connected()) {
      terminal.print("CMD >> "); terminal.println(cmd); terminal.flush();
  }

  auto reply = [](String msg) {
      Serial.println(msg);
      if (Blynk.connected()) { terminal.println(msg); }
  };
  if (cmd.equalsIgnoreCase("HELP")) {
    reply(F("=== DANH SACH LENH ==="));
    reply(F("NGAYCHOT:1 -> Chot so ngay 1 hang thang"));
    reply(F("GIOCHOT:5  -> Reset luc 5h sang"));
    reply(F("WIFI:TenWifi -> Cai Wifi"));
    reply(F("PASS:MatKhau -> Cai Pass"));
    reply(F("TOKEN:BlynkToken"));
    reply(F("SERVER:BlynkServer"));
    reply(F("SETSHUNT:100"));
    reply(F("RESETWIFI -> Xoa Wifi"));
    reply(F("REBOOT -> Khoi dong lai"));
    reply(F("RESETAC -> Xoa chi so dien AC")); // <-- BẠN THÊM DÒNG NÀY
    reply(F("RESETDC -> Xoa chi so dien DC")); // <-- BẠN THÊM DÒNG NÀY
  }
  else if (cmd.startsWith("WIFI:")) {
    String s = cmd.substring(5);
    preferences.begin("solar_cfg", false); preferences.putString("cssid", s); preferences.end();
    reply("-> DA LUU SSID: " + s);
    reply("-> Go 'PASS:xxx' de nhap Pass.");
  }
  else if (cmd.startsWith("PASS:")) {
    String p = cmd.substring(5);
    preferences.begin("solar_cfg", false); preferences.putString("cpass", p); preferences.end();
    reply("-> DA LUU PASS. Go 'REBOOT'.");
  }
  else if (cmd.startsWith("TOKEN:")) {
    String t = cmd.substring(6);
    preferences.begin("solar_cfg", false); preferences.putString("token", t); preferences.end();
    reply("-> DA LUU TOKEN.");
  }
  else if (cmd.startsWith("SERVER:")) {
    String s = cmd.substring(7);
    preferences.begin("solar_cfg", false); preferences.putString("server", s); preferences.end();
    reply("-> DA LUU SERVER.");
  }
  else if (cmd.startsWith("SETSHUNT:")) {
     int val = cmd.substring(9).toInt();
     uint16_t regVal = 0x0000;
     bool valid = true;
     if (val == 100) regVal = 0x0000;
     else if (val == 50) regVal = 0x0001;
     else if (val == 200) regVal = 0x0002;
     else if (val == 300) regVal = 0x0003;
     else valid = false;
     if (valid) {
         val_req_shunt = regVal;
         flag_req_setShunt = true;
         reply("-> DANG CAI SHUNT " + String(val) + "A...");
     } else {
         reply("-> LOI: Chi ho tro 50, 100, 200, 300");
     }
  }
  else if (cmd.equalsIgnoreCase("RESETWIFI")) {
    reply("-> DANG RESET WIFI...");
    WiFiManager wm; wm.resetSettings();
    preferences.begin("solar_cfg", false); preferences.putString("cssid", ""); preferences.putString("cpass", ""); preferences.end();
    reply("-> XONG! REBOOT SAU 2S.");
    delay(2000); ESP.restart();
  }
  else if (cmd.equalsIgnoreCase("REBOOT")) {
    reply("-> REBOOTING...");
    delay(1000); ESP.restart();
  }
  // --- LỆNH RESET PZEM ---
  else if (cmd.equalsIgnoreCase("RESETAC")) {
    reply("-> DANG RESET KWH AC VE 0...");
    // Đã xóa hàm resetEnergyAC() ở đây để chống đụng độ UART
    flag_reset_ac = true; 
    reply("-> XONG! DA RA LENH XOA AC.");
  }
  else if (cmd.equalsIgnoreCase("RESETDC")) {
    reply("-> DANG RESET KWH DC VE 0...");
    // Đã xóa hàm resetEnergyDC() ở đây để chống đụng độ UART
    flag_reset_dc = true; 
    reply("-> XONG! DA RA LENH XOA DC.");
  }
  // -----------------------


  else if (cmd.startsWith("NGAYCHOT:")) {
    int val = cmd.substring(9).toInt();
    if (val >= 1 && val <= 31) { // Cho phép đến 31 (nhưng nên đặt < 28 để an toàn cho tháng 2)
        energyBillingDate = val;
        // Lưu vào bộ nhớ ngay lập tức
        preferences.begin("solar_cfg", false); 
        preferences.putInt("bDate", val); 
        preferences.end();
        reply("-> OK! NGAY CHOT MOI: " + String(val));
    } else {
        reply("-> LOI: Ngay phai tu 1 den 31");
    }
  }
  else if (cmd.startsWith("GIOCHOT:")) {
    int val = cmd.substring(8).toInt();
    if (val >= 0 && val <= 23) {
        energyResetHour = val;
        // Lưu vào bộ nhớ ngay lập tức
        preferences.begin("solar_cfg", false); 
        preferences.putInt("rHour", val); 
        preferences.end();
        reply("-> OK! GIO CHOT MOI: " + String(val) + "h00");
    } else {
        reply("-> LOI: Gio phai tu 0 den 23");
    }
  }


  // --------------------------------
  else {
    reply(F("-> Lenh sai! Go 'HELP'."));
  }
 
  if (Blynk.connected()) terminal.flush();
}

// ==========================================
// MODULE MQTT (KẾT NỐI & GIAO TIẾP VPS)
// ==========================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg = "";
    for (int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }
    Serial.print("MQTT Nhan lenh tu VPS: ");
    Serial.println(msg);

    // Xử lý lệnh điều khiển Relay cơ bản
    msg.trim();
    if (msg.equalsIgnoreCase("RELAY_ON")) {
        manualRelayCmd = true;
        if (systemMode == MODE_MANUAL) setRelay(true);
    } 
    else if (msg.equalsIgnoreCase("RELAY_OFF")) {
        manualRelayCmd = false;
        if (systemMode == MODE_MANUAL) setRelay(false);
    }
}

void handleMQTT() {
    // Nếu khách hàng TẮT tính năng VPS -> Ngắt kết nối và Thoát
    if (!enableMQTT) {
        if (mqttClient.connected()) mqttClient.disconnect();
        return;
    }
    if (WiFi.status() != WL_CONNECTED) return;
    
    if (!mqttClient.connected()) {
        if (millis() - t_mqtt_reconnect > 5000) { // Thử kết nối lại mỗi 5 giây
            t_mqtt_reconnect = millis();
            if (mqttClient.connect(device_id, mqtt_user, mqtt_pass)) {
                // Đăng ký nhận lệnh từ Web/App
                String subTopic = String("cmd/") + device_id;
                mqttClient.subscribe(subTopic.c_str());
            }
        }
    } else {
        mqttClient.loop(); // Duy trì kết nối ngầm
    }
}

void pushDataToMQTT() {
    // Nếu đang tắt hoặc chưa kết nối thì không đẩy
    if (!enableMQTT || !mqttClient.connected()) return; 
    
    // --- KHỐI COPY DỮ LIỆU AN TOÀN TUYỆT ĐỐI CHO MQTT ---
    DataBox mqttData;
    if (xSemaphoreTake(dataMutex, (TickType_t) 10) == pdTRUE) { 
        mqttData = sharedData; 
        xSemaphoreGive(dataMutex); 
    } else {
        mqttData = currentData; // Dự phòng nếu Mutex đang bận
    }
    // -----------------------------------------------------

    char jsonPayload[512];
    float dc_kwh = mqttData.dc_energy / 1000.0f; // Thay bằng mqttData
    
    // Đóng gói JSON theo chuẩn API (Sửa toàn bộ currentData thành mqttData)
    snprintf(jsonPayload, sizeof(jsonPayload),
        "{"
        "\"ac\":{\"v\":%.1f, \"a\":%.2f, \"w\":%.0f, \"kwh\":%.2f},"
        "\"dc\":{\"v\":%.1f, \"a\":%.2f, \"w\":%.0f, \"kwh\":%.2f},"
        "\"temp\":{\"t1\":%.1f, \"t2\":%.1f},"
        "\"sys\":{\"mode\":%d, \"relay\":%d}"
        "}",
        mqttData.ac_volt, mqttData.ac_amp, mqttData.ac_power, mqttData.ac_energy,
        mqttData.dc_volt, mqttData.dc_amp, mqttData.dc_power, dc_kwh,
        mqttData.t1, mqttData.t2,
        systemMode, relayState ? 1 : 0
    );

    // Gửi lên Topic Telemetry
    String pubTopic = String("telemetry/") + device_id;
    mqttClient.publish(pubTopic.c_str(), jsonPayload);
}
// ======================================================================================
// TASK XỬ LÝ MẠNG ĐỘC LẬP (CHỐNG LAG MÀN HÌNH)
// ======================================================================================
// ======================================================================================
// TASK XỬ LÝ MẠNG ĐỘC LẬP (CHỐNG LAG MÀN HÌNH VÀ CRASH)
// ======================================================================================
void TaskNetworkCode(void * parameter) {
  const TickType_t xDelay = 15 / portTICK_PERIOD_MS; // Nhường CPU 15ms
  
  for(;;) {
    // 1. AN TOÀN OTA: Ngủ đông hoàn toàn nếu đang nạp Firmware để chống giật băng thông
    if (isWebOTA || currentScreen == SCREEN_OTA) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
        continue; 
    }

    unsigned long currentMillis = millis();
    bool isUserBusy = (currentScreen != SCREEN_MAIN) || (currentMillis - t_last_user_action < 30000);

   // 2. KHỐI LOGIC TỰ KẾT NỐI (Đã tách rời điều kiện custom_ssid)
    if (!isUserBusy && !isWifiScanning && currentScreen != SCREEN_WIFI_CONNECTING && next_screen_target == -1) {
        static unsigned long t_wifi_check = 0;
        if (currentMillis - t_wifi_check >= 60000) {
            t_wifi_check = currentMillis;
            
            if (WiFi.status() != WL_CONNECTED) {
                if (strlen(custom_ssid) > 0) { WiFi.reconnect(); }
                is_blynk_ip_resolved = false;
            }
            else if (Blynk.connected() == false) {
                // Đã tháo gông, hệ thống sẽ tự động cứu hộ Blynk khi có WiFi
                if (!is_blynk_ip_resolved || (currentMillis - t_last_dns_resolve > 300000)) { 
                    is_blynk_ip_resolved = WiFi.hostByName(blynk_server, blynk_cached_ip);
                    t_last_dns_resolve = currentMillis;
                }
                if (is_blynk_ip_resolved) {
                    WiFiClient testClient;
                    if (testClient.connect(blynk_cached_ip, 8080, 200)) {
                        testClient.stop();    
                        Blynk.connect(1000);  
                    }
                }
            }
        }
    }

    // 3. XỬ LÝ DUY TRÌ & GỬI DỮ LIỆU
    if (WiFi.status() == WL_CONNECTED) {
      // ĐỂ TRẦN HÀM NÀY, tuyệt đối không bọc trong lệnh if(Blynk.connected())
      Blynk.run(); 
      
      if (Blynk.connected()) {
          if (flag_sync_relay) { syncBlynkRelayState(relayState); flag_sync_relay = false; }
          if (flag_sync_settings) { syncSettingsToBlynk(); flag_sync_settings = false; }
      }
      
      handleMQTT();
      
      // Kỹ thuật lệch pha đẩy dữ liệu
      if (currentMillis - t_blynk_update >= 1000) { 
          t_blynk_update = currentMillis; 
          updateBlynk(); 
      }
      
      if (currentMillis - t_mqtt_push >= 1000 && (currentMillis - t_blynk_update >= 500)) {
          t_mqtt_push = currentMillis;
          pushDataToMQTT();
      }
    }
    
    // Bắt buộc phải có lệnh này để Task không nuốt trọn CPU
    vTaskDelay(xDelay); 
  }
}
void setup() {
  Serial.begin(115200); Serial.setTimeout(50); SerialAC.setTimeout(200); SerialDC.setTimeout(200);
  snprintf(device_id, sizeof(device_id), "SOLAR_%s", getMacSuffix());
  dataMutex = xSemaphoreCreateMutex();
  pinMode(RELAY_PIN, OUTPUT); digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY2_PIN, OUTPUT); digitalWrite(RELAY2_PIN, LOW);
  pinMode(RELAY3_PIN, OUTPUT); digitalWrite(RELAY3_PIN, LOW);
  // --- THÊM 2 DÒNG NÀY ĐỂ GIỮ CHÂN VÀ THIẾT LẬP AN TOÀN ---
  pinMode(TFT_BL_PIN, OUTPUT); digitalWrite(TFT_BL_PIN, HIGH); // Bật sáng đèn nền mặc định
  pinMode(RELAY4_PIN, OUTPUT); digitalWrite(RELAY4_PIN, LOW);  // Giữ Relay 4 luôn tắt
  // ---------------------------------------------------------
  pinMode(GRID_DETECT_PIN, INPUT_PULLUP);
  pinMode(BTN_UP_PIN, INPUT_PULLUP); pinMode(BTN_MENU_PIN, INPUT_PULLUP); pinMode(BTN_DOWN_PIN, INPUT_PULLUP);

  tft.init(); tft.setRotation(1); tft.invertDisplay(0); tft.fillScreen(C_BLACK);
  tft.setTextFont(4); tft.setTextColor(C_POWER, C_BLACK); tft.setTextDatum(MC_DATUM);
  tft.drawString("SMART SOLAR METER", 160, 60);
  tft.setTextFont(2); tft.setTextColor(TFT_WHITE, C_BLACK);
  tft.drawString("Developed by Thien An", 160, 95);
  
  loadSettings();
  
  preferences.begin("solar_cfg", false); 
  relayState = preferences.getBool("lastRelay", false); 
  preferences.end();
  
  if (forceOff && systemMode == MODE_AUTO) {
      relayState = false;
  }
  digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
  
  if (systemMode == MODE_MANUAL) manualRelayCmd = relayState;
  
  loadEnergyLatch(); setupTime();

  WiFiManager wm;
  String apName = String("SolarMeter_") + getMacSuffix();
  WiFiManagerParameter ct("token", "Blynk Token", blynk_token, 34); 
  WiFiManagerParameter cs("server", "Blynk Server", blynk_server, 40);
  wm.addParameter(&ct); wm.addParameter(&cs);
  wm.setAPCallback(configModeCallback); wm.setSaveConfigCallback(saveConfigCallback);
  wm.setDebugOutput(false);
  wm.setConfigPortalBlocking(false); wm.setConfigPortalTimeout(120);
  WiFi.setAutoReconnect(true); WiFi.persistent(true);

  if (strlen(custom_ssid) > 0) { WiFi.mode(WIFI_STA); WiFi.begin(custom_ssid, custom_pass); }
  bool wifiResult = wm.autoConnect(apName.c_str());

  if (!wifiResult) {
      unsigned long startConfigTime = millis();
      bool userExit = false;
      while(WiFi.status() != WL_CONNECTED && millis() - startConfigTime < 120000) {
           wm.process();
           if (digitalRead(BTN_MENU_PIN) == LOW) {
               delay(50);
               if (digitalRead(BTN_MENU_PIN) == LOW) {
                   userExit = true; wm.stopConfigPortal(); tft.fillScreen(C_BLACK);
                   tft.setTextColor(TFT_RED, C_BLACK); tft.drawString("User Exit -> Offline Mode", 160, 120); delay(1000); break;
               }
           }
           delay(10);
      }
      if (WiFi.status() != WL_CONNECTED) {
          tft.fillScreen(C_BLACK); tft.setTextColor(TFT_RED, C_BLACK);
          if(!userExit) tft.drawString("Timeout -> Offline Mode", 160, 120); delay(1000);
      } else {
          tft.fillScreen(C_BLACK); tft.setTextColor(TFT_GREEN, C_BLACK); tft.drawString("WiFi Connected!", 160, 120);
          if (shouldSaveConfig) {
              strcpy(blynk_token, ct.getValue()); strcpy(blynk_server, cs.getValue());
              preferences.begin("solar_cfg", false); preferences.putString("token", blynk_token); preferences.putString("server", blynk_server); preferences.end();
          }
          delay(1000);
      }
  } else {
      tft.setTextColor(TFT_GREEN, C_BLACK); tft.drawString("WiFi Connected!", 160, 200); delay(1000);
  }

  if (String(blynk_token).length() > 0) { Blynk.config(blynk_token, blynk_server, 8080); if (WiFi.status() == WL_CONNECTED) Blynk.connect(); }
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512); // [MỞ RỘNG BỘ ĐỆM] Nới kích thước gói tin lên 512 byte để chứa trọn JSON
  setupWebOTA(); 
  // --- CẤU HÌNH GIAO DIỆN KHI NẠP TỪ ARDUINO IDE (PRO VERSION) ---
  ArduinoOTA.onStart([]() {
      // [FIX 1] Bật cờ ngưng đọc phần cứng thay vì Suspend Task
      isSystemUpdating = true;

      // --- FIX: LƯU GẤP DỮ LIỆU TRƯỚC KHI UPDATE ---
      saveEnergyData(); // <-- GỌI HÀM LƯU GOM CỤM
      delay(50);
      isWebOTA = false;
      currentScreen = SCREEN_OTA; // Khóa loop() không cho vẽ đè màn hình chính
      tft.fillScreen(C_BLACK);
// ... (Phần vẽ tft bên dưới giữ nguyên) ...
      
      // 1. Header sắc nét
      tft.fillRect(0, 0, 320, 30, 0x18E3);
      tft.setTextFont(2); 
      tft.setTextColor(TFT_WHITE, 0x18E3);
      tft.setTextDatum(MC_DATUM);
      tft.drawString("SYSTEM FIRMWARE UPDATE", 160, 15);
      
      // 2. Thông tin IP
      tft.setTextDatum(TL_DATUM);
      tft.setTextColor(TFT_CYAN, C_BLACK);
      tft.drawString("Nguon nap: Arduino IDE (LAN)", 15, 45);
      tft.setTextColor(TFT_SILVER, C_BLACK);
      tft.drawString("Dia chi IP: " + WiFi.localIP().toString(), 15, 65);
      
      // 3. Nền xám của thanh Progress
      tft.drawRect(28, 128, 264, 24, TFT_SILVER);
      tft.fillRect(30, 130, 260, 20, 0x2124);
      
      // 4. Trạng thái khởi tạo
      tft.setTextColor(TFT_YELLOW, C_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(280); 
      tft.drawString("DANG CHUAN BI DATA...", 160, 175);
      tft.setTextPadding(0);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      // [BẢO VỆ CRASH CHIA CHO 0]
      if (total == 0 || total == UPDATE_SIZE_UNKNOWN) total = 1; 
      int percent = (progress * 100) / total;
      if (percent > 100) percent = 100;
      
      static int lastPercent = -1;
      static int lastBarWidth = 0;

      // [AUTO RESET] Dọn dẹp rác bộ nhớ nếu đây là chu kỳ nạp mới
      if (progress == 0) {
          lastPercent = -1;
          lastBarWidth = 0;
      }
      
      if (percent != lastPercent) {
          lastPercent = percent;
          
          // [RENDER TỐI ƯU SIÊU MƯỢT] Chỉ vẽ thêm phần chênh lệch
          int barWidth = (percent * 260) / 100;
          if (barWidth > lastBarWidth) {
              tft.fillRect(30 + lastBarWidth, 130, barWidth - lastBarWidth, 20, TFT_GREEN);
              lastBarWidth = barWidth;
          }
          
          // In phần trăm (%)
          tft.setTextFont(4);
          tft.setTextColor(TFT_WHITE, C_BLACK);
          tft.setTextDatum(MC_DATUM);
          tft.setTextPadding(100); 
          char pctBuf[10]; 
          snprintf(pctBuf, sizeof(pctBuf), "%d%%", percent);
          tft.drawString(pctBuf, 160, 105); 
          
          // In lưu lượng (KB)
          tft.setTextFont(2);
          tft.setTextColor(TFT_SILVER, C_BLACK);
          tft.setTextPadding(280); 
          char infoBuf[50];
          snprintf(infoBuf, sizeof(infoBuf), "Da tai: %u KB / %u KB", progress / 1024, total / 1024);
          tft.drawString(infoBuf, 160, 175);
          
          tft.setTextPadding(0); 
      }
  });
  
  ArduinoOTA.onEnd([]() {
      // [BÙ 100%] Ép hiển thị hoàn hảo ở giây cuối cùng
      tft.fillRect(30, 130, 260, 20, TFT_GREEN);
      tft.setTextFont(4);
      tft.setTextColor(TFT_WHITE, C_BLACK);
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(100);
      tft.drawString("100%", 160, 105);

      // Box thông báo Thành công
      tft.fillRoundRect(40, 195, 240, 30, 4, TFT_GREEN);
      tft.setTextFont(2);
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(0);
      tft.drawString("THANH CONG! DANG REBOOT...", 160, 210);
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
      // Box thông báo Lỗi
      tft.fillRoundRect(40, 195, 240, 30, 4, TFT_RED);
      tft.setTextFont(2);
      tft.setTextColor(TFT_WHITE, TFT_RED);
      tft.setTextDatum(MC_DATUM);
      tft.setTextPadding(0);
      
      String errStr = "LOI: ";
      if (error == OTA_AUTH_ERROR) errStr += "Sai Mat Khau";
      else if (error == OTA_BEGIN_ERROR) errStr += "Loi Khoi Tao";
      else if (error == OTA_CONNECT_ERROR) errStr += "Mat Ket Noi";
      else if (error == OTA_RECEIVE_ERROR) errStr += "Loi Nhan Data";
      else if (error == OTA_END_ERROR) errStr += "Loi Ket Thuc";
      
      tft.drawString(errStr, 160, 210);
      
      // Giữ màn hình 3 giây để đọc lỗi, rồi giải cứu mạch
      delay(3000);
      ESP.restart();
  });
  ArduinoOTA.setHostname(device_id);
  
  ArduinoOTA.begin();
  sensors.begin(); sensors.setWaitForConversion(false);
  SerialDC.begin(9600, SERIAL_8N1, PZEM_DC_RX_PIN, PZEM_DC_TX_PIN); nodeDC.begin(1, SerialDC);
  SerialAC.begin(9600, SERIAL_8N1, PZEM_AC_RX_PIN, PZEM_AC_TX_PIN); nodeAC.begin(1, SerialAC);
  esp_task_wdt_init(WDT_TIMEOUT, true); 
  esp_task_wdt_add(NULL); 
  xTaskCreatePinnedToCore(TaskSensorsCode, "SensorsTask", 8192, NULL, 1, &Task0, 0);
  // <-- THÊM DÒNG NÀY ĐỂ KÍCH HOẠT TASK MẠNG -->
  xTaskCreatePinnedToCore(TaskNetworkCode, "NetworkTask", 8192, NULL, 1, &TaskNetwork, 1);
  drawInterface();
}


void loop() {
  esp_task_wdt_reset();
  checkButtons();
  ArduinoOTA.handle();
  server.handleClient();
  
  unsigned long currentMillis = millis();
  static String serialBuffer = "";
  while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
          processSerialCommand(serialBuffer);
          serialBuffer = "";
      } else if (c != '\r') {
          serialBuffer += c;
      }
  }

  if (isWifiScanning) {
      int n = WiFi.scanComplete();
      if (n >= 0) {
          wifiNetworksFound = n; isWifiScanning = false;
          if (n > 0) currentScreen = SCREEN_WIFI_SCAN;
          else { currentScreen = SCREEN_MENU; } 
          menuRedraw = true; menuNeedsUpdate = true;
      } else if (n == WIFI_SCAN_FAILED) {
          // Bắt lỗi để giải cứu hệ thống nếu quét thất bại
          isWifiScanning = false; 
          currentScreen = SCREEN_MENU;
          menuRedraw = true; menuNeedsUpdate = true;
      }
  }

  if (xSemaphoreTake(dataMutex, (TickType_t) 2) == pdTRUE) { currentData = sharedData; xSemaphoreGive(dataMutex); }
  static unsigned long t_energy_check = 0; if (currentMillis - t_energy_check >= 1000) { checkEnergyReset(); t_energy_check = currentMillis; }
  
  // --- CƠ CHẾ AUTO-SAVE BẢO VỆ DỮ LIỆU ĐIỆN NĂNG MỖI 15 PHÚT ---
  static unsigned long t_auto_save = 0;
  if (currentMillis - t_auto_save >= 900000) { 
      t_auto_save = currentMillis;
      
      bool shouldSave = false;
      // SỬA: Dùng currentData thay vì esp_total_ac_energy
      if (fabs(currentData.ac_energy - last_saved_ac_energy) >= 0.1) { 
          shouldSave = true;
      }
      // SỬA: Dùng currentData thay vì esp_total_dc_energy
      if (fabs(currentData.dc_energy - last_saved_dc_energy) >= 100.0) { 
          shouldSave = true;
      }

      if (shouldSave) {
          saveEnergyData();
          // SỬA: Cập nhật lại mốc lưu bằng currentData
          last_saved_ac_energy = currentData.ac_energy; 
          last_saved_dc_energy = currentData.dc_energy;
          Serial.println(F("[SYSTEM] Da Auto-Save dien nang vao Flash."));
      }
  }

  // [UI REFRESH RATE] 20Hz
  static unsigned long t_ui_update = 0;
  if (currentMillis - t_ui_update >= 50) {
      unsigned long t_start = micros();
      updateRelay(); updateACRelayProtection(); updateRelay3Logic();
      
      if (currentScreen == SCREEN_MAIN) updateScreenSmart(); 
      else if (currentScreen == SCREEN_MENU) drawMenu();
      else if (currentScreen == SCREEN_WIFI_SCAN) drawWifiScan();
      else if (currentScreen == SCREEN_WIFI_PASS) drawWifiKeyboard();
      else if (currentScreen == SCREEN_WIFI_CONNECTING) drawWifiConnecting();
      else if (currentScreen == SCREEN_ABOUT) drawAboutScreen();
      else if (currentScreen == SCREEN_SENSOR_CHECK) drawSensorCheckScreen();
      else if (currentScreen == SCREEN_VPS_INPUT) drawVPSKeyboard();
      
      t_ui_update = currentMillis; long t_work = micros() - t_start; load_core1 = (t_work / 200000.0) * 100.0;
  }
  
  // Chuyển màn hình an toàn
  if (next_screen_target != -1) {
      if (currentMillis - t_state_delay >= 1000) {
          currentScreen = next_screen_target;
          next_screen_target = -1;
          if (currentScreen == SCREEN_MAIN) drawInterface();
          else { menuRedraw = true; menuNeedsUpdate = true; }
      }
  }
  else if (currentScreen == SCREEN_WIFI_CONNECTING) {
      if (WiFi.status() == WL_CONNECTED) {
          tft.fillScreen(C_BLACK); tft.setTextColor(TFT_GREEN, C_BLACK); 
          tft.setTextDatum(MC_DATUM); tft.setTextFont(4);
          tft.drawString("KET NOI THANH CONG!", 160, 120);
          
          is_blynk_ip_resolved = false;
          t_state_delay = currentMillis;
          next_screen_target = SCREEN_MAIN;
      } else if (currentMillis - t_wifi_connect_start > 15000) {
          tft.fillScreen(C_BLACK); tft.setTextColor(TFT_RED, C_BLACK); 
          tft.setTextDatum(MC_DATUM); tft.setTextFont(4);
          tft.drawString("KET NOI THAT BAI!", 160, 120);
          WiFi.disconnect(); 
          
          t_state_delay = currentMillis;
          next_screen_target = SCREEN_MENU;
      }
  }
  vTaskDelay(pdMS_TO_TICKS(1)); // [FIX 4] Nhường CPU Core 1 chuẩn FreeRTOS
}





