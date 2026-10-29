// ============================================================
// ПРИЁМНИК (receiver 1.0), Heltec Wireless Stick Lite V3 (ESP32-S3 + SX1262)
// Рабочая прошивка приёмника гидрораспределителя на 8 секций (16 пропорциональных катушек).
// Протокол HydroLink v1: полное описание в remote_config.md §52 (пульт pult 1.0 реализует то же самое).
//
//  Ядро 0: радио (приём управления, ответ телеметрией, настройки, привязка, Wi-Fi команды).
//  Ядро 1: loop (управление выходами, реле, защиты, мигалка, Wi-Fi страница), отдельная задача опроса АЦП тока.
//
//  Выходы включаются только при: есть связь с пультом, нет аварии (E-Stop пульта, кнопка приёмника, сбой), реле замкнуло
//  (проверка обратной связи), рычаги побывали в нуле. Любая авария: PCA9685 в ноль, OE выключен, EN снят, реле разомкнуто.
//  Сбой реле (залипло / не замкнулось) и потеря PCA9685 защёлкиваются до перезагрузки приёмника.
//
//  ТОК: пока ADS1115 (стенд): пик за скользящее окно 40 мс, обрыв и режим CURRENT выключены (быстрого АЦП нет).
//  С MCP3208 слой измерения заменяется (функция currentBackend), обрыв и CURRENT включаются сами.
//  Калибровка IS лежит в NVS "calib" (пишет вкладка Test этой прошивки или hwtest), формат прежний.
//
//  Wi-Fi: по команде пульта (Receiver -> Wi-Fi): Phone (точка RX-xxxx, страница Update/Test) или GitHub (обновление).
//  Привязка: долгое нажатие USER (3 с) на работающем приёмнике; Bind ID по умолчанию как в пульте.
//
//  Arduino IDE: плата "Heltec Wireless Stick Lite (V3)", библиотека RadioLib 7.1.2.
//  12 В и USB одновременно НЕ подключать.
//
//  Журнал версий:
//  1.0 - первая рабочая версия: радио, протокол, ШИМ и защиты, реле и сбои, мигалка, Wi-Fi Update/Test, GitHub
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <mbedtls/sha256.h>
#include <math.h>
#include <stdarg.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define PROJECT_NAME  "receiver"
#define FW_VERSION    "1.0"
static const char FW_TAG[] __attribute__((used)) = "FWTAG:" PROJECT_NAME ":" FW_VERSION ";";

#define GH_REPO       "potska9992-hub/forester"
#define GH_TAG        "receiver-latest"
#define AP_PASS       "88888888"
#define AP_ADDR       "192.168.4.1"

// ---- Пины (remote_config.md §47) ----
#define PIN_BUS0_SDA  2      // Wire : PCA9685
#define PIN_BUS0_SCL  3
#define PIN_BUS1_SDA  4      // Wire1: две ADS1115
#define PIN_BUS1_SCL  5
#define PIN_VBUS      6      // напряжение бортсети (делитель 100к/10к)
#define PIN_VFB       7      // напряжение после реле
#define PIN_RED       15     // мигалка красный (HIGH = включено)
#define PIN_GREEN     16     // мигалка зелёный
#define PIN_BUZ       17     // зуммер
#define PIN_RELAY     21     // реле питания драйверов: LOW = включено, HIGH = выключено
#define PIN_OE        33     // PCA9685 OE: LOW = выходы разрешены
#define PIN_ESTOP     34     // аварийная кнопка NC: HIGH = нажата или обрыв
#define PIN_EN        38     // EN всех драйверов: HIGH = разрешено
#define PIN_LED       35     // белый светодиод платы
#define PIN_USER      0      // кнопка USER, LOW = нажата
#define PIN_NSS       8
#define PIN_SCK       9
#define PIN_MOSI      10
#define PIN_MISO      11
#define PIN_RST       12
#define PIN_BUSY      13
#define PIN_DIO1      14

#define I2C_HZ        100000
#define I2C1_HZ       400000
#define PCA_ADDR      0x40
#define ADS_ADDR0     0x48
#define ADS_ADDR1     0x49

#define K_BUS_DEF     11.09f
#define K_FB_DEF      10.96f
#define V_DEADBAND    0.8f

// ---- Радио (одинаково с пультом) ----
#define HL_PROTO        1
#define RF_FREQ_MHZ     868.0f
#define RF_BW_KHZ       250.0f
#define RF_SF           7
#define RF_CR           5
#define RF_SYNC         0x12
#define RF_PREAMBLE     8
#define RF_TCXO_V       1.8f
#define DEF_BIND_ID     0xA5C3E17BUL
#define HL_FACTORY_KEY  0xA55A

enum { HLT_CONTROL = 1, HLT_TELEM, HLT_CFG, HLT_ACK, HLT_BATCH, HLT_NET, HLT_INFO, HLT_BIND, HLT_BINDOK };
enum { HLF_ESTOP = 1, HLF_REQT = 2, HLF_LEVER = 4, HLF_BOOT = 8 };
enum { HLO_SET, HLO_GET, HLO_BATCH, HLO_CMD, HLO_NET };
enum { HLC_PROTO = 1, HLC_CHMASK, HLC_CHTYPES, HLC_TXPWR, HLC_FACTORY = 0x10, HLC_WIFI_PHONE = 0x20, HLC_WIFI_GH, HLC_WIFI_OFF, HLC_WIFI_YES };
enum { HLW_OFF = 0, HLW_PH_WAIT = 2, HLW_PH_CONN, HLW_PH_TEST, HLW_PH_UP, HLW_PH_DONE,
       HLW_GH_CONN = 10, HLW_GH_CHECK, HLW_GH_ASK, HLW_GH_DL, HLW_GH_VERIFY, HLW_GH_DONE, HLW_ERR = 20 };
enum { WE_NONET = 1, WE_PASS, WE_GHDOWN, WE_NOREL, WE_DLFAIL, WE_SUM, WE_WRONG, WE_UPD, WE_BUSY };

// ---- Параметры (порядок и пределы те же, что в пульте) ----
enum { PS_SOL, PS_CH, PS_GLOB };
enum { RXP_LIMIT, RXP_DUTY, RXP_MAXD, RXP_RAMPUP, RXP_RAMPDN, RXP_CHEN, RXP_OVLDLY, RXP_LINKTMO, RXP_PWMFREQ, RXP_DITHAMP, RXP_DITHFRQ, RXP_PWR, RXP_MODE, RXP_STARTI, RXP_MAXI, NUM_RXP };
struct RxParamDef { uint8_t scope; int16_t mn, mx, def; };
static const RxParamDef RXP[NUM_RXP] = {
  { PS_SOL,     5,  100,  20 },   // LIMIT, 0.1 А
  { PS_SOL,     0, 1000,  50 },   // DUTY: стартовый ШИМ, 0.1 %
  { PS_SOL,   100, 1000, 600 },   // MAX DUTY, 0.1 %
  { PS_CH,      0, 2000, 200 },   // RAMP UP, мс
  { PS_CH,      0, 2000, 200 },   // RAMP DN, мс
  { PS_CH,      0,    1,   1 },   // ENABLE
  { PS_GLOB,   20, 1000, 150 },   // OVL DELAY, мс
  { PS_GLOB,  200, 2000, 500 },   // LINK TMO, мс
  { PS_GLOB,   50,  500, 100 },   // PWM FREQ, Гц
  { PS_GLOB,    0,   20,   0 },   // DITH AMP, %
  { PS_GLOB,   10,   60,  30 },   // DITH FREQ, Гц
  { PS_GLOB,    2,   22,  10 },   // RX POWER, дБм
  { PS_GLOB,    0,    1,   0 },   // CONTROL: 0 PWM, 1 CURRENT
  { PS_SOL,     0,  100,   5 },   // START I, 0.1 А
  { PS_SOL,     5,  100,  15 }    // MAX I, 0.1 А
};

#define NMOD          8
#define NUM_CH        8
#define NET_MAX       3

// ---- Измерение тока ----
#define ADS_LSB       0.000125f
#define ADS_CONV_US   1400
#define IDLE_SCAN_MS  25
#define BK_MS         5
#define BK_N          8             // окно 8 x 5 мс = 40 мс
#define K_DEF         1.70f         // А на вольт по умолчанию, пока модуль не откалиброван
#define I_LIM_DEF     2.0f
#define OVL_WINDOWS   3
#define ZERO_MS       1200
#define R_COIL_NOM    5.6f          // номинальное сопротивление катушки, Ом (для начального приближения режима CURRENT)

struct Bk { uint32_t ep; float mx; float sum; uint16_t n; };
static Bk bk[NMOD][BK_N];
struct SamplerState {
  int8_t   cur[2]     = { -1, -1 };
  int64_t  startUs[2] = { 0, 0 };
  uint8_t  rr[2]      = { 0, 0 };
  int64_t  idleAt[2]  = { 0, 0 };
  uint32_t cnt[2]     = { 0, 0 };
  uint32_t errRun[2]  = { 0, 0 };
  int64_t  secAt = 0, pubAt = 0;
};

// ---- Журнал (для вкладки Test) ----
#define LOG_SZ        4096
static char     logBuf[LOG_SZ];
static uint32_t logTotal = 0;
static portMUX_TYPE shMux = portMUX_INITIALIZER_UNLOCKED;

struct CtlShared { int8_t v[NUM_CH]; uint32_t rxAt; uint32_t nzAt; bool estop; bool lever; bool have; };
static CtlShared ctlSh = { { 0 }, 0, 0, false, false, false };
struct TelShared { uint8_t cur[NUM_CH]; int8_t pwm[NUM_CH]; uint8_t mLim, mOvl, mOpen, side; uint8_t kill, reason; uint8_t vrx; };
static TelShared telSh = { { 0 }, { 0 }, 0, 0, 0, 0, 1, 3, 0 };
static void logRaw(const char* s) { while (*s) { logBuf[logTotal % LOG_SZ] = *s++; logTotal++; } }
static void plog(const char* fmt, ...) {
  char msg[200];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  unsigned long s = millis() / 1000UL;
  char line[240];
  snprintf(line, sizeof(line), "[%02lu:%02lu] %s\n", (s / 60UL) % 100UL, s % 60UL, msg);
  logRaw(line);
  if (Serial) Serial.print(line);
}

// ---- Общие данные между ядрами ----
static volatile uint8_t wifiSt = HLW_OFF, wifiArg = 0;     // состояние Wi-Fi, его параметр (в телеметрию)
static volatile uint8_t wifiReq = 0;                        // команда радио -> loop: 1 phone, 2 github, 3 off, 4 yes
static volatile bool    btnLatched = false;                 // аварийная кнопка приёмника (держит до нулевого пакета после отпускания)
static volatile bool    relayFault = false;                 // сбой реле / PCA (защёлка до перезагрузки)
static volatile bool    testOK = false;                     // вкладка Test подтверждена
static volatile bool    bindMode = false;
static volatile bool    pwrDirty = false;                   // радио: мощность изменилась
static volatile uint32_t bindOkUntil = 0;

// ---- Настройки приёмника: RAM, сохранённые (NVS) ----
static uint16_t ramP[NUM_RXP][NUM_CH][2];
static uint16_t nvsP[NUM_RXP][NUM_CH][2];
static volatile bool cfgSavePending = false;
static uint8_t  bootCount = 0;
static uint16_t chMaskPult = 0xFF, chTypesPult = 0, pultTxPwr = 0;
static uint32_t bindId = DEF_BIND_ID;

static void cfgDefaults(uint16_t a[NUM_RXP][NUM_CH][2]) {
  memset(a, 0, sizeof(uint16_t) * NUM_RXP * NUM_CH * 2);
  for (int p = 0; p < NUM_RXP; p++) {
    int nc = (RXP[p].scope == PS_GLOB) ? 1 : NUM_CH, nd = (RXP[p].scope == PS_SOL) ? 2 : 1;
    for (int c = 0; c < nc; c++) for (int d = 0; d < nd; d++) a[p][c][d] = (uint16_t)RXP[p].def;
  }
}

// привести (канал, сторона) к виду, в котором параметр хранится
static void paramNorm(int p, int& ch, int& dir) {
  if (RXP[p].scope == PS_GLOB) { ch = 0; dir = 0; }
  else if (RXP[p].scope == PS_CH) dir = 0;
}

static void cfgFix(uint16_t a[NUM_RXP][NUM_CH][2]) {          // значения вне пределов -> по умолчанию
  for (int p = 0; p < NUM_RXP; p++) {
    int nc = (RXP[p].scope == PS_GLOB) ? 1 : NUM_CH, nd = (RXP[p].scope == PS_SOL) ? 2 : 1;
    for (int c = 0; c < nc; c++) for (int d = 0; d < nd; d++)
      if ((int)a[p][c][d] < RXP[p].mn || (int)a[p][c][d] > RXP[p].mx) a[p][c][d] = (uint16_t)RXP[p].def;
  }
}

// CRC8 сохранённой конфигурации: poly 0x07, init 0, значения по порядку параметров, каждое младшим байтом первым
static uint8_t cfgCrc8() {
  uint8_t c = 0;
  for (int p = 0; p < NUM_RXP; p++) {
    int nc = (RXP[p].scope == PS_GLOB) ? 1 : NUM_CH, nd = (RXP[p].scope == PS_SOL) ? 2 : 1;
    for (int ch = 0; ch < nc; ch++)
      for (int d = 0; d < nd; d++) {
        uint16_t v = nvsP[p][ch][d];
        for (int k = 0; k < 2; k++) {
          c ^= (uint8_t)(k ? (v >> 8) : (v & 0xFF));
          for (int j = 0; j < 8; j++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
        }
      }
  }
  return c;
}

static Preferences prefs;                 // namespace "calib" (открыт всё время, как в hwtest)
static void cfgLoad() {
  Preferences q;
  cfgDefaults(nvsP);
  q.begin("rxcfg", false);
  uint16_t tmp[NUM_RXP][NUM_CH][2];
  if (q.getBytes("p", tmp, sizeof(tmp)) == sizeof(tmp)) { memcpy(nvsP, tmp, sizeof(nvsP)); cfgFix(nvsP); }
  bootCount = (uint8_t)(q.getUChar("boots", 0) + 1);
  q.putUChar("boots", bootCount);
  q.end();
  memcpy(ramP, nvsP, sizeof(ramP));
}

static void cfgSave() {
  Preferences q;
  q.begin("rxcfg", false);
  q.putBytes("p", nvsP, sizeof(nvsP));
  q.putUChar("boots", bootCount);
  q.end();
}

static void bindLoad() {
  Preferences q;
  q.begin("bind", true);
  bindId = q.getUInt("id", DEF_BIND_ID);
  q.end();
}
static void bindSave(uint32_t id) {
  Preferences q;
  q.begin("bind", false);
  q.putUInt("id", id);
  q.end();
}

static inline uint16_t pairId() {
  uint32_t b = bindId;
  uint16_t p = (uint16_t)(b ^ (b >> 16));
  if (p == 0 || p == 0xFFFF) p ^= 0x5A5A;
  return p;
}

// ============================================================
// Состояние силовой части
// ============================================================
static WebServer server(80);

static bool  oeOn = false, enOn = false;
static float pwmFreq = 100.0f;
static float oscHz = 25000000.0f;          // реальная частота генератора PCA9685 (калибровка fcal, NVS "calib")
static int   pcaPre = 60;
static bool  pcaOk = false, pcaRestartPend = false;
static unsigned long pcaRestartAt = 0;
static uint8_t pcaFail = 0;
static float pwmDuty[16];                  // фактическая скважность каналов PCA, %

static volatile uint8_t faultCode = 0;     // 0 нет, 1 реле залипло, 2 реле не замкнулось / предохранитель, 3 потеряна PCA9685
static float kBus = K_BUS_DEF, kFb = K_FB_DEF;
static float emaB = 0, emaF = 0, vBus = 0, vFb = 0;
static bool  adcInit = false;
static unsigned long adcAt = 0;

enum { RS_OFF, RS_CLOSING, RS_ON, RS_OPENING };
static uint8_t relayState = RS_OFF;
static unsigned long relayChangeAt = 0, relayOffAt = 0, relayBadAt = 0;
static char relayMsg[100] = "-";

static bool estopRaw = false, estopBtn = false;   // аварийная кнопка приёмника (HIGH = нажата или обрыв)
static unsigned long estopRawAt = 0;

// ---- ток ----
static bool     adsPresent[2] = { false, false };
static SemaphoreHandle_t i2c1Mux = nullptr;
static volatile uint8_t  wantMask = 0;
static volatile float    pubPeakV[NMOD], pubMeanV[NMOD];
static volatile uint16_t pubN[NMOD];
static volatile float    lastV[NMOD];
static volatile bool     lastHave[NMOD];
static volatile uint16_t pubSps[2];
static volatile uint32_t adsErr[2];
static float iK[NMOD], iZ[NMOD];
static bool  iCal[NMOD];
static float iLim = I_LIM_DEF;               // предел вкладки Test (в рабочем режиме лимиты берутся из параметров LIMIT)
static float iPeakA[NMOD];                   // пик тока модуля, А (-1 = нет данных)
static uint8_t zeroMask = 0;
static unsigned long zeroDoneAt = 0;

// вкладка Test
static bool manRed = false, manGreen = false, manBuz = false;
static uint8_t blinkMode = 0;
static unsigned long blinkT0 = 0;
static uint16_t sweepMask = 0;
static unsigned long sweepT0 = 0, sweepAt = 0;
static bool testRelay = false, testOe = false, testEn = false;
static uint8_t tOvlMask = 0, tOvlCnt[NMOD];
static unsigned long tOvlAt = 0;

// ============================================================
// PCA9685
// ============================================================
static bool pcaWriteBytes(uint8_t reg, const uint8_t* d, uint8_t n) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(d, n);
  return Wire.endTransmission(true) == 0;
}
static bool pcaWrite1(uint8_t reg, uint8_t v) { return pcaWriteBytes(reg, &v, 1); }

static void pcaSetFreq(float f) {
  if (f < 24.0f) f = 24.0f;
  if (f > 1500.0f) f = 1500.0f;
  pwmFreq = f;
  if (!pcaOk) return;
  int pre = (int)lroundf(oscHz / (4096.0f * f)) - 1;
  if (pre < 3) pre = 3;
  if (pre > 255) pre = 255;
  pcaPre = pre;
  pcaWrite1(0x00, 0x10);
  pcaWrite1(0xFE, (uint8_t)pre);
  pcaWrite1(0x00, 0x20);
  pcaRestartPend = true;
  pcaRestartAt = millis() + 2;
}

static void duty2bytes(float duty, uint8_t* b) {
  long v = lroundf(duty * 40.96f);
  if (v <= 0)         { b[0] = 0; b[1] = 0;    b[2] = 0; b[3] = 0x10; }
  else if (v >= 4096) { b[0] = 0; b[1] = 0x10; b[2] = 0; b[3] = 0; }
  else                { b[0] = 0; b[1] = 0;    b[2] = (uint8_t)(v & 0xFF); b[3] = (uint8_t)(v >> 8); }
}

static void setFault(uint8_t code, const char* why);

static void pcaNote(bool ok) {
  if (ok) { pcaFail = 0; return; }
  if (++pcaFail >= 3) setFault(3, "PCA9685 does not answer");
}

static void pcaSetCh(uint8_t ch, float duty) {
  if (!pcaOk || ch > 15) return;
  pwmDuty[ch] = duty;
  uint8_t b[4];
  duty2bytes(duty, b);
  pcaNote(pcaWriteBytes(0x06 + 4 * ch, b, 4));
}

static void pcaSetAll(float duty) {
  if (!pcaOk) return;
  for (uint8_t i = 0; i < 16; i++) pwmDuty[i] = duty;
  uint8_t b[4];
  duty2bytes(duty, b);
  pcaNote(pcaWriteBytes(0xFA, b, 4));
}

static void pcaInit() {
  pcaOk = false;
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission(true) != 0) return;
  pcaOk = true;
  pcaWrite1(0x01, 0x04);            // MODE2: двухтактные выходы
  pcaSetAll(0);
  pcaSetFreq(pwmFreq);
}

// ============================================================
// Выходы: OE, EN, реле
// ============================================================
static void setOe(bool on) { oeOn = on; digitalWrite(PIN_OE, on ? LOW : HIGH); }
static void setEn(bool on) { enOn = on; digitalWrite(PIN_EN, on ? HIGH : LOW); }
static void relayPin(bool on) { digitalWrite(PIN_RELAY, on ? LOW : HIGH); }

// все выходы в безопасное состояние: сначала аппаратно (OE, EN), потом PCA, потом реле
static void hardOff() {
  setOe(false);
  setEn(false);
  pcaSetAll(0);
  sweepMask = 0;
}

static void relayOpen(unsigned long now) {
  relayPin(false);
  if (relayState != RS_OFF && relayState != RS_OPENING) { relayState = RS_OPENING; relayChangeAt = now; }
}

static void setFault(uint8_t code, const char* why) {
  if (faultCode) return;
  faultCode = code;
  setOe(false);
  setEn(false);
  relayPin(false);
  if (relayState == RS_CLOSING || relayState == RS_ON) { relayState = RS_OPENING; relayChangeAt = millis(); }
  plog("FAULT %u: %s. Outputs, EN and relay switched off until the receiver restarts", (unsigned)code, why);
}

static void relayMsgSet(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(relayMsg, sizeof(relayMsg), fmt, ap);
  va_end(ap);
  plog("%s", relayMsg);
}

// конечный автомат реле: want = нужно ли реле замкнуто
static void relayTick(unsigned long now, bool want) {
  bool busOk = vBus > 5.0f;
  if (faultCode) want = false;
  switch (relayState) {
    case RS_OFF:
      if (adcInit && now > 1500UL && (now - relayOffAt) > 1000UL) {          // залипание: реле выключено, а за ним напряжение
        if (vFb > 3.0f) { if (!relayBadAt) relayBadAt = now; else if ((now - relayBadAt) > 500UL) setFault(1, "relay stuck: voltage after relay while OFF"); }
        else relayBadAt = 0;
      }
      if (want && busOk && vFb < 1.5f && adcInit && now > 1500UL) {
        relayPin(true);
        relayState = RS_CLOSING;
        relayChangeAt = now;
        relayBadAt = 0;
      }
      break;
    case RS_CLOSING:
      if (!want) { relayPin(false); relayState = RS_OPENING; relayChangeAt = now; break; }
      if (busOk && vFb > 0.9f * vBus && (now - relayChangeAt) >= 300UL) {
        relayState = RS_ON;
        relayMsgSet("Relay ON OK (bus %.2f V, after relay %.2f V)", vBus, vFb);
      } else if ((now - relayChangeAt) >= 800UL) {
        if (busOk && vFb > 0.8f * vBus) { relayState = RS_ON; relayMsgSet("Relay ON OK, slow rise (bus %.2f V, after relay %.2f V)", vBus, vFb); }
        else if (!busOk) { relayPin(false); relayState = RS_OPENING; relayChangeAt = now; relayMsgSet("Relay ON: no bus voltage (12 V missing), cannot check"); }
        else setFault(2, "relay ON but no voltage after it (relay or fuse)");
      }
      break;
    case RS_ON:
      if (!want) { relayPin(false); relayState = RS_OPENING; relayChangeAt = now; break; }
      if (busOk && vFb < 0.5f * vBus) { if (!relayBadAt) relayBadAt = now; else if ((now - relayBadAt) > 500UL) setFault(2, "voltage after relay lost while relay ON (relay or fuse)"); }
      else relayBadAt = 0;
      break;
    case RS_OPENING:
      if (vFb < 1.5f) { relayState = RS_OFF; relayOffAt = now; relayBadAt = 0; relayMsgSet("Relay OFF OK (after relay %.2f V)", vFb); }
      else if ((now - relayChangeAt) >= 2500UL) setFault(1, "relay stuck: voltage after relay 2.5 s after opening");
      break;
  }
}

// ============================================================
// Сэмплер тока: ADS1115 в отдельной задаче (слой измерения; с MCP3208 заменяется)
// ============================================================
static bool i1Lock() { return i2c1Mux && xSemaphoreTake(i2c1Mux, portMAX_DELAY) == pdTRUE; }
static void i1Unlock() { if (i2c1Mux) xSemaphoreGive(i2c1Mux); }

static bool adsStart(uint8_t addr, uint8_t ch) {
  uint16_t cfg = 0x8000 | ((uint16_t)(4 + ch) << 12) | 0x0200 | 0x0100 | 0x00E0 | 0x0003;
  Wire1.beginTransmission(addr);
  Wire1.write(0x01);
  Wire1.write((uint8_t)(cfg >> 8));
  Wire1.write((uint8_t)(cfg & 0xFF));
  return Wire1.endTransmission(true) == 0;
}

static bool adsRead(uint8_t addr, int16_t& out) {
  Wire1.beginTransmission(addr);
  Wire1.write(0x00);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((int)addr, 2) != 2) return false;
  uint8_t hi = Wire1.read(), lo = Wire1.read();
  out = (int16_t)((hi << 8) | lo);
  return true;
}

static void bkAdd(uint8_t m, float v, uint32_t tms) {
  uint32_t ep = tms / BK_MS;
  Bk& b = bk[m][ep % BK_N];
  if (b.ep != ep || b.n == 0) { b.ep = ep; b.mx = v; b.sum = v; b.n = 1; }
  else { if (v > b.mx) b.mx = v; b.sum += v; b.n++; }
}

static void bkPublish(uint32_t tms) {
  uint32_t ep0 = tms / BK_MS;
  for (uint8_t m = 0; m < NMOD; m++) {
    float mx = -1000.0f, sum = 0; uint32_t n = 0;
    for (uint8_t i = 0; i < BK_N; i++) {
      const Bk& b = bk[m][i];
      if (b.n && ep0 >= b.ep && (ep0 - b.ep) < BK_N) { if (b.mx > mx) mx = b.mx; sum += b.sum; n += b.n; }
    }
    if (n) { pubPeakV[m] = mx; pubMeanV[m] = sum / n; }
    pubN[m] = (uint16_t)n;
  }
}

static void samplerStep(SamplerState& S) {
  int64_t now = esp_timer_get_time();
  bool busyWait = false;
  for (uint8_t d = 0; d < 2; d++) {
    if (!adsPresent[d]) { S.cur[d] = -1; continue; }
    uint8_t addr = d ? ADS_ADDR1 : ADS_ADDR0;
    if (S.cur[d] >= 0) {
      if ((now - S.startUs[d]) < ADS_CONV_US) { busyWait = true; continue; }
      int16_t raw = 0;
      bool ok = false;
      if (i1Lock()) { ok = adsRead(addr, raw); i1Unlock(); }
      uint8_t m = d * 4 + S.cur[d];
      S.cur[d] = -1;
      if (ok) {
        float v = raw * ADS_LSB;
        if (v < 0) v = 0;
        lastV[m] = v; lastHave[m] = true;
        bkAdd(m, v, (uint32_t)(esp_timer_get_time() / 1000));
        S.cnt[d]++; S.errRun[d] = 0;
      } else { adsErr[d]++; if (++S.errRun[d] > 20) S.errRun[d] = 20; }
      now = esp_timer_get_time();
    }
    uint8_t want = (wantMask >> (d * 4)) & 0x0F;
    int8_t next = -1;
    if (want) {
      for (uint8_t k = 1; k <= 4; k++) { uint8_t c = (S.rr[d] + k) & 3; if (want & (1u << c)) { next = c; break; } }
    } else if (now >= S.idleAt[d]) {
      next = (S.rr[d] + 1) & 3;
      S.idleAt[d] = now + (int64_t)IDLE_SCAN_MS * 1000;
    }
    if (next >= 0) {
      bool ok = false;
      if (i1Lock()) { ok = adsStart(addr, next); i1Unlock(); }
      if (ok) { S.cur[d] = next; S.rr[d] = next; S.startUs[d] = esp_timer_get_time(); busyWait = true; }
      else { adsErr[d]++; }
    }
  }
  now = esp_timer_get_time();
  if (now - S.pubAt >= 5000) { S.pubAt = now; bkPublish((uint32_t)(now / 1000)); }
  if (now - S.secAt >= 1000000) { S.secAt = now; pubSps[0] = (uint16_t)S.cnt[0]; pubSps[1] = (uint16_t)S.cnt[1]; S.cnt[0] = S.cnt[1] = 0; }
  if (busyWait) {
    int64_t left = ADS_CONV_US;
    for (uint8_t d = 0; d < 2; d++) if (S.cur[d] >= 0) { int64_t l = ADS_CONV_US - (esp_timer_get_time() - S.startUs[d]); if (l < left) left = l; }
    if (left > 1100) vTaskDelay(1);
  } else {
    vTaskDelay(1);
  }
}

static void samplerTask(void*) {
  static SamplerState S;
  S.secAt = esp_timer_get_time();
  for (;;) samplerStep(S);
}

static float voltToAmp(uint8_t m, float v) { float a = (v - iZ[m]) * iK[m]; return a < 0 ? 0 : a; }
static float modDuty(uint8_t m) { return pwmDuty[2 * m] > pwmDuty[2 * m + 1] ? pwmDuty[2 * m] : pwmDuty[2 * m + 1]; }
static uint8_t modActive() {
  uint8_t mask = 0;
  for (uint8_t m = 0; m < NMOD; m++) if (pwmDuty[2 * m] > 0.0f || pwmDuty[2 * m + 1] > 0.0f) mask |= (1u << m);
  return mask;
}

// слой измерения: быстрый АЦП (MCP3208) пока не установлен. Когда появится, функция возвращает true:
// включаются обнаружение обрыва и режим CURRENT (на ADS1115 эти функции недостоверны: импульс 2-15 % короче замера)
static bool currentBackendFast() { return false; }

static void keyName(char* k, const char* base, uint8_t m) { snprintf(k, 8, "%s%u", base, (unsigned)m); }

static void calibLoad() {
  prefs.begin("calib", false);
  prefs.putUInt("cver", 1);
  iLim = prefs.getFloat("ilim", I_LIM_DEF);
  if (iLim < 0.3f || iLim > 6.0f) iLim = I_LIM_DEF;
  for (uint8_t m = 0; m < NMOD; m++) {
    char k[8];
    keyName(k, "ik", m);
    iCal[m] = prefs.isKey(k);
    iK[m] = prefs.getFloat(k, K_DEF);
    if (iK[m] < 0.8f || iK[m] > 3.5f) { iK[m] = K_DEF; iCal[m] = false; }
    keyName(k, "iz", m);
    iZ[m] = prefs.getFloat(k, 0.0f);
    if (iZ[m] < 0.0f || iZ[m] > 0.3f) iZ[m] = 0.0f;
    iPeakA[m] = -1.0f; pubN[m] = 0; lastHave[m] = false;
  }
  kBus = prefs.getFloat("kbus", K_BUS_DEF);
  kFb  = prefs.getFloat("kfb", K_FB_DEF);
  if (kBus < 5 || kBus > 20) kBus = K_BUS_DEF;
  if (kFb < 5 || kFb > 20) kFb = K_FB_DEF;
  oscHz = prefs.getFloat("osc", 25000000.0f);
  if (oscHz < 20e6f || oscHz > 32e6f) oscHz = 25000000.0f;
}

static uint8_t calMaskBits() { uint8_t m = 0; for (uint8_t i = 0; i < NMOD; i++) if (iCal[i]) m |= (1u << i); return m; }
static bool anyUncal() { for (uint8_t i = 0; i < NMOD; i++) if (!iCal[i]) return true; return false; }

// ============================================================
// Рабочий режим: цели ШИМ, разгон/торможение, дизеринг, защиты по току
// ============================================================
static float sDuty[16];                    // команда скважности соленоида до дизеринга, %
static float sCap[16];                     // потолок скважности от ограничения по току, %
static float sReg[16];                     // регулятор тока (режим CURRENT), %
static bool  limFlag[NUM_CH];
static bool  ovlLatch[NUM_CH];             // перегрузка: канал выключен, пока рычаг не вернётся в центр
static unsigned long ovlSince[NUM_CH];
static float ovlPeak[NUM_CH];
static uint8_t ovlSide[NUM_CH];
static bool  openFlag[NUM_CH];
static unsigned long openSince[NUM_CH];
static bool  armedRun = false;             // рычаги побывали в нуле после включения выходов
static bool  outAct = false;               // выходы включены (EN, OE)
static unsigned long ctlAt = 0, evalAt = 0, telAt = 0;
static unsigned long lastCtlTick = 0;
static uint8_t killReasonNow = 3;
static bool    killNow = true;
static bool    anyActive = false;

static void runReset() {
  for (int i = 0; i < 16; i++) { sDuty[i] = 0; sCap[i] = 100.0f; sReg[i] = 0; }
  for (int c = 0; c < NUM_CH; c++) { limFlag[c] = false; ovlLatch[c] = false; ovlSince[c] = 0; openFlag[c] = false; openSince[c] = 0; }
  armedRun = false;
}

static inline float paramPct(int p, int ch, int side) { return ramP[p][ch][side] / 10.0f; }

static bool modeCurrent() { return ramP[RXP_MODE][0][0] == 1 && currentBackendFast(); }

static float triWave(unsigned long now, float hz) {
  float ph = fmodf((float)now * hz / 1000.0f, 1.0f);
  return ph < 0.5f ? (ph * 4.0f - 1.0f) : (3.0f - ph * 4.0f);
}

// проверки по току раз в 40 мс (окно сэмплера): ограничение, перегрузка, обрыв
static void currentEval(unsigned long now) {
  uint8_t act = modActive();
  wantMask = act | zeroMask;
  for (uint8_t m = 0; m < NMOD; m++) iPeakA[m] = ((act & (1u << m)) && pubN[m] > 0) ? voltToAmp(m, pubPeakV[m]) : -1.0f;
  if (zeroMask && (long)(now - zeroDoneAt) >= 0) {
    uint8_t mask = zeroMask; zeroMask = 0;
    for (uint8_t m = 0; m < NMOD; m++) {
      if (!(mask & (1u << m))) continue;
      if (pubN[m] < 3) { plog("M%u: no samples (ADS1115 missing?), zero not saved", m + 1); continue; }
      float v = pubMeanV[m];
      if (v < 0.0f || v > 0.3f) { plog("M%u: zero %.3f V is outside 0..0.3 V (input floating, IS wire not connected, or current flows). Not saved", m + 1, v); continue; }
      iZ[m] = v;
      char k[8]; keyName(k, "iz", m);
      prefs.putFloat(k, v);
      plog("M%u: zero offset %.4f V (saved)", m + 1, v);
    }
  }
}

static void runCurrentChecks(unsigned long now) {
  if ((now - evalAt) < 40UL) return;
  evalAt = now;
  uint16_t ovlDelay = ramP[RXP_OVLDLY][0][0];
  for (int ch = 0; ch < NUM_CH; ch++) {
    int side = (sDuty[2 * ch + 1] > sDuty[2 * ch]) ? 1 : 0;
    int s = 2 * ch + side;
    float d = sDuty[s];
    float I = iPeakA[ch];
    if (d <= 0.0f || I < 0.0f || pubN[ch] < 2) {
      ovlSince[ch] = 0;
      if (d <= 0.0f) { limFlag[ch] = false; openSince[ch] = 0; if (!ovlLatch[ch]) openFlag[ch] = false; }
      continue;
    }
    float lim = ramP[RXP_LIMIT][ch][side] / 10.0f;
    float startD = paramPct(RXP_DUTY, ch, side);
    if (I > lim * 1.3f) {                                   // перегрузка: ток в импульсе выше лимита дольше задержки
      if (!ovlSince[ch]) ovlSince[ch] = now;
      else if ((now - ovlSince[ch]) >= ovlDelay) {
        ovlLatch[ch] = true; ovlPeak[ch] = I; ovlSide[ch] = (uint8_t)side;
        sDuty[2 * ch] = 0; sDuty[2 * ch + 1] = 0; sReg[2 * ch] = sReg[2 * ch + 1] = 0;
        pcaSetCh(2 * ch, 0); pcaSetCh(2 * ch + 1, 0);
        plog("OVERLOAD ch %d side %c: %.2f A > limit %.2f A", ch + 1, side ? 'B' : 'A', I, lim);
      }
    } else ovlSince[ch] = 0;
    if (I >= lim) {                                         // ограничение: потолок ШИМ снижаем, пока ток не войдёт в лимит
      limFlag[ch] = true;
      float base = d < sCap[s] ? d : sCap[s];
      sCap[s] = base * 0.95f;
      if (sCap[s] < startD) sCap[s] = startD;
    } else if (I < lim * 0.9f) {
      limFlag[ch] = false;
      if (sCap[s] < 100.0f) { sCap[s] += 0.5f; if (sCap[s] > 100.0f) sCap[s] = 100.0f; }
    }
    if (currentBackendFast()) {                             // обрыв: ШИМ выше стартового, ток в импульсе меньше 0.1 А дольше 300 мс
      if (d > startD && I < 0.1f) { if (!openSince[ch]) openSince[ch] = now; else if ((now - openSince[ch]) >= 300UL) openFlag[ch] = true; }
      else { openSince[ch] = 0; openFlag[ch] = false; }
    } else { openFlag[ch] = false; }
    if (modeCurrent()) {                                    // режим CURRENT: подстройка скважности по току в импульсе
      int v = abs((int)ctlSh.v[ch]);
      float iT = (ramP[RXP_STARTI][ch][side] + (ramP[RXP_MAXI][ch][side] - ramP[RXP_STARTI][ch][side]) * v / 100.0f) / 10.0f;
      sReg[s] += constrain(0.5f * (iT - I), -1.0f, 1.0f);
    }
  }
}

static void controlRun(unsigned long now, const CtlShared& c, float dtMs) {
  anyActive = false;
  bool cur = modeCurrent();
  float vb = vBus > 5.0f ? vBus : 12.0f;
  for (int ch = 0; ch < NUM_CH; ch++) {
    int v = c.v[ch];
    int mag = abs(v);
    bool en = ramP[RXP_CHEN][ch][0] != 0;
    if (ovlLatch[ch] && mag == 0) { ovlLatch[ch] = false; openFlag[ch] = false; }
    int dirIdx = v > 0 ? 0 : 1;
    for (int side = 0; side < 2; side++) {
      int s = 2 * ch + side;
      float startD = paramPct(RXP_DUTY, ch, side);
      float maxD = paramPct(RXP_MAXD, ch, side);
      if (maxD < startD) maxD = startD;
      float tgt = 0;
      if (side == dirIdx && mag > 0 && en && armedRun && !ovlLatch[ch]) {
        if (cur) {
          float iT = (ramP[RXP_STARTI][ch][side] + (ramP[RXP_MAXI][ch][side] - ramP[RXP_STARTI][ch][side]) * mag / 100.0f) / 10.0f;
          if (sReg[s] <= 0.0f) sReg[s] = constrain(iT * R_COIL_NOM / vb * 100.0f, startD, maxD);
          tgt = constrain(sReg[s], startD, maxD);
        } else {
          tgt = startD + (maxD - startD) * mag / 100.0f;
        }
        if (tgt > sCap[s]) tgt = sCap[s] > startD ? sCap[s] : startD;
      } else sReg[s] = 0;
      if (tgt > 0 && sDuty[s ^ 1] > 0) tgt = 0;               // в паре одновременно активен один соленоид
      float d = sDuty[s];
      float rUp = ramP[RXP_RAMPUP][ch][0], rDn = ramP[RXP_RAMPDN][ch][0];
      float rateUp = rUp > 0 ? maxD / rUp : 1e9f, rateDn = rDn > 0 ? maxD / rDn : 1e9f;
      if (tgt > d) {
        if (d < startD) d = startD;
        d += rateUp * dtMs;
        if (d > tgt) d = tgt;
      } else if (tgt < d) {
        if (tgt <= 0.0f) { d -= rateDn * dtMs; if (d <= startD) d = 0; }
        else { d -= rateDn * dtMs; if (d < tgt) d = tgt; }
      }
      sDuty[s] = d;
      float out = d;
      if (d > 0.0f && ramP[RXP_DITHAMP][0][0] > 0) {
        out = d + ramP[RXP_DITHAMP][0][0] * triWave(now, (float)ramP[RXP_DITHFRQ][0][0]);
        out = constrain(out, 0.1f, 100.0f);
      }
      if (d > 0.0f) anyActive = true;
      if (fabsf(out - pwmDuty[s]) >= 0.04f || (out == 0.0f) != (pwmDuty[s] == 0.0f)) pcaSetCh((uint8_t)s, out);
    }
  }
}

// телеметрия для радио
static void telPublish() {
  TelShared t;
  memset(&t, 0, sizeof(t));
  for (int ch = 0; ch < NUM_CH; ch++) {
    int side = (sDuty[2 * ch + 1] > sDuty[2 * ch] || (ovlLatch[ch] && ovlSide[ch])) ? 1 : 0;
    float d = sDuty[2 * ch + side];
    int p = (int)lroundf(d);
    t.pwm[ch] = (int8_t)(side ? -p : p);
    float I = ovlLatch[ch] ? ovlPeak[ch] : iPeakA[ch];
    if (I < 0) I = 0;
    int ci = (int)lroundf(I * 10.0f);
    t.cur[ch] = (uint8_t)(ci > 255 ? 255 : ci);
    if (limFlag[ch]) t.mLim |= (uint8_t)(1 << ch);
    if (ovlLatch[ch]) t.mOvl |= (uint8_t)(1 << ch);
    if (openFlag[ch]) t.mOpen |= (uint8_t)(1 << ch);
    if (side && (d > 0 || ovlLatch[ch])) t.side |= (uint8_t)(1 << ch);
  }
  float vb = vBus * 10.0f;
  t.vrx = (uint8_t)(vb > 255.0f ? 255 : (vb < 0 ? 0 : (int)lroundf(vb)));
  t.kill = killNow ? 1 : 0;
  t.reason = killReasonNow;
  taskENTER_CRITICAL(&shMux);
  telSh = t;
  taskEXIT_CRITICAL(&shMux);
}

// ============================================================
// Аварийная кнопка, напряжения, главный такт управления
// ============================================================
static void powerMeasure(unsigned long now) {
  if ((now - adcAt) < 10UL) return;
  adcAt = now;
  float pb = analogReadMilliVolts(PIN_VBUS) / 1000.0f;
  float pf = analogReadMilliVolts(PIN_VFB) / 1000.0f;
  if (!adcInit) { emaB = pb; emaF = pf; adcInit = true; }
  else { emaB += (pb - emaB) * 0.05f; emaF += (pf - emaF) * 0.05f; }
  vBus = emaB * kBus; if (vBus < V_DEADBAND) vBus = 0;
  vFb  = emaF * kFb;  if (vFb  < V_DEADBAND) vFb = 0;
}

static void estopTick(unsigned long now) {
  bool raw = (digitalRead(PIN_ESTOP) == HIGH);
  if (raw != estopRaw) { estopRaw = raw; estopRawAt = now; }
  if (estopRaw != estopBtn && (now - estopRawAt) >= 30UL) {
    estopBtn = estopRaw;
    if (estopBtn) { btnLatched = true; plog("RX E-STOP button pressed (or wire broken)"); }
    else plog("RX E-STOP button released (alarm stays until the pult sends zeros)");
  }
}

static bool wifiActive() { return wifiSt != HLW_OFF; }
static bool testMode() { return testOK && wifiSt != HLW_OFF; }

static void testCurrentTick(unsigned long now);

static void controlTick(unsigned long now) {
  CtlShared c;
  taskENTER_CRITICAL(&shMux);
  c = ctlSh;
  taskEXIT_CRITICAL(&shMux);
  uint16_t tmo = ramP[RXP_LINKTMO][0][0];
  bool link = c.have && c.rxAt != 0 && (now - c.rxAt) < tmo;
  bool pultEs = link && c.estop;
  // killswitch и его причина (0 пульт, 1 кнопка приёмника, 2 сбой, 3 нет связи): приоритет сбой > кнопка > пульт > связь
  bool kill = false; uint8_t reason = 3;
  if (faultCode)        { kill = true; reason = 2; }
  else if (btnLatched)  { kill = true; reason = 1; }
  else if (pultEs)      { kill = true; reason = 0; }
  else if (!link)       { kill = true; reason = 3; }
  killNow = kill; killReasonNow = reason;

  bool tm = testMode();
  if (tm) {
    relayTick(now, testRelay && !kill);
    bool allow = !faultCode && !btnLatched;
    if (!allow) { testRelay = false; testOe = false; testEn = false; hardOff(); }
    setOe(allow && testOe);
    setEn(allow && testEn);
    outAct = false;
    runReset();
    testCurrentTick(now);
    ctlAt = now;
    return;
  }

  // рабочий режим
  bool busOk = vBus > 5.0f;
  bool runEnable = !kill && !bindMode && !wifiActive() && busOk;
  relayTick(now, runEnable);
  bool act = runEnable && relayState == RS_ON && !faultCode;
  if (act != outAct) {
    outAct = act;
    if (act) { runReset(); setEn(true); setOe(true); plog("Outputs ENABLED"); }
    else { hardOff(); runReset(); plog("Outputs OFF"); }
  }
  if (!act && (oeOn || enOn)) hardOff();
  if (!act) { for (int i = 0; i < NMOD; i++) iPeakA[i] = -1.0f; currentEval(now); telPublish(); ctlAt = now; return; }

  if (!armedRun) {                                          // Safe Start приёмника: сначала нули
    bool z = true;
    for (int i = 0; i < NUM_CH; i++) if (c.v[i] != 0) z = false;
    if (z) armedRun = true;
  }
  float dt = (float)(now - ctlAt);
  if (dt > 50.0f) dt = 50.0f;
  ctlAt = now;
  currentEval(now);
  runCurrentChecks(now);
  controlRun(now, c, dt);
  telPublish();
}

// ============================================================
// Мигалка, зуммер, светодиод платы
// ============================================================
static unsigned long ledAt = 0;
static bool ledOn = false;

static void lampsTick(unsigned long now) {
  bool r = false, g = false, b = false;
  bool tm = testMode();
  if (tm && (manRed || manGreen || manBuz || blinkMode)) {
    r = manRed; g = manGreen; b = manBuz;
    if (blinkMode) {
      unsigned long ph = now - blinkT0;
      r = g = b = false;
      if (blinkMode == 1) g = (ph % 1000UL) < 500UL;
      else if (blinkMode == 2) { bool x = (ph % 250UL) < 125UL; r = x; b = x; }
      else if (blinkMode == 3) { bool x = (ph % 1000UL) < 500UL; r = x; g = !x; }
    }
  } else if (wifiActive()) {
    r = g = true;                                           // оба вместе: режим обновления / Test
  } else if (killNow && (killReasonNow == 0 || killReasonNow == 1 || killReasonNow == 2)) {
    bool x = (now % 250UL) < 125UL; r = x; b = x;           // авария: красный быстро + зуммер
  } else if (killNow) {
    r = (now % 1000UL) < 500UL;                             // нет связи: красный медленно
  } else if (anyActive) {
    g = (now % 250UL) < 125UL;                              // канал активен: зелёный быстро
  } else {
    g = (now % 1000UL) < 500UL;                             // норма: зелёный медленно
  }
  if (now < 200UL) b = true;                                // короткий сигнал при включении
  digitalWrite(PIN_RED, r);
  digitalWrite(PIN_GREEN, g);
  digitalWrite(PIN_BUZ, b);

  unsigned long period = 1000UL;
  if (bindMode) period = 200UL;
  if ((long)(now - bindOkUntil) < 0 && bindOkUntil) { digitalWrite(PIN_LED, HIGH); ledOn = true; return; }
  if (!ledOn && (now - ledAt) >= period) { ledOn = true; ledAt = now; digitalWrite(PIN_LED, HIGH); }
  else if (ledOn && (now - ledAt) >= 60UL) { ledOn = false; digitalWrite(PIN_LED, LOW); }
}

// ============================================================
// РАДИО: HydroLink v1 (задача на ядре 0). Формат пакетов: remote_config.md §52
// ============================================================
static SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);
static volatile bool hlIrq = false;
static void IRAM_ATTR hlIsr() { hlIrq = true; }
static bool hlInit = false, hlTxBusy = false;
static int  hlErr = 0;

static uint8_t  qBuf[4][64];
static size_t   qLen[4];
static uint8_t  qHead = 0, qCount = 0;

static uint8_t  tSeq = 0, lastCtlSeq = 0, missedCnt = 0;
static bool     haveSeq = false;
static int8_t   lastSnr = 0, lastRssi = 0;
static uint8_t  lastCmdTag = 0;
static uint8_t  lastCmdAck[12];
static bool     haveCmdAck = false;
static unsigned long infoAt = 0;
static bool     infoVerPending = false;
static char     rxVer[24] = "";                 // версия, найденная на GitHub (INFO 1)
static char     apSsid[16] = "";                // RX-xxxx (INFO 2)
static char     netSsid[NET_MAX][33];
static char     netPass[NET_MAX][64];

static uint16_t hlCrc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

static size_t hlFinish(uint8_t* b, size_t n) {
  uint16_t c = hlCrc16(b, n);
  b[n] = (uint8_t)(c & 0xFF); b[n + 1] = (uint8_t)(c >> 8);
  return n + 2;
}

static void qPush(const uint8_t* b, size_t n) {
  if (qCount >= 4 || n > 64) return;
  uint8_t i = (uint8_t)((qHead + qCount) & 3);
  memcpy(qBuf[i], b, n);
  qLen[i] = n;
  qCount++;
}

static void hdr(uint8_t* b, uint8_t type, uint8_t flags, uint16_t pair, uint8_t tag) {
  b[0] = (uint8_t)((type << 4) | (flags & 0x0F));
  b[1] = (uint8_t)(pair & 0xFF); b[2] = (uint8_t)(pair >> 8);
  b[3] = tag;
}

static void sendAck(uint8_t tag, uint8_t st, uint8_t p, uint8_t ch, uint8_t d, uint16_t val, bool cache) {
  uint8_t b[12];
  hdr(b, HLT_ACK, 0, pairId(), tag);
  b[4] = st; b[5] = p; b[6] = ch; b[7] = d; b[8] = (uint8_t)(val & 0xFF); b[9] = (uint8_t)(val >> 8);
  hlFinish(b, 10);
  if (cache) { memcpy(lastCmdAck, b, 12); lastCmdTag = tag; haveCmdAck = true; }
  qPush(b, 12);
}

static void sendBatch(uint8_t tag, uint8_t prm) {
  uint8_t b[48];
  hdr(b, HLT_BATCH, 0, pairId(), tag);
  int n = RXP[prm].scope == PS_SOL ? 16 : (RXP[prm].scope == PS_CH ? NUM_CH : 1);
  b[4] = prm; b[5] = (uint8_t)n;
  for (int k = 0; k < n; k++) {
    uint16_t v;
    if (RXP[prm].scope == PS_SOL) v = nvsP[prm][k / 2][k % 2];
    else if (RXP[prm].scope == PS_CH) v = nvsP[prm][k][0];
    else v = nvsP[prm][0][0];
    b[6 + 2 * k] = (uint8_t)(v & 0xFF); b[7 + 2 * k] = (uint8_t)(v >> 8);
  }
  qPush(b, hlFinish(b, 6 + 2 * n));
}

static void sendTelem() {
  TelShared t;
  taskENTER_CRITICAL(&shMux);
  t = telSh;
  taskEXIT_CRITICAL(&shMux);
  uint8_t b[36];
  memset(b, 0, sizeof(b));
  hdr(b, HLT_TELEM, 0, pairId(), ++tSeq);
  b[4] = lastCtlSeq;
  b[5] = (uint8_t)lastSnr;
  b[6] = missedCnt; missedCnt = 0;
  for (int i = 0; i < NUM_CH; i++) { b[7 + i] = t.cur[i]; b[15 + i] = (uint8_t)t.pwm[i]; }
  b[23] = t.mLim; b[24] = t.mOvl; b[25] = t.mOpen; b[26] = t.side;
  b[27] = (uint8_t)((t.kill ? 1 : 0) | ((t.reason & 3) << 1));
  b[28] = t.vrx;
  b[29] = (uint8_t)lastRssi;
  b[30] = bootCount;
  b[31] = cfgCrc8();
  b[32] = wifiSt; b[33] = wifiArg;
  qPush(b, hlFinish(b, 34));
}

static void sendInfo(uint8_t kind, const char* text) {
  uint8_t b[40];
  size_t l = strlen(text);
  if (l > 23) l = 23;
  hdr(b, HLT_INFO, 0, pairId(), 0);
  b[4] = kind; b[5] = (uint8_t)l;
  memcpy(b + 6, text, l);
  qPush(b, hlFinish(b, 6 + l));
}

static void hlSetPower() {
  int p = ramP[RXP_PWR][0][0];
  radio.setOutputPower((int8_t)p);
  radio.setCurrentLimit(p > 14 ? 140.0f : 60.0f);
}

static void hlBegin() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  int p = ramP[RXP_PWR][0][0];
  int st = radio.begin(RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, RF_CR, RF_SYNC, (int8_t)p, RF_PREAMBLE, RF_TCXO_V);
  if (st != RADIOLIB_ERR_NONE) st = radio.begin(RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, RF_CR, RF_SYNC, (int8_t)p, RF_PREAMBLE, 0.0f);
  if (st == RADIOLIB_ERR_NONE) {
    radio.setDio2AsRfSwitch(true);
    radio.setDio1Action(hlIsr);
    st = radio.startReceive();
  }
  hlErr = st;
  hlInit = (st == RADIOLIB_ERR_NONE);
  if (hlInit) hlSetPower();
  plog("Radio: %s (%d), %d dBm", hlInit ? "started" : "init failed", st, p);
}

// условие для команд, которым нужны нулевые рычаги и отсутствие аварии
static bool hlSafeNow(unsigned long now) {
  CtlShared c;
  taskENTER_CRITICAL(&shMux);
  c = ctlSh;
  taskEXIT_CRITICAL(&shMux);
  bool zero = (c.nzAt == 0) || ((now - c.nzAt) >= 300UL);
  return zero && !c.estop && !btnLatched && !faultCode;
}

static unsigned long lastCtlAt = 0;
// Настройки/сеть во время синхронизации идут без пакетов управления: продлеваем связь, но не дольше 3 с после последнего управления
static void cfgKeepAlive(unsigned long now) {
  if (lastCtlAt == 0 || (now - lastCtlAt) >= 3000UL) return;
  taskENTER_CRITICAL(&shMux);
  if (ctlSh.have) ctlSh.rxAt = now ? now : 1;
  taskEXIT_CRITICAL(&shMux);
}

static void handleControl(const uint8_t* p, size_t len, unsigned long now) {
  if (len < 7) return;
  uint8_t flags = p[0] & 0x0F, mask = p[4];
  int n = __builtin_popcount(mask);
  if (len != (size_t)(7 + n)) return;
  int snrI = (int)lroundf(radio.getSNR()), rssiI = (int)lroundf(radio.getRSSI());
  lastSnr = (int8_t)constrain(snrI, -127, 127);
  lastRssi = (int8_t)constrain(rssiI, -127, 0);
  uint8_t seq = p[3];
  if (haveSeq) { uint8_t gap = (uint8_t)(seq - lastCtlSeq - 1); if (gap < 128) { int m = missedCnt + gap; missedCnt = (uint8_t)(m > 255 ? 255 : m); } }
  haveSeq = true; lastCtlSeq = seq; lastCtlAt = now ? now : 1;
  int8_t v[NUM_CH] = { 0 };
  int k = 5;
  bool nz = false;
  for (int i = 0; i < NUM_CH; i++) if (mask & (1 << i)) { int raw = (int)(int8_t)p[k++]; v[i] = (int8_t)constrain(raw, -100, 100); if (v[i]) nz = true; }
  if (flags & HLF_ESTOP) { digitalWrite(PIN_OE, HIGH); digitalWrite(PIN_EN, LOW); }       // E-Stop: аппаратно сразу, остальное в такте управления
  taskENTER_CRITICAL(&shMux);
  memcpy(ctlSh.v, v, sizeof(v));
  ctlSh.rxAt = now ? now : 1;
  if (nz) ctlSh.nzAt = now ? now : 1;
  ctlSh.estop = (flags & HLF_ESTOP) != 0;
  ctlSh.lever = (flags & HLF_LEVER) != 0;
  ctlSh.have = true;
  taskEXIT_CRITICAL(&shMux);
  if (btnLatched && !estopBtn && !nz && !(flags & HLF_ESTOP)) { btnLatched = false; plog("RX E-STOP alarm cleared"); }
  if (flags & HLF_REQT) { sendTelem(); infoAt = now; }
  else if ((long)(now - infoAt) >= 1500L) {                           // неподтверждаемая информация для экрана пульта
    if (wifiSt == HLW_PH_WAIT || wifiSt == HLW_PH_CONN) { sendInfo(2, apSsid); infoAt = now; }
    else if (wifiSt == HLW_GH_ASK && rxVer[0]) { sendInfo(1, rxVer); infoAt = now; }
  }
}

static void handleCfg(const uint8_t* p, size_t len, unsigned long now) {
  if (len != 12) return;
  cfgKeepAlive(now);
  uint8_t tag = p[3], op = p[4], prm = p[5], ch = p[6], d = p[7];
  uint16_t val = (uint16_t)(p[8] | (p[9] << 8));
  bool save = (p[0] & 1) != 0;
  if ((op == HLO_CMD || op == HLO_NET) && haveCmdAck && tag == lastCmdTag) { qPush(lastCmdAck, 12); return; }   // повтор уже выполненной команды
  if (op == HLO_SET || op == HLO_GET) {
    if (prm >= NUM_RXP || ch >= NUM_CH || d > 1) { sendAck(tag, 1, prm, ch, d, val, false); return; }
    int c = ch, dd = d;
    paramNorm(prm, c, dd);
    if (op == HLO_GET) { sendAck(tag, 0, prm, ch, d, ramP[prm][c][dd], false); return; }
    int vv = constrain((int)val, (int)RXP[prm].mn, (int)RXP[prm].mx);
    if (prm == RXP_MODE && vv == 1 && !currentBackendFast()) { sendAck(tag, 2, prm, ch, d, ramP[prm][c][dd], false); return; }   // режим CURRENT ждёт быстрый АЦП
    ramP[prm][c][dd] = (uint16_t)vv;
    if (save) { nvsP[prm][c][dd] = (uint16_t)vv; cfgSavePending = true; }
    if (prm == RXP_PWR) pwrDirty = true;
    sendAck(tag, 0, prm, ch, d, (uint16_t)vv, false);
  } else if (op == HLO_BATCH) {
    if (prm >= NUM_RXP) { sendAck(tag, 1, prm, 0, 0, 0, false); return; }
    sendBatch(tag, prm);
  } else if (op == HLO_CMD) {
    uint8_t st = 0;
    switch (prm) {
      case HLC_PROTO:   st = (val == HL_PROTO) ? 0 : 1; break;
      case HLC_CHMASK:  chMaskPult = val; break;
      case HLC_CHTYPES: chTypesPult = val; break;
      case HLC_TXPWR:   pultTxPwr = val; break;
      case HLC_FACTORY:
        if (val != HL_FACTORY_KEY) st = 1;
        else if (!hlSafeNow(now) || wifiSt != HLW_OFF) st = 2;
        else {
          cfgDefaults(ramP); cfgDefaults(nvsP);
          bootCount++;
          cfgSavePending = true;
          pwrDirty = true;
          plog("FACTORY RESET of the receiver settings");
        }
        break;
      case HLC_WIFI_PHONE:
      case HLC_WIFI_GH:
        if (wifiSt != HLW_OFF || wifiReq) st = 3;
        else if (!hlSafeNow(now)) st = 2;
        else wifiReq = (prm == HLC_WIFI_PHONE) ? 1 : 2;
        break;
      case HLC_WIFI_OFF:
        if (wifiSt == HLW_PH_UP || wifiSt == HLW_GH_DL || wifiSt == HLW_GH_VERIFY) st = 3;
        else wifiReq = 3;
        break;
      case HLC_WIFI_YES:
        if (wifiSt == HLW_GH_ASK) wifiReq = 4; else st = 2;
        break;
      default: st = 1; break;
    }
    sendAck(tag, st, prm, ch, d, val, true);
  } else {
    sendAck(tag, 1, prm, ch, d, val, false);
  }
}

static void handleNet(const uint8_t* p, size_t len) {
  if (len < 10) return;
  cfgKeepAlive(millis());
  uint8_t tag = p[3], slot = p[4];
  uint8_t ls = p[6];
  if (6 + 1 + (size_t)ls + 1 > len) return;
  uint8_t lp = p[7 + ls];
  if (len != (size_t)(8 + ls + lp + 2)) return;
  if (haveCmdAck && tag == lastCmdTag) { qPush(lastCmdAck, 12); return; }
  if (slot >= NET_MAX || ls > 32 || lp > 63 || wifiSt >= HLW_GH_CONN) { sendAck(tag, 1, 0xE0, slot, 0, 0, true); return; }
  memcpy(netSsid[slot], p + 7, ls); netSsid[slot][ls] = 0;
  memcpy(netPass[slot], p + 8 + ls, lp); netPass[slot][lp] = 0;
  sendAck(tag, 0, 0xE0, slot, 0, 0, true);
}

static void handleBind(const uint8_t* p, size_t len, unsigned long now) {
  if (len != 10) return;
  uint32_t id = (uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
  bool inBind = bindMode;
  bool again = (bindOkUntil != 0) && ((long)(now - bindOkUntil) < 0) && id == bindId;      // пульт повторяет запрос, если не получил ответ
  if (!inBind && !again) return;
  if (inBind) {
    bindId = id;
    bindSave(id);
    bindMode = false;
    bindOkUntil = now + 4000UL;
    plog("Bind: new ID stored %08lX", (unsigned long)id);
    haveSeq = false;
  }
  uint8_t b[12];
  hdr(b, HLT_BINDOK, 0, pairId(), p[3]);
  b[4] = p[4]; b[5] = p[5]; b[6] = p[6]; b[7] = p[7];
  qPush(b, hlFinish(b, 8));
}

static void hlRead(unsigned long now) {
  size_t len = radio.getPacketLength();
  uint8_t p[64];
  if (len > sizeof(p) || len < 6) { uint8_t d[64]; radio.readData(d, len > sizeof(d) ? sizeof(d) : len); return; }
  if (radio.readData(p, len) != RADIOLIB_ERR_NONE) return;
  uint16_t c = hlCrc16(p, len - 2);
  if (c != (uint16_t)(p[len - 2] | (p[len - 1] << 8))) return;
  int type = p[0] >> 4;
  uint16_t pr = (uint16_t)(p[1] | (p[2] << 8));
  if (type == HLT_BIND) { if (pr == 0xFFFF) handleBind(p, len, now); return; }
  if (pr != pairId()) return;
  switch (type) {
    case HLT_CONTROL: handleControl(p, len, now); break;
    case HLT_CFG:     handleCfg(p, len, now); break;
    case HLT_NET:     handleNet(p, len); break;
    default: break;
  }
}

static void radioStep(unsigned long now) {
  if (!hlInit) return;
  if (hlIrq) {
    hlIrq = false;
    if (hlTxBusy) {
      radio.finishTransmit();
      hlTxBusy = false;
      radio.startReceive();
    } else {
      hlRead(now);
    }
  }
  if (!hlTxBusy && qCount) {
    if (pwrDirty) { pwrDirty = false; hlSetPower(); }
    if (radio.startTransmit(qBuf[qHead], qLen[qHead]) == RADIOLIB_ERR_NONE) hlTxBusy = true;
    else radio.startReceive();
    qHead = (uint8_t)((qHead + 1) & 3);
    qCount--;
  } else if (!hlTxBusy && pwrDirty) {
    pwrDirty = false; hlSetPower();
  }
}

static void radioTask(void*) {
  for (;;) { radioStep(millis()); vTaskDelay(1); }
}

// ============================================================
// Вкладка Test: команды (порт из hwtest) и защита
// ============================================================
static bool outR = false, outG = false, outB = false;
static unsigned long restartAt = 0;
static unsigned long lastPoll = 0;
static float lastRunFreq = -1;

static void testOff(const char* why) {
  bool was = testOK;
  testOK = false;
  testRelay = testOe = testEn = false;
  manRed = manGreen = manBuz = false; blinkMode = 0;
  sweepMask = 0;
  tOvlMask = 0;
  hardOff();
  relayOpen(millis());
  if (wifiSt == HLW_PH_TEST) wifiSt = HLW_PH_CONN;
  lastRunFreq = -1;                                          // вернуть рабочую частоту ШИМ
  if (was) plog("Test mode OFF (%s)", why);
}

static void testCurrentTick(unsigned long now) {
  if (sweepMask && (long)(now - sweepAt) >= 0) {
    sweepAt = now + 40;
    float p = (float)((now - sweepT0) % 10000UL) / 10000.0f;
    float tri = p < 0.5f ? p * 2.0f : (1.0f - p) * 2.0f;
    float duty = 10.0f + 80.0f * tri;
    if (sweepMask == 0xFFFF) pcaSetAll(duty);
    else for (uint8_t i = 0; i < 16; i++) if (sweepMask & (1u << i)) pcaSetCh(i, duty);
  }
  currentEval(now);
  if ((now - tOvlAt) >= 40UL) {
    tOvlAt = now;
    uint8_t act = modActive();
    for (uint8_t m = 0; m < NMOD; m++) {
      bool over = iCal[m] && (act & (1u << m)) && pubN[m] >= 2 && iPeakA[m] > iLim;
      if (!over) { tOvlCnt[m] = 0; continue; }
      if (++tOvlCnt[m] >= OVL_WINDOWS && !(tOvlMask & (1u << m))) {
        tOvlMask |= (1u << m);
        pcaSetCh(2 * m, 0); pcaSetCh(2 * m + 1, 0);
        sweepMask &= (uint16_t)~(3u << (2 * m));
        plog("OVERLOAD module %u: peak %.2f A > limit %.2f A, outputs of the module switched off", m + 1, iPeakA[m], iLim);
      }
    }
  }
  telPublish();
}

static bool parseMod(const char* s, uint8_t& mask) {
  if (!strcmp(s, "all")) { mask = 0xFF; return true; }
  char* e;
  long v = strtol(s, &e, 10);
  if (*s == 0 || *e != 0 || v < 1 || v > NMOD) return false;
  mask = (uint8_t)(1u << (v - 1));
  return true;
}
static bool parseCh(const char* s, uint16_t& mask) {
  if (!strcmp(s, "all")) { mask = 0xFFFF; return true; }
  char* e;
  long v = strtol(s, &e, 10);
  if (*s == 0 || *e != 0 || v < 0 || v > 15) return false;
  mask = (uint16_t)(1u << v);
  return true;
}
static void clearOvlCh(uint16_t chmask) { for (uint8_t ch = 0; ch < 16; ch++) if (chmask & (1u << ch)) tOvlMask &= (uint8_t)~(1u << (ch >> 1)); }

static void printHelp() {
  plog("Commands (Test tab):");
  plog(" s | scan | adc | i      status, I2C scan, ADS inputs, currents");
  plog(" r 1|0   oe 1|0   en 1|0   relay, PCA OE, drivers EN");
  plog(" red/green/buz 1|0, blink 0..3");
  plog(" pwm 5 40 | pwm all 40   channel duty %%; sweep 5|all|off; freq 200");
  plog(" izero 1|all   ical 1 2.0   ireset 1|all   ilim 2.0   (current sensors)");
  plog(" cal bus 12.05 | cal fb 12.05 | cal reset   fcal 110 | fcal reset");
  plog(" stop        everything off");
}

static void printStatus() {
  plog("STATUS receiver %s, uptime %lu s, boots %u", FW_VERSION, millis() / 1000UL, (unsigned)bootCount);
  plog("Bus %.2f V (k %.3f), after relay %.2f V (k %.3f)", vBus, kBus, vFb, kFb);
  plog("Relay state %d, OE %s, EN %s, RX E-STOP %s, fault %u", (int)relayState, oeOn ? "ON" : "off", enOn ? "ON" : "off", estopBtn ? "ACTIVE" : "released", (unsigned)faultCode);
  plog("PCA9685 %s, PWM %.0f Hz; ADS1115 0x48 %s, 0x49 %s", pcaOk ? "found" : "NOT FOUND", pwmFreq, adsPresent[0] ? "found" : "NOT FOUND", adsPresent[1] ? "found" : "NOT FOUND");
  plog("Current: ADS samples/s %u + %u, I2C errors %lu + %lu, calibrated %u of 8", (unsigned)pubSps[0], (unsigned)pubSps[1], (unsigned long)adsErr[0], (unsigned long)adsErr[1], (unsigned)__builtin_popcount(calMaskBits()));
  plog("Radio: %s", hlInit ? "OK" : "FAILED");
}

static bool probe(TwoWire& w, uint8_t a) {
  bool one = (&w == &Wire1) && i2c1Mux;
  if (one) xSemaphoreTake(i2c1Mux, portMAX_DELAY);
  w.beginTransmission(a);
  bool ok = (w.endTransmission(true) == 0);
  if (one) xSemaphoreGive(i2c1Mux);
  return ok;
}

static void scanBuses() {
  for (int b = 0; b < 2; b++) {
    TwoWire& w = b ? Wire1 : Wire;
    char buf[160];
    int n = snprintf(buf, sizeof(buf), "BUS%d:", b);
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
      if (probe(w, a) && n < (int)sizeof(buf) - 24) { n += snprintf(buf + n, sizeof(buf) - n, " 0x%02X;", a); found++; }
    }
    if (!found) snprintf(buf + n, sizeof(buf) - n, " nothing found");
    plog("%s", buf);
  }
}

static void printCur() {
  plog("Current per module (peak over 40 ms), samples/s: 0x48 %u, 0x49 %u", (unsigned)pubSps[0], (unsigned)pubSps[1]);
  for (uint8_t m = 0; m < NMOD; m++) {
    char pk[24];
    if (iPeakA[m] >= 0) snprintf(pk, sizeof(pk), "%.2f A", iPeakA[m]); else snprintf(pk, sizeof(pk), "--");
    plog("M%u: duty %.0f pct, peak %s, k %.3f A/V (%s), zero %.4f V%s", m + 1, modDuty(m), pk, iK[m], iCal[m] ? "calibrated" : "NOT calibrated", iZ[m],
         (tOvlMask & (1u << m)) ? "  OVERLOAD (switched off)" : "");
  }
}

static void doCal(const char* which, const char* val) {
  bool bus = !strcmp(which, "bus");
  float real = atof(val);
  float pin = bus ? emaB : emaF;
  if (real < 3.0f || real > 40.0f) { plog("Enter the real voltage in volts, 3..40 (example: cal %s 12.05)", which); return; }
  if (pin < 0.15f) { plog("No voltage on the input now, cannot calibrate"); return; }
  float k = real / pin;
  if (k < 5.0f || k > 20.0f) { plog("Result k=%.2f is out of range 5..20, check the divider and the value", k); return; }
  if (bus) { kBus = k; prefs.putFloat("kbus", k); } else { kFb = k; prefs.putFloat("kfb", k); }
  plog("Calibrated %s: pin %.3f V, real %.2f V, k = %.3f (saved)", bus ? "bus" : "after-relay", pin, real, k);
}

static void execCmd(char* line) {
  for (char* p = line; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;
  char* t[4] = { 0, 0, 0, 0 };
  int n = 0;
  char* p = line;
  while (*p && n < 4) {
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') *p++ = 0;
    if (!*p) break;
    t[n++] = p;
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') p++;
  }
  if (n == 0) return;
  const char* c = t[0];
  const char* a1 = n > 1 ? t[1] : "";
  const char* a2 = n > 2 ? t[2] : "";
  bool on = (a1[0] == '1' || !strcmp(a1, "on"));
  bool arg01 = (a1[0] == '0' || a1[0] == '1' || !strcmp(a1, "on") || !strcmp(a1, "off"));
  bool bad = faultCode || btnLatched;

  if (!strcmp(c, "h") || !strcmp(c, "help") || !strcmp(c, "?")) printHelp();
  else if (!strcmp(c, "s") || !strcmp(c, "status")) printStatus();
  else if (!strcmp(c, "scan")) scanBuses();
  else if (!strcmp(c, "adc")) {
    for (int d = 0; d < 2; d++) {
      char buf[120];
      int k = snprintf(buf, sizeof(buf), "ADS 0x%02X (V):", d ? ADS_ADDR1 : ADS_ADDR0);
      if (!adsPresent[d]) { plog("%s not found", buf); continue; }
      for (int ch = 0; ch < 4; ch++) {
        int m = d * 4 + ch;
        if (lastHave[m]) k += snprintf(buf + k, sizeof(buf) - k, "  A%d=%.3f", ch, lastV[m]);
        else k += snprintf(buf + k, sizeof(buf) - k, "  A%d=--", ch);
      }
      plog("%s", buf);
    }
  }
  else if (!strcmp(c, "i") || !strcmp(c, "ishow")) printCur();
  else if (!strcmp(c, "izero")) {
    uint8_t mask;
    if (!parseMod(a1, mask)) { plog("Usage: izero <1..8|all>  (no load, PWM 0)"); return; }
    if (zeroMask) { plog("Zero measurement is already running"); return; }
    if (mask & modActive()) { plog("REFUSED: PWM of the module is not 0. Send 'pwm all 0' and remove the load first"); return; }
    if (!adsPresent[0] && !adsPresent[1]) { plog("ADS1115 not found"); return; }
    zeroMask = mask; zeroDoneAt = millis() + ZERO_MS;
    plog("Measuring zero offset (no load!) for %s, wait 1.2 s...", a1);
  }
  else if (!strcmp(c, "ical")) {
    uint8_t mask;
    float amps = atof(a2);
    if (!parseMod(a1, mask) || mask == 0xFF || a2[0] == 0 || amps < 0.3f || amps > 6.0f) { plog("Usage: ical <1..8> <A>  (module at PWM 100 pct, electronic load in CC mode)"); return; }
    uint8_t m = 0;
    while (!(mask & (1u << m))) m++;
    if (modDuty(m) < 99.0f) { plog("REFUSED: calibrate at DC. Set PWM 100 pct on channel %u (Extend) or %u (Retract) of module %u", 2 * m, 2 * m + 1, m + 1); return; }
    if (pubN[m] < 5) { plog("M%u: not enough samples (%u), try again in a second", m + 1, (unsigned)pubN[m]); return; }
    float v = pubMeanV[m] - iZ[m];
    if (v < 0.15f) { plog("M%u: signal %.3f V is too small. Is the load connected and ON? (zero offset %.4f V)", m + 1, pubMeanV[m], iZ[m]); return; }
    float k = amps / v;
    if (k < 0.8f || k > 3.5f) { plog("M%u: result %.3f A/V is outside 0.8..3.5, check the IS wiring and the entered current", m + 1, k); return; }
    iK[m] = k; iCal[m] = true;
    char key[8]; keyName(key, "ik", m);
    prefs.putFloat(key, k);
    plog("M%u calibrated: %.3f V at %.2f A, k = %.3f A/V (saved)", m + 1, pubMeanV[m], amps, k);
  }
  else if (!strcmp(c, "ireset")) {
    uint8_t mask;
    if (!parseMod(a1, mask)) { plog("Usage: ireset <1..8|all>"); return; }
    for (uint8_t m = 0; m < NMOD; m++) {
      if (!(mask & (1u << m))) continue;
      char k[8];
      keyName(k, "ik", m); prefs.remove(k);
      keyName(k, "iz", m); prefs.remove(k);
      iK[m] = K_DEF; iZ[m] = 0; iCal[m] = false;
    }
    plog("Current calibration reset for %s", a1);
  }
  else if (!strcmp(c, "ilim")) {
    float v = atof(a1);
    if (v < 0.3f || v > 6.0f) { plog("Usage: ilim 2.0  (Test tab current limit, A, 0.3..6; now %.2f)", iLim); return; }
    iLim = v; prefs.putFloat("ilim", v); plog("Test current limit %.2f A (saved)", v);
  }
  else if (!strcmp(c, "stop")) { testRelay = testOe = testEn = false; manRed = manGreen = manBuz = false; blinkMode = 0; hardOff(); relayOpen(millis()); plog("ALL OFF"); }
  else if (!strcmp(c, "r") || !strcmp(c, "relay")) {
    if (!arg01) { plog("Usage: r 1 or r 0"); return; }
    if (on && bad) { plog("REFUSED: alarm or fault is active"); return; }
    testRelay = on;
    plog("Relay %s, checking feedback...", on ? "ON" : "OFF");
  }
  else if (!strcmp(c, "oe")) {
    if (!arg01) { plog("Usage: oe 1 or oe 0"); return; }
    if (on && bad) { plog("REFUSED: alarm or fault is active"); return; }
    testOe = on; plog("PCA9685 outputs %s", on ? "ENABLED (OE low)" : "disabled (OE high)");
  }
  else if (!strcmp(c, "en")) {
    if (!arg01) { plog("Usage: en 1 or en 0"); return; }
    if (on && bad) { plog("REFUSED: alarm or fault is active"); return; }
    testEn = on; plog("EN line %s", on ? "HIGH (drivers enabled)" : "LOW (drivers disabled)");
  }
  else if (!strcmp(c, "red") || !strcmp(c, "green") || !strcmp(c, "buz")) {
    if (!arg01) { plog("Usage: %s 1 or %s 0", c, c); return; }
    blinkMode = 0;
    if (!strcmp(c, "red")) manRed = on; else if (!strcmp(c, "green")) manGreen = on; else manBuz = on;
    plog("%s %s", c, on ? "ON" : "off");
  }
  else if (!strcmp(c, "blink")) {
    int m = atoi(a1);
    if (a1[0] < '0' || a1[0] > '3') { plog("Usage: blink 0..3"); return; }
    blinkMode = (uint8_t)m; blinkT0 = millis();
    if (m == 0) manRed = manGreen = manBuz = false;
    plog("Blink mode %d", m);
  }
  else if (!strcmp(c, "pwm")) {
    uint16_t mask;
    if (!parseCh(a1, mask) || a2[0] == 0) { plog("Usage: pwm <0..15|all> <duty 0..100>"); return; }
    float d = atof(a2);
    if (d < 0 || d > 100) { plog("Duty must be 0..100"); return; }
    if (!pcaOk) { plog("PCA9685 not found"); return; }
    sweepMask &= (uint16_t)~mask;
    if (d > 0) clearOvlCh(mask);
    if (mask == 0xFFFF) pcaSetAll(d); else for (uint8_t i = 0; i < 16; i++) if (mask & (1u << i)) pcaSetCh(i, d);
    plog("PWM %s = %.1f pct at %.0f Hz%s", a1, d, pwmFreq, oeOn ? "" : "  (OE is off: send 'oe 1' to see it on the outputs)");
  }
  else if (!strcmp(c, "sweep")) {
    if (!strcmp(a1, "off")) { for (uint8_t i = 0; i < 16; i++) if (sweepMask & (1u << i)) pcaSetCh(i, 0); sweepMask = 0; plog("Sweep stopped"); return; }
    uint16_t mask;
    if (!parseCh(a1, mask)) { plog("Usage: sweep <0..15|all> or sweep off"); return; }
    if (!pcaOk) { plog("PCA9685 not found"); return; }
    if (!sweepMask) sweepT0 = millis();
    sweepMask |= mask; clearOvlCh(mask);
    plog("Sweep %s: duty 10-90-10 pct, period 10 s", a1);
  }
  else if (!strcmp(c, "freq")) {
    float f = atof(a1);
    if (f < 24 || f > 1500) { plog("Usage: freq 24..1500"); return; }
    pcaSetFreq(f); plog("PWM frequency %.0f Hz (Test tab only; working frequency comes from the PWM FREQ parameter)", pwmFreq);
  }
  else if (!strcmp(c, "fcal")) {
    if (!strcmp(a1, "reset")) { oscHz = 25000000.0f; prefs.putFloat("osc", oscHz); pcaSetFreq(pwmFreq); plog("PWM oscillator reset to 25.000 MHz"); }
    else {
      float meas = atof(a1);
      if (!pcaOk) { plog("PCA9685 not found"); return; }
      if (meas < 10 || meas > 2000) { plog("Usage: fcal <measured Hz>  (set a frequency first, measure it, then fcal 110)"); return; }
      float nominal = oscHz / (4096.0f * (pcaPre + 1));
      float osc = oscHz * meas / nominal;
      if (osc < 20e6f || osc > 32e6f) { plog("Result %.2f MHz is out of range 20..32, check the measured value", osc / 1e6f); return; }
      oscHz = osc; prefs.putFloat("osc", oscHz); pcaSetFreq(pwmFreq);
      plog("PWM oscillator calibrated: %.3f MHz (saved), now measure again", oscHz / 1e6f);
    }
  }
  else if (!strcmp(c, "cal")) {
    if (!strcmp(a1, "reset")) { kBus = K_BUS_DEF; kFb = K_FB_DEF; prefs.putFloat("kbus", kBus); prefs.putFloat("kfb", kFb); plog("Voltage calibration reset to defaults"); }
    else if (!strcmp(a1, "bus") || !strcmp(a1, "fb")) doCal(a1, a2);
    else plog("Usage: cal bus 12.05 | cal fb 12.05 | cal reset");
  }
  else plog("Unknown command '%s', send h", c);
}

// ============================================================
// Wi-Fi: страница (Update + Test), загрузка файла, состояния
// ============================================================
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>RECEIVER</title>
<style>
:root{--bg:#10151c;--card:#1a222d;--line:#2c3848;--tx:#e8edf3;--mut:#8b9ab0;--ok:#2fbf71;--bad:#e5484d;--acc:#3b82f6}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--tx);font:15px/1.35 system-ui,sans-serif}
.w{max-width:680px;margin:0 auto;padding:10px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px;margin-bottom:10px}
h2{margin:0 0 8px;font-size:12px;letter-spacing:.07em;text-transform:uppercase;color:var(--mut)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:6px}
button{background:#263243;color:var(--tx);border:1px solid var(--line);border-radius:10px;padding:9px 12px;font-size:15px;min-height:44px;flex:1 1 auto}
button:active{filter:brightness(1.3)}
button:disabled{opacity:.45}
button.on{background:var(--ok);border-color:var(--ok);color:#04210f;font-weight:700}
button.r.on{background:var(--bad);border-color:var(--bad);color:#fff}
button.stop{background:var(--bad);border-color:var(--bad);color:#fff;font-weight:800;font-size:18px;width:100%;min-height:52px;margin-top:6px}
button.sm{min-height:34px;padding:4px 10px;font-size:13px;flex:0 0 auto}
button.tab{border-radius:10px 10px 0 0;font-weight:700}
button.tab.on{background:var(--acc);border-color:var(--acc);color:#fff}
input[type=number],input[type=text]{background:#0d1219;color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:9px;font-size:16px;width:96px}
input[type=range]{flex:1 1 120px;min-width:100px;height:30px}
.chip{padding:6px 12px;border-radius:20px;font-weight:700;font-size:14px;background:#263243;flex:1 1 auto;text-align:center}
.chip.ok{background:var(--ok);color:#04210f}
.chip.bad{background:var(--bad);color:#fff}
.vv>div{flex:1 1 140px;background:#0d1219;border-radius:10px;padding:8px 12px}
.vv small{display:block;color:var(--mut);font-size:12px}
.vv b{font-size:30px;font-variant-numeric:tabular-nums}
.mut,small{color:var(--mut);font-size:13px}
.ch{display:flex;align-items:center;gap:8px;margin-bottom:4px}
.ch .n{width:34px;color:var(--mut);font-size:13px}
.ch .p{width:44px;text-align:right;font-variant-numeric:tabular-nums;font-size:13px}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
td,th{border:1px solid var(--line);padding:6px;text-align:center;font-size:14px}
th{color:var(--mut);font-weight:600}
#log{background:#000;color:#6f6;font:12px/1.3 monospace;height:34vh;overflow:auto;white-space:pre-wrap;padding:6px;border:1px solid var(--line);border-radius:8px}
.warn{border-color:var(--bad)}
.warn li{margin-bottom:4px}
.hid{display:none}
progress{width:100%;height:18px}
</style></head><body><div class="w">

<div class="card">
 <div class="row"><div id="es" class="chip">E-STOP ...</div><div id="lnk" class="chip">link ...</div></div>
 <div class="row"><small id="inf"></small></div>
 <div class="row"><button class="tab on" id="tu" onclick="tab('u')">Update</button><button class="tab" id="tt" onclick="tab('t')">Test</button></div>
</div>

<div id="pu">
 <div class="card"><h2>Firmware update</h2>
  <div class="row"><small id="dev"></small></div>
  <div class="row"><input type="file" id="f" accept=".bin"></div>
  <div class="row"><button id="go" disabled>Upload and install</button></div>
  <progress id="pg" value="0" max="100" class="hid"></progress>
  <div id="ust" class="mut"></div>
  <small>Outputs are off, the relay is open. The receiver restarts after the install. Only files of this project (receiver) are accepted.</small>
 </div>
</div>

<div id="pt" class="hid">
 <div class="card warn" id="wrn"><h2>Warning: Test mode</h2>
  <ul>
   <li>Outputs and the relay become ACTIVE. Hydraulics may move.</li>
   <li>Nobody must be near the valves or the machine.</li>
   <li>Never apply 12 V while the board is powered from USB.</li>
   <li>The emergency button stays working. Leaving this tab, closing the page or losing the phone switches everything off.</li>
  </ul>
  <div class="row"><label><input type="checkbox" id="ck"> I understand</label></div>
  <div class="row"><button id="cf" disabled>Confirm</button><button onclick="tab('u')">Cancel</button></div>
 </div>
 <div id="tp" class="hid">
<div class="card"><button class="stop" onclick="cmd('stop')">STOP ALL</button></div>

<div class="card"><h2>Voltages</h2>
 <div class="row vv"><div><small>Bus</small><b id="vb">-</b> V</div><div><small>After relay</small><b id="vf">-</b> V</div></div>
 <div class="mut" id="rm"></div>
 <div class="row" style="margin-top:8px"><input id="cv" type="number" step="0.01" value="12.00" inputmode="decimal">
  <button onclick="cal('bus')">Calibrate bus</button><button onclick="cal('fb')">Calibrate after relay</button></div>
 <small>Measure with a multimeter, type the real voltage, press the button. (After-relay needs the relay ON.) k: <span id="k"></span></small>
 <button class="sm" onclick="cmd('cal reset')">Reset k</button>
</div>

<div class="card"><h2>Power</h2>
 <div class="row"><button id="t_rl" onclick="tog('rl','r')">Relay</button>
  <button id="t_oe" onclick="tog('oe','oe')">PCA OE</button>
  <button id="t_en" onclick="tog('en','en')">Drivers EN</button></div>
 <small>Relay: LOW on GPIO21 = ON (feedback is checked). OE: PCA9685 outputs. EN: all driver enables.</small>
</div>

<div class="card"><h2>Beacon and buzzer</h2>
 <div class="row"><button id="t_rd" class="r" onclick="tog('rd','red')">Red</button>
  <button id="t_gn" onclick="tog('gn','green')">Green</button>
  <button id="t_bz" onclick="tog('bz','buz')">Buzzer</button></div>
 <div class="row"><button id="bl0" onclick="cmd('blink 0')">Off</button><button id="bl1" onclick="cmd('blink 1')">Green 1 Hz</button>
  <button id="bl2" onclick="cmd('blink 2')">Red 4 Hz + buzzer</button><button id="bl3" onclick="cmd('blink 3')">Red/green</button></div>
</div>

<div class="card"><h2>PWM (PCA9685)</h2>
 <div class="row"><small>Frequency, Hz</small><input id="fq" type="number" min="24" max="1500" value="100"><button onclick="cmd('freq '+$('fq').value)">Set</button></div>
 <div class="row"><small>Scope shows, Hz</small><input id="fm" type="number" step="0.1" placeholder="e.g. 110"><button onclick="cmd('fcal '+$('fm').value)">Calibrate freq</button><button class="sm" onclick="cmd('fcal reset')">Reset</button></div>
 <small>Set 100 Hz, measure the real frequency on the scope, type it here, press Calibrate freq. Oscillator: <span id="osc"></span> MHz</small>
 <div class="row" style="margin-top:6px"><button id="swall" onclick="cmd('sweep all')">Sweep all 10-90%</button><button onclick="cmd('sweep off')">Sweep stop</button></div>
 <div class="ch"><span class="n">ALL</span><input id="sa" type="range" min="0" max="100" value="0"><span class="p" id="pa">0%</span><button class="sm" onclick="cmd('pwm all '+$('sa').value)">Set</button></div>
 <div id="chs"></div>
 <small>Outputs are visible only with PCA OE ON.</small>
</div>

<div class="card"><h2>Current sensors (IS)</h2>
 <table id="ct"><tr><th>Mod</th><th>Duty</th><th>Peak, A</th><th>k, A/V</th><th>State</th></tr></table>
 <small id="ci"></small>
 <div class="row" style="margin-top:8px"><small>Module</small><select id="cm" style="background:#0d1219;color:#e8edf3;border:1px solid #2c3848;border-radius:8px;padding:8px;font-size:16px"></select>
  <button onclick="cmd('pwm '+(2*($('cm').value-1))+' 100')">PWM 100%</button><button onclick="cmd('pwm all 0')">PWM 0</button></div>
 <div class="row"><button onclick="cmd('izero '+$('cm').value)">1. Measure zero (no load)</button></div>
 <div class="row"><small>Load current, A</small><input id="ca" type="number" step="0.01" value="2.00" inputmode="decimal"><button onclick="cmd('ical '+$('cm').value+' '+$('ca').value)">2. Calibrate (PWM 100%, load ON)</button></div>
 <div class="row"><small>Test limit, A</small><input id="cl2" type="number" step="0.1" value="2.0" inputmode="decimal"><button onclick="cmd('ilim '+$('cl2').value)">Set limit</button><button class="sm" onclick="cmd('ireset '+$('cm').value)">Reset module</button></div>
 <small>Procedure: pick the module, press 1 with the solenoids and load disconnected. Then connect the electronic load in CC mode to the module output, press PWM 100%, enter the current shown by the load, press 2. Values are saved in flash and used by the working firmware. Peak = highest reading in the last 40 ms. Valid only when the PWM pulse is long enough (about 20% at 100 Hz).</small>
</div>

<div class="card"><h2>ADS1115 inputs (volts, latest sample)</h2>
 <table><tr><th></th><th>A0</th><th>A1</th><th>A2</th><th>A3</th></tr>
 <tr><th>0x48</th><td id="a0">-</td><td id="a1">-</td><td id="a2">-</td><td id="a3">-</td></tr>
 <tr><th>0x49</th><td id="a4">-</td><td id="a5">-</td><td id="a6">-</td><td id="a7">-</td></tr></table>
 <small id="devs"></small>
</div>

<div class="card"><h2>Log</h2>
 <div id="log"></div>
 <div class="row" style="margin-top:8px"><button class="sm" onclick="cmd('s')">Status</button><button class="sm" onclick="cmd('scan')">I2C scan</button><button class="sm" onclick="cmd('h')">Commands</button>
 <input id="c" type="text" style="flex:1 1 140px;width:auto" autocomplete="off" autocapitalize="off" placeholder="command"><button class="sm" onclick="go()">Send</button></div>
</div>
 </div>
</div>
</div>
<script>
var $=function(i){return document.getElementById(i)},S=null,drag=[],lastSend=[],off=0,busy=false,fails=0,cur='u';
function send(c){return fetch('/cmd?c='+encodeURIComponent(c)).catch(function(){})}
function cmd(c){send(c).then(function(){setTimeout(tick,120)})}
function tog(k,c){if(S)cmd(c+' '+(S[k]?0:1))}
function cal(w){cmd('cal '+w+' '+$('cv').value)}
function go(){var v=$('c').value;if(v){cmd(v);$('c').value=''}}
$('c').addEventListener('keydown',function(e){if(e.key==='Enter')go()});
function tab(t){cur=t;$('tu').className='tab'+(t==='u'?' on':'');$('tt').className='tab'+(t==='t'?' on':'');
 $('pu').className=t==='u'?'':'hid';$('pt').className=t==='t'?'':'hid';
 if(t==='u')fetch('/tab?t=u').catch(function(){});setTimeout(tick,100)}
$('ck').onchange=function(){$('cf').disabled=!this.checked};
$('cf').onclick=function(){fetch('/testok',{method:'POST'}).then(function(r){return r.text()}).then(function(t){if(t!=='ok'){alert('Not possible now: '+t)}setTimeout(tick,100)}).catch(function(){})};
$('f').onchange=function(){$('go').disabled=!this.files.length;$('ust').textContent=this.files.length?this.files[0].name+', '+this.files[0].size+' bytes':''};
$('go').onclick=function(){var f=$('f').files[0];if(!f)return;$('go').disabled=true;$('pg').className='';$('pg').value=0;$('ust').textContent='Uploading...';
 var x=new XMLHttpRequest(),d=new FormData();d.append('f',f,'fw.bin');x.open('POST','/update?size='+f.size);
 x.upload.onprogress=function(e){if(e.lengthComputable)$('pg').value=Math.round(e.loaded*100/e.total)};
 x.onload=function(){try{var j=JSON.parse(x.responseText);$('ust').textContent=j.ok?'Installed. The receiver restarts, close this page.':('Failed: '+j.err)}catch(e){$('ust').textContent='Failed'}$('go').disabled=false};
 x.onerror=function(){$('ust').textContent='Connection lost';$('go').disabled=false};x.send(d)};
var H='';for(var i=0;i<16;i++){H+='<div class="ch"><span class="n">CH'+i+'</span><input type="range" min="0" max="100" value="0" id="s'+i+'" oninput="sl('+i+',this.value)" onchange="fin('+i+',this.value)"><span class="p" id="p'+i+'">0%</span><button class="sm" id="w'+i+'" onclick="swp('+i+')">Sweep</button></div>'}
$('chs').innerHTML=H;
var T='',O='';for(var i=0;i<8;i++){T+='<tr><td>M'+(i+1)+'</td><td id="cd'+i+'">-</td><td id="ca'+i+'">-</td><td id="ck'+i+'">-</td><td id="cs'+i+'">-</td></tr>';O+='<option value="'+(i+1)+'">Module '+(i+1)+'</option>'}
$('ct').insertAdjacentHTML('beforeend',T);$('cm').innerHTML=O;
$('sa').oninput=function(){$('pa').textContent=this.value+'%'};
function sl(i,v){drag[i]=Date.now()+1500;$('p'+i).textContent=v+'%';var t=Date.now();if(t-(lastSend[i]||0)>120){lastSend[i]=t;send('pwm '+i+' '+v)}}
function fin(i,v){drag[i]=Date.now()+1500;send('pwm '+i+' '+v).then(function(){setTimeout(tick,150)})}
function swp(i){if(!S)return;if(S.sw&(1<<i))cmd('pwm '+i+' '+Math.round(S.pw[i]));else cmd('sweep '+i)}
function on(id,f){$(id).className=$(id).className.replace(/ ?\bon\b/,'')+(f?' on':'')}
function render(d){S=d;fails=0;
 $('lnk').textContent='link OK';$('lnk').className='chip ok';
 $('es').textContent=d.es?'E-STOP ACTIVE':'E-STOP OK';$('es').className='chip '+(d.es?'bad':'ok');
 $('inf').textContent='uptime '+d.up+' s, receiver '+d.fw+(d.fl?', FAULT '+d.fl:'');
 $('dev').textContent='Device: receiver, firmware '+d.fw;
 if(d.tk){$('wrn').className='card warn hid';$('tp').className=''}else{$('wrn').className='card warn';$('tp').className='hid';if(cur==='t'&&!d.tk){}}
 if(!d.tk)return;
 $('vb').textContent=d.vb.toFixed(2);$('vf').textContent=d.vf.toFixed(2);$('rm').textContent=d.rm;
 $('k').textContent='bus '+d.kb.toFixed(3)+', after relay '+d.kf.toFixed(3);
 ['rl','oe','en','rd','gn','bz'].forEach(function(k){on('t_'+k,d[k])});
 for(var b=0;b<4;b++)on('bl'+b,d.bl===b);
 on('swall',d.sw===65535);
 for(var i=0;i<16;i++){if(Date.now()>(drag[i]||0)){$('s'+i).value=Math.round(d.pw[i]);$('p'+i).textContent=Math.round(d.pw[i])+'%'}on('w'+i,(d.sw>>i)&1)}
 for(var a=0;a<8;a++)$('a'+a).textContent=d.ad[a]===null?'--':d.ad[a].toFixed(3);
 for(var m=0;m<8;m++){var du=Math.max(d.pw[2*m],d.pw[2*m+1]),st='idle',cl='',cal=(d.ic>>m)&1;
  if((d.ov>>m)&1){st='OVERLOAD';cl='color:#e5484d;font-weight:700'}
  else if((d.zm>>m)&1){st='zero...';cl='color:#e0b030'}
  else if(du>0){if(du/100/d.fq*1000<2.0){st='PWM too low';cl='color:#e0b030'}else if(!cal){st='not calibrated';cl='color:#e0b030'}else{st='ok';cl='color:#2fbf71'}}
  else if(!cal){st='not calibrated';cl='color:#8b9ab0'}
  $('cd'+m).textContent=du>0?Math.round(du)+'%':'-';
  $('ca'+m).textContent=d.ia[m]===null?'--':d.ia[m].toFixed(2);
  $('ck'+m).textContent=d.ik[m].toFixed(3)+(cal?'':'*');
  $('cs'+m).textContent=st;$('cs'+m).style.cssText=cl}
 $('ci').textContent='ADS samples/s: '+d.sps[0]+' + '+d.sps[1]+', I2C errors '+d.er[0]+' + '+d.er[1]+', limit '+d.lim.toFixed(2)+' A. * = default k, not calibrated.';
 if(document.activeElement!==$('cl2'))$('cl2').value=d.lim.toFixed(1);
 $('devs').textContent='PCA9685 '+(d.pca?'found':'NOT FOUND')+', ADS 0x48 '+(d.ap[0]?'found':'NOT FOUND')+', ADS 0x49 '+(d.ap[1]?'found':'NOT FOUND');
 $('osc').textContent=d.os.toFixed(3);
 if(document.activeElement!==$('fq'))$('fq').value=d.fq}
function tick(){if(busy)return;busy=true;
 fetch('/state').then(function(r){return r.json()}).then(render).catch(function(){if(++fails>2){$('lnk').textContent='NO LINK';$('lnk').className='chip bad'}}).then(function(){busy=false})}
function poll(){fetch('/log?from='+off).then(function(r){return r.text()}).then(function(t){
 var i=t.indexOf('\n'),L=$('log');off=parseInt(t.slice(0,i));var d=t.slice(i+1);
 if(d){L.textContent+=d;if(L.textContent.length>20000)L.textContent=L.textContent.slice(-12000);L.scrollTop=L.scrollHeight}
 }).catch(function(){}).then(function(){setTimeout(poll,700)})}
fetch('/info').catch(function(){});
setInterval(tick,400);tick();poll();
</script></body></html>)HTML";

static char     upErr[40] = "";
static char     upVer[16] = "";
static size_t   upTotal = 0, upDone = 0;
static bool     upActive = false, upOk = false, upChecked = false, upFound = false;
static bool     pageSeen = false;
static unsigned long wifiT0 = 0, wifiAct = 0;
static bool     wifiRoutes = false;
static char     scanPre[24];
static size_t   scanPreLen = 0;
static uint8_t  scanBuf[1536 + 64];
static uint8_t  scanTail[48];
static size_t   scanTailLen = 0;

static void upFail(const char* msg) { snprintf(upErr, sizeof(upErr), "%s", msg); Update.abort(); }

// ищет в потоке метку "FWTAG:receiver:<версия>;" (может оказаться на стыке двух блоков)
static void upScan(const uint8_t* d, size_t n, bool last) {
  if (upFound) return;
  const size_t need = scanPreLen + 12;
  size_t carry = scanTailLen;
  if (carry + n > sizeof(scanBuf)) n = sizeof(scanBuf) - carry;
  memcpy(scanBuf, scanTail, carry);
  if (n) memcpy(scanBuf + carry, d, n);
  size_t tot = carry + n;
  size_t lim = 0;
  if (last) lim = (tot > scanPreLen) ? tot - scanPreLen : 0;
  else      lim = (tot >= need) ? tot - need + 1 : 0;
  for (size_t i = 0; i < lim && !upFound; i++) {
    if (scanBuf[i] != 'F' || memcmp(scanBuf + i, scanPre, scanPreLen) != 0) continue;
    uint8_t c = scanBuf[i + scanPreLen];
    if (c < '0' || c > '9') continue;
    size_t k = 0, j = i + scanPreLen;
    while (j < tot && scanBuf[j] != ';' && scanBuf[j] != 0 && k < sizeof(upVer) - 1) upVer[k++] = (char)scanBuf[j++];
    upVer[k] = 0;
    if (j < tot && scanBuf[j] == ';') upFound = true;
  }
  size_t keep = need - 1;
  if (keep > tot) keep = tot;
  memcpy(scanTail, scanBuf + tot - keep, keep);
  scanTailLen = keep;
}

static void wifiClose(const char* why);
static void wifiOpenPhone();

static void hRoot() { wifiAct = millis(); server.sendHeader("Cache-Control", "no-store"); server.send_P(200, "text/html; charset=utf-8", PAGE); }
static void hInfo() { pageSeen = true; wifiAct = millis(); server.send(200, "application/json", "{\"dev\":\"" PROJECT_NAME "\",\"ver\":\"" FW_VERSION "\"}"); }
static void hLog() {
  wifiAct = millis();
  uint32_t from = (uint32_t)strtoul(server.arg("from").c_str(), nullptr, 10);
  uint32_t oldest = logTotal > LOG_SZ ? logTotal - LOG_SZ : 0;
  if (from < oldest) from = oldest;
  if (from > logTotal) from = logTotal;
  String out;
  out += String((unsigned long)logTotal);
  out += "\n";
  if (testOK) for (uint32_t i = from; i < logTotal; i++) { char ch[2] = { logBuf[i % LOG_SZ], 0 }; out += ch; }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", out.c_str());
}

static void jf(char* b, int& n, int cap, const char* name, const volatile float* a, int cnt, const char* fmt, bool nullIfNeg) {
  n += snprintf(b + n, cap - n, "\"%s\":[", name);
  for (int i = 0; i < cnt && n < cap - 24; i++) {
    float v = a[i];
    if (nullIfNeg && v < 0) n += snprintf(b + n, cap - n, "%snull", i ? "," : "");
    else { n += snprintf(b + n, cap - n, i ? "," : ""); n += snprintf(b + n, cap - n, fmt, v); }
  }
  n += snprintf(b + n, cap - n, "],");
}

static void hState() {
  wifiAct = millis();
  lastPoll = wifiAct;
  static char b[2600];
  const int cap = sizeof(b);
  bool es = estopBtn || btnLatched;
  int n = snprintf(b, cap,
    "{\"up\":%lu,\"tk\":%d,\"fl\":%u,\"vb\":%.2f,\"vf\":%.2f,\"kb\":%.3f,\"kf\":%.3f,\"rl\":%d,\"oe\":%d,\"en\":%d,\"es\":%d,"
    "\"rd\":%d,\"gn\":%d,\"bz\":%d,\"bl\":%d,\"fq\":%.0f,\"os\":%.3f,\"sw\":%u,\"pca\":%d,\"ap\":[%d,%d],\"rm\":\"%s\","
    "\"lim\":%.2f,\"ov\":%u,\"ic\":%u,\"zm\":%u,\"sps\":[%u,%u],\"er\":[%lu,%lu],\"fw\":\"%s\",\"pw\":[",
    millis() / 1000UL, testOK ? 1 : 0, (unsigned)faultCode, vBus, vFb, kBus, kFb, (relayState == RS_ON || relayState == RS_CLOSING) ? 1 : 0, oeOn, enOn, es,
    outR, outG, outB, blinkMode, pwmFreq, oscHz / 1e6f, (unsigned)sweepMask, pcaOk, adsPresent[0], adsPresent[1], relayMsg,
    iLim, (unsigned)tOvlMask, (unsigned)calMaskBits(), (unsigned)zeroMask, (unsigned)pubSps[0], (unsigned)pubSps[1], (unsigned long)adsErr[0], (unsigned long)adsErr[1], FW_VERSION);
  for (int i = 0; i < 16 && n < cap - 20; i++) n += snprintf(b + n, cap - n, "%s%.1f", i ? "," : "", pwmDuty[i]);
  n += snprintf(b + n, cap - n, "],");
  jf(b, n, cap, "ia", iPeakA, NMOD, "%.2f", true);
  jf(b, n, cap, "ik", iK, NMOD, "%.3f", false);
  n += snprintf(b + n, cap - n, "\"ad\":[");
  for (int i = 0; i < 8 && n < cap - 20; i++) {
    if (lastHave[i]) n += snprintf(b + n, cap - n, "%s%.3f", i ? "," : "", lastV[i]);
    else n += snprintf(b + n, cap - n, "%snull", i ? "," : "");
  }
  snprintf(b + n, cap - n, "]}");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

static void hCmd() {
  wifiAct = millis();
  if (!testOK) { server.send(403, "text/plain", "test not confirmed"); return; }
  char buf[96];
  strncpy(buf, server.arg("c").c_str(), sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  plog("> %s", buf);
  execCmd(buf);
  server.send(200, "text/plain", "ok");
}

static void hTestOk() {
  wifiAct = millis();
  unsigned long now = millis();
  if (wifiSt != HLW_PH_CONN && wifiSt != HLW_PH_TEST) { server.send(200, "text/plain", "Wi-Fi mode"); return; }
  if (faultCode)  { server.send(200, "text/plain", "receiver fault"); return; }
  if (btnLatched) { server.send(200, "text/plain", "emergency button"); return; }
  if (!hlSafeNow(now)) { server.send(200, "text/plain", "levers not at zero"); return; }
  if (!testOK) {
    testOK = true; lastPoll = now;
    wifiSt = HLW_PH_TEST;
    testRelay = testOe = testEn = false;
    hardOff();
    plog("Test mode ON (confirmed on the page)");
  }
  server.send(200, "text/plain", "ok");
}

static void hTab() {
  wifiAct = millis();
  if (server.arg("t") == "u" && testOK) testOff("tab changed");
  server.send(200, "text/plain", "ok");
}

static void hUploadFile() {
  HTTPUpload& up = server.upload();
  wifiAct = millis();
  if (up.status == UPLOAD_FILE_START) {
    if (testOK) testOff("upload");
    upActive = true; upOk = false; upChecked = false; upFound = false;
    upErr[0] = 0; upVer[0] = 0; upDone = 0; scanTailLen = 0;
    upTotal = (size_t)server.arg("size").toInt();
    snprintf(scanPre, sizeof(scanPre), "FWTAG:%s:", PROJECT_NAME);
    scanPreLen = strlen(scanPre);
    wifiSt = HLW_PH_UP; wifiArg = 0;
    plog("Wi-Fi Update: upload started");
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (upErr[0]) return;
    if (!upChecked) {
      const uint8_t* b = up.buf;
      if (up.currentSize < 64 || b[0] != 0xE9 || (b[12] | (b[13] << 8)) != 9) { upFail("not an ESP32-S3 image"); return; }
      if (!Update.begin(upTotal ? upTotal : UPDATE_SIZE_UNKNOWN)) { upFail(Update.errorString()); return; }
      upChecked = true;
    }
    upScan(up.buf, up.currentSize, false);
    if (Update.write(up.buf, up.currentSize) != up.currentSize) { upFail(Update.errorString()); return; }
    upDone += up.currentSize;
    if (upTotal) wifiArg = (uint8_t)(upDone * 100 / upTotal > 100 ? 100 : upDone * 100 / upTotal);
  } else if (up.status == UPLOAD_FILE_END) {
    if (!upErr[0]) {
      upScan(nullptr, 0, true);
      if (!upChecked)        upFail("empty file");
      else if (!upFound)     upFail("wrong firmware (not receiver)");
      else if (Update.end(true)) upOk = true;
      else                   upFail(Update.errorString());
    }
    upActive = false;
    plog("Wi-Fi Update: upload end, %s %s", upOk ? "OK" : "FAIL", upOk ? upVer : upErr);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (upChecked) Update.abort();
    upActive = false;
    snprintf(upErr, sizeof(upErr), "aborted");
  }
}

static void hUpdateDone() {
  wifiAct = millis();
  char b[96];
  if (upOk) {
    server.send(200, "application/json", "{\"ok\":true}");
    wifiSt = HLW_PH_DONE; wifiArg = 100;
    restartAt = millis() + 2500UL;
  } else {
    snprintf(b, sizeof(b), "{\"ok\":false,\"err\":\"%s\"}", upErr[0] ? upErr : "unknown");
    server.send(200, "application/json", b);
    wifiSt = HLW_PH_CONN; wifiArg = 0;
  }
}

static void hOk204() { server.send(204, "text/plain", ""); }
static void hOkApple() { server.send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"); }
static void hOkMsft() { server.send(200, "text/plain", "Microsoft Connect Test"); }
static void hOkNcsi() { server.send(200, "text/plain", "Microsoft NCSI"); }
static void hRedirect() { server.sendHeader("Location", "http://" AP_ADDR "/", true); server.send(302, "text/plain", ""); }

static void wifiOpenPhone() {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(apSsid, sizeof(apSsid), "RX-%02X%02X", (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSsid, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  if (!wifiRoutes) {
    server.on("/", HTTP_GET, hRoot);
    server.on("/info", HTTP_GET, hInfo);
    server.on("/state", HTTP_GET, hState);
    server.on("/log", HTTP_GET, hLog);
    server.on("/cmd", HTTP_GET, hCmd);
    server.on("/testok", HTTP_POST, hTestOk);
    server.on("/tab", HTTP_GET, hTab);
    server.on("/update", HTTP_POST, hUpdateDone, hUploadFile);
    server.on("/generate_204", HTTP_GET, hOk204);
    server.on("/gen_204", HTTP_GET, hOk204);
    server.on("/hotspot-detect.html", HTTP_GET, hOkApple);
    server.on("/library/test/success.html", HTTP_GET, hOkApple);
    server.on("/connecttest.txt", HTTP_GET, hOkMsft);
    server.on("/ncsi.txt", HTTP_GET, hOkNcsi);
    server.onNotFound(hRedirect);
    wifiRoutes = true;
  }
  server.begin();
  pageSeen = false; upActive = false; upOk = false; upErr[0] = 0;
  wifiT0 = millis(); wifiAct = wifiT0; lastPoll = wifiT0;
  wifiSt = HLW_PH_WAIT; wifiArg = 0;
  infoAt = 0;
  hardOff();
  relayOpen(wifiT0);
  testOK = false;
  plog("Wi-Fi (Update/Test): on, net %s, pass %s, %s", apSsid, AP_PASS, AP_ADDR);
}

// ============================================================
// Обновление с GitHub (сеть и пароль приходят от пульта по радио, NET)
// ============================================================
enum { G_OFF, G_SCAN, G_CONN, G_API, G_ASK, G_DL, G_OK, G_ERR };
static uint8_t ghStage = G_OFF;
static unsigned long ghT0 = 0;
static char   ghVer[24] = "", ghSha[65] = "", ghUrl[256] = "", ghNet[33] = "";
static size_t ghSize = 0, ghDone = 0;
static bool   ghBegan = false, ghYes = false;
static int    ghReason = 0;
static uint8_t ghBuf[1024];

static void ghWifiEvt(arduino_event_id_t ev, arduino_event_info_t info) {
  if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) ghReason = info.wifi_sta_disconnected.reason;
}
static bool ghAuthReason(int r) { return r == 15 || r == 202 || r == 204; }

static void ghFail(uint8_t code, const char* why) {
  if (ghBegan) { Update.abort(); ghBegan = false; }
  WiFi.disconnect();
  ghStage = G_ERR;
  ghT0 = millis();
  wifiSt = HLW_ERR; wifiArg = code;
  plog("GitHub: error %u (%s)", (unsigned)code, why);
}

static void ghStart(unsigned long now) {
  ghVer[0] = ghSha[0] = ghUrl[0] = ghNet[0] = 0; rxVer[0] = 0;
  ghSize = ghDone = 0; ghBegan = false; ghYes = false; ghReason = 0;
  ghT0 = now;
  testOK = false;
  hardOff();
  relayOpen(now);
  wifiSt = HLW_GH_CONN; wifiArg = 0;
  int cnt = 0;
  for (int i = 0; i < NET_MAX; i++) if (netSsid[i][0]) cnt++;
  if (cnt == 0) { ghFail(WE_NONET, "no saved network received from the pult"); return; }
  ghStage = G_SCAN;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.scanNetworks(true);
  plog("GitHub Update: start");
}

static bool ghParse(const String& body) {
  int pos = 0;
  while (true) {
    int a = body.indexOf("\"name\":\"" PROJECT_NAME "_", pos);
    if (a < 0) return false;
    int nb = a + 8;
    int ne = body.indexOf('"', nb);
    if (ne < 0) return false;
    pos = ne;
    String nm = body.substring(nb, ne);
    if (!nm.endsWith(".bin") || nm.indexOf("_full") >= 0) continue;
    String ver = nm.substring(strlen(PROJECT_NAME) + 1, nm.length() - 4);
    if (ver.length() == 0 || ver.length() >= sizeof(ghVer)) continue;
    int sz = body.indexOf("\"size\":", ne);
    int dg = body.indexOf("\"digest\":\"sha256:", ne);
    int ur = body.indexOf("\"browser_download_url\":\"", ne);
    if (sz < 0 || dg < 0 || ur < 0) continue;
    long size = body.substring(sz + 7).toInt();
    String sha = body.substring(dg + 17, dg + 17 + 64);
    int ue = body.indexOf('"', ur + 24);
    if (size <= 0 || sha.length() != 64 || ue < 0) continue;
    if (body.indexOf('"', dg + 17) != dg + 17 + 64) continue;
    bool hexOk = true;
    for (int i = 0; i < 64; i++) { char c = sha.c_str()[i]; if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) hexOk = false; }
    if (!hexOk) continue;
    String url = body.substring(ur + 24, ue);
    if (!url.startsWith("https://") || url.length() >= sizeof(ghUrl)) continue;
    snprintf(ghVer, sizeof(ghVer), "%s", ver.c_str());
    snprintf(ghSha, sizeof(ghSha), "%s", sha.c_str());
    snprintf(ghUrl, sizeof(ghUrl), "%s", url.c_str());
    ghSize = (size_t)size;
    return true;
  }
}

static void ghCheck() {
  WiFiClientSecure cl;
  cl.setInsecure();
  HTTPClient http;
  http.setUserAgent("FORESTER-receiver");
  http.addHeader("Accept", "application/vnd.github+json");
  http.setTimeout(10000);
  if (!http.begin(cl, "https://api.github.com/repos/" GH_REPO "/releases/tags/" GH_TAG)) { ghFail(WE_GHDOWN, "begin"); return; }
  int code = http.GET();
  if (code <= 0)                  { http.end(); ghFail(WE_GHDOWN, "GitHub not reachable"); return; }
  if (code == 404)                { http.end(); ghFail(WE_NOREL, "release not found"); return; }
  if (code != 200)                { http.end(); ghFail(WE_GHDOWN, "GitHub http error"); return; }
  String body = http.getString();
  http.end();
  if (!ghParse(body)) { ghFail(WE_NOREL, "no firmware in the release"); return; }
  snprintf(rxVer, sizeof(rxVer), "%s", ghVer);
  plog("GitHub: found %s %u B", ghVer, (unsigned)ghSize);
  ghStage = G_ASK; wifiSt = HLW_GH_ASK; wifiArg = 0;
  ghT0 = millis(); ghYes = false; infoAt = 0;
}

static bool ghChunk(const uint8_t* b, size_t n, mbedtls_sha256_context* sha) {
  upScan(b, n, false);
  mbedtls_sha256_update(sha, b, n);
  if (Update.write((uint8_t*)b, n) != n) { ghFail(WE_UPD, Update.errorString()); return false; }
  ghDone += n;
  wifiArg = (uint8_t)(ghDone * 100 / ghSize);
  return true;
}

static void ghDownload() {
  ghStage = G_DL; wifiSt = HLW_GH_DL; wifiArg = 0;
  ghDone = 0;
  upFound = false; upVer[0] = 0; scanTailLen = 0;
  snprintf(scanPre, sizeof(scanPre), "FWTAG:%s:", PROJECT_NAME);
  scanPreLen = strlen(scanPre);
  WiFiClientSecure cl;
  cl.setInsecure();
  HTTPClient http;
  http.setUserAgent("FORESTER-receiver");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(15000);
  if (!http.begin(cl, ghUrl)) { ghFail(WE_DLFAIL, "begin"); return; }
  int code = http.GET();
  if (code != 200) { http.end(); ghFail(WE_DLFAIL, "download http error"); return; }
  int len = http.getSize();
  if (len > 0 && (size_t)len != ghSize) { http.end(); ghFail(WE_DLFAIL, "size mismatch"); return; }
  WiFiClient* st = http.getStreamPtr();
  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);
  bool ok = true;
  uint8_t h[64];
  if (st->readBytes(h, sizeof(h)) != sizeof(h)) { ok = false; ghFail(WE_DLFAIL, "short read"); }
  else if (h[0] != 0xE9 || (h[12] | (h[13] << 8)) != 9) { ok = false; ghFail(WE_WRONG, "not an ESP32-S3 image"); }
  else if (!Update.begin(ghSize)) { ok = false; ghFail(WE_UPD, Update.errorString()); }
  else { ghBegan = true; ok = ghChunk(h, sizeof(h), &sha); }
  unsigned long last = millis();
  while (ok && ghDone < ghSize) {
    size_t av = st->available();
    if (av == 0) {
      if (!http.connected() || (millis() - last) > 15000UL) { ok = false; ghFail(WE_DLFAIL, "stalled"); break; }
      yield();
      continue;
    }
    size_t want = ghSize - ghDone;
    if (want > sizeof(ghBuf)) want = sizeof(ghBuf);
    if (want > av) want = av;
    size_t n = st->readBytes(ghBuf, want);
    if (n == 0) continue;
    last = millis();
    ok = ghChunk(ghBuf, n, &sha);
  }
  http.end();
  uint8_t dig[32];
  mbedtls_sha256_finish(&sha, dig);
  mbedtls_sha256_free(&sha);
  if (!ok) return;
  wifiSt = HLW_GH_VERIFY; wifiArg = 0;
  upScan(nullptr, 0, true);
  char hex[65];
  for (int i = 0; i < 32; i++) snprintf(hex + i * 2, 3, "%02x", dig[i]);
  if (ghDone != ghSize)             { ghFail(WE_DLFAIL, "incomplete"); return; }
  if (strcasecmp(hex, ghSha) != 0)  { ghFail(WE_SUM, "checksum"); return; }
  if (!upFound)                     { ghFail(WE_WRONG, "wrong firmware tag"); return; }
  if (!Update.end(true))            { ghFail(WE_UPD, Update.errorString()); return; }
  ghBegan = false;
  ghStage = G_OK; wifiSt = HLW_GH_DONE; wifiArg = 100;
  restartAt = millis() + 2500UL;
  plog("GitHub Update: installed %s", upVer);
}

static void ghTick(unsigned long now) {
  switch (ghStage) {
    case G_SCAN: {
      int n = WiFi.scanComplete();
      if (n == WIFI_SCAN_RUNNING) { if ((now - ghT0) > 15000UL) ghFail(WE_NONET, "scan timeout"); break; }
      if (n < 0) { ghFail(WE_NONET, "scan failed"); break; }
      int best = -1, bestR = -1000;
      for (int i = 0; i < n; i++) {
        String sd = WiFi.SSID(i);
        int r = WiFi.RSSI(i);
        for (int k = 0; k < NET_MAX; k++)
          if (netSsid[k][0] && sd == netSsid[k] && r > bestR) { best = k; bestR = r; }
      }
      WiFi.scanDelete();
      if (best < 0) { ghFail(WE_NONET, "no saved network in range"); break; }
      snprintf(ghNet, sizeof(ghNet), "%s", netSsid[best]);
      static bool evSet = false;
      if (!evSet) { WiFi.onEvent(ghWifiEvt, ARDUINO_EVENT_WIFI_STA_DISCONNECTED); evSet = true; }
      ghReason = 0;
      WiFi.begin(netSsid[best], netPass[best][0] ? netPass[best] : nullptr);
      ghStage = G_CONN; ghT0 = now;
      break;
    }
    case G_CONN: {
      wl_status_t st = WiFi.status();
      if (st == WL_CONNECTED) { ghStage = G_API; wifiSt = HLW_GH_CHECK; ghT0 = now; }
      else if (st == WL_CONNECT_FAILED || ghAuthReason(ghReason)) ghFail(WE_PASS, ghNet);
      else if ((now - ghT0) > 30000UL) ghFail(WE_NONET, "connect timeout");
      break;
    }
    case G_API:
      ghCheck();
      break;
    case G_ASK:
      if (ghYes) ghDownload();
      else if ((now - ghT0) > 120000UL) wifiClose("no confirmation");
      break;
    case G_ERR:
      if ((now - ghT0) > 60000UL) wifiClose("error timeout");
      break;
    default:
      break;
  }
}

// ============================================================
// Wi-Fi: общее управление состоянием
// ============================================================
static void wifiClose(const char* why) {
  if (testOK) testOff(why);
  server.stop();
  if (ghBegan) { Update.abort(); ghBegan = false; }
  WiFi.scanDelete();
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  wifiSt = HLW_OFF; wifiArg = 0;
  ghStage = G_OFF; ghYes = false;
  pageSeen = false; upActive = false;
  rxVer[0] = 0;
  memset(netSsid, 0, sizeof(netSsid)); memset(netPass, 0, sizeof(netPass));
  lastRunFreq = -1;
  plog("Wi-Fi closed (%s)", why);
}

static void wifiTick(unsigned long now) {
  uint8_t r = wifiReq;
  if (r) {
    wifiReq = 0;
    if (r == 1 && wifiSt == HLW_OFF) wifiOpenPhone();
    else if (r == 2 && wifiSt == HLW_OFF) ghStart(now);
    else if (r == 3 && wifiSt != HLW_OFF) wifiClose("closed from the pult");
    else if (r == 4 && wifiSt == HLW_GH_ASK) ghYes = true;
  }
  if (wifiSt == HLW_OFF) return;
  CtlShared c;
  taskENTER_CRITICAL(&shMux);
  c = ctlSh;
  taskEXIT_CRITICAL(&shMux);
  bool link = c.have && c.rxAt != 0 && (now - c.rxAt) < ramP[RXP_LINKTMO][0][0];
  bool busy = upActive || wifiSt == HLW_PH_UP || wifiSt == HLW_PH_DONE || wifiSt == HLW_GH_DL || wifiSt == HLW_GH_VERIFY || wifiSt == HLW_GH_DONE;
  if ((btnLatched || (link && c.estop)) && !busy) { wifiClose("E-Stop"); return; }
  if (wifiSt >= HLW_PH_WAIT && wifiSt <= HLW_PH_DONE) {
    int st = WiFi.softAPgetStationNum();
    if (wifiSt == HLW_PH_WAIT && (pageSeen || st > 0)) wifiSt = HLW_PH_CONN;
    if (wifiSt == HLW_PH_WAIT && (now - wifiT0) > 120000UL) { wifiClose("no phone"); return; }
    if (!busy && (now - wifiAct) > 300000UL) { wifiClose("idle"); return; }
    if (testOK) {
      if ((now - lastPoll) > 4000UL) testOff("page closed");
      else if (faultCode || btnLatched) testOff("alarm");
    }
  } else {
    ghTick(now);
  }
}

// ============================================================
// Кнопка USER: привязка (долгое нажатие 3 с)
// ============================================================
static void userTick(unsigned long now) {
  static bool prev = false, longDone = false;
  static unsigned long downAt = 0, bindT0 = 0;
  bool u = (digitalRead(PIN_USER) == LOW);
  if (u && !prev) { downAt = now; longDone = false; }
  if (u && !longDone && (now - downAt) >= 3000UL) {
    longDone = true;
    if (bindMode) { bindMode = false; plog("Bind cancelled"); }
    else if (!wifiActive()) { bindMode = true; bindT0 = now; plog("Bind mode ON (60 s): start Bind in the pult menu"); }
  }
  prev = u;
  if (bindMode && (now - bindT0) > 60000UL) { bindMode = false; plog("Bind mode timeout"); }
}

// ============================================================
// setup / loop
// ============================================================
static void pinOutHigh(int pin) { gpio_set_level((gpio_num_t)pin, 1); pinMode(pin, OUTPUT); }
static void pinOutLow(int pin)  { gpio_set_level((gpio_num_t)pin, 0); pinMode(pin, OUTPUT); }

void setup() {
  pinOutHigh(PIN_RELAY);     // реле выключено
  pinOutHigh(PIN_OE);        // выходы PCA9685 выключены
  pinOutLow(PIN_EN);         // драйверы выключены
  pinOutLow(PIN_RED); pinOutLow(PIN_GREEN); pinOutLow(PIN_BUZ);
  pinOutLow(PIN_LED);
  pinMode(PIN_USER, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT);
  pinMode(PIN_VBUS, INPUT);
  pinMode(PIN_VFB, INPUT);
  Serial.begin(115200);

  cfgLoad();
  bindLoad();
  calibLoad();
  runReset();

  Wire.begin(PIN_BUS0_SDA, PIN_BUS0_SCL, I2C_HZ);
  Wire1.begin(PIN_BUS1_SDA, PIN_BUS1_SCL, I2C1_HZ);
  Wire.setTimeOut(50);
  Wire1.setTimeOut(10);
  i2c1Mux = xSemaphoreCreateMutex();

  pwmFreq = (float)ramP[RXP_PWMFREQ][0][0];
  lastRunFreq = pwmFreq;
  pcaInit();
  adsPresent[0] = probe(Wire1, ADS_ADDR0);
  adsPresent[1] = probe(Wire1, ADS_ADDR1);
  plog("RECEIVER %s, boots %u", FW_VERSION, (unsigned)bootCount);
  plog("%s", FW_TAG);
  plog("PCA9685 %s; ADS1115 0x%02X %s, 0x%02X %s; current sensors calibrated: %u of 8%s", pcaOk ? "found" : "NOT FOUND", ADS_ADDR0, adsPresent[0] ? "found" : "NOT FOUND",
       ADS_ADDR1, adsPresent[1] ? "found" : "NOT FOUND", (unsigned)__builtin_popcount(calMaskBits()), anyUncal() ? " (NOT CALIBRATED: default coefficient is used)" : "");
  if (!pcaOk) setFault(3, "PCA9685 not found at start");
  xTaskCreatePinnedToCore(samplerTask, "ads", 6144, nullptr, 2, nullptr, 1);

  estopRaw = (digitalRead(PIN_ESTOP) == HIGH);
  estopBtn = estopRaw;
  btnLatched = estopRaw;
  plog("RX E-STOP %s", estopBtn ? "ACTIVE (pressed or wire broken)" : "released");

  hlBegin();
  xTaskCreatePinnedToCore(radioTask, "radio", 8192, nullptr, 3, nullptr, 0);
}

void loop() {
  unsigned long now = millis();
  if (wifiSt >= HLW_PH_WAIT && wifiSt <= HLW_PH_DONE) server.handleClient();
  powerMeasure(now);
  estopTick(now);
  wifiTick(now);
  controlTick(now);
  lampsTick(now);
  userTick(now);
  outR = digitalRead(PIN_RED); outG = digitalRead(PIN_GREEN); outB = digitalRead(PIN_BUZ);
  if (pcaRestartPend && (long)(now - pcaRestartAt) >= 0) { pcaRestartPend = false; pcaWrite1(0x00, 0xA0); }
  if (!testMode() && lastRunFreq != (float)ramP[RXP_PWMFREQ][0][0]) { lastRunFreq = (float)ramP[RXP_PWMFREQ][0][0]; pcaSetFreq(lastRunFreq); }
  if (cfgSavePending) { cfgSavePending = false; cfgSave(); }
  if (restartAt && (long)(now - restartAt) >= 0) { restartAt = 0; ESP.restart(); }
}
