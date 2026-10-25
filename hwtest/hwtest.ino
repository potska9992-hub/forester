// ============================================================
// ПРИЁМНИК: ТЕСТ ЖЕЛЕЗА И КАЛИБРОВКА (hwtest 1.0), Heltec Wireless Stick Lite V3
// Это не рабочая прошивка приёмника, а проверочная. Её ставят ПЕРВОЙ на каждое изделие: проверка всех узлов
// и калибровка датчиков тока (IS драйверов BTS7960), делителей напряжения и генератора PCA9685.
// РАДИО НЕ ИНИЦИАЛИЗИРУЕТСЯ (антенна не нужна).
//  - плата поднимает Wi-Fi сеть RECV-TEST (пароль 88888888), веб-панель в браузере телефона: http://192.168.4.1
//  - те же команды работают в строке панели и в Serial Monitor (115200); команда h выводит список
//  - сообщения только при изменении состояния
//  - питание 12 В и USB одновременно НЕ подключать: для проверки с 12 В отключите USB и работайте через Wi-Fi
//
// ТОК: АЦП ADS1115 опрашивается непрерывно в отдельной задаче (только модули, где скважность > 0), для каждого
// модуля берётся ПИК за скользящее окно 40 мс (окно постоянное, настройки нет). Пик = ток в импульсе ШИМ.
//
// КАЛИБРОВКА ХРАНИТСЯ В ЭНЕРГОНЕЗАВИСИМОЙ ПАМЯТИ (NVS, namespace "calib"), её читает рабочая прошивка приёмника:
//   ik0..ik7  float  А на вольт для модуля 1..8 (ток = (U - iz) * ik, U = напряжение на входе ADS1115)
//   iz0..iz7  float  нулевое смещение входа, В
//   ilim      float  предел тока, А (по умолчанию 2.0)
//   kbus, kfb float  коэффициенты делителей бортсети и после реле
//   osc       float  реальная частота генератора PCA9685, Гц
//   cver      uint   версия формата (1)
//   Нет ключа ikN = модуль не откалиброван (рабочая прошивка должна об этом предупредить).
//
// Arduino IDE: плата "Heltec Wireless Stick Lite (V3)", Tools -> USB CDC On Boot: Enabled. Библиотеки не нужны.
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <stdarg.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define PROJECT_NAME  "hwtest"      // не "receiver", чтобы защита обновлений по GitHub не приняла проверочную прошивку за прошивку приёмника
#define FW_VERSION    "1.0"
static const char FW_TAG[] __attribute__((used)) = "FWTAG:" PROJECT_NAME ":" FW_VERSION ";";

// ---- Wi-Fi консоли ----
#define AP_SSID   "RECV-TEST"
#define AP_PASS   "88888888"

// ---- Пины ----
#define PIN_BUS0_SDA  2      // Wire : PCA9685
#define PIN_BUS0_SCL  3
#define PIN_BUS1_SDA  4      // Wire1: две ADS1115
#define PIN_BUS1_SCL  5
#define PIN_VBUS      6      // напряжение бортсети (делитель 100к/10к), ADC1
#define PIN_VFB       7      // напряжение после реле (обратная связь), ADC1
#define PIN_RED       15     // мигалка красный   (HIGH = включено, через NPN)
#define PIN_GREEN     16     // мигалка зелёный
#define PIN_BUZ       17     // зуммер
#define PIN_RELAY     21     // реле питания драйверов: LOW = включено, HIGH = выключено
#define PIN_OE        33     // PCA9685 OE: LOW = выходы разрешены, HIGH = выключены
#define PIN_ESTOP     34     // аварийная кнопка NC: HIGH = нажата или обрыв
#define PIN_EN        38     // EN всех драйверов: HIGH = разрешено
#define PIN_LED       35     // белый светодиод платы
#define PIN_USER      0      // кнопка USER, LOW = нажата

#define I2C_HZ        100000        // шина 0 (PCA9685)
#define I2C1_HZ       400000        // шина 1 (ADS1115): быстрее, чтобы опрос АЦП не терял время
#define PCA_ADDR      0x40
#define ADS_ADDR0     0x48
#define ADS_ADDR1     0x49

// ---- Коэффициенты делителей по умолчанию (меняются командой cal) ----
#define K_BUS_DEF     11.09f
#define K_FB_DEF      10.96f
#define V_DEADBAND    0.8f   // ниже этого напряжения показываем 0.0 (смещение нуля АЦП)

// ---- Измерение тока ----
#define NMOD          8             // модулей драйверов (каналов тока); модуль n: PCA выходы 2n (Extend) и 2n+1 (Retract)
#define ADS_LSB       0.000125f     // В на отсчёт, диапазон +-4.096 В
#define ADS_CONV_US   1400          // ожидание конца преобразования 860 SPS (1.16 мс + запас на разброс тактов АЦП)
#define IDLE_SCAN_MS  25            // пауза между опросами, когда на устройстве нет активных модулей (для таблицы входов)
#define BK_MS         5             // ширина корзины окна
#define BK_N          8             // корзин: окно 8 x 5 мс = 40 мс (постоянное)
#define K_DEF         1.70f         // А на вольт по умолчанию (расчёт для 8500 / 5 кОм), пока модуль не откалиброван
#define I_LIM_DEF     2.0f          // предел тока по умолчанию, А
#define OVL_WINDOWS   3             // сколько проверок подряд (каждые 40 мс) пик выше предела до отключения модуля
#define MIN_ON_MS     2.0f          // минимальная длительность импульса для верного измерения (сам АЦП 1.16 мс + запас)
#define ZERO_MS       1200          // время набора нуля командой izero

// типы заданы здесь, до первой функции (иначе сборщик Arduino вставит прототипы раньше типов)
struct Bk { uint32_t ep; float mx; float sum; uint16_t n; };
static Bk bk[NMOD][BK_N];
struct SamplerState {
  int8_t   cur[2]     = { -1, -1 };      // канал, который сейчас преобразуется (-1 = ничего)
  int64_t  startUs[2] = { 0, 0 };
  uint8_t  rr[2]      = { 0, 0 };
  int64_t  idleAt[2]  = { 0, 0 };
  uint32_t cnt[2]     = { 0, 0 };
  uint32_t errRun[2]  = { 0, 0 };
  int64_t  secAt = 0, pubAt = 0;
};

// ---- Журнал ----
#define LOG_SZ        4096
static char     logBuf[LOG_SZ];
static uint32_t logTotal = 0;

static void logRaw(const char* s) {
  while (*s) { logBuf[logTotal % LOG_SZ] = *s++; logTotal++; }
}

static void plog(const char* fmt, ...) {
  char msg[240];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  unsigned long s = millis() / 1000UL;
  char line[280];
  snprintf(line, sizeof(line), "[%02lu:%02lu] %s\n", (s / 60UL) % 100UL, s % 60UL, msg);
  logRaw(line);
  if (Serial) Serial.print(line);
}

char  relayMsg[100] = "-";          // последний результат проверки реле

static void relayResult(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(relayMsg, sizeof(relayMsg), fmt, ap);
  va_end(ap);
  plog("%s", relayMsg);
}

// ---- Состояние ----
WebServer server(80);
Preferences prefs;

bool estop = false, estopRaw = false;
unsigned long estopRawAt = 0;
bool relayOn = false, oeOn = false, enOn = false;
bool manRed = false, manGreen = false, manBuz = false;
uint8_t blinkMode = 0;
unsigned long blinkT0 = 0;
float pwmFreq = 100.0f;
float oscHz = 25000000.0f;          // реальная частота генератора PCA9685 (калибруется командой fcal)
int   pcaPre = 60;                  // текущий предделитель
uint16_t sweepMask = 0;
unsigned long sweepT0 = 0, sweepAt = 0;
bool pcaOk = false, pcaRestartPend = false;
unsigned long pcaRestartAt = 0;

float kBus = K_BUS_DEF, kFb = K_FB_DEF;
float emaB = 0, emaF = 0;           // сглаженное напряжение на пине, В
bool  adcInit = false;
unsigned long adcAt = 0;
float vBus = 0, vFb = 0;
float lastRepBus = -100, lastRepFb = -100;
unsigned long firstRepAt = 0;
float settleBus = 0, settleFb = 0;
unsigned long settleBusAt = 0, settleFbAt = 0;

int  checkState = 0;                // 0 нет, 1 ждём ON, 2 ждём OFF
unsigned long checkDeadline = 0, relayChangeAt = 0;
int  warnState = 0;

unsigned long ledAt = 0, userAt = 0, userDownAt = 0, stationAt = 0;
bool ledOn = false, userPrev = false, userLongDone = false;
int  lastStations = 0;

// ---- Измерение тока (сэмплер ADS1115 работает в своей задаче, loop читает только опубликованные значения) ----
bool     adsPresent[2] = { false, false };
SemaphoreHandle_t i2c1Mux = nullptr;                  // шина 1 (Wire1) используют сэмплер и команда scan
volatile uint8_t  wantMask = 0;                       // модули, которые надо опрашивать непрерывно (скважность > 0 или набор нуля)
volatile float    pubPeakV[NMOD], pubMeanV[NMOD];     // пик и среднее за окно 40 мс, В на входе АЦП
volatile uint16_t pubN[NMOD];                         // сколько выборок в окне (0 = данных нет)
volatile float    lastV[NMOD];                        // последняя выборка, В (для таблицы входов)
volatile bool     lastHave[NMOD];
volatile uint16_t pubSps[2];                          // выборок в секунду на каждом ADS1115
volatile uint32_t adsErr[2];                          // ошибки обмена

float iK[NMOD], iZ[NMOD];                             // калибровка модулей (А/В и нулевое смещение, В)
bool  iCal[NMOD];                                     // коэффициент k откалиброван (есть в NVS)
float iLim = I_LIM_DEF;
uint8_t ovlMask = 0;                                  // модули, отключённые по перегрузке (до новой команды pwm)
uint8_t ovlCnt[NMOD];
uint8_t zeroMask = 0;                                 // идёт набор нуля
unsigned long zeroDoneAt = 0, ovlAt = 0;
float iPeakA[NMOD];                                   // пик тока модуля, А (для панели); -1 = нет данных

float pwmDuty[16];                                    // текущая скважность каналов для панели
bool  outR = false, outG = false, outB = false;   // фактическое состояние красной, зелёной, зуммера

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
  pcaWrite1(0x00, 0x10);            // сон
  pcaWrite1(0xFE, (uint8_t)pre);    // предделитель
  pcaWrite1(0x00, 0x20);            // проснуться, автоинкремент
  pcaRestartPend = true;
  pcaRestartAt = millis() + 2;      // осциллятору нужно время, потом restart
}

static void duty2bytes(float duty, uint8_t* b) {
  long v = lroundf(duty * 40.96f);
  if (v <= 0)         { b[0] = 0; b[1] = 0;    b[2] = 0; b[3] = 0x10; }
  else if (v >= 4096) { b[0] = 0; b[1] = 0x10; b[2] = 0; b[3] = 0; }
  else                { b[0] = 0; b[1] = 0;    b[2] = (uint8_t)(v & 0xFF); b[3] = (uint8_t)(v >> 8); }
}

static void pcaSetCh(uint8_t ch, float duty) {
  if (!pcaOk || ch > 15) return;
  pwmDuty[ch] = duty;
  uint8_t b[4];
  duty2bytes(duty, b);
  pcaWriteBytes(0x06 + 4 * ch, b, 4);
}

static void pcaSetAll(float duty) {
  if (!pcaOk) return;
  for (uint8_t i = 0; i < 16; i++) pwmDuty[i] = duty;
  uint8_t b[4];
  duty2bytes(duty, b);
  pcaWriteBytes(0xFA, b, 4);
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
// Выходы
// ============================================================
static void setOe(bool on) {
  oeOn = on;
  digitalWrite(PIN_OE, on ? LOW : HIGH);
}

static void setEn(bool on) {
  enOn = on;
  digitalWrite(PIN_EN, on ? HIGH : LOW);
}

static void setRelay(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, on ? LOW : HIGH);
  relayChangeAt = millis();
  checkState = on ? 1 : 2;
  snprintf(relayMsg, sizeof(relayMsg), "checking...");
  checkDeadline = relayChangeAt + (on ? 800UL : 2500UL);
}

static void allOff(const char* why) {
  bool anything = relayOn || oeOn || enOn || sweepMask || manRed || manGreen || manBuz || blinkMode;
  sweepMask = 0;
  pcaSetAll(0);
  setOe(false);
  setEn(false);
  if (relayOn) { setRelay(false); plog("Relay OFF, checking..."); }
  blinkMode = 0; manRed = manGreen = manBuz = false;
  plog("ALL OFF (%s)%s", why, anything ? "" : " - already off");
}

// ============================================================
// Вывод справки, статуса, сканера, ADS
// ============================================================
static void printHelp() {
  plog("Commands:");
  plog(" h               this list");
  plog(" s               full status");
  plog(" scan            I2C scan of both buses");
  plog(" r 1 | r 0       relay on/off (checks feedback)");
  plog(" oe 1 | oe 0     PCA9685 outputs enable/disable");
  plog(" en 1 | en 0     driver EN line");
  plog(" red/green/buz 1|0   lamp and buzzer");
  plog(" blink 0..3      0 off, 1 green 1Hz, 2 red 4Hz+buzzer, 3 red/green");
  plog(" pwm 5 40        channel 5 fixed duty 40 pct; pwm all 40");
  plog(" sweep 5|all     duty 10-90-10 pct, period 10 s; sweep off");
  plog(" freq 200        PWM frequency, Hz (default 100)");
  plog(" cal bus 12.05   calibrate bus voltage (real value, V)");
  plog(" cal fb 12.05    calibrate after-relay voltage");
  plog(" cal show | cal reset");
  plog(" fcal 110        PWM oscillator calibration: enter the frequency measured on the scope");
  plog(" fcal reset      back to nominal 25 MHz");
  plog(" adc             latest voltage of all 8 ADS1115 inputs");
  plog(" i               current of all modules (peak over 40 ms) and calibration");
  plog(" izero 1|all     measure sensor zero (PWM 0, no load), saved");
  plog(" ical 1 2.0      calibrate module 1: PWM 100 pct, load CC, type the load current in A");
  plog(" ireset 1|all    reset current calibration");
  plog(" ilim 2.0        current limit, A (overload switches the module off; only calibrated modules)");
  plog(" stop            everything off");
  plog("USER button: short = relay toggle, long 2 s = stop");
}

static void printStatus() {
  plog("STATUS hwtest %s, uptime %lu s", FW_VERSION, millis() / 1000UL);
  plog("Bus %.2f V (k %.3f), after relay %.2f V (k %.3f)", vBus, kBus, vFb, kFb);
  plog("Relay %s, OE %s, EN %s, E-STOP %s", relayOn ? "ON" : "off", oeOn ? "ON" : "off", enOn ? "ON" : "off",
       estop ? "ACTIVE (pressed or wire broken)" : "released");
  plog("Red %d, green %d, buzzer %d, blink mode %d", manRed, manGreen, manBuz, blinkMode);
  plog("PCA9685 %s, PWM %.0f Hz, sweep mask 0x%04X", pcaOk ? "found" : "NOT FOUND", pwmFreq, sweepMask);
  plog("ADS1115 0x%02X %s, 0x%02X %s", ADS_ADDR0, adsPresent[0] ? "found" : "NOT FOUND", ADS_ADDR1, adsPresent[1] ? "found" : "NOT FOUND");
  plog("Current: limit %.2f A, ADS samples/s %u + %u, I2C errors %lu + %lu", iLim, (unsigned)pubSps[0], (unsigned)pubSps[1], (unsigned long)adsErr[0], (unsigned long)adsErr[1]);
  plog("Wi-Fi clients: %d", WiFi.softAPgetStationNum());
}

static const char* devName(uint8_t a) {
  if (a == 0x40) return "PCA9685";
  if (a == 0x70) return "PCA9685 ALLCALL";
  if (a >= 0x48 && a <= 0x4B) return "ADS1115";
  if (a == 0x3C || a == 0x3D) return "OLED";
  if (a >= 0x20 && a <= 0x27) return "MCP23017";
  return "unknown";
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
    char buf[200];
    int n = snprintf(buf, sizeof(buf), "BUS%d:", b);
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
      if (probe(w, a) && n < (int)sizeof(buf) - 40) { n += snprintf(buf + n, sizeof(buf) - n, " 0x%02X %s;", a, devName(a)); found++; }
    }
    if (!found) snprintf(buf + n, sizeof(buf) - n, " nothing found");
    plog("%s", buf);
  }
  pcaOk = probe(Wire, PCA_ADDR);
  adsPresent[0] = probe(Wire1, ADS_ADDR0);
  adsPresent[1] = probe(Wire1, ADS_ADDR1);
}

// ============================================================
// Сэмплер тока: ADS1115 в отдельной задаче
// ============================================================
// Одноразовые преобразования 860 SPS, конвейер: пока один АЦП преобразует, обслуживается второй.
// Опрашиваются только модули из wantMask (скважность > 0), по кругу. Если на устройстве активных нет, один вход
// опрашивается раз в IDLE_SCAN_MS (чтобы в таблице видеть входы без тока).

static bool i1Lock() { return i2c1Mux && xSemaphoreTake(i2c1Mux, portMAX_DELAY) == pdTRUE; }
static void i1Unlock() { if (i2c1Mux) xSemaphoreGive(i2c1Mux); }

static bool adsStart(uint8_t addr, uint8_t ch) {
  uint16_t cfg = 0x8000 | ((uint16_t)(4 + ch) << 12) | 0x0200 | 0x0100 | 0x00E0 | 0x0003;   // AINch/GND, +-4.096 V, single, 860 SPS
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

// один проход сэмплера (в задаче вызывается по кругу)
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
    // выбрать следующий вход
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
    // ждать конец преобразования: сначала отдать процессор на тик, остаток добрать короткой паузой
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

static uint8_t modActive() {
  uint8_t mask = 0;
  for (uint8_t m = 0; m < NMOD; m++) if (pwmDuty[2 * m] > 0.0f || pwmDuty[2 * m + 1] > 0.0f) mask |= (1u << m);
  return mask;
}

static float modDuty(uint8_t m) { return pwmDuty[2 * m] > pwmDuty[2 * m + 1] ? pwmDuty[2 * m] : pwmDuty[2 * m + 1]; }

static float voltToAmp(uint8_t m, float v) { float a = (v - iZ[m]) * iK[m]; return a < 0 ? 0 : a; }

// ============================================================
// Команды
// ============================================================
// ---- ток: команды и контроль ----
static bool parseMod(const char* s, uint8_t& mask) {          // модуль 1..8 или all
  if (!strcmp(s, "all")) { mask = 0xFF; return true; }
  char* e;
  long v = strtol(s, &e, 10);
  if (*s == 0 || *e != 0 || v < 1 || v > NMOD) return false;
  mask = (uint8_t)(1u << (v - 1));
  return true;
}

static void keyName(char* k, const char* base, uint8_t m) { snprintf(k, 8, "%s%u", base, (unsigned)m); }

static void clearOvlCh(uint16_t chmask) {
  for (uint8_t ch = 0; ch < 16; ch++) if (chmask & (1u << ch)) ovlMask &= (uint8_t)~(1u << (ch >> 1));
}

static uint8_t calMaskBits() { uint8_t m = 0; for (uint8_t i = 0; i < NMOD; i++) if (iCal[i]) m |= (1u << i); return m; }

static void printAdc() {
  for (int d = 0; d < 2; d++) {
    char buf[160];
    int n = snprintf(buf, sizeof(buf), "ADS 0x%02X (latest, V):", d ? ADS_ADDR1 : ADS_ADDR0);
    if (!adsPresent[d]) { plog("%s not found", buf); continue; }
    for (int c = 0; c < 4; c++) {
      int m = d * 4 + c;
      if (lastHave[m]) n += snprintf(buf + n, sizeof(buf) - n, "  A%d=%.3f", c, lastV[m]);
      else n += snprintf(buf + n, sizeof(buf) - n, "  A%d=--", c);
    }
    plog("%s", buf);
  }
}

static void printCur() {
  plog("Current per module (peak over 40 ms), limit %.2f A, samples/s: ADS0x48 %u, ADS0x49 %u", iLim, (unsigned)pubSps[0], (unsigned)pubSps[1]);
  for (uint8_t m = 0; m < NMOD; m++) {
    char pk[24];
    if (iPeakA[m] >= 0) snprintf(pk, sizeof(pk), "%.2f A", iPeakA[m]); else snprintf(pk, sizeof(pk), "--");
    plog("M%u: duty %.0f pct, peak %s, k %.3f A/V (%s), zero %.4f V%s", m + 1, modDuty(m), pk, iK[m], iCal[m] ? "calibrated" : "NOT calibrated", iZ[m],
         (ovlMask & (1u << m)) ? "  OVERLOAD (switched off)" : "");
  }
}

static void cmdZero(const char* a1) {
  uint8_t mask;
  if (!parseMod(a1, mask)) { plog("Usage: izero <1..8|all>  (no load, PWM 0: measures the sensor zero offset)"); return; }
  if (zeroMask) { plog("Zero measurement is already running"); return; }
  uint8_t act = modActive();
  if (mask & act) { plog("REFUSED: PWM of the module is not 0. Send 'pwm all 0' and remove the load first"); return; }
  if (!adsPresent[0] && !adsPresent[1]) { plog("ADS1115 not found"); return; }
  zeroMask = mask;
  zeroDoneAt = millis() + ZERO_MS;
  plog("Measuring zero offset (no load!) for %s, wait 1.2 s...", a1);
}

static void finishZero() {
  uint8_t mask = zeroMask;
  zeroMask = 0;
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

static void cmdIcal(const char* a1, const char* a2) {
  uint8_t mask;
  float amps = atof(a2);
  if (!parseMod(a1, mask) || mask == 0xFF || a2[0] == 0 || amps < 0.3f || amps > 6.0f) {
    plog("Usage: ical <1..8> <A>  (module at PWM 100 pct, electronic load in CC mode, type the current shown by the load)");
    return;
  }
  uint8_t m = 0;
  while (!(mask & (1u << m))) m++;
  if (modDuty(m) < 99.0f) { plog("REFUSED: calibrate at DC. Set PWM 100 pct on channel %u (Extend) or %u (Retract) of module %u", 2 * m, 2 * m + 1, m + 1); return; }
  if (pubN[m] < 5) { plog("M%u: not enough samples (%u), try again in a second", m + 1, (unsigned)pubN[m]); return; }
  float v = pubMeanV[m] - iZ[m];
  if (v < 0.15f) { plog("M%u: signal %.3f V is too small. Is the load connected and ON? Is the IS wire connected? (zero offset %.4f V)", m + 1, pubMeanV[m], iZ[m]); return; }
  float k = amps / v;
  if (k < 0.8f || k > 3.5f) { plog("M%u: result %.3f A/V is outside 0.8..3.5, check the IS wiring and the entered current", m + 1, k); return; }
  iK[m] = k; iCal[m] = true;
  char key[8]; keyName(key, "ik", m);
  prefs.putFloat(key, k);
  plog("M%u calibrated: %.3f V at %.2f A, k = %.3f A/V, zero %.4f V (saved)", m + 1, pubMeanV[m], amps, k, iZ[m]);
}

static void cmdIreset(const char* a1) {
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

static void currentTick(unsigned long now) {
  uint8_t act = modActive();
  wantMask = act | zeroMask;
  for (uint8_t m = 0; m < NMOD; m++) iPeakA[m] = ((act & (1u << m)) && pubN[m] > 0) ? voltToAmp(m, pubPeakV[m]) : -1.0f;
  if (zeroMask && (long)(now - zeroDoneAt) >= 0) finishZero();
  if ((now - ovlAt) >= 40) {
    ovlAt = now;
    for (uint8_t m = 0; m < NMOD; m++) {
      bool over = iCal[m] && (act & (1u << m)) && pubN[m] >= 2 && iPeakA[m] > iLim;
      if (!over) { ovlCnt[m] = 0; continue; }
      if (++ovlCnt[m] >= OVL_WINDOWS && !(ovlMask & (1u << m))) {
        ovlMask |= (1u << m);
        pcaSetCh(2 * m, 0); pcaSetCh(2 * m + 1, 0);
        sweepMask &= (uint16_t)~(3u << (2 * m));
        plog("OVERLOAD module %u: peak %.2f A > limit %.2f A, outputs of the module switched off", m + 1, iPeakA[m], iLim);
      }
    }
  }
}

static bool parseCh(const char* s, uint16_t& mask) {
  if (!strcmp(s, "all")) { mask = 0xFFFF; return true; }
  char* e;
  long v = strtol(s, &e, 10);
  if (*s == 0 || *e != 0 || v < 0 || v > 15) return false;
  mask = (uint16_t)(1u << v);
  return true;
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
  lastRepBus = lastRepFb = -100;
  plog("Calibrated %s: pin %.3f V, real %.2f V, k = %.3f (saved)", bus ? "bus" : "after-relay", pin, real, k);
}

static void execCmd(char* line) {
  // нижний регистр, обрезка пробелов, разбор на слова
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

  if (!strcmp(c, "h") || !strcmp(c, "help") || !strcmp(c, "?")) { printHelp(); }
  else if (!strcmp(c, "s") || !strcmp(c, "status")) { printStatus(); }
  else if (!strcmp(c, "scan")) { scanBuses(); }
  else if (!strcmp(c, "adc")) { printAdc(); }
  else if (!strcmp(c, "ishow") || !strcmp(c, "i")) { printCur(); }
  else if (!strcmp(c, "izero")) { cmdZero(a1); }
  else if (!strcmp(c, "ical")) { cmdIcal(a1, a2); }
  else if (!strcmp(c, "ireset")) { cmdIreset(a1); }
  else if (!strcmp(c, "ilim")) {
    float v = atof(a1);
    if (v < 0.3f || v > 6.0f) { plog("Usage: ilim 2.0  (current limit, A, 0.3..6; now %.2f)", iLim); return; }
    iLim = v; prefs.putFloat("ilim", v); plog("Current limit %.2f A (saved)", v);
  }
  else if (!strcmp(c, "stop")) { allOff("stop command"); }
  else if (!strcmp(c, "r") || !strcmp(c, "relay")) {
    if (!arg01) { plog("Usage: r 1 or r 0"); return; }
    if (on && estop) { plog("REFUSED: E-STOP is active"); return; }
    if (on == relayOn) { plog("Relay already %s", on ? "ON" : "off"); return; }
    setRelay(on);
    plog("Relay %s, checking feedback...", on ? "ON" : "OFF");
  }
  else if (!strcmp(c, "oe")) {
    if (!arg01) { plog("Usage: oe 1 or oe 0"); return; }
    if (on && estop) { plog("REFUSED: E-STOP is active"); return; }
    setOe(on);
    plog("PCA9685 outputs %s", on ? "ENABLED (OE low)" : "disabled (OE high)");
  }
  else if (!strcmp(c, "en")) {
    if (!arg01) { plog("Usage: en 1 or en 0"); return; }
    if (on && estop) { plog("REFUSED: E-STOP is active"); return; }
    setEn(on);
    plog("EN line %s", on ? "HIGH (drivers enabled)" : "LOW (drivers disabled)");
  }
  else if (!strcmp(c, "red") || !strcmp(c, "green") || !strcmp(c, "buz")) {
    if (!arg01) { plog("Usage: %s 1 or %s 0", c, c); return; }
    blinkMode = 0;
    if (!strcmp(c, "red")) manRed = on; else if (!strcmp(c, "green")) manGreen = on; else manBuz = on;
    plog("%s %s", c, on ? "ON" : "off");
  }
  else if (!strcmp(c, "blink")) {
    int m = atoi(a1);
    if (a1[0] < '0' || a1[0] > '3' || m < 0 || m > 3) { plog("Usage: blink 0..3"); return; }
    blinkMode = (uint8_t)m;
    blinkT0 = millis();
    if (m == 0) { manRed = manGreen = manBuz = false; }
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
    if (!strcmp(a1, "off")) { if (sweepMask) { for (uint8_t i = 0; i < 16; i++) if (sweepMask & (1u << i)) pcaSetCh(i, 0); sweepMask = 0; plog("Sweep stopped, channels set to 0"); } else plog("Sweep is not running"); return; }
    uint16_t mask;
    if (!parseCh(a1, mask)) { plog("Usage: sweep <0..15|all> or sweep off"); return; }
    if (!pcaOk) { plog("PCA9685 not found"); return; }
    if (!sweepMask) sweepT0 = millis();
    sweepMask |= mask;
    clearOvlCh(mask);
    plog("Sweep %s: duty 10-90-10 pct, period 10 s, %.0f Hz%s", a1, pwmFreq, oeOn ? "" : "  (OE is off: send 'oe 1')");
  }
  else if (!strcmp(c, "freq")) {
    float f = atof(a1);
    if (f < 24 || f > 1500) { plog("Usage: freq 24..1500"); return; }
    pcaSetFreq(f);
    plog("PWM frequency %.0f Hz", pwmFreq);
  }
  else if (!strcmp(c, "fcal")) {
    if (!strcmp(a1, "reset")) { oscHz = 25000000.0f; prefs.putFloat("osc", oscHz); pcaSetFreq(pwmFreq); plog("PWM oscillator reset to 25.000 MHz, frequency set to %.0f Hz", pwmFreq); }
    else {
      float meas = atof(a1);
      if (!pcaOk) { plog("PCA9685 not found"); return; }
      if (meas < 10 || meas > 2000) { plog("Usage: fcal <measured Hz>  (set a frequency first, e.g. freq 100, measure it, then fcal 110)"); return; }
      float nominal = oscHz / (4096.0f * (pcaPre + 1));          // частота, которую мы рассчитывали получить
      float osc = oscHz * meas / nominal;                         // реальный генератор
      if (osc < 20e6f || osc > 32e6f) { plog("Result %.2f MHz is out of range 20..32, check the measured value", osc / 1e6f); return; }
      oscHz = osc;
      prefs.putFloat("osc", oscHz);
      pcaSetFreq(pwmFreq);
      plog("PWM oscillator calibrated: %.3f MHz (saved), frequency set to %.0f Hz, now measure it again", oscHz / 1e6f, pwmFreq);
    }
  }
  else if (!strcmp(c, "cal")) {
    if (!strcmp(a1, "show")) { plog("k bus %.3f, k after-relay %.3f", kBus, kFb); }
    else if (!strcmp(a1, "reset")) { kBus = K_BUS_DEF; kFb = K_FB_DEF; prefs.putFloat("kbus", kBus); prefs.putFloat("kfb", kFb); lastRepBus = lastRepFb = -100; plog("Calibration reset to defaults (%.2f / %.2f)", kBus, kFb); }
    else if (!strcmp(a1, "bus") || !strcmp(a1, "fb")) { doCal(a1, a2); }
    else plog("Usage: cal bus 12.05 | cal fb 12.05 | cal show | cal reset");
  }
  else { plog("Unknown command '%s', send h", c); }
}

// ============================================================
// Веб-консоль
// ============================================================
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>RECV-TEST</title>
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
button.on{background:var(--ok);border-color:var(--ok);color:#04210f;font-weight:700}
button.r.on{background:var(--bad);border-color:var(--bad);color:#fff}
button.stop{background:var(--bad);border-color:var(--bad);color:#fff;font-weight:800;font-size:18px;width:100%;min-height:52px;margin-top:6px}
button.sm{min-height:34px;padding:4px 10px;font-size:13px;flex:0 0 auto}
input[type=number]{background:#0d1219;color:var(--tx);border:1px solid var(--line);border-radius:8px;padding:9px;font-size:16px;width:96px}
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
</style></head><body><div class="w">

<div class="card">
 <div class="row"><div id="es" class="chip">E-STOP ...</div><div id="lnk" class="chip">link ...</div></div>
 <div class="row"><small id="inf"></small></div>
 <button class="stop" onclick="cmd('stop')">STOP ALL</button>
</div>

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
 <small>Outputs are visible only with PCA OE ON. Sweep: 10 to 90 to 10 percent, 10 s period.</small>
</div>

<div class="card"><h2>Current sensors (IS)</h2>
 <table id="ct"><tr><th>Mod</th><th>Duty</th><th>Peak, A</th><th>k, A/V</th><th>State</th></tr></table>
 <small id="ci"></small>
 <div class="row" style="margin-top:8px"><small>Module</small><select id="cm" style="background:#0d1219;color:#e8edf3;border:1px solid #2c3848;border-radius:8px;padding:8px;font-size:16px"></select>
  <button onclick="cmd('pwm '+(2*($('cm').value-1))+' 100')">PWM 100%</button><button onclick="cmd('pwm all 0')">PWM 0</button></div>
 <div class="row"><button onclick="cmd('izero '+$('cm').value)">1. Measure zero (no load)</button></div>
 <div class="row"><small>Load current, A</small><input id="ca" type="number" step="0.01" value="2.00" inputmode="decimal"><button onclick="cmd('ical '+$('cm').value+' '+$('ca').value)">2. Calibrate (PWM 100%, load ON)</button></div>
 <div class="row"><small>Limit, A</small><input id="cl2" type="number" step="0.1" value="2.0" inputmode="decimal"><button onclick="cmd('ilim '+$('cl2').value)">Set limit</button><button class="sm" onclick="cmd('ireset '+$('cm').value)">Reset module</button></div>
 <small>Procedure: pick the module, press 1 with the solenoids and load disconnected. Then connect the electronic load in CC mode to the module output, press PWM 100%, enter the current shown by the load, press 2. Values are saved in flash and used by the receiver firmware. Peak = highest reading in the last 40 ms. Valid only when the PWM pulse is long enough (about 20% at 100 Hz); the overload limit works only for calibrated modules.</small>
</div>

<div class="card"><h2>ADS1115 inputs (volts, latest sample)</h2>
 <table><tr><th></th><th>A0</th><th>A1</th><th>A2</th><th>A3</th></tr>
 <tr><th>0x48</th><td id="a0">-</td><td id="a1">-</td><td id="a2">-</td><td id="a3">-</td></tr>
 <tr><th>0x49</th><td id="a4">-</td><td id="a5">-</td><td id="a6">-</td><td id="a7">-</td></tr></table>
 <small id="devs"></small>
</div>

<div class="card"><h2>E-STOP test</h2>
 <div id="esb" class="chip">-</div>
 <small>Press the emergency button: this must turn red and everything switches off. Release: green. Unplug its wire: also red.</small>
</div>

<div class="card"><h2>Log</h2>
 <div id="log"></div>
 <div class="row" style="margin-top:8px"><button class="sm" onclick="cmd('s')">Status</button><button class="sm" onclick="cmd('scan')">I2C scan</button><button class="sm" onclick="cmd('h')">Commands</button>
 <input id="c" type="text" style="flex:1 1 140px;background:#0d1219;color:#e8edf3;border:1px solid #2c3848;border-radius:8px;padding:8px;font-size:16px" autocomplete="off" autocapitalize="off" placeholder="command"><button class="sm" onclick="go()">Send</button></div>
</div>
</div>
<script>
var $=function(i){return document.getElementById(i)},S=null,drag=[],lastSend=[],off=0,busy=false,fails=0;
function send(c){return fetch('/cmd?c='+encodeURIComponent(c)).catch(function(){})}
function cmd(c){send(c).then(function(){setTimeout(tick,120)})}
function tog(k,c){if(S)cmd(c+' '+(S[k]?0:1))}
function cal(w){cmd('cal '+w+' '+$('cv').value)}
function go(){var v=$('c').value;if(v){cmd(v);$('c').value=''}}
$('c').addEventListener('keydown',function(e){if(e.key==='Enter')go()});
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
 $('esb').textContent=d.es?'PRESSED or WIRE BROKEN':'Circuit closed, button released';$('esb').className='chip '+(d.es?'bad':'ok');
 $('inf').textContent='uptime '+d.up+' s, Wi-Fi clients '+d.cl+', PWM '+d.fq+' Hz';
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
  $('ck'+m).textContent=d.ik[m].toFixed(3)+(cal?'':'*');$('ck'+m).title='zero '+d.iz[m].toFixed(4)+' V';
  $('cs'+m).textContent=st;$('cs'+m).style.cssText=cl}
 $('ci').textContent='ADS samples/s: '+d.sps[0]+' + '+d.sps[1]+', I2C errors '+d.er[0]+' + '+d.er[1]+', limit '+d.lim.toFixed(2)+' A. * = default k, not calibrated. hwtest '+d.fw;
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
setInterval(tick,400);tick();poll();
</script></body></html>)HTML";

static void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

static void handleLog() {
  uint32_t from = (uint32_t)strtoul(server.arg("from").c_str(), nullptr, 10);
  uint32_t oldest = logTotal > LOG_SZ ? logTotal - LOG_SZ : 0;
  if (from < oldest) from = oldest;
  if (from > logTotal) from = logTotal;
  String out;
  out.reserve(logTotal - from + 16);
  out += String((unsigned long)logTotal);
  out += '\n';
  for (uint32_t i = from; i < logTotal; i++) out += logBuf[i % LOG_SZ];
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", out);
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

static void handleState() {
  static char b[2600];
  const int cap = sizeof(b);
  int n = snprintf(b, cap,
    "{\"up\":%lu,\"vb\":%.2f,\"vf\":%.2f,\"kb\":%.3f,\"kf\":%.3f,\"rl\":%d,\"oe\":%d,\"en\":%d,\"es\":%d,"
    "\"rd\":%d,\"gn\":%d,\"bz\":%d,\"bl\":%d,\"fq\":%.0f,\"os\":%.3f,\"sw\":%u,\"pca\":%d,\"ap\":[%d,%d],\"cl\":%d,\"rm\":\"%s\","
    "\"lim\":%.2f,\"ov\":%u,\"ic\":%u,\"zm\":%u,\"sps\":[%u,%u],\"er\":[%lu,%lu],\"fw\":\"%s\",\"pw\":[",
    millis() / 1000UL, vBus, vFb, kBus, kFb, relayOn, oeOn, enOn, estop, outR, outG, outB, blinkMode, pwmFreq, oscHz / 1e6f, (unsigned)sweepMask,
    pcaOk, adsPresent[0], adsPresent[1], WiFi.softAPgetStationNum(), relayMsg,
    iLim, (unsigned)ovlMask, (unsigned)calMaskBits(), (unsigned)zeroMask, (unsigned)pubSps[0], (unsigned)pubSps[1], (unsigned long)adsErr[0], (unsigned long)adsErr[1], FW_VERSION);
  for (int i = 0; i < 16 && n < cap - 20; i++) n += snprintf(b + n, cap - n, "%s%.1f", i ? "," : "", pwmDuty[i]);
  n += snprintf(b + n, cap - n, "],");
  jf(b, n, cap, "ia", iPeakA, NMOD, "%.2f", true);
  jf(b, n, cap, "ik", iK, NMOD, "%.3f", false);
  jf(b, n, cap, "iz", iZ, NMOD, "%.4f", false);
  n += snprintf(b + n, cap - n, "\"ad\":[");
  for (int i = 0; i < 8 && n < cap - 20; i++) {
    if (lastHave[i]) n += snprintf(b + n, cap - n, "%s%.3f", i ? "," : "", lastV[i]);
    else n += snprintf(b + n, cap - n, "%snull", i ? "," : "");
  }
  snprintf(b + n, cap - n, "]}");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

static void handleCmd() {
  char buf[96];
  strncpy(buf, server.arg("c").c_str(), sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  plog("> %s", buf);
  execCmd(buf);
  server.send(200, "text/plain", "ok");
}

// ============================================================
// setup / loop
// ============================================================
static void pinOutHigh(int pin) { gpio_set_level((gpio_num_t)pin, 1); pinMode(pin, OUTPUT); }   // без провала в LOW при старте
static void pinOutLow(int pin)  { gpio_set_level((gpio_num_t)pin, 0); pinMode(pin, OUTPUT); }

void setup() {
  pinOutHigh(PIN_RELAY);    // реле выключено
  pinOutHigh(PIN_OE);       // выходы PCA9685 выключены
  pinOutLow(PIN_EN);        // драйверы выключены
  pinOutLow(PIN_RED); pinOutLow(PIN_GREEN); pinOutLow(PIN_BUZ);
  pinOutLow(PIN_LED);
  pinMode(PIN_USER, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT);
  pinMode(PIN_VBUS, INPUT);
  pinMode(PIN_VFB, INPUT);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 1500) { yield(); }

  prefs.begin("calib", false);
  {
    // перенос из старого теста (namespace "rtest"): делители и генератор PCA9685 не терять
    Preferences old;
    if (old.begin("rtest", true)) {
      const char* ks[3] = { "kbus", "kfb", "osc" };
      for (int i = 0; i < 3; i++) if (!prefs.isKey(ks[i]) && old.isKey(ks[i])) prefs.putFloat(ks[i], old.getFloat(ks[i], 0));
      old.end();
    }
  }
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

  Wire.begin(PIN_BUS0_SDA, PIN_BUS0_SCL, I2C_HZ);
  Wire1.begin(PIN_BUS1_SDA, PIN_BUS1_SCL, I2C1_HZ);
  Wire.setTimeOut(50);
  Wire1.setTimeOut(10);
  i2c1Mux = xSemaphoreCreateMutex();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/log", HTTP_GET, handleLog);
  server.on("/cmd", HTTP_GET, handleCmd);
  server.on("/state", HTTP_GET, handleState);
  server.begin();

  plog("RECEIVER HARDWARE TEST AND CALIBRATION, hwtest %s (radio is OFF, no antenna needed)", FW_VERSION);
  plog("%s", FW_TAG);
  plog("Wi-Fi: network %s, password %s, open http://%s", AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  plog("Do NOT connect 12 V while USB is plugged in. Send h for commands.");

  pcaInit();
  adsPresent[0] = probe(Wire1, ADS_ADDR0);
  adsPresent[1] = probe(Wire1, ADS_ADDR1);
  plog("PCA9685 %s; ADS1115 0x%02X %s, 0x%02X %s", pcaOk ? "found" : "NOT FOUND", ADS_ADDR0, adsPresent[0] ? "found" : "NOT FOUND",
       ADS_ADDR1, adsPresent[1] ? "found" : "NOT FOUND");
  {
    int nc = 0;
    for (uint8_t m = 0; m < NMOD; m++) if (iCal[m]) nc++;
    plog("Current sensors calibrated: %d of %d, limit %.2f A. Send 'i' for details", nc, NMOD, iLim);
  }
  xTaskCreatePinnedToCore(samplerTask, "ads", 6144, nullptr, 2, nullptr, 1);

  estopRaw = (digitalRead(PIN_ESTOP) == HIGH);
  estop = estopRaw;
  plog("E-STOP %s", estop ? "ACTIVE (pressed or wire broken)" : "released");
  firstRepAt = millis() + 1500;
}

static void serialInput() {
  static char buf[96];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { buf[len] = 0; len = 0; execCmd(buf); }
    } else if (len < sizeof(buf) - 1) buf[len++] = ch;
  }
}

void loop() {
  unsigned long now = millis();

  server.handleClient();
  serialInput();

  // --- измерения напряжений ---
  if ((now - adcAt) >= 10) {
    adcAt = now;
    float pb = analogReadMilliVolts(PIN_VBUS) / 1000.0f;
    float pf = analogReadMilliVolts(PIN_VFB) / 1000.0f;
    if (!adcInit) { emaB = pb; emaF = pf; adcInit = true; }
    else { emaB += (pb - emaB) * 0.05f; emaF += (pf - emaF) * 0.05f; }
    vBus = emaB * kBus; if (vBus < V_DEADBAND) vBus = 0;
    vFb  = emaF * kFb;  if (vFb  < V_DEADBAND) vFb = 0;
  }
  if (adcInit && (long)(now - firstRepAt) >= 0) {
    // сообщаем при изменении на 0.3 В и ещё раз, когда значение успокоилось (вдруг остановилось между порогами)
    if (fabsf(vBus - settleBus) > 0.05f) { settleBus = vBus; settleBusAt = now; }
    if (fabsf(vFb - settleFb) > 0.05f)   { settleFb = vFb;  settleFbAt = now; }
    if (fabsf(vBus - lastRepBus) >= 0.3f || ((now - settleBusAt) > 1000 && fabsf(vBus - lastRepBus) >= 0.05f)) { lastRepBus = vBus; plog("Bus voltage: %.2f V", vBus); }
    if (fabsf(vFb - lastRepFb) >= 0.3f || ((now - settleFbAt) > 1000 && fabsf(vFb - lastRepFb) >= 0.05f))     { lastRepFb = vFb;  plog("After-relay voltage: %.2f V", vFb); }
  }

  // --- проверка реле ---
  if (checkState == 1) {
    if (vBus > 5.0f && vFb > 0.9f * vBus && (now - relayChangeAt) >= 300) { relayResult("RELAY ON OK (bus %.2f V, after relay %.2f V)", vBus, vFb); checkState = 0; }
    else if ((long)(now - checkDeadline) >= 0) {
      if (vBus > 5.0f && vFb > 0.8f * vBus) relayResult("RELAY ON OK, slow rise (bus %.2f V, after relay %.2f V)", vBus, vFb);
      else if (vBus <= 5.0f) relayResult("Relay ON: no bus voltage (12 V missing?), feedback cannot be checked");
      else relayResult("RELAY FAIL: relay is ON but after-relay voltage is only %.2f V (bus %.2f V)", vFb, vBus);
      checkState = 0;
    }
  } else if (checkState == 2) {
    if (vFb < 1.5f) { relayResult("RELAY OFF OK (after relay %.2f V)%s", vFb, vBus > 5.0f ? "" : "  note: no bus voltage, weak check"); checkState = 0; }
    else if ((long)(now - checkDeadline) >= 0) { relayResult("RELAY STUCK? relay is OFF but after-relay voltage is %.2f V", vFb); checkState = 0; }
  } else if ((now - relayChangeAt) > 3000) {
    int w = 0;
    if (!relayOn && vFb > 3.0f) w = 1;
    else if (relayOn && vBus > 5.0f && vFb < 0.5f * vBus) w = 2;
    if (w != warnState) {
      warnState = w;
      if (w == 1) relayResult("WARNING: voltage after relay (%.2f V) while relay is OFF - stuck contacts?", vFb);
      else if (w == 2) relayResult("WARNING: relay is ON but after-relay voltage is low (%.2f V)", vFb);
      else relayResult("Relay feedback is consistent again");
    }
  }

  // --- аварийная кнопка ---
  bool raw = (digitalRead(PIN_ESTOP) == HIGH);
  if (raw != estopRaw) { estopRaw = raw; estopRawAt = now; }
  if (estopRaw != estop && (now - estopRawAt) >= 30) {
    estop = estopRaw;
    if (estop) { plog("E-STOP ACTIVE (pressed or wire broken)"); allOff("E-STOP"); }
    else plog("E-STOP released (outputs stay off, switch them on again by command)");
  }

  // --- кнопка USER: короткое = реле, длинное = всё выключить ---
  bool u = (digitalRead(PIN_USER) == LOW);
  if (u && !userPrev) { userDownAt = now; userLongDone = false; }
  if (u && !userLongDone && (now - userDownAt) >= 2000) { userLongDone = true; allOff("USER long press"); }
  if (!u && userPrev && !userLongDone && (now - userDownAt) >= 30) {
    plog("USER short press");
    if (!relayOn && estop) plog("REFUSED: E-STOP is active");
    else { setRelay(!relayOn); plog("Relay %s, checking feedback...", relayOn ? "ON" : "OFF"); }
  }
  userPrev = u;

  // --- PCA9685: перезапуск после смены частоты, качание скважности ---
  if (pcaRestartPend && (long)(now - pcaRestartAt) >= 0) { pcaRestartPend = false; pcaWrite1(0x00, 0xA0); }
  if (sweepMask && (long)(now - sweepAt) >= 0) {
    sweepAt = now + 40;
    float p = (float)((now - sweepT0) % 10000UL) / 10000.0f;
    float tri = p < 0.5f ? p * 2.0f : (1.0f - p) * 2.0f;
    float duty = 10.0f + 80.0f * tri;
    if (sweepMask == 0xFFFF) pcaSetAll(duty);
    else for (uint8_t i = 0; i < 16; i++) if (sweepMask & (1u << i)) pcaSetCh(i, duty);
  }

  // --- мигалка и зуммер ---
  bool r = manRed, g = manGreen, b = manBuz;
  if (blinkMode) {
    unsigned long ph = now - blinkT0;
    r = g = b = false;
    if (blinkMode == 1) g = (ph % 1000UL) < 500UL;
    else if (blinkMode == 2) { bool x = (ph % 250UL) < 125UL; r = x; b = x; }
    else if (blinkMode == 3) { bool x = (ph % 1000UL) < 500UL; r = x; g = !x; }
  }
  if (estop) r = true;                  // при аварийной кнопке красная горит постоянно
  outR = r; outG = g; outB = b;
  digitalWrite(PIN_RED, r);
  digitalWrite(PIN_GREEN, g);
  digitalWrite(PIN_BUZ, b);

  // --- сообщения о клиентах Wi-Fi ---
  if ((now - stationAt) >= 500) {
    stationAt = now;
    int st = WiFi.softAPgetStationNum();
    if (st != lastStations) { plog(st > lastStations ? "Wi-Fi client connected (%d)" : "Wi-Fi client disconnected (%d)", st); lastStations = st; }
  }

  currentTick(now);

  // --- светодиод платы: вспышка раз в секунду, при аварийной кнопке часто ---
  unsigned long period = estop ? 300 : 1000;
  if (!ledOn && (now - ledAt) >= period) { ledOn = true; ledAt = now; digitalWrite(PIN_LED, HIGH); }
  else if (ledOn && (now - ledAt) >= 60) { ledOn = false; digitalWrite(PIN_LED, LOW); }
}
