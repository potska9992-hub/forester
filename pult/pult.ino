// ============================================================
// Пульт гидрораспределителя. Журнал версий
//  1.0 - РАДИО (HydroLink v1, SX1262 868 МГц, SF7 BW250): пакеты управления с маской каналов и heartbeat, телеметрия по запросу
//        (TELEM RATE), настройки приёмника с подтверждением и повторами (SetConfig / Ack / Batch), полная синхронизация
//        после включения и по CRC конфигурации, привязка по радио (Bind), Wi-Fi приёмника командами (Phone: Update/Test,
//        GitHub), Safe Start после возврата связи. Источник приёмника в Transmitter -> RX: Radio / Sim link / Sim lost
//        (имитация оставлена для отладки экранов, убрать после полевых испытаний).
//  0.8 - подготовка к радио: стартовый и максимальный ШИМ в единицах 0.1 % (шаг 0.5 %, стартовый по умолчанию 5 %);
//        Receiver -> Wi-Fi: "Phone: Update/Test" (вкладки Update и Test на странице приёмника, имитация статусов
//        Client connected / Test active / Uploading); ток приёмника подписан как ток в импульсе (Ipulse);
//        событие SHORT (КЗ) больше не создаётся; Safe Start при возврате связи; запрос режима CONTROL у приёмника
//        при подключении (имитация); при ошибках загрузки с GitHub на экране код (HTTP / Update)
//  0.3 - игра Dino (главное меню: Transmitter, Receiver, Dino, Exit; CH3 прыжок/присесть, CH6 "+" старт, "-" и MENU выход,
//        при проигрыше один раз вспышка красных светодиодов, рекорд в памяти); Receiver -> Wi-Fi вкладка (Phone update,
//        GitHub update - пока имитация, сети берутся из пульта); из LED & Alerts убран пункт Heartbeat (настройка осталась
//        в Alert events); единое правило CH6: "+" подтвердить, "-" отмена, на экранах без подтверждения выходит только "-"
//  0.2 - (нумерация начата заново, до этого было 0.1..0.7) Wi-Fi: код причины отказа подключения (неверный пароль,
//        сеть не найдена и т.д.), длина сохранённого пароля на экране проверки, ожидание подключения 30 с;
//        полоса загрузки больше не заходит на значок батареи
//  0.7 - выход из экранов Wi-Fi (Phone update, GitHub update, Test connection, ошибки) также рычагом CH6,
//        в любую сторону; в калибровке CH6 не работает как отмена (рычаги там двигаются)
//  0.6 - Wi-Fi собран в одну вкладку меню (Transmitter -> Wi-Fi: Phone update, GitHub update, Networks); у сохранённой
//        сети пункты Test connection и Delete; после ввода сети с пульта подключение проверяется сразу
//  0.5 - исправлено: метка прошивки FWTAG выбрасывалась линкером из файла, поэтому страница и пульт считали
//        любой файл "не той прошивкой" (теперь на метку есть ссылка из кода, сборка на GitHub проверяет её наличие)
//  0.4 - загрузка прошивки с GitHub (меню Transmitter -> GitHub Update): пульт сам подключается к сохранённой
//        сети, находит последнюю прошивку, показывает версию, после подтверждения (CH6) скачивает, проверяет
//        (SHA-256, метка проекта) и прошивается. До 3 сохранённых сетей (страница Wi-Fi Update и меню
//        Transmitter -> Wi-Fi Networks, ввод названия и пароля с пульта). Вывод в Serial убран (журнал на экране).
//  0.3 - исправлено: Wi-Fi отключался сразу после запроса от телефона (ошибка сравнения времени)
//  0.2 - Wi-Fi: адрес и пароль остаются на экране, пока страница не открыта (а не по первому запросу
//        телефона); телефон получает ответ "интернет есть" и не отключается; причина выхода в сообщении
//  0.1 - Wi-Fi Update: обновление прошивки с телефона (меню Transmitter -> Wi-Fi Update), пункт
//        Receiver -> Wi-Fi Update (работает с радио, пока имитация). Нумерация: пока радио нет,
//        версии 0.x; 1.0 будет вместе с радио. Содержит все изменения 1.2.
//  1.2 - заставка FORESTER (2 с), экран тревог в рабочем режиме (у каждого события пункт Screen),
//        Max Duty по умолчанию 60 %, настройка частоты телеметрии TELEM RATE (1..10, по умолчанию 3)
//  1.1 - экран выводится по одной странице за проход цикла (цикл больше не замирает на 30 мс),
//        шина I2C больше не падает до 100 кГц после каждого кадра, меню отсортированы
//        по важности (Back / Exit всегда внизу), нумерация версий 1.x
//  1.0 - первая полная версия (раньше называлась v3.1): экраны, журнал, Debug, Receiver,
//        Power / Bind, Factory Reset, режим CONTROL PWM / CURRENT
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include <esp_system.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <mbedtls/sha256.h>
#include <stdarg.h>

// ============ ПИНЫ И АДРЕСА ============
#define SDA_PIN        2
#define SCL_PIN        3
#define I2C_CLOCK      400000      // если будут глюки на шине, поставить 100000

#define BTN_MENU_PIN   33          // нажата = LOW
#define ESTOP_PIN      34          // NC-контакт: HIGH = нажата или обрыв
#define VBAT_PIN       4

#define OLED_ADDR      0x3C
#define MCP_ADDR       0x20
#define ADS1_ADDR      0x48
#define ADS2_ADDR      0x49

#define SCREEN_W       128
#define SCREEN_H       64

// ============ НАВИГАЦИЯ ПО МЕНЮ (индексы 0..7 = CH1..CH8) ============
// Меняются только здесь, логика меню не трогается.
#define MENU_NAV_CH     2          // CH3: вверх/вниз по списку, +/- при вводе числа
#define MENU_ENTER_CH   5          // CH6: ввод/назад
#define TEST_LEVER_CH   1          // CH2: тестовый джойстик (Receiver -> Current Limits / Start Duty)

// ============ ПАРАМЕТРЫ ============
#define NUM_CH          8
#define NUM_EV          12
#define DEF_DEADZONE    500        // в отсчётах ADC (GAIN_ONE) от центра
#define DZ_MIN          100
#define DZ_MAX          3000
#define DZ_STEP         50
#define MIN_SIDE        2500       // минимальный ход в каждую сторону при калибровке
#define DEF_CENTER      13200      // значения по умолчанию до первой калибровки
#define DEF_MIN         0
#define DEF_MAX         26400

#define EXPO_STEP       5
#define MAXP_MIN        10         // минимум для Max, %
#define MAXP_STEP       5

#define NAV_ON          60         // порог срабатывания тумблера навигации, %
#define NAV_OFF         30         // порог возврата в центр, %
#define NAV_REARM_MS    80
#define REPEAT_DELAY_MS 600        // автоповтор при удержании (числовые экраны)
#define REPEAT_MS       120
#define SAFE_HOLD_MS    400        // сколько всё должно стоять в центре для Safe Start
#define LONG_PRESS_MS   700
#define DEBOUNCE_MS     30
#define READ_PERIOD_MS  20
#define DISP_PERIOD_MS  150
#define GAME_FRAME_MS   50         // кадры игры Dino (20 к/с)
#define LOG_PERIOD_MS   1000
#define BOOT_MS         1500
#define SPLASH_MS       2000       // заставка FORESTER перед Boot check
#define ALERT_MIN_MS    2000       // короткое событие на экране тревог держится не меньше
#define ALERT_SLOT_MS   2000       // при нескольких событиях каждое показывается столько
#define WIFI_PASS       "88888888"      // пароль точки доступа (фиксированный)
#define WIFI_ADDR       "192.168.4.1"
#define WIFI_NOCONN_MS  120000UL        // 2 мин, если телефон не подключился
#define WIFI_IDLE_MS    300000UL        // 5 мин без действий, если телефон подключён
#define TELEM_RATE_MIN  1
#define TELEM_RATE_MAX  10
#define TELEM_RATE_DEF  3          // запросов телеметрии в секунду
#define CAL_COUNT_MS    5000       // отсчёт перед записью центра
#define CAL_AVG_MS      500
#define CAL_RANGE_MS    10000
#define MENU_ROWS       4
#define TOAST_MS        1800
#define RESTORED_MS     700        // длительность вспышки «связь восстановлена»
#define HB_FLASH_MS     100        // длительность вспышки пульса

// ============ ВЕРСИЯ ПРОШИВКИ ============
#define FW_VERSION      "1.0"
#define PBAR_W          108             // ширина полосы загрузки (значок батареи справа)
#define GH_REPO         "potska9992-hub/forester"   // репозиторий с готовыми прошивками
#define GH_TAG          "pult-latest"               // релиз с последней прошивкой пульта
#define NET_MAX         3           // сохранённых Wi-Fi сетей
#define PROJECT_NAME    "pult"      // имя устройства внутри метки прошивки (проверка при обновлении)
#define FACTORY_PIN     1111       // код для Factory Reset
#define DEF_BIND_ID     0xA5C3E17BUL
#define BIND_TIMEOUT_MS 60000UL
#define LOG_MAX         10         // событий в журнале
#define LOG_SHOW        5          // событий на экране Event Log
#define LOG_NVS_GAP_MS  10000UL    // одна и та же авария в NVS не чаще
#define SLOG_LINES      6          // строк serial-лога для страницы Debug -> Log
#define NUM_SCR         6          // рабочих экранов
#define NUM_DP          6          // страниц Debug
#define TX_PWR_DEF      10         // dBm по умолчанию
#define RF_FREQ_TXT     "868.0MHz SF7 BW250"   // справочно (настоящие значения RF_* в блоке радио)

// ============ БАТАРЕЯ ============
#define DEF_DIVIDER     3.3503f    // коэффициент делителя по умолчанию
#define DEF_VFULL       8.40f      // напряжение 100%
#define DEF_VEMPTY      6.00f      // напряжение 0%
#define DEF_WARN        20         // % для надписи LOW BATTERY
#define DEF_CRIT        10         // % для надписи BATT CRITICAL
#define BAT_UPDATE_MS   30000UL    // обновление процента на экранах
#define BAT_FAST_MS     2000UL     // фоновый замер (поиск минимума)
#define BAT_LIVE_MS     1000UL     // замер на странице Battery
#define BAT_USB_V       4.0f       // ниже этого считаем, что батареи нет (USB)
#define BAT_MIN_SAVE_MS 10000UL    // не чаще раз в 10 с писать минимум в NVS

// ============ КАРТА КАНАЛОВ ============
// ads: 0 = ADS1 (0x48), 1 = ADS2 (0x49); input: вход A0..A3
// redPin / greenPin: пин MCP23017 (0-7 порт A, 8-15 порт B)
struct ChannelHw { uint8_t ads; uint8_t input; uint8_t redPin; uint8_t greenPin; };
const ChannelHw CH[NUM_CH] = {
  {0, 0,  7,  6},   // CH1
  {0, 1,  5,  4},   // CH2
  {0, 2,  3,  2},   // CH3
  {0, 3,  1,  0},   // CH4
  {1, 0,  8,  9},   // CH5
  {1, 1, 10, 11},   // CH6
  {1, 2, 12, 13},   // CH7
  {1, 3, 14, 15}    // CH8
};

// ============ ОБЪЕКТЫ ============
Adafruit_SH1106G  display(SCREEN_W, SCREEN_H, &Wire, -1, I2C_CLOCK, I2C_CLOCK);   // библиотека по умолчанию роняет шину до 100 кГц после кадра
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115  ads1, ads2;
Adafruit_ADS1115* adsBank[2] = { &ads1, &ads2 };
Preferences       prefs;

bool okOled = false, okMcp = false;
bool adsOk[2] = { false, false };

// ============ КНОПКИ ============
struct Button {
  uint8_t pin;
  bool activeHigh;
  bool raw, state, changed, pressedEv, shortEv, longEv, longFired;
  unsigned long t, pressStart;
};
Button btnMenu = { BTN_MENU_PIN, false, false, false, false, false, false, false, true, 0, 0 };
Button estop   = { ESTOP_PIN,    true,  false, false, false, false, false, false, true, 0, 0 };

// ============ ТИПЫ (объявлены ДО функций, иначе Arduino IDE ломает автопрототипы) ============
enum State { ST_BOOT, ST_SAFE, ST_RUN, ST_MENU, ST_CAL_C, ST_CAL_R, ST_CAL_RES, ST_ESTOP, ST_WIFI, ST_GH };
enum MenuScr {
  MS_MAIN, MS_TX, MS_CAL, MS_CHLIST, MS_CHEDIT, MS_DZ,
  MS_BAT, MS_BATEDIT, MS_LED, MS_LEDSTAT, MS_EVLIST, MS_EVEDIT, MS_PRESET, MS_TEST,
  MS_EXPO, MS_MAXV,
  MS_SCREENS, MS_SCRITEM, MS_SCRMOVE, MS_POWER, MS_TXPWR, MS_TRATE,
  MS_BIND, MS_BINDRUN, MS_BINDID, MS_EVLOG,
  MS_DBGLIST, MS_DBGDESC, MS_DBGPAGE,
  MS_PIN, MS_FRTARGET, MS_CONFIRM,
  MS_RX, MS_RXCH, MS_RXDIR, MS_RXGLOB, MS_RXEDIT, MS_RXSTAT, MS_RXWIFI,
  MS_NETS, MS_NETPICK, MS_TEXTIN, MS_WIFI, MS_NETACT, MS_RXWIFIM, MS_GAME
};
enum ChItem { CI_TYPE, CI_REV, CI_EN, CI_EXPO, CI_MAX, CI_LED, CI_BACK };
enum EvItem { EI_EN, EI_COLOR, EI_PAT, EI_INT, EI_WHERE, EI_SCR, EI_FIXED, EI_BACK };
enum BatKind { BE_DIV, BE_FULL, BE_EMPTY, BE_WARN, BE_CRIT };
enum SimMode { SIM_RADIO, SIM_LINK, SIM_LOST };   // источник приёмника: настоящее радио или имитация (отладка)
struct Nav { bool armed; unsigned long since; unsigned long held; unsigned long last; int dir; };
struct EvCfg { uint8_t en, color, pat, whereAll, ival, scr; };
struct GObs { float x; uint8_t kind; bool on; };      // 0 малый кактус, 1 большой, 2 двойной, 3 птица (присесть), 4 птица высоко   // тип выше функций (Arduino делает прототипы)
struct NetFound { char ssid[33]; int8_t rssi; };      // найденная Wi-Fi сеть (тип выше функций: Arduino делает прототипы)

// порядок = порядок в меню: частое и важное сверху, опасное и временное внизу, Back последний
enum TxItem { TX_CAL, TX_CHSET, TX_PWR, TX_BIND, TX_TRT, TX_SCR, TX_LED, TX_BAT, TX_LOG, TX_DBG, TX_WIFI, TX_SIM, TX_FR, TX_BACK, TX_COUNT };
enum RxTopic { RT_LIMIT, RT_DUTY, RT_MAXD, RT_RAMP, RT_CHEN, RT_GLOB, RT_STAT, RT_WIFI, RT_BACK, RT_COUNT };
enum WifiItem { WF_PHONE, WF_GH, WF_NETS, WF_BACK, WF_COUNT };
enum RxWifiItem { RW_PHONE, RW_GH, RW_BACK, RW_COUNT };
enum ScrId { SC_MAIN, SC_TX, SC_RX, SC_LINK, SC_POWER, SC_LOG };
enum DbgPage { DP_RADIO, DP_TELEM, DP_LIM, DP_RAW, DP_SYS, DP_LOG };
enum ConfAct { CA_CLEARLOG, CA_FORGET, CA_FR_TX, CA_FR_RX, CA_NETDEL, CA_NETALL };
// LT_SHORT больше не создаётся (решение 28.10.2026), номер оставлен, чтобы не сломать старые записи журнала в NVS
enum LogType { LT_ESTOP, LT_RXALARM, LT_NOLINK, LT_LINKOK, LT_RXFAULT, LT_LIMIT, LT_BATTTX, LT_OVL, LT_SHORT, LT_OPEN, LT_BATTRX, LT_COUNT };
// параметры приёмника (SetConfig): SOL = на соленоид (канал x сторона A/B), CH = на канал, GLOB = общие
enum RxParam { RXP_LIMIT, RXP_DUTY, RXP_MAXD, RXP_RAMPUP, RXP_RAMPDN, RXP_CHEN,
               RXP_OVLDLY, RXP_LINKTMO, RXP_PWMFREQ, RXP_DITHAMP, RXP_DITHFRQ, RXP_PWR,
               RXP_MODE, RXP_STARTI, RXP_MAXI, NUM_RXP };
enum RxScope { PS_SOL, PS_CH, PS_GLOB };
enum RxFmt { F_AMP, F_PCT, F_MS, F_HZ, F_ONOFF, F_DBM, F_MODE, F_PCT10 };
struct RxParamDef { const char* name; uint8_t scope; int16_t mn, mx, step, def; uint8_t fmt; };
struct LogEv { uint8_t type, ch, side, cnt; uint16_t dA; uint32_t t; bool old; };
struct Telem {
  int16_t cur[NUM_CH];      // ток, 0.1 А (у отключённого по перегрузке: пиковый)
  int8_t  pwm[NUM_CH];      // ШИМ со знаком, минус = сторона B
  uint8_t mLim, mOvl, mOpen, side;   // маски по каналам; side: бит = сторона B
  uint8_t vrx;              // напряжение приёмника, 0.1 В
  int8_t  rssi;             // RSSI приёмника, dBm
  uint8_t kill, reason;     // killswitch: 0/1; причина 0..3
  int8_t  snr;              // SNR, с которым приёмник слышит пульт, дБ
  uint8_t missed;           // пропущено пакетов управления с прошлой телеметрии
  uint8_t boots;            // счётчик перезагрузок приёмника
  uint8_t cfgCrc;           // CRC8 сохранённой конфигурации приёмника
  uint8_t wifi, wifiArg;    // состояние Wi-Fi приёмника и параметр (процент / код ошибки)
};
// ---- радио HydroLink: константы (описание протокола ниже, в блоке РАДИО) ----
#define HL_PROTO        1
#define RF_FREQ_MHZ     868.0f
#define RF_BW_KHZ       250.0f         // BW и SF одинаковые в пульте и приёмнике, меняются только в коде
#define RF_SF           7
#define RF_CR           5
#define RF_SYNC         0x12
#define RF_PREAMBLE     8
#define RF_TCXO_V       1.8f
#define HL_PIN_NSS      8
#define HL_PIN_SCK      9
#define HL_PIN_MOSI     10
#define HL_PIN_MISO     11
#define HL_PIN_RST      12
#define HL_PIN_BUSY     13
#define HL_PIN_DIO1     14
#define HL_RESP_MS      90             // ждать ответ (телеметрия, подтверждение) после конца передачи
#define HL_GAP_MS       2              // пауза между пакетами управления
#define HL_HB_MS        66             // heartbeat 15 раз в секунду
#define HL_HB_AFTER_MS  300            // рычаги в нуле дольше этого времени: heartbeat
#define HL_SEARCH_MS    150            // поиск приёмника: запрос телеметрии
#define HL_BIND_MS      150
#define HL_CFG_TRIES    3
#define HL_RXBATT_LOW   105            // 10.5 В: ниже событие Batt RX
enum { HLT_CONTROL = 1, HLT_TELEM, HLT_CFG, HLT_ACK, HLT_BATCH, HLT_NET, HLT_INFO, HLT_BIND, HLT_BINDOK };
enum { HLF_ESTOP = 1, HLF_REQT = 2, HLF_LEVER = 4, HLF_BOOT = 8 };
enum { HLO_SET, HLO_GET, HLO_BATCH, HLO_CMD, HLO_NET };
enum { HLC_PROTO = 1, HLC_CHMASK, HLC_CHTYPES, HLC_TXPWR, HLC_FACTORY = 0x10, HLC_WIFI_PHONE = 0x20, HLC_WIFI_GH, HLC_WIFI_OFF, HLC_WIFI_YES };
enum { HLW_OFF = 0, HLW_PH_WAIT = 2, HLW_PH_CONN, HLW_PH_TEST, HLW_PH_UP, HLW_PH_DONE,
       HLW_GH_CONN = 10, HLW_GH_CHECK, HLW_GH_ASK, HLW_GH_DL, HLW_GH_VERIFY, HLW_GH_DONE, HLW_ERR = 20 };
#define HL_FACTORY_KEY  0xA55A

struct HlCfg { uint8_t op, param, ch, dir; uint16_t val; bool save; uint8_t kind, tag, tries; uint16_t old; };   // запрос настройки в очереди радио

const RxParamDef RXP[NUM_RXP] = {
  { "LIMIT",     PS_SOL,    5,  100,  1,  20, F_AMP   },   // 0.5..10.0 А, по умолчанию 2.0 А
  { "DUTY",      PS_SOL,    0, 1000,  5,  50, F_PCT10 },   // стартовый ШИМ, 0.1 % (0..100.0 %), шаг 0.5 %, по умолчанию 5.0 %
  { "MAX DUTY",  PS_SOL,  100, 1000,  5, 600, F_PCT10 },   // максимальный ШИМ, 0.1 % (10.0..100.0 %), по умолчанию 60.0 %
  { "RAMP UP",   PS_CH,     0, 2000, 50, 200, F_MS    },
  { "RAMP DN",   PS_CH,     0, 2000, 50, 200, F_MS    },
  { "ENABLE",    PS_CH,     0,    1,  1,   1, F_ONOFF },
  { "OVL DELAY", PS_GLOB,  20, 1000, 10, 150, F_MS    },
  { "LINK TMO",  PS_GLOB, 200, 2000, 50, 500, F_MS    },
  { "PWM FREQ",  PS_GLOB,  50,  500, 10, 100, F_HZ    },
  { "DITH AMP",  PS_GLOB,   0,   20,  1,   0, F_PCT   },
  { "DITH FREQ", PS_GLOB,  10,   60,  5,  30, F_HZ    },
  { "RX POWER",  PS_GLOB,   2,   22,  1,  10, F_DBM   },
  { "CONTROL",   PS_GLOB,   0,    1,  1,   0, F_MODE  },   // 0 = PWM (без обратной связи), 1 = CURRENT (по току)
  { "START I",   PS_SOL,    0,  100,  1,   5, F_AMP   },   // стартовый ток, режим CURRENT (0.1 А)
  { "MAX I",     PS_SOL,    5,  100,  1,  15, F_AMP   }    // максимальный рабочий ток, режим CURRENT
};

// цвета: 0 красный, 1 зелёный, 2 янтарный (оба)
#define COL_RED    0
#define COL_GREEN  1
#define COL_AMBER  2
// ритмы
#define PAT_STEADY 0
#define PAT_SLOW   1
#define PAT_FAST   2
#define PAT_PULSE  3
#define PAT_DOUBLE 4
#define PAT_ALT    5      // красный/янтарный попеременно
// события (порядок = приоритет, 0 самый важный)
#define EV_ESTOP    0
#define EV_RXALARM  1
#define EV_NOLINK   2
#define EV_RXFAULT  3
#define EV_OVERLOAD 4
#define EV_OPEN     5
#define EV_LIMIT    6
#define EV_NOTCTR   7
#define EV_RESTORED 8
#define EV_HEART    9
#define EV_BATTTX   10
#define EV_BATTRX   11
// маска цвета светодиода: бит0 красный, бит1 зелёный, 3 = янтарный; 0x80 = погасить (тёмная вспышка)
#define MASK_DARK  0x80

void initButton(Button &b) {
  bool lvl = digitalRead(b.pin);
  b.raw = b.state = b.activeHigh ? lvl : !lvl;
  b.t = millis();
  b.longFired = true;   // кнопка, зажатая при старте, событий не даёт
}

void updateButton(Button &b) {
  unsigned long now = millis();
  b.changed = b.pressedEv = b.shortEv = b.longEv = false;
  bool lvl = digitalRead(b.pin);
  bool p = b.activeHigh ? lvl : !lvl;
  if (p != b.raw) { b.raw = p; b.t = now; }
  if (b.state != b.raw && (now - b.t) >= DEBOUNCE_MS) {
    b.state = b.raw;
    b.changed = true;
    if (b.state) { b.pressedEv = true; b.pressStart = now; b.longFired = false; }
    else if (!b.longFired) { b.shortEv = true; }
  }
  if (b.state && !b.longFired && (now - b.pressStart) >= LONG_PRESS_MS) {
    b.longFired = true;
    b.longEv = true;
  }
}

// ============ ДАННЫЕ КАНАЛОВ ============
#define T_JOY 0
#define T_SW3 1
#define LED_BOTH  0
#define LED_RED   1
#define LED_GREEN 2
#define LED_OFF   3
const uint8_t DEF_TYPE[NUM_CH] = { T_JOY, T_JOY, T_SW3, T_JOY, T_JOY, T_SW3, T_JOY, T_JOY };

// Стандартные настройки событий (пресет Standard)   en color pat        whereAll ival scr
const EvCfg DEF_EV[NUM_EV] = {
  { 1, COL_RED,   PAT_FAST,   1, 0, 0 },   // 0  E-Stop пульта (фиксировано, свой экран)
  { 1, COL_RED,   PAT_FAST,   1, 0, 1 },   // 1  аварийная кнопка приёмника
  { 1, COL_AMBER, PAT_SLOW,   1, 0, 1 },   // 2  нет связи
  { 1, COL_RED,   PAT_ALT,    1, 0, 1 },   // 3  неисправность приёмника
  { 1, COL_RED,   PAT_FAST,   0, 0, 1 },   // 4  перегрузка секции
  { 1, COL_AMBER, PAT_FAST,   0, 0, 1 },   // 5  обрыв соленоида
  { 1, COL_RED,   PAT_SLOW,   0, 0, 0 },   // 6  лимит тока достигнут
  { 1, COL_RED,   PAT_SLOW,   0, 0, 0 },   // 7  Safe Start: канал не в центре (свой экран)
  { 1, COL_GREEN, PAT_PULSE,  1, 0, 0 },   // 8  связь восстановлена
  { 1, COL_GREEN, PAT_PULSE,  1, 2, 0 },   // 9  пульс (ival 2 = 5 с)
  { 0, COL_AMBER, PAT_DOUBLE, 1, 0, 0 },   // 10 батарея пульта (по умолчанию выключено)
  { 1, COL_AMBER, PAT_DOUBLE, 1, 0, 0 }    // 11 батарея приёмника
};
const char* EV_NAME[NUM_EV] = {
  "E-Stop pult", "RX alarm btn", "No link", "RX fault", "Overload", "Open circuit",
  "Cur. limit", "Not centered", "Link back", "Heartbeat", "Batt pult", "Batt RX"
};
const uint8_t HB_SEC[4] = { 2, 3, 5, 10 };

int16_t raw[NUM_CH];
int16_t cenV[NUM_CH], mnV[NUM_CH], mxV[NUM_CH];   // калибровка
uint8_t chType[NUM_CH];
bool    chRev[NUM_CH];
bool    chEn[NUM_CH];
uint8_t chExpo[NUM_CH];      // 0..100 %
uint8_t chMaxP[NUM_CH];      // MAXP_MIN..100 %
uint8_t chLed[NUM_CH];       // LED_BOTH / RED / GREEN / OFF
bool    ledGreenOn = true, ledRedOn = true;
EvCfg   ev[NUM_EV];
int16_t deadzone = DEF_DEADZONE;
unsigned long readUs = 0, dispUs = 0;
int     ctl[NUM_CH];         // что пульт отправляет приёмнику (-100..100); в меню нули

// ---- батарея (настройки) ----
float   divRatio = DEF_DIVIDER;
float   vFull = DEF_VFULL, vEmpty = DEF_VEMPTY;
float   vMinSeen = 99.0f;    // самое низкое напряжение, которое видел пульт
uint8_t batWarn = DEF_WARN, batCrit = DEF_CRIT;
uint8_t batCurve = 1;        // 1 = типовая Li-ion кривая, 0 = линейно

// ---- экраны (порядок, включение, стартовый) ----
uint8_t scrOrder[NUM_SCR] = { SC_MAIN, SC_TX, SC_RX, SC_LINK, SC_POWER, SC_LOG };
uint8_t scrEn = 0x3F;
uint8_t scrStart = SC_MAIN;
const char* SCR_NAME[NUM_SCR] = { "Main", "TX channels", "RX channels", "Link", "Power", "Event log" };

// ---- радио (пока только настройки, радио подключим позже) ----
const uint8_t  TXP_LIST[8] = { 2, 5, 8, 10, 14, 17, 20, 22 };
const uint16_t TXP_MW[8]   = { 2, 3, 6, 10, 25, 50, 100, 158 };
uint8_t  txPower = TX_PWR_DEF;
uint8_t  telemRate = TELEM_RATE_DEF;   // запросов телеметрии в секунду (пульт -> приёмник), 1..10
uint32_t bindId = DEF_BIND_ID;

void keyOf(char* k, size_t n, char p, int i) { snprintf(k, n, "%c%d", p, i); }

// Бит 9 хранит отличие Screen от значения по умолчанию: старые записи (бит 0) автоматически
// получают стандартное значение Screen, миграция не нужна.
uint16_t packEv(const EvCfg &c, int e) {
  uint16_t d = (c.scr != DEF_EV[e].scr) ? 1 : 0;
  return (uint16_t)((c.en & 1) | ((c.color & 3) << 1) | ((c.pat & 7) << 3) | ((c.whereAll & 1) << 6) | ((c.ival & 3) << 7) | (d << 9));
}
EvCfg unpackEv(uint16_t v, int e) {
  EvCfg c;
  c.en = v & 1;
  c.color = (v >> 1) & 3;
  c.pat = (v >> 3) & 7;
  c.whereAll = (v >> 6) & 1;
  c.ival = (v >> 7) & 3;
  c.scr = ((v >> 9) & 1) ? (DEF_EV[e].scr ? 0 : 1) : DEF_EV[e].scr;
  if (c.color > COL_AMBER) c.color = COL_AMBER;
  if (c.pat > PAT_ALT) c.pat = PAT_STEADY;
  return c;
}

// ============ СЕРИЙНЫЙ ЛОГ (последние строки видны на странице Debug -> Log) ============
char slog[SLOG_LINES][24];
int  slogCount = 0;

void logLine(const char* t) {
  snprintf(slog[slogCount % SLOG_LINES], sizeof(slog[0]), "%.19s", t);
  slogCount++;
}

void logFmt(const char* fmt, ...) {
  char b[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  logLine(b);
}
#define LOGF(...) logFmt(__VA_ARGS__)

// ============ ЖУРНАЛ СОБЫТИЙ ============
// RAM: 10 последних событий, одинаковые подряд (тип+канал+сторона) сливаются со счётчиком.
// NVS: только события силовой части приёмника (перегрузка, КЗ, обрыв, критическое напряжение),
// пишутся в момент события, повторы не пишутся.
LogEv   logRam[LOG_MAX];        // [0] = самое новое
int     logN = 0;
uint8_t plog[LOG_MAX][5];       // сохранённые: type, ch, side, dA lo, dA hi; [0] = новое
int     plogN = 0;
uint32_t lastPersist[LT_COUNT][NUM_CH];
const char* LT_NAME[LT_COUNT] = { "ESTP", "RXAL", "NOLK", "LINK", "RXFT", "LIM", "BATT", "OVL", "SHRT", "OPEN", "RXBT" };

bool logPersistent(uint8_t t) { return t == LT_OVL || t == LT_SHORT || t == LT_OPEN || t == LT_BATTRX; }
bool logHasChannel(uint8_t t) { return t == LT_LIMIT || t == LT_OVL || t == LT_SHORT || t == LT_OPEN; }

void savePersistLog() {
  uint8_t blob[LOG_MAX * 5 + 1];
  blob[0] = (uint8_t)plogN;
  memcpy(blob + 1, plog, LOG_MAX * 5);
  prefs.begin("log", false);
  prefs.putBytes("ring", blob, sizeof(blob));
  prefs.end();
}

void loadPersistLog() {
  uint8_t blob[LOG_MAX * 5 + 1];
  memset(blob, 0, sizeof(blob));
  memset(plog, 0, sizeof(plog));
  prefs.begin("log", false);
  size_t n = prefs.getBytes("ring", blob, sizeof(blob));
  prefs.end();
  plogN = 0;
  logN = 0;
  if (n != sizeof(blob)) return;
  int cnt = blob[0];
  if (cnt > LOG_MAX) cnt = LOG_MAX;
  memcpy(plog, blob + 1, LOG_MAX * 5);
  plogN = cnt;
  for (int i = 0; i < cnt && logN < LOG_MAX; i++) {
    if (plog[i][0] >= LT_COUNT || plog[i][1] >= NUM_CH) continue;
    LogEv &e = logRam[logN++];
    e.type = plog[i][0];
    e.ch = plog[i][1];
    e.side = plog[i][2] & 1;
    e.dA = (uint16_t)(plog[i][3] | (plog[i][4] << 8));
    e.cnt = 1;
    e.t = 0;
    e.old = true;     // событие до последней перезагрузки
  }
}

void logEvent(uint8_t type, uint8_t ch, uint8_t side, uint16_t dA) {
  uint32_t now = millis();
  if (logN > 0) {
    LogEv &l = logRam[0];
    if (l.type == type && l.ch == ch && l.side == side && !l.old) {   // повтор: только счётчик, в NVS не пишем
      if (l.cnt < 255) l.cnt++;
      if (dA > l.dA) l.dA = dA;
      l.t = now;
      return;
    }
  }
  int n = (logN < LOG_MAX) ? logN : LOG_MAX - 1;
  for (int i = n; i > 0; i--) logRam[i] = logRam[i - 1];
  LogEv &e = logRam[0];
  e.type = type; e.ch = ch; e.side = side; e.cnt = 1; e.dA = dA; e.t = now; e.old = false;
  if (logN < LOG_MAX) logN++;
  if (logPersistent(type)) {
    uint8_t c = (ch < NUM_CH) ? ch : 0;
    if (lastPersist[type][c] == 0 || (now - lastPersist[type][c]) >= LOG_NVS_GAP_MS) {
      lastPersist[type][c] = now ? now : 1;
      memmove(plog[1], plog[0], (LOG_MAX - 1) * 5);
      plog[0][0] = type; plog[0][1] = c; plog[0][2] = side;
      plog[0][3] = (uint8_t)(dA & 0xFF); plog[0][4] = (uint8_t)(dA >> 8);
      if (plogN < LOG_MAX) plogN++;
      savePersistLog();
    }
  }
}

void logClear() {
  logN = 0;
  plogN = 0;
  memset(plog, 0, sizeof(plog));
  memset(lastPersist, 0, sizeof(lastPersist));
  prefs.begin("log", false);
  prefs.clear();
  prefs.end();
}

// Текст строки журнала (до 19 символов): "3m OVL 3B 5.2 x5"
void logText(const LogEv &e, char* b, size_t n) {
  char tm[10];
  if (e.old) {
    snprintf(tm, sizeof(tm), "old");
  } else {
    unsigned long sec = (millis() - e.t) / 1000UL;
    if (sec < 60)        snprintf(tm, sizeof(tm), "%lus", sec);
    else if (sec < 3600) snprintf(tm, sizeof(tm), "%lum", sec / 60);
    else                 snprintf(tm, sizeof(tm), "%luh", sec / 3600);
  }
  int p = snprintf(b, n, "%s %s", tm, LT_NAME[e.type]);
  if (logHasChannel(e.type) && p < (int)n) p += snprintf(b + p, n - p, " %d%c", e.ch + 1, e.side ? 'B' : 'A');
  if (e.dA > 0 && p < (int)n) p += snprintf(b + p, n - p, " %d.%d", e.dA / 10, e.dA % 10);
  if (e.cnt > 1 && p < (int)n) {
    if (e.cnt > 99) snprintf(b + p, n - p, " x99+");
    else            snprintf(b + p, n - p, " x%d", e.cnt);
  }
}

// ============ СОХРАНЕНИЕ: экраны, мощность, привязка ============
void saveScreens() {
  prefs.begin("screens", false);
  prefs.putBytes("ord", scrOrder, NUM_SCR);
  prefs.putUChar("en", scrEn);
  prefs.putUChar("st", scrStart);
  prefs.end();
}

void loadScreens() {
  uint8_t o[NUM_SCR];
  prefs.begin("screens", false);
  size_t n = prefs.getBytes("ord", o, NUM_SCR);
  uint8_t en = prefs.getUChar("en", 0x3F);
  uint8_t st = prefs.getUChar("st", SC_MAIN);
  prefs.end();
  bool ok = (n == NUM_SCR);
  if (ok) {
    uint8_t seen = 0;
    for (int i = 0; i < NUM_SCR; i++) {
      if (o[i] >= NUM_SCR) ok = false;
      else seen |= (uint8_t)(1 << o[i]);
    }
    if (seen != 0x3F) ok = false;
  }
  if (ok) memcpy(scrOrder, o, NUM_SCR);
  en &= 0x3F;
  en |= 1;                                   // Main всегда включён
  if (st >= NUM_SCR || !((en >> st) & 1)) st = SC_MAIN;
  scrEn = en;
  scrStart = st;
}

void saveRadio() {
  prefs.begin("radio", false);
  prefs.putUChar("txp", txPower);
  prefs.putUChar("trt", telemRate);
  prefs.end();
}

void saveBind() {
  prefs.begin("bind", false);
  prefs.putUInt("id", bindId);
  prefs.end();
}

void loadConfig() {
  char k[8];
  prefs.begin("joy", false);
  for (int i = 0; i < NUM_CH; i++) {
    keyOf(k, sizeof(k), 'c', i); cenV[i] = prefs.getShort(k, DEF_CENTER);
    keyOf(k, sizeof(k), 'n', i); mnV[i]  = prefs.getShort(k, DEF_MIN);
    keyOf(k, sizeof(k), 'x', i); mxV[i]  = prefs.getShort(k, DEF_MAX);
  }
  deadzone = prefs.getShort("dz", DEF_DEADZONE);
  prefs.end();

  prefs.begin("chcfg", false);
  for (int i = 0; i < NUM_CH; i++) {
    keyOf(k, sizeof(k), 't', i); chType[i] = prefs.getUChar(k, DEF_TYPE[i]);
    keyOf(k, sizeof(k), 'r', i); chRev[i]  = prefs.getBool(k, false);
    keyOf(k, sizeof(k), 'e', i); chEn[i]   = prefs.getBool(k, true);
    keyOf(k, sizeof(k), 'p', i); chExpo[i] = prefs.getUChar(k, 0);
    keyOf(k, sizeof(k), 'm', i); chMaxP[i] = prefs.getUChar(k, 100);
    keyOf(k, sizeof(k), 'l', i); chLed[i]  = prefs.getUChar(k, LED_BOTH);
    if (chExpo[i] > 100) chExpo[i] = 100;
    if (chMaxP[i] > 100) chMaxP[i] = 100;
    if (chMaxP[i] < MAXP_MIN) chMaxP[i] = MAXP_MIN;
    if (chLed[i] > LED_OFF) chLed[i] = LED_BOTH;
  }
  prefs.end();

  prefs.begin("led", false);
  ledGreenOn = prefs.getBool("g", true);
  ledRedOn   = prefs.getBool("r", true);
  prefs.end();

  prefs.begin("alerts", false);
  for (int e = 0; e < NUM_EV; e++) {
    snprintf(k, sizeof(k), "e%d", e);
    uint16_t v = prefs.getUShort(k, 0xFFFF);
    ev[e] = (v == 0xFFFF) ? DEF_EV[e] : unpackEv(v, e);
  }
  prefs.end();

  prefs.begin("bat", false);
  divRatio = prefs.getFloat("div", DEF_DIVIDER);
  vFull    = prefs.getFloat("full", DEF_VFULL);
  vEmpty   = prefs.getFloat("empty", DEF_VEMPTY);
  vMinSeen = prefs.getFloat("min", 99.0f);
  batWarn  = prefs.getUChar("warn", DEF_WARN);
  batCrit  = prefs.getUChar("crit", DEF_CRIT);
  batCurve = prefs.getUChar("curve", 1);
  prefs.end();
  if (divRatio < 2.0f || divRatio > 5.0f) divRatio = DEF_DIVIDER;
  if (vFull - vEmpty < 0.5f) { vFull = DEF_VFULL; vEmpty = DEF_VEMPTY; }
  if (batCrit < 1 || batCrit >= batWarn) { batWarn = DEF_WARN; batCrit = DEF_CRIT; }

  loadScreens();

  prefs.begin("radio", false);
  uint8_t tp = prefs.getUChar("txp", TX_PWR_DEF);
  uint8_t tr = prefs.getUChar("trt", TELEM_RATE_DEF);
  prefs.end();
  telemRate = (tr < TELEM_RATE_MIN || tr > TELEM_RATE_MAX) ? TELEM_RATE_DEF : tr;
  txPower = TX_PWR_DEF;
  for (int i = 0; i < 8; i++) if (TXP_LIST[i] == tp) txPower = tp;

  prefs.begin("bind", false);
  bindId = prefs.getUInt("id", DEF_BIND_ID);
  prefs.end();

  loadPersistLog();

  LOGF("Config loaded from NVS, deadzone=%d, divider=%.4f, full=%.2f, empty=%.2f",
                deadzone, divRatio, vFull, vEmpty);
}

void saveCenter(int i) {
  char k[8];
  prefs.begin("joy", false);
  keyOf(k, sizeof(k), 'c', i); prefs.putShort(k, cenV[i]);
  prefs.end();
}

void saveRange(int i) {
  char k[8];
  prefs.begin("joy", false);
  keyOf(k, sizeof(k), 'n', i); prefs.putShort(k, mnV[i]);
  keyOf(k, sizeof(k), 'x', i); prefs.putShort(k, mxV[i]);
  prefs.end();
}

void saveDeadzone() {
  prefs.begin("joy", false);
  prefs.putShort("dz", deadzone);
  prefs.end();
}

void saveChannelCfg(int i) {
  char k[8];
  prefs.begin("chcfg", false);
  keyOf(k, sizeof(k), 't', i); prefs.putUChar(k, chType[i]);
  keyOf(k, sizeof(k), 'r', i); prefs.putBool(k, chRev[i]);
  keyOf(k, sizeof(k), 'e', i); prefs.putBool(k, chEn[i]);
  keyOf(k, sizeof(k), 'p', i); prefs.putUChar(k, chExpo[i]);
  keyOf(k, sizeof(k), 'm', i); prefs.putUChar(k, chMaxP[i]);
  keyOf(k, sizeof(k), 'l', i); prefs.putUChar(k, chLed[i]);
  prefs.end();
}

void saveLedCfg() {
  prefs.begin("led", false);
  prefs.putBool("g", ledGreenOn);
  prefs.putBool("r", ledRedOn);
  prefs.end();
}

void saveEv(int e) {
  char k[8];
  snprintf(k, sizeof(k), "e%d", e);
  prefs.begin("alerts", false);
  prefs.putUShort(k, packEv(ev[e], e));
  prefs.end();
}

void saveAllEv() {
  for (int e = 0; e < NUM_EV; e++) saveEv(e);
}

void saveBatCfg() {
  prefs.begin("bat", false);
  prefs.putFloat("div", divRatio);
  prefs.putFloat("full", vFull);
  prefs.putFloat("empty", vEmpty);
  prefs.putUChar("warn", batWarn);
  prefs.putUChar("crit", batCrit);
  prefs.putUChar("curve", batCurve);
  prefs.end();
}

void saveBatMin() {
  prefs.begin("bat", false);
  prefs.putFloat("min", vMinSeen);
  prefs.end();
}

void readAll() {
  unsigned long t0 = micros();
  for (int i = 0; i < NUM_CH; i++) {
    uint8_t b = CH[i].ads;
    raw[i] = adsOk[b] ? adsBank[b]->readADC_SingleEnded(CH[i].input) : cenV[i];
  }
  readUs = micros() - t0;
}

bool isMoved(int i) {
  return adsOk[CH[i].ads] && abs(raw[i] - cenV[i]) > deadzone;
}

// -100..+100 по калибровке, симметрично относительно центра
int normalizeBase(int i) {
  int c = cenV[i];
  long v = 0;
  if (raw[i] >= c) {
    int span = mxV[i] - c;
    if (span > 0) v = (long)(raw[i] - c) * 100 / span;
  } else {
    int span = c - mnV[i];
    if (span > 0) v = (long)(raw[i] - c) * 100 / span;
  }
  return (int)constrain(v, -100L, 100L);
}

// Значение канала БЕЗ Expo/Max: мёртвая зона -> Reverse -> защёлка для тумблера.
// Именно оно используется для навигации по меню (Expo/Max на меню не влияют).
int valBase(int i) {
  if (!isMoved(i)) return 0;
  int v = normalizeBase(i);
  if (chRev[i]) v = -v;
  if (chType[i] == T_SW3) {
    if (v > 50) v = 100;
    else if (v < -50) v = -100;
    else v = 0;
  }
  return v;
}

// Кривая: плавный центр (expo 0..100 %) и ограничение максимума (maxp %)
int applyCurve(int v, int expo, int maxp) {
  float f = v / 100.0f;
  float k = expo / 100.0f;
  float o = ((1.0f - k) * f + k * f * f * f) * (maxp / 100.0f) * 100.0f;
  return (int)(o >= 0 ? o + 0.5f : o - 0.5f);
}

// Итоговое значение канала (то, что уйдёт в приёмник): valBase -> Expo -> Max.
// Для тумблера Switch3 Expo и Max не применяются.
int val(int i) {
  int v = valBase(i);
  if (v != 0 && chType[i] == T_JOY) v = applyCurve(v, chExpo[i], chMaxP[i]);
  return v;
}

// канал мешает Safe Start: включён и (не в центре или нет связи с АЦП)
bool chBad(int i) {
  if (!chEn[i]) return false;
  if (!adsOk[CH[i].ads]) return true;
  return isMoved(i);
}

// ============ СОСТОЯНИЯ ============
State   state = ST_BOOT;
MenuScr menuScr = MS_MAIN;
int     sel = 0, menuTop = 0;
int     idxMain = 0, idxTx = 0, idxCal = 0, idxChList = 0, idxChEdit = 0, idxBat = 0;
int     idxLed = 0, idxEvList = 0, idxEvEdit = 0, idxPreset = 0, idxRx = 0, idxRxWifi = 0, idxRxCh = 0, idxRxDir = 0;
int     idxNets = 0, idxWifi = 0, netSlot = 0;
// выбор сети (скан) и ввод текста с пульта
NetFound pickNet[8];
int     pickN = 0;
bool    pickScanning = false;
unsigned long pickT0 = 0;
char    txtBuf[64];
int     txtLen = 0, txtMax = 63, txtCur = 0, txtStage = 0;
char    txtSsid[33];
static const char TXT_SET[] = "abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.-_@!?#$%&*+=/:;,~()[]{} ";
#define TXT_N  ((int)(sizeof(TXT_SET) - 1))      // символов; дальше два служебных: DEL и OK
int     editCh = 0;
int     dzEdit = DEF_DEADZONE;
int     editInt = 0;            // для Expo / Max
int     batKind = BE_DIV;
int     batEditCv = 0;          // сотые доли вольта
int     batEditI = 0;           // проценты
int     evSel = 0;              // выбранное событие в LED & Alerts
unsigned long gameFlashUntil = 0;   // до этого момента все красные светодиоды горят (проигрыш в игре)
bool    evFromLed = false;      // открыто ярлыком Heartbeat -> назад в LED & Alerts
int     testEv = 0;             // событие на экране Test alerts
int     screenNo = 0;
SimMode simMode = SIM_RADIO;    // источник приёмника: SIM_RADIO = настоящее радио
unsigned long restoredAt = 0;   // когда была вспышка «связь восстановлена»
unsigned long bootStart = 0;
unsigned long safeSince = 0;

// ---- состояния меню ----
int  idxScr = 0, idxPower = 0, idxBind = 0, idxEvLog = 0, idxDbg = 0, idxFr = 0, idxRxGlob = 0;
int  scrSel = 0;                 // позиция выбранного экрана в scrOrder
int  dbgPage = 0, dbgTop = 0;    // страница Debug и первая видимая строка описания
int  pinDigit[4] = { 0, 0, 0, 0 };
int  pinPos = 0;
ConfAct confAct = CA_CLEARLOG;   // что подтверждаем
MenuScr confBack = MS_TX;        // куда вернуться
int  confIdx = 0;
unsigned long bindStart = 0, bindDoneAt = 0;
int  bindPhase = 0;              // 0 ждём, 1 успех, 2 тайм-аут
unsigned long restartAt = 0;     // не 0 = перезагрузка в этот момент (после Factory Reset)

const char* stateName() {
  switch (state) {
    case ST_BOOT:    return "BOOT";
    case ST_SAFE:    return "SAFE";
    case ST_RUN:     return "RUN";
    case ST_MENU:    return "MENU";
    case ST_CAL_C:   return "CAL-C";
    case ST_CAL_R:   return "CAL-R";
    case ST_CAL_RES: return "CAL-RES";
    case ST_ESTOP:   return "ESTOP";
    case ST_WIFI:    return "WIFI";
    case ST_GH:      return "GITHUB";
  }
  return "?";
}

bool hlUp = false;                // радио: свежая телеметрия от приёмника есть
bool hlSyncBusy = false;          // радио: идёт полное чтение настроек приёмника
bool rxLinked() { return simMode == SIM_LINK || (simMode == SIM_RADIO && hlUp); }
bool rxReady() { return rxLinked() && !hlSyncBusy; }   // меню Receiver доступно

// ---- короткие сообщения внизу экрана ----
char toastText[20] = "";
unsigned long toastUntil = 0;
void toast(const char* t) {
  snprintf(toastText, sizeof(toastText), "%s", t);
  toastUntil = millis() + TOAST_MS;
}
bool toastActive() { return (long)(toastUntil - millis()) > 0; }

// ---- Wi-Fi Update: состояние (реализация ниже, после goSafe) ----
bool wifiUp = false;             // точка доступа включена (state == ST_WIFI)
bool wifiEver = false;           // телефон хотя бы раз подключился
bool wifiPage = false;           // страница открыта (после этого адрес и пароль убираются с экрана)
unsigned long wifiT0 = 0, wifiAct = 0;
bool upActive = false;           // идёт приём файла прошивки
bool upOk = false;
char upErr[48] = "";
char upVer[16] = "";
size_t upTotal = 0, upDone = 0;
char wifiSsid[16] = "";
uint8_t rxWifiMode = 0;          // 0 с телефона, 1 GitHub (имитация)
uint8_t rxWifiSt = 0;            // Wi-Fi приёмника: 0 выкл, 1 запрос отправлен, 2 приёмник ответил
unsigned long rxWifiAt = 0;
char rxWifiSsid[16] = "";
void wifiBegin();
void wifiStop(const char* why);
void rxWifiBegin(int mode);
void gameEnter();
void gameStep(unsigned long now, int b);
bool gameOn();

// ---- сохранённые Wi-Fi сети (NVS "wifi"): до NET_MAX штук ----
char netSsid[NET_MAX][33];
char netPass[NET_MAX][64];

void netLoad() {
  memset(netSsid, 0, sizeof(netSsid));
  memset(netPass, 0, sizeof(netPass));
  prefs.begin("wifi", true);
  for (int i = 0; i < NET_MAX; i++) {
    char k[4];
    snprintf(k, sizeof(k), "s%d", i); prefs.getString(k, netSsid[i], sizeof(netSsid[i]));
    snprintf(k, sizeof(k), "p%d", i); prefs.getString(k, netPass[i], sizeof(netPass[i]));
  }
  prefs.end();
}

static void netStore(int i) {
  char k[4];
  prefs.begin("wifi", false);
  snprintf(k, sizeof(k), "s%d", i);
  if (netSsid[i][0]) prefs.putString(k, netSsid[i]); else prefs.remove(k);
  snprintf(k, sizeof(k), "p%d", i);
  if (netSsid[i][0]) prefs.putString(k, netPass[i]); else prefs.remove(k);
  prefs.end();
}

int netCount() {
  int c = 0;
  for (int i = 0; i < NET_MAX; i++) if (netSsid[i][0]) c++;
  return c;
}

// 0 = сохранено, 1 = нет места, 2 = неверные данные
int netAdd(const char* ssid, const char* pass) {
  if (!ssid || !ssid[0] || strlen(ssid) > 32 || strlen(pass ? pass : "") > 63) return 2;
  int slot = -1;
  for (int i = 0; i < NET_MAX; i++) if (strcmp(netSsid[i], ssid) == 0) slot = i;     // та же сеть: обновить пароль
  if (slot < 0) for (int i = 0; i < NET_MAX; i++) if (!netSsid[i][0]) { slot = i; break; }
  if (slot < 0) return 1;
  snprintf(netSsid[slot], sizeof(netSsid[slot]), "%s", ssid);
  snprintf(netPass[slot], sizeof(netPass[slot]), "%s", pass ? pass : "");
  netStore(slot);
  return 0;
}

void netDel(int i) {
  if (i < 0 || i >= NET_MAX) return;
  netSsid[i][0] = 0; netPass[i][0] = 0;
  netStore(i);
}

void netClearAll() {
  for (int i = 0; i < NET_MAX; i++) netDel(i);
}

// ---- загрузка с GitHub: состояние (реализация ниже, после wifiTick) ----
enum GhStage { G_SCAN, G_CONN, G_API, G_ASK, G_DL, G_OK, G_ERR, G_TEST };
GhStage ghStage = G_SCAN;
char ghNet[33] = "";             // сеть, к которой подключаемся
char ghVer[16] = "";             // версия найденной прошивки
char ghErr[40] = "";
char ghErr2[33] = "";
int  ghTestSlot = -1;            // >= 0: только проверка подключения к этой сохранённой сети
bool ghFresh = false;            // проверка сразу после ввода сети
MenuScr ghBackScr = MS_WIFI;     // куда вернуться при выходе
int  ghBackIdx = 0;
int  ghRssi = 0;
volatile int ghReason = 0;       // последний код причины отключения Wi-Fi (событие)
int  ghErrCode = 0;              // код для экрана ошибки
int  ghPassLen = 0;              // длина пароля выбранной сети
char ghIp[20] = "";
char ghSha[65] = "";             // SHA-256 из ответа GitHub
char ghUrl[200] = "";
size_t ghSize = 0, ghDone = 0;
unsigned long ghT0 = 0;
void ghBegin();
void ghTestBegin(int slot, MenuScr back, int idx, bool fresh);
void ghStop(const char* why);

// ============ БАТАРЕЯ ============
float vbat = 0;                 // напряжение, показываемое на экране
float pinV = 0;                 // последнее напряжение на пине (до делителя)
int   vbatPct = 0;
int   batLevel = 0;             // 0 норма, 1 LOW BATTERY, 2 BATT CRITICAL
int   batCnt = 0;
bool  batUsb = false;
bool  batInit = false;
unsigned long lastBatSlow = 0, lastBatFast = 0, lastMinSave = 0;

float measurePin() {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(VBAT_PIN);
  return sum / 16.0f / 1000.0f;
}

// Типовая кривая Li-ion: доля диапазона напряжения -> процент заряда
const float CURVE_F[7] = { 0.00f, 0.25f, 0.42f, 0.58f, 0.75f, 0.92f, 1.00f };
const float CURVE_P[7] = { 0.0f,  5.0f,  15.0f, 45.0f, 75.0f, 95.0f, 100.0f };

int pctFromV(float v) {
  float span = vFull - vEmpty;
  if (span < 0.1f) span = 0.1f;
  float f = (v - vEmpty) / span;
  if (f <= 0.0f) return 0;
  if (f >= 1.0f) return 100;
  if (batCurve == 0) return (int)(f * 100.0f + 0.5f);
  for (int i = 1; i < 7; i++) {
    if (f <= CURVE_F[i]) {
      float t = (f - CURVE_F[i - 1]) / (CURVE_F[i] - CURVE_F[i - 1]);
      return (int)(CURVE_P[i - 1] + t * (CURVE_P[i] - CURVE_P[i - 1]) + 0.5f);
    }
  }
  return 100;
}

void batTick(unsigned long now) {
  bool live = (state == ST_MENU && (menuScr == MS_BAT || menuScr == MS_BATEDIT));
  unsigned long period = live ? BAT_LIVE_MS : BAT_FAST_MS;
  if (batInit && (now - lastBatFast) < period) return;
  lastBatFast = now;

  pinV = measurePin();
  float v = pinV * divRatio;
  batUsb = (v < BAT_USB_V);

  // самое низкое напряжение, которое видел пульт (пригодится для Empty)
  if (!batUsb && v > 5.0f && v < vMinSeen - 0.03f && (now - lastMinSave) >= BAT_MIN_SAVE_MS) {
    vMinSeen = v;
    lastMinSave = now;
    saveBatMin();
  }

  bool slow = !batInit || (now - lastBatSlow) >= BAT_UPDATE_MS;
  if (live || slow) {
    vbat = v;
    vbatPct = pctFromV(v);
  }
  if (slow) {
    lastBatSlow = now;
    if (batUsb) {
      batLevel = 0; batCnt = 0;
    } else {
      int up   = (vbatPct <= batCrit) ? 2 : (vbatPct <= batWarn) ? 1 : 0;
      int down = (vbatPct <= batCrit + 3) ? 2 : (vbatPct <= batWarn + 3) ? 1 : 0;
      if (!batInit) {
        batLevel = up; batCnt = 0;      // при старте сразу
      } else {
        int cand = batLevel;
        if (up > batLevel) cand = up;
        else if (down < batLevel) cand = down;
        if (cand != batLevel) {
          if (++batCnt >= 3) { batLevel = cand; batCnt = 0; }   // 3 замера подряд
        } else {
          batCnt = 0;
        }
      }
    }
    LOGF("Battery: %.2f V (%d%%) level=%d%s", vbat, vbatPct, batLevel, batUsb ? " [USB]" : "");
  }
  batInit = true;
}

void drawBattery() {
  const int battW = 10, battH = 30, nubW = 4, nubH = 3;
  int battX = SCREEN_W - 12;
  int battY = SCREEN_H - battH - nubH - 10;

  bool show = !(batLevel == 2 && ((millis() / 500) % 2 == 1));   // критический: значок мигает
  if (show) {
    display.fillRect(battX + (battW - nubW) / 2, battY, nubW, nubH, SH110X_WHITE);
    display.drawRect(battX, battY + nubH, battW, battH, SH110X_WHITE);
    int innerH = battH - 4;
    int fillH = (int)(innerH * (vbatPct / 100.0f));
    if (batUsb) fillH = 0;
    display.fillRect(battX + 2, battY + nubH + battH - 2 - fillH, battW - 4, fillH, SH110X_WHITE);
  }

  char s[8];
  if (batUsb) snprintf(s, sizeof(s), "USB");
  else        snprintf(s, sizeof(s), "%d%%", vbatPct);
  int16_t x1, y1; uint16_t w, h;
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(battX + battW - w, battY + nubH + battH + 2);
  display.print(s);
}

// Нижняя строка: подсказка. Если батарея низкая, раз в секунду вместо подсказки
// показывается предупреждение. Короткие сообщения (toast) имеют приоритет.
void drawHintText(const char* t, int y) {
  const char* s = t;
  if (toastActive()) s = toastText;
  else if (batLevel > 0 && ((millis() / 1000) % 2 == 0)) s = (batLevel == 2) ? "BATT CRITICAL!" : "LOW BATTERY";
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, y);
  display.print(s);
}

// ---- навигация тумблерами: один шаг за отклонение, повтор только через центр ----
// repeat = true (числовые экраны): при удержании тумблера значение идёт само.
Nav navA = { false, 0, 0, 0, 0 };   // MENU_NAV_CH
Nav navB = { false, 0, 0, 0, 0 };   // MENU_ENTER_CH

void navReset() {
  navA = { false, 0, 0, 0, 0 };
  navB = { false, 0, 0, 0, 0 };
}

int navPoll(int ch, Nav &n, unsigned long now, bool repeat) {
  int v = valBase(ch);   // без Expo/Max
  if (!n.armed) {
    if (repeat && n.dir != 0) {
      int d = (v >= NAV_ON) ? 1 : (v <= -NAV_ON) ? -1 : 0;
      if (d == n.dir) {
        if ((now - n.held) >= REPEAT_DELAY_MS && (now - n.last) >= REPEAT_MS) {
          n.last = now;
          return d;
        }
        return 0;
      }
      n.dir = 0;
    }
    if (abs(v) < NAV_OFF) {
      if (n.since == 0) n.since = now;
      if (now - n.since >= NAV_REARM_MS) n.armed = true;
    } else {
      n.since = 0;
    }
    return 0;
  }
  if (v >= NAV_ON)  { n.armed = false; n.since = 0; n.dir = 1;  n.held = now; n.last = now; return 1; }
  if (v <= -NAV_ON) { n.armed = false; n.since = 0; n.dir = -1; n.held = now; n.last = now; return -1; }
  return 0;
}

// множитель шага при долгом удержании: 1 .. 10
int navMult(const Nav &n, unsigned long now) {
  if (n.dir == 0) return 1;
  int m = 1 + (int)((now - n.held) / 1000UL);
  return m > 10 ? 10 : m;
}

// ============ ПРИЁМНИК (пока симулятор; потом заменится радио: SetConfig -> Ack) ============
// Значения: [параметр][канал][сторона]. Сторона 0 = Extend (A), 1 = Retract (B).
// Параметры канала хранятся со стороной 0, общие параметры: канал 0, сторона 0.
uint16_t simNvs[NUM_RXP][NUM_CH][2];
uint16_t simRam[NUM_RXP][NUM_CH][2];
// Режим CONTROL хранится в памяти приёмника; пульт запрашивает его при подключении (на радио: SetConfig 0xFFFF)
bool     rxCfgKnown = false;
uint16_t rxModeCache = 0;

void simInit() {
  for (int p = 0; p < NUM_RXP; p++)
    for (int c = 0; c < NUM_CH; c++)
      for (int d = 0; d < 2; d++) {
        simNvs[p][c][d] = RXP[p].def;
        simRam[p][c][d] = RXP[p].def;
      }
}

void rxNorm(int param, int &ch, int &dir) {
  if (RXP[param].scope == PS_GLOB) { ch = 0; dir = 0; }
  else if (RXP[param].scope == PS_CH) { dir = 0; }
}

// «Только прочитать» (на радио это SetConfig со значением 0xFFFF): вернуть текущее значение
uint16_t rxQuery(int ch, int dir, int param) {
  rxNorm(param, ch, dir);
  return simRam[param][ch][dir];
}

// Применить значение (на радио: SetConfig -> Ack). commit = сохранить в память приёмника.
// Возвращает значение, которое реально применил приёмник.
bool hlCfgPut(uint8_t op, uint8_t param, uint8_t ch, uint8_t dir, uint16_t val, bool save, uint8_t kind, uint16_t old);
uint16_t rxSet(int ch, int dir, int param, int value, bool commit) {
  rxNorm(param, ch, dir);
  value = constrain(value, (int)RXP[param].mn, (int)RXP[param].mx);
  if (simMode == SIM_RADIO) {                          // радио: копия меняется сразу, приёмник подтвердит (ACK) или значение вернётся
    uint16_t old = simRam[param][ch][dir];
    simRam[param][ch][dir] = value;
    if (param == RXP_MODE) rxModeCache = value;
    hlCfgPut(HLO_SET, (uint8_t)param, (uint8_t)ch, (uint8_t)dir, (uint16_t)value, commit, 0, old);
    return value;
  }
  simRam[param][ch][dir] = value;
  if (commit) simNvs[param][ch][dir] = value;
  if (param == RXP_MODE) rxModeCache = value;
  return value;
}

// потеря связи: приёмник сам откатывает несохранённое
void rxRevertAll() {
  for (int p = 0; p < NUM_RXP; p++)
    for (int c = 0; c < NUM_CH; c++)
      for (int d = 0; d < 2; d++) simRam[p][c][d] = simNvs[p][c][d];
  rxModeCache = simRam[RXP_MODE][0][0];
}

void rxCfgRequest() {
  rxModeCache = simRam[RXP_MODE][0][0];
  rxCfgKnown = true;
  LOGF("RX config: CONTROL=%s", rxModeCache ? "CURRENT" : "PWM");
}

// сброс настроек приёмника (привязка не затрагивается)
void rxFactoryReset() {
  if (simMode == SIM_RADIO) { hlCfgPut(HLO_CMD, HLC_FACTORY, 0, 0, HL_FACTORY_KEY, false, 0, 0); return; }   // приёмник проверит условия и ответит
  simInit(); rxModeCache = simRam[RXP_MODE][0][0];
}

int  rxTopic = RT_LIMIT;           // что выбрано в меню Receiver
int  rxParam = RXP_LIMIT, rxCh = 0, rxDir = 0;
int  rxCur = 0, rxOrig = 0;        // текущее (из Ack) и исходное значение
MenuScr rxRetScr = MS_RXDIR;       // куда вернуться после правки
bool leverArmed = false;           // тестовый джойстик разрешён, когда побывал в центре

bool isRxScreen() {
  return menuScr == MS_RX || menuScr == MS_RXCH || menuScr == MS_RXDIR || menuScr == MS_RXGLOB ||
         menuScr == MS_RXEDIT || menuScr == MS_RXSTAT || menuScr == MS_RXWIFI || menuScr == MS_RXWIFIM;
}

// все включённые каналы, кроме тумблеров навигации и тестового джойстика, в центре
bool othersCentered() {
  for (int i = 0; i < NUM_CH; i++) {
    if (i == MENU_NAV_CH || i == MENU_ENTER_CH || i == TEST_LEVER_CH) continue;
    if (chBad(i)) return false;
  }
  return true;
}

// ---- симулятор телеметрии приёмника (пока нет радио) ----
// Условный ток = 3.5 А на 100% ШИМ. Выше 1.3 x лимита: перегрузка (канал отключается, пока рычаг не в центре),
// от лимита до 1.3 x лимита: ограничение по лимиту.
Telem tel;
bool  telValid = false;
bool  ovlLatch[NUM_CH];
uint16_t simPeak[NUM_CH];
uint32_t rfTx = 0, rfRx = 0, rfLost = 0;
int   rfRssi = -63, rfSnr10 = 95, rfRate = 26;
unsigned long rfAgeMs = 0;
uint16_t rxId = 0x3F2A;

void simTelemetry(unsigned long now) {
  if (simMode == SIM_RADIO) {                            // телеметрия приходит по радио (hlStep), здесь только сброс при потере связи
    telValid = hlUp;
    if (!hlUp) memset(&tel, 0, sizeof(tel));
    return;
  }
  memset(&tel, 0, sizeof(tel));
  if (!rxLinked()) {
    telValid = false;
    memset(ovlLatch, 0, sizeof(ovlLatch));
    return;
  }
  telValid = true;
  tel.vrx = 124;
  tel.rssi = (int8_t)rfRssi;
  tel.kill = (state == ST_ESTOP) ? 1 : 0;
  tel.reason = 0;
  for (int c = 0; c < NUM_CH; c++) {
    int p = ctl[c];
    int side = (p < 0) ? 1 : 0;
    int mag = abs(p);
    if (!simRam[RXP_CHEN][c][0]) continue;
    bool curMode = simRam[RXP_MODE][0][0] != 0;
    int maxd = simRam[RXP_MAXD][c][side] / 10;       // 0.1 % -> %
    if (!curMode && mag > maxd) mag = maxd;
    if (ovlLatch[c]) {
      if (p == 0) {
        ovlLatch[c] = false;
      } else {
        tel.mOvl |= (uint8_t)(1 << c);
        tel.side |= (uint8_t)(side << c);
        tel.cur[c] = simPeak[c];
        continue;
      }
    }
    int raw = mag * 35 / 100;                // 0.1 А при 100 % ШИМ около 3.5 А
    if (curMode) {                           // ток: от стартового до максимального, ШИМ считается из тока
      int i0 = simRam[RXP_STARTI][c][side], i1 = simRam[RXP_MAXI][c][side];
      raw = (mag > 0) ? i0 + mag * (i1 - i0) / 100 : 0;
      mag = constrain(raw * 100 / 35, 0, 100);
    }
    int lim = simRam[RXP_LIMIT][c][side];
    if (raw > 0 && raw > lim * 13 / 10) {
      ovlLatch[c] = true;
      simPeak[c] = raw;
      tel.mOvl |= (uint8_t)(1 << c);
      tel.side |= (uint8_t)(side << c);
      tel.cur[c] = raw;
      continue;
    }
    tel.pwm[c] = (int8_t)(side ? -mag : mag);
    if (raw > 0 && raw >= lim) { tel.mLim |= (uint8_t)(1 << c); raw = lim; }
    tel.cur[c] = raw;
    if (mag > 0) tel.side |= (uint8_t)(side << c);
  }
  rfTx++; rfRx++;
  rfAgeMs = now % 300;
}

// ============================================================
// РАДИО: HydroLink v1 (SX1262, 868 МГц). Пульт (передатчик).
// Все числа little-endian. Первый байт: старший полубайт = тип пакета, младший = флаги.
// Байты 1..2 = ID пары (из bindId), в конце CRC16-CCITT (poly 0x1021, init 0xFFFF) по всем предыдущим байтам.
//  CONTROL (1) пульт -> RX: [0]=0x1F, [1..2]=пара, [3]=номер, [4]=маска каналов, [5..]=значения int8 по возрастанию номера канала, CRC16.
//     Длина = 7 + n, n = число единиц в маске. Флаги: 1 E-Stop, 2 запрос телеметрии, 4 тест рычага, 8 пульт только что включён.
//     Heartbeat = CONTROL с маской 0.
//  TELEM (2) RX -> пульт, 36 байт: [3]=номер, [4]=последний принятый номер управления, [5]=SNR (int8, дБ), [6]=пропущено пакетов,
//     [7..14]=ток в импульсе по 8 каналам (0.1 А), [15..22]=ШИМ int8 (минус = сторона B), [23]=лимит, [24]=перегрузка, [25]=обрыв (маски),
//     [26]=сторона ошибки (бит на канал, 1 = B), [27]=статус (бит0 killswitch, биты1..2 причина: 0 пульт, 1 кнопка RX, 2 сбой RX, 3 нет связи),
//     [28]=напряжение RX (0.1 В), [29]=RSSI (int8, дБм), [30]=счётчик перезагрузок RX, [31]=CRC8 сохранённой конфигурации,
//     [32]=состояние Wi-Fi, [33]=параметр Wi-Fi (процент или код ошибки), CRC16.
//  CFG (3) пульт -> RX, 12 байт: [0] флаг 1 = сохранить в память RX, [3]=метка, [4]=операция (0 SET, 1 GET, 2 BATCH, 3 CMD, 4 NET внутр.),
//     [5]=параметр/команда, [6]=канал, [7]=сторона, [8..9]=значение, CRC16. Значение 0xFFFF в SET не используется.
//  ACK (4) RX -> пульт, 12 байт: [3]=метка, [4]=статус (0 ок, 1 ошибка, 2 отказ сейчас, 3 занят), [5..7]=параметр/канал/сторона, [8..9]=значение.
//  BATCH (5) RX -> пульт: [3]=метка, [4]=параметр, [5]=число значений n, [6..]=n значений uint16, CRC16. Для параметров на соленоид n=16 (канал*2+сторона),
//     на канал n=8, общие n=1.
//  NET (6) пульт -> RX: [3]=метка, [4]=слот, [5]=всего сетей, [6]=длина имени, имя, длина пароля, пароль, CRC16. Ответ: ACK с параметром 0xE0.
//  INFO (7) RX -> пульт: [3]=0, [4]=вид (1 версия на GitHub, 2 имя сети RX-xxxx), [5]=длина, текст, CRC16.
//  BIND (8) пульт -> все: [1..2]=0xFFFF, [3]=номер, [4..7]=новый bindId, CRC16. BINDOK (9) RX -> пульт: [1..2]=новая пара, [3]=номер, [4..7]=bindId.
// ============================================================
SX1262 hlRadio = new Module(HL_PIN_NSS, HL_PIN_DIO1, HL_PIN_RST, HL_PIN_BUSY);
volatile bool hlIrq = false;
void IRAM_ATTR hlIsr() { hlIrq = true; }

bool hlInit = false, hlTxBusy = false;
int  hlErr = 0;
uint8_t  hlSeq = 0, hlTag = 0, hlBindSeq = 0;
unsigned long hlTxStart = 0, hlTxEnd = 0, hlNextTx = 0, hlLastReq = 0, hlLastRx = 0, hlWaitUntil = 0, hlLastNonZero = 0, hlRateAt = 0;
unsigned long hlReqAt = 0, hlBootAt = 0;
uint32_t hlRateCnt = 0;
int  hlRtt = 0;
uint8_t hlWait = 0;                    // 0 нет, 1 ждём телеметрию, 2 ждём подтверждение, 3 ждём BATCH
bool hlMissReq = false;                // последний запрос телеметрии остался без ответа
bool hlBootFlag = true;                // пульт только что включён (до первой телеметрии)
bool hlHbSched = false;                // следующая передача запланирована как heartbeat
bool hlWaitEs = false;                 // ожидаемый пакет управления уже содержал E-Stop

// синхронизация (раздел 26)
bool hlSynced = false;                 // после включения пульта конфигурация считана
int  hlSyncStep = 0;
uint8_t hlBoots = 0;                   // счётчик перезагрузок приёмника при последней синхронизации
uint8_t hlBootsNow = 0;                // счётчик перезагрузок в телеметрии, с которой началась синхронизация
uint8_t hlCrcIgnore = 0;               // CRC приёмника, которое не совпадает с копией и повторно не читается
bool hlCrcIgnoreOn = false;
unsigned long hlSyncRetryAt = 0;
bool hlMirrorDirty = false;
unsigned long hlMirrorAt = 0;
#define HL_SYNC_STEPS   (4 + NUM_RXP)

// очередь настроек
HlCfg hlQ[8];
int   hlQn = 0;
HlCfg hlCur;
bool  hlCurOn = false;

// привязка
bool hlBindOn = false, hlBindDone = false;
uint32_t hlBindNew = 0;
unsigned long hlBindAt = 0;

// Wi-Fi приёмника (по телеметрии и INFO)
char hlRxVer[24] = "";                 // версия на GitHub (INFO 1)
bool hlWifiAck = false;                // приёмник принял команду Wi-Fi
uint8_t hlWifiAckSt = 0;

bool rxLeverParam(int p);

uint16_t hlCrc16(const uint8_t* d, size_t n) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    c ^= (uint16_t)d[i] << 8;
    for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

uint16_t hlPair() {
  uint32_t b = bindId;
  uint16_t p = (uint16_t)(b ^ (b >> 16));
  if (p == 0 || p == 0xFFFF) p ^= 0x5A5A;
  return p;
}

static size_t hlFinish(uint8_t* b, size_t n) {
  uint16_t c = hlCrc16(b, n);
  b[n] = (uint8_t)(c & 0xFF); b[n + 1] = (uint8_t)(c >> 8);
  return n + 2;
}

static bool hlTx(const uint8_t* b, size_t n, unsigned long now) {
  if (hlRadio.startTransmit((uint8_t*)b, n) == RADIOLIB_ERR_NONE) { hlTxBusy = true; hlTxStart = now; return true; }
  hlRadio.startReceive();
  return false;
}

// CRC8 сохранённой конфигурации приёмника (копия пульта): те же значения и тот же порядок на приёмнике
uint8_t hlCfgCrc() {
  uint8_t c = 0;
  for (int p = 0; p < NUM_RXP; p++) {
    int nc = (RXP[p].scope == PS_GLOB) ? 1 : NUM_CH;
    int nd = (RXP[p].scope == PS_SOL) ? 2 : 1;
    for (int ch = 0; ch < nc; ch++)
      for (int d = 0; d < nd; d++) {
        uint16_t v = simNvs[p][ch][d];
        for (int k = 0; k < 2; k++) {
          c ^= (uint8_t)(k ? (v >> 8) : (v & 0xFF));
          for (int j = 0; j < 8; j++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
        }
      }
  }
  return c;
}

void hlMirrorSave() {
  prefs.begin("rxcopy", false);
  prefs.putBytes("nvs", simNvs, sizeof(simNvs));
  prefs.putUChar("boots", hlBoots);
  prefs.end();
  hlMirrorDirty = false;
}

void hlMirrorLoad() {
  prefs.begin("rxcopy", true);
  size_t n = prefs.getBytes("nvs", simNvs, sizeof(simNvs));
  hlBoots = prefs.getUChar("boots", 0);
  prefs.end();
  if (n != sizeof(simNvs)) { simInit(); return; }          // нет копии: значения по умолчанию
  memcpy(simRam, simNvs, sizeof(simRam));
  rxModeCache = simRam[RXP_MODE][0][0];
}

void hlMirrorForget() {
  prefs.begin("rxcopy", false);
  prefs.clear();
  prefs.end();
  simInit();
  rxModeCache = simRam[RXP_MODE][0][0];
  hlSynced = false;
  hlSyncBusy = false;
  hlCurOn = false; hlQn = 0;
  hlCrcIgnoreOn = false;
}

// ---- радио ----
void hlSetPower() {
  if (!hlInit) return;
  hlRadio.setOutputPower((int8_t)txPower);
  hlRadio.setCurrentLimit(txPower > 14 ? 140.0f : 60.0f);
}

void hlBegin() {
  SPI.begin(HL_PIN_SCK, HL_PIN_MISO, HL_PIN_MOSI, HL_PIN_NSS);
  int st = hlRadio.begin(RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, RF_CR, RF_SYNC, (int8_t)txPower, RF_PREAMBLE, RF_TCXO_V);
  if (st != RADIOLIB_ERR_NONE) st = hlRadio.begin(RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, RF_CR, RF_SYNC, (int8_t)txPower, RF_PREAMBLE, 0.0f);
  if (st == RADIOLIB_ERR_NONE) {
    hlRadio.setDio2AsRfSwitch(true);
    hlRadio.setDio1Action(hlIsr);
    st = hlRadio.startReceive();
  }
  hlErr = st;
  hlInit = (st == RADIOLIB_ERR_NONE);
  if (hlInit) hlSetPower();
  LOGF("Radio: %s (%d)", hlInit ? "started" : "init failed", st);
  hlBootAt = millis();
  hlNextTx = millis() + 50;
}

bool hlRadioMode() { return simMode == SIM_RADIO; }

unsigned long hlLostMs() { return 3000UL / telemRate + 450UL; }

// ---- очередь настроек ----
void hlCfgClear() { hlQn = 0; hlCurOn = false; if (hlWait == 2 || hlWait == 3) hlWait = 0; }

bool hlCfgPut(uint8_t op, uint8_t param, uint8_t ch, uint8_t dir, uint16_t val, bool save, uint8_t kind, uint16_t old = 0) {
  if (op == HLO_SET)                                       // повторная правка того же параметра: заменить ещё не отправленную
    for (int i = 0; i < hlQn; i++)
      if (hlQ[i].op == HLO_SET && hlQ[i].param == param && hlQ[i].ch == ch && hlQ[i].dir == dir && hlQ[i].kind == kind) {
        hlQ[i].val = val; hlQ[i].save = hlQ[i].save || save;
        return true;
      }
  if (hlQn >= 8) return false;
  HlCfg &q = hlQ[hlQn++];
  q.op = op; q.param = param; q.ch = ch; q.dir = dir; q.val = val; q.save = save; q.kind = kind; q.tag = 0; q.tries = 0; q.old = old;
  return true;
}

// Приёмник ответил (статус 0) или нет: применить результат к копии пульта
void hlCfgDone(const HlCfg &c, uint8_t status, uint16_t applied) {
  if (c.kind == 1) {                                       // синхронизация
    if (status != 0 && !(c.op == HLO_CMD && c.param == HLC_TXPWR)) {
      LOGF("Sync: step failed (op %d p %d st %d)", c.op, c.param, status);
      hlSyncBusy = false; hlSynced = false; hlSyncRetryAt = millis() + 2000UL;
      if (c.op == HLO_CMD && c.param == HLC_PROTO) toast("Protocol mismatch");
      return;
    }
    return;
  }
  if (c.op == HLO_SET) {
    if (status == 0) {
      simRam[c.param][c.ch][c.dir] = applied;
      if (c.param == RXP_MODE) rxModeCache = applied;
      if (c.save) { simNvs[c.param][c.ch][c.dir] = applied; hlMirrorDirty = true; hlMirrorAt = millis(); }
      if (menuScr == MS_RXEDIT && rxParam == c.param && rxCh == c.ch && rxDir == c.dir && rxCur != (int)applied) rxCur = applied;
    } else {
      simRam[c.param][c.ch][c.dir] = c.old;                // не получилось: возвращаем прежнее
      if (c.param == RXP_MODE) rxModeCache = c.old;
      toast(status == 2 ? "RX refused" : "RX error");
    }
  } else if (c.op == HLO_CMD) {
    if (c.param == HLC_FACTORY) {
      if (status == 0) { hlMirrorForget(); toast("RX reset"); }
      else toast("RX refused");
    } else if (c.param == HLC_WIFI_PHONE || c.param == HLC_WIFI_GH) {
      hlWifiAck = true; hlWifiAckSt = status;
    }
  } else if (c.op == HLO_NET) {
    if (status != 0) { hlWifiAck = true; hlWifiAckSt = status; hlQn = 0; }
  }
}

static void hlSendCfg(unsigned long now) {
  HlCfg &c = hlCur;
  uint8_t b[128];
  size_t n;
  memset(b, 0, sizeof(b));
  b[0] = (uint8_t)((HLT_CFG << 4) | (c.save ? 1 : 0));
  uint16_t pr = hlPair();
  b[1] = (uint8_t)(pr & 0xFF); b[2] = (uint8_t)(pr >> 8);
  b[3] = c.tag;
  if (c.op == HLO_NET) {
    b[0] = (uint8_t)(HLT_NET << 4);
    int slot = c.ch;
    b[4] = (uint8_t)slot; b[5] = (uint8_t)netCount();
    uint8_t ls = (uint8_t)strlen(netSsid[slot]), lp = (uint8_t)strlen(netPass[slot]);
    b[6] = ls; memcpy(b + 7, netSsid[slot], ls);
    b[7 + ls] = lp; memcpy(b + 8 + ls, netPass[slot], lp);
    n = hlFinish(b, 8 + ls + lp);
  } else {
    b[4] = c.op; b[5] = c.param; b[6] = c.ch; b[7] = c.dir;
    b[8] = (uint8_t)(c.val & 0xFF); b[9] = (uint8_t)(c.val >> 8);
    n = hlFinish(b, 10);
  }
  hlWait = (c.op == HLO_BATCH) ? 3 : 2;
  hlWaitUntil = now + 1000UL;
  hlHbSched = false;
  if (!hlTx(b, n, now)) { hlWait = 0; hlNextTx = now + 20; }
}

static void hlSendControl(unsigned long now) {
  uint8_t b[16];
  uint8_t mask = 0;
  bool syncZero = hlSyncBusy || hlBindOn;
  for (int i = 0; i < NUM_CH; i++) if (ctl[i] != 0 && !syncZero) mask |= (uint8_t)(1 << i);
  bool es = (state == ST_ESTOP) || estop.state;
  bool lever = (state == ST_MENU && menuScr == MS_RXEDIT && rxLeverParam(rxParam) && leverArmed);
  bool req;
  if (!hlUp || hlMissReq) req = ((long)(now - hlLastReq) >= (long)HL_SEARCH_MS);
  else                    req = ((long)(now - hlLastReq) >= (long)(1000UL / telemRate));
  uint8_t fl = 0;
  if (es) fl |= HLF_ESTOP;
  if (req) fl |= HLF_REQT;
  if (lever) fl |= HLF_LEVER;
  if (hlBootFlag) fl |= HLF_BOOT;
  b[0] = (uint8_t)((HLT_CONTROL << 4) | fl);
  uint16_t pr = hlPair();
  b[1] = (uint8_t)(pr & 0xFF); b[2] = (uint8_t)(pr >> 8);
  b[3] = ++hlSeq;
  b[4] = mask;
  size_t n = 5;
  for (int i = 0; i < NUM_CH; i++)
    if (mask & (1 << i)) b[n++] = (uint8_t)(int8_t)constrain(ctl[i], -100, 100);
  n = hlFinish(b, n);
  if (req) { hlWait = 1; hlWaitUntil = now + 1000UL; hlLastReq = now; hlReqAt = now; hlWaitEs = es; }
  bool fast = (mask != 0) || es || lever;
  hlHbSched = !fast && (now - hlLastNonZero) >= HL_HB_AFTER_MS;
  if (!hlTx(b, n, now)) { hlWait = 0; hlNextTx = now + 20; return; }
  hlRateCnt++;
  rfTx++;
}

static void hlSendBind(unsigned long now) {
  uint8_t b[16];
  b[0] = (uint8_t)(HLT_BIND << 4);
  b[1] = 0xFF; b[2] = 0xFF;
  b[3] = ++hlBindSeq;
  b[4] = (uint8_t)(hlBindNew & 0xFF); b[5] = (uint8_t)(hlBindNew >> 8);
  b[6] = (uint8_t)(hlBindNew >> 16); b[7] = (uint8_t)(hlBindNew >> 24);
  size_t n = hlFinish(b, 8);
  hlWait = 0;
  if (!hlTx(b, n, now)) hlNextTx = now + 20;
}

// следующий шаг синхронизации (кладёт запрос в очередь)
static void hlSyncNext() {
  if (!hlSyncBusy || hlCurOn || hlQn > 0) return;
  int s = hlSyncStep;
  if (s >= HL_SYNC_STEPS) {                                // готово
    hlSyncBusy = false; hlSynced = true; rxCfgKnown = true;
    hlBoots = hlBootsNow;
    memcpy(simRam, simNvs, sizeof(simRam));
    rxModeCache = simRam[RXP_MODE][0][0];
    hlMirrorSave();
    LOGF("Sync: done, cfg crc %02X", hlCfgCrc());
    return;
  }
  hlSyncStep++;
  if (s == 0)      hlCfgPut(HLO_CMD, HLC_PROTO, 0, 0, HL_PROTO, false, 1);
  else if (s == 1) { uint16_t m = 0; for (int i = 0; i < NUM_CH; i++) if (chEn[i]) m |= (1 << i); hlCfgPut(HLO_CMD, HLC_CHMASK, 0, 0, m, false, 1); }
  else if (s == 2) { uint16_t m = 0; for (int i = 0; i < NUM_CH; i++) if (chType[i] == T_SW3) m |= (1 << i); hlCfgPut(HLO_CMD, HLC_CHTYPES, 0, 0, m, false, 1); }
  else if (s == 3) hlCfgPut(HLO_CMD, HLC_TXPWR, 0, 0, txPower, false, 1);
  else             hlCfgPut(HLO_BATCH, (uint8_t)(s - 4), 0, 0, 0, false, 1);
}

void hlSyncStart(const char* why) {
  if (hlSyncBusy) return;
  LOGF("Sync: start (%s)", why);
  hlSyncBusy = true; hlSyncStep = 0;
  hlCfgClear();
}

int hlSyncPct() { return hlSyncStep * 100 / HL_SYNC_STEPS; }

static void hlOnTelem(const uint8_t* p, size_t len, unsigned long now) {
  if (len != 36) return;
  Telem t;
  memset(&t, 0, sizeof(t));
  t.snr = (int8_t)p[5];
  t.missed = p[6];
  for (int i = 0; i < NUM_CH; i++) { t.cur[i] = p[7 + i]; t.pwm[i] = (int8_t)p[15 + i]; }
  t.mLim = p[23]; t.mOvl = p[24]; t.mOpen = p[25]; t.side = p[26];
  t.kill = p[27] & 1; t.reason = (p[27] >> 1) & 3;
  t.vrx = p[28]; t.rssi = (int8_t)p[29];
  t.boots = p[30]; t.cfgCrc = p[31]; t.wifi = p[32]; t.wifiArg = p[33];
  bool first = !hlUp;
  hlBootsNow = t.boots;
  tel = t;
  telValid = true;
  hlUp = true;
  hlMissReq = false;
  hlBootFlag = false;
  hlLastRx = now;
  rfRssi = (int)lroundf(hlRadio.getRSSI());
  rfSnr10 = (int)lroundf(hlRadio.getSNR() * 10.0f);
  rfLost += p[6];
  rfRx++;
  rfAgeMs = 0;
  if (hlWait == 1) { hlWait = 0; hlRtt = (int)(now - hlReqAt); hlNextTx = now + 1; }
  if (first) LOGF("Link: up (rssi %d snr %.1f)", rfRssi, rfSnr10 / 10.0f);
  // синхронизация: полная после включения пульта или перезагрузки приёмника, иначе по CRC конфигурации
  if (!hlSyncBusy && now > hlSyncRetryAt) {
    if (!hlSynced)                                          hlSyncStart("first link");
    else if (t.boots != hlBoots)                            hlSyncStart("rx rebooted");
    else if (t.cfgCrc != hlCfgCrc() && !(hlCrcIgnoreOn && hlCrcIgnore == t.cfgCrc)) hlSyncStart("cfg crc");
  }
  if (!hlSyncBusy && hlSynced) {
    hlBoots = t.boots;
    if (t.cfgCrc != hlCfgCrc()) { hlCrcIgnore = t.cfgCrc; hlCrcIgnoreOn = true; }
  }
}

static void hlOnAck(const uint8_t* p, size_t len, unsigned long now) {
  if (len != 12 || !hlCurOn || p[3] != hlCur.tag || (hlWait != 2 && hlWait != 3)) return;
  hlLastRx = now; hlWait = 0; hlMissReq = false;
  hlCurOn = false;
  hlCfgDone(hlCur, p[4], (uint16_t)(p[8] | (p[9] << 8)));
  hlNextTx = now + 1;
}

static void hlOnBatch(const uint8_t* p, size_t len, unsigned long now) {
  if (len < 8 || !hlCurOn || hlCur.op != HLO_BATCH || p[3] != hlCur.tag || hlWait != 3) return;
  int prm = p[4], n = p[5];
  if (prm != hlCur.param || prm >= NUM_RXP || len != (size_t)(8 + 2 * n)) return;
  hlLastRx = now; hlWait = 0; hlMissReq = false;
  hlCurOn = false;
  for (int k = 0; k < n; k++) {
    uint16_t v = (uint16_t)(p[6 + 2 * k] | (p[7 + 2 * k] << 8));
    if (RXP[prm].scope == PS_SOL) { if (k < 16) simNvs[prm][k / 2][k % 2] = v; }
    else if (RXP[prm].scope == PS_CH) { if (k < NUM_CH) simNvs[prm][k][0] = v; }
    else { simNvs[prm][0][0] = v; }
  }
  hlNextTx = now + 1;
}

static void hlOnInfo(const uint8_t* p, size_t len) {
  if (len < 8) return;
  int kind = p[4], l = p[5];
  if (len != (size_t)(8 + l)) return;
  char t[24];
  int m = l < 23 ? l : 23;
  memcpy(t, p + 6, m); t[m] = 0;
  if (kind == 1) snprintf(hlRxVer, sizeof(hlRxVer), "%s", t);
  else if (kind == 2) snprintf(rxWifiSsid, sizeof(rxWifiSsid), "%s", t);
}

static void hlRead(unsigned long now) {
  size_t len = hlRadio.getPacketLength();
  uint8_t p[64];
  if (len > sizeof(p) || len < 6) { uint8_t d[64]; hlRadio.readData(d, len > sizeof(d) ? sizeof(d) : len); return; }
  if (hlRadio.readData(p, len) != RADIOLIB_ERR_NONE) return;
  uint16_t c = hlCrc16(p, len - 2);
  if (c != (uint16_t)(p[len - 2] | (p[len - 1] << 8))) return;
  int type = p[0] >> 4;
  uint16_t pr = (uint16_t)(p[1] | (p[2] << 8));
  if (type == HLT_BINDOK) {
    if (hlBindOn && len == 10) {
      uint32_t id = (uint32_t)p[4] | ((uint32_t)p[5] << 8) | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
      if (id == hlBindNew) hlBindDone = true;
    }
    return;
  }
  if (pr != hlPair()) return;
  switch (type) {
    case HLT_TELEM: hlOnTelem(p, len, now); break;
    case HLT_ACK:   hlOnAck(p, len, now); break;
    case HLT_BATCH: hlOnBatch(p, len, now); break;
    case HLT_INFO:  hlOnInfo(p, len); break;
    default: break;
  }
}

// Радио работает на каждом проходе главного цикла (ответ приходит быстро, пульт должен успеть на приём)
void hlStep(unsigned long now) {
  if (!hlRadioMode() || !hlInit) return;
  if (state == ST_WIFI || state == ST_GH) {                 // Wi-Fi пульта: радио молчит, приёмник сам остановится по таймауту
    if (hlUp) { hlUp = false; hlSyncBusy = false; hlCfgClear(); }
    return;
  }
  if (hlIrq) {
    hlIrq = false;
    if (hlTxBusy) {
      hlRadio.finishTransmit();
      hlTxBusy = false;
      hlRadio.startReceive();
      hlTxEnd = now;
      if (hlWait) hlWaitUntil = now + HL_RESP_MS;
      else hlNextTx = hlHbSched ? (hlTxStart + HL_HB_MS) : (now + HL_GAP_MS);
    } else {
      hlRead(now);
    }
  }
  // таймаут ожидания ответа
  if (hlWait && !hlTxBusy && (long)(now - hlWaitUntil) >= 0) {
    if (hlWait == 1) { hlWait = 0; hlMissReq = true; hlNextTx = now + 1; }
    else {                                                  // нет подтверждения: повтор до 3 раз
      hlWait = 0;
      hlCur.tries++;
      if (hlCur.tries >= HL_CFG_TRIES) {
        hlCurOn = false;
        LOGF("Cfg: no answer (op %d p %d)", hlCur.op, hlCur.param);
        if (hlCur.kind == 1) { hlSyncBusy = false; hlSynced = false; hlSyncRetryAt = now + 2000UL; }
        else {
          if (hlCur.op == HLO_SET) { simRam[hlCur.param][hlCur.ch][hlCur.dir] = hlCur.old; if (hlCur.param == RXP_MODE) rxModeCache = hlCur.old; }
          if (hlCur.op == HLO_CMD || hlCur.op == HLO_NET) { hlWifiAck = true; hlWifiAckSt = 255; }
          toast("RX no answer");
        }
      }
      hlNextTx = now + 1;
    }
  }
  // связь потеряна
  if (hlUp && (long)(now - hlLastRx) > (long)hlLostMs()) {
    hlUp = false; hlSyncBusy = false; hlCfgClear();
    LOGF("Link: lost (age %lu ms)", now - hlLastRx);
  }
  if (now - hlRateAt >= 1000UL) { rfRate = (int)hlRateCnt; hlRateCnt = 0; hlRateAt = now; }
  if (hlUp) rfAgeMs = now - hlLastRx;
  // быстрые передачи: рычаг, E-Stop или тест рычага не ждут интервала heartbeat
  bool es = (state == ST_ESTOP) || estop.state;
  bool anyNZ = false;
  for (int i = 0; i < NUM_CH; i++) if (ctl[i] != 0) anyNZ = true;
  if (anyNZ || es) hlLastNonZero = now;
  if (es && hlWait == 1 && !hlWaitEs && !hlTxBusy) { hlWait = 0; hlMissReq = true; hlNextTx = now; }   // E-Stop не ждёт ответа
  if (hlHbSched && !hlWait && (anyNZ || es) && !hlSyncBusy && !hlBindOn) { unsigned long t = hlTxEnd + (unsigned long)HL_GAP_MS; if ((long)(t - hlNextTx) < 0) hlNextTx = t; }
  if (hlTxBusy || hlWait || (long)(now - hlNextTx) < 0) return;
  // что отправить
  if (hlBindOn) {
    if ((long)(now - hlBindAt) >= 0) { hlBindAt = now + HL_BIND_MS; hlSendBind(now); }
    return;
  }
  if (hlSyncBusy) hlSyncNext();
  if (!hlCurOn && hlQn > 0 && (hlUp || hlSyncBusy)) {
    hlCur = hlQ[0];
    for (int i = 1; i < hlQn; i++) hlQ[i - 1] = hlQ[i];
    hlQn--;
    if (++hlTag == 0) hlTag = 1;
    hlCur.tag = hlTag;
    hlCur.tries = 0;
    hlCurOn = true;
  }
  static unsigned long lastCtl = 0;
  if (hlCurOn) {
    bool leverTest = anyNZ && state == ST_MENU && menuScr == MS_RXEDIT && rxLeverParam(rxParam) && leverArmed;
    if (!(leverTest && (long)(now - lastCtl) >= 60)) { hlSendCfg(now); return; }   // тест рычага: управление чередуется с настройкой
  }
  hlSendControl(now);
  lastCtl = now;
}

// Фронты событий: связь, телеметрия, батарея -> журнал
void eventsTick(unsigned long now) {
  static bool pLinked = false;
  static uint8_t pOvl = 0, pOpen = 0, pLim = 0;
  static int pBat = 0;

  bool linked = rxLinked();
  if (linked && !pLinked) {
    restoredAt = now ? now : 1;
    logEvent(LT_LINKOK, 0, 0, 0);
    if (simMode == SIM_RADIO) { if (hlSynced) rxCfgKnown = true; }   // иначе станет известен после синхронизации
    else rxCfgRequest();                 // имитация: режим CONTROL и т.п. берём у приёмника
  }
  if (!linked && pLinked) {
    rxRevertAll();                       // несохранённое откатывается
    rxCfgKnown = false;
    rxWifiSt = 0;
    logEvent(LT_NOLINK, 0, 0, 0);
  }
  pLinked = linked;

  static bool pKillRx = false, pKillFault = false, pRxBatt = false;
  bool kRx = telValid && tel.kill && tel.reason == 1;
  bool kFt = telValid && tel.kill && tel.reason == 2;
  bool bRx = telValid && tel.vrx != 0 && tel.vrx < HL_RXBATT_LOW;
  if (kRx && !pKillRx)    logEvent(LT_RXALARM, 0, 0, 0);
  if (kFt && !pKillFault) logEvent(LT_RXFAULT, 0, 0, 0);
  if (bRx && !pRxBatt)    logEvent(LT_BATTRX, 0, 0, tel.vrx);
  pKillRx = kRx; pKillFault = kFt; pRxBatt = bRx;
  uint8_t ovl = telValid ? tel.mOvl : 0;
  uint8_t opn = telValid ? tel.mOpen : 0;
  uint8_t lim = telValid ? tel.mLim : 0;
  for (int c = 0; c < NUM_CH; c++) {
    uint8_t bit = (uint8_t)(1 << c);
    int side = (tel.side >> c) & 1;
    if ((ovl & bit) && !(pOvl & bit)) {
      logEvent(LT_OVL, c, side, (uint16_t)tel.cur[c]);     // отдельного события КЗ нет, только перегрузка
    }
    if ((opn & bit) && !(pOpen & bit)) logEvent(LT_OPEN, c, side, 0);
    if ((lim & bit) && !(pLim & bit))  logEvent(LT_LIMIT, c, side, (uint16_t)tel.cur[c]);
  }
  pOvl = ovl; pOpen = opn; pLim = lim;

  if (batLevel >= 1 && pBat == 0) logEvent(LT_BATTTX, 0, 0, (uint16_t)(vbat * 10.0f));
  pBat = batLevel;
}

// ============ МЕНЮ ============
// Пункты меню канала зависят от типа: Expo и Max только у джойстика
int chItems(int ch, uint8_t* ids) {
  int n = 0;
  ids[n++] = CI_TYPE;
  ids[n++] = CI_REV;
  ids[n++] = CI_EN;
  if (chType[ch] == T_JOY) { ids[n++] = CI_EXPO; ids[n++] = CI_MAX; }
  ids[n++] = CI_LED;
  ids[n++] = CI_BACK;
  return n;
}

// Пункты экрана настройки события
int evItems(int e, uint8_t* ids) {
  int n = 0;
  if (e == EV_ESTOP) { ids[n++] = EI_FIXED; ids[n++] = EI_BACK; return n; }
  if (e != EV_RXALARM && e != EV_NOLINK) ids[n++] = EI_EN;     // эти два выключить нельзя
  ids[n++] = EI_COLOR;
  if (e == EV_HEART) ids[n++] = EI_INT;
  else               ids[n++] = EI_PAT;
  if (e >= EV_OVERLOAD && e <= EV_NOTCTR) ids[n++] = EI_WHERE;
  if (e != EV_NOTCTR && e != EV_HEART) ids[n++] = EI_SCR;       // у E-Stop и Safe Start свои экраны
  ids[n++] = EI_BACK;
  return n;
}

int menuCount() {
  switch (menuScr) {
    case MS_MAIN:    return 4;
    case MS_TX:      return TX_COUNT;
    case MS_CAL:     return 4;
    case MS_CHLIST:  return NUM_CH + 1;
    case MS_CHEDIT:  { uint8_t ids[8]; return chItems(editCh, ids); }
    case MS_BAT:     return 10;
    case MS_LED:     return 6;
    case MS_LEDSTAT: return 3;
    case MS_EVLIST:  return NUM_EV + 1;
    case MS_EVEDIT:  { uint8_t ids[8]; return evItems(evSel, ids); }
    case MS_PRESET:  return 4;
    case MS_SCREENS: return NUM_SCR + 1;
    case MS_SCRITEM: return 4;
    case MS_SCRMOVE: return NUM_SCR;
    case MS_POWER:   return 3;
    case MS_BIND:    return 4;
    case MS_EVLOG:   return logN + 2;
    case MS_DBGLIST: return NUM_DP + 1;
    case MS_FRTARGET: return 3;
    case MS_NETS:    return NET_MAX + 3;
    case MS_WIFI:    return WF_COUNT;
    case MS_NETACT:  return 3;
    case MS_NETPICK: return pickScanning ? 1 : pickN + 2;
    case MS_RX:      return RT_COUNT;
    case MS_RXWIFIM: return RW_COUNT;
    case MS_RXCH:    return NUM_CH + 1;
    case MS_RXDIR:   return 3;
    case MS_RXGLOB:  return 7;
    default:         return 0;
  }
}

void adjustTop() {
  if (sel < menuTop) menuTop = sel;
  if (sel >= menuTop + MENU_ROWS) menuTop = sel - MENU_ROWS + 1;
  if (menuTop < 0) menuTop = 0;
}

void gotoScr(MenuScr s, int index) {
  menuScr = s;
  sel = index;
  menuTop = 0;
  adjustTop();
  navReset();
}

const char* COL_NAME[3] = { "Red", "Green", "Amber" };
const char* PAT_NAME[6] = { "Steady", "Slow", "Fast", "Pulse", "Double", "Alt" };

// эффективные настройки события: E-Stop фиксирован, у двух обязательных Enable всегда ON
EvCfg cfgOf(int e) {
  EvCfg c = ev[e];
  if (e == EV_ESTOP) c = DEF_EV[EV_ESTOP];
  if (e == EV_ESTOP || e == EV_RXALARM || e == EV_NOLINK) c.en = 1;
  return c;
}

// ---- пресеты ----
void presetFill(int p, EvCfg* out, bool &g, bool &r) {
  for (int e = 0; e < NUM_EV; e++) out[e] = DEF_EV[e];
  g = true; r = true;
  if (p == 0) {                     // Quiet
    g = false;
    out[EV_LIMIT].en = 0; out[EV_NOTCTR].en = 0; out[EV_RESTORED].en = 0;
    out[EV_HEART].en = 0; out[EV_BATTTX].en = 0;  out[EV_BATTRX].en = 0;
  } else if (p == 2) {              // Full
    for (int e = 0; e < NUM_EV; e++) out[e].en = 1;
    out[EV_HEART].ival = 0;
  }
}

const char* presetName() {
  static const char* names[3] = { "Quiet", "Standard", "Full" };
  EvCfg t[NUM_EV];
  for (int p = 0; p < 3; p++) {
    bool g, r;
    presetFill(p, t, g, r);
    if (g != ledGreenOn || r != ledRedOn) continue;
    bool same = true;
    for (int e = 1; e < NUM_EV && same; e++) {
      EvCfg c = ev[e];
      if (e == EV_RXALARM || e == EV_NOLINK) c.en = t[e].en;
      if (c.en != t[e].en || c.color != t[e].color || c.pat != t[e].pat ||
          c.whereAll != t[e].whereAll || c.ival != t[e].ival || c.scr != t[e].scr) same = false;
    }
    if (same) return names[p];
  }
  return "Custom";
}

void presetApply(int p) {
  bool g, r;
  presetFill(p, ev, g, r);
  ledGreenOn = g; ledRedOn = r;
  saveAllEv();
  saveLedCfg();
}

// ============ ВСПОМОГАТЕЛЬНОЕ (меню, экраны, журнал) ============
void fmtParam(int p, int v, char* b, size_t n) {
  switch (RXP[p].fmt) {
    case F_AMP:   snprintf(b, n, "%d.%dA", v / 10, v % 10); break;
    case F_PCT:   snprintf(b, n, "%d%%", v); break;
    case F_PCT10: snprintf(b, n, "%d.%d%%", v / 10, v % 10); break;
    case F_MS:    snprintf(b, n, "%dms", v); break;
    case F_HZ:    snprintf(b, n, "%dHz", v); break;
    case F_ONOFF: snprintf(b, n, "%s", v ? "ON" : "OFF"); break;
    case F_MODE:  snprintf(b, n, "%s", v ? "CURRENT" : "PWM"); break;
    default:      snprintf(b, n, "%ddBm", v); break;
  }
}

bool rxCurMode() { return rxCfgKnown && rxModeCache != 0; }   // пока режим не получен от приёмника, показываем PWM

// параметры, при правке которых работает тестовый рычаг
bool rxLeverParam(int p) { return p == RXP_LIMIT || p == RXP_DUTY || p == RXP_STARTI; }

int rxTopicParam(int topic) {
  switch (topic) {
    case RT_LIMIT: return RXP_LIMIT;
    case RT_DUTY:  return rxCurMode() ? RXP_STARTI : RXP_DUTY;
    case RT_MAXD:  return rxCurMode() ? RXP_MAXI : RXP_MAXD;
    case RT_CHEN:  return RXP_CHEN;
    default:       return RXP_RAMPUP;
  }
}

// ---- экраны ----
bool scrEnabled(uint8_t id) { return ((scrEn >> id) & 1) != 0; }

uint8_t scrNext(uint8_t cur) {
  int pos = 0;
  for (int i = 0; i < NUM_SCR; i++) if (scrOrder[i] == cur) pos = i;
  for (int k = 1; k <= NUM_SCR; k++) {
    uint8_t id = scrOrder[(pos + k) % NUM_SCR];
    if (scrEnabled(id)) return id;
  }
  return cur;
}

void scrToggle(uint8_t id) {
  if (id == SC_MAIN) { toast("Main always on"); return; }
  if (scrEnabled(id)) {
    if (id == scrStart) { toast("Is start screen"); return; }
    scrEn &= (uint8_t)~(1 << id);
  } else {
    scrEn |= (uint8_t)(1 << id);
  }
  saveScreens();
}

void scrMove(int dir) {
  int j = scrSel + dir;
  if (j < 0 || j >= NUM_SCR) return;
  uint8_t t = scrOrder[scrSel];
  scrOrder[scrSel] = scrOrder[j];
  scrOrder[j] = t;
  scrSel = j;
}

// ---- подтверждение ----
void startConfirm(ConfAct act, MenuScr back, int idx) {
  confAct = act;
  confBack = back;
  confIdx = idx;
  gotoScr(MS_CONFIRM, 0);
}

void factoryResetTx() {
  static const char* ns[] = { "joy", "chcfg", "led", "alerts", "bat", "screens", "radio", "log", "wifi", "game", "rxcopy" };
  for (int i = 0; i < 11; i++) {
    prefs.begin(ns[i], false);
    prefs.clear();
    prefs.end();
  }
  memset(netSsid, 0, sizeof(netSsid)); memset(netPass, 0, sizeof(netPass));
  logLine("FACTORY RESET: pult");
  restartAt = millis() + 1500UL;      // привязка (namespace "bind") не затрагивается
}

void doConfirm() {
  if (restartAt) return;
  switch (confAct) {
    case CA_CLEARLOG:
      logClear();
      logLine("Event log cleared");
      toast("Log cleared");
      gotoScr(confBack, 0);
      break;
    case CA_FORGET:
      bindId = DEF_BIND_ID;
      saveBind();
      hlMirrorForget();                    // копия настроек приёмника стирается, при следующей связи полная синхронизация
      logLine("Bind forgotten");
      toast("Bind forgotten");
      gotoScr(confBack, confIdx);
      break;
    case CA_NETDEL:
      netDel(confIdx);
      logLine("Wi-Fi network deleted");
      toast("Deleted");
      gotoScr(confBack, 0);
      break;
    case CA_NETALL:
      netClearAll();
      logLine("Wi-Fi networks cleared");
      toast("All forgotten");
      gotoScr(confBack, 0);
      break;
    case CA_FR_TX:
      factoryResetTx();               // экран остаётся до перезагрузки
      break;
    case CA_FR_RX:
      if (!rxReady() || !othersCentered()) {
        toast("Not possible now");
      } else {
        rxFactoryReset();
        logLine("FACTORY RESET: RX");
        toast(simMode == SIM_RADIO ? "RX reset sent" : "RX reset");
      }
      gotoScr(confBack, confIdx);
      break;
  }
}

// ---- код для Factory Reset ----
void startPin() {
  for (int i = 0; i < 4; i++) pinDigit[i] = 0;
  pinPos = 0;
  gotoScr(MS_PIN, 0);
}

void pinStep(int a, int b) {
  if (a) pinDigit[pinPos] = (pinDigit[pinPos] + a + 10) % 10;
  if (b > 0) {
    if (++pinPos >= 4) {
      int code = pinDigit[0] * 1000 + pinDigit[1] * 100 + pinDigit[2] * 10 + pinDigit[3];
      if (code == FACTORY_PIN) {
        gotoScr(MS_FRTARGET, 0);
      } else {
        logLine("Factory reset: wrong code");
        toast("Wrong code");
        gotoScr(MS_TX, idxTx);
      }
    }
  } else if (b < 0) {
    if (pinPos > 0) pinPos--;
    else gotoScr(MS_TX, idxTx);
  }
}

// ---- привязка (пока без радио: LINK в симуляторе имитирует успех) ----
void startBind() {
  bindStart = millis();
  bindPhase = 0;
  if (simMode == SIM_RADIO) {                  // приёмник в режиме привязки (долгое нажатие PRG) принимает новый ID
    hlBindNew = (uint32_t)esp_random();
    if (hlBindNew == DEF_BIND_ID || hlBindNew == 0) hlBindNew ^= 0x1B2D3F5UL;
    hlBindOn = true; hlBindDone = false; hlBindAt = millis();
  }
  logLine("Bind: started");
  gotoScr(MS_BINDRUN, 0);
}

void bindTick(unsigned long now) {
  if (bindPhase == 0) {
    if (simMode == SIM_RADIO && hlBindDone) {
      bindId = hlBindNew;
      saveBind();
      hlBindOn = false; hlBindDone = false;
      hlMirrorForget();                        // после привязки: полная синхронизация
      bindPhase = 1;
      bindDoneAt = now;
      LOGF("Bind OK %08lX", (unsigned long)bindId);
    } else if (simMode == SIM_LINK && (now - bindStart) >= 3000UL) {
      bindId = (uint32_t)esp_random();
      if (bindId == DEF_BIND_ID) bindId ^= 1UL;
      saveBind();
      bindPhase = 1;
      bindDoneAt = now;
      LOGF("Bind OK %08lX", (unsigned long)bindId);
    } else if ((now - bindStart) >= BIND_TIMEOUT_MS) {
      bindPhase = 2;
      hlBindOn = false;
      bindDoneAt = now;
      logLine("Bind: timeout");
    }
  } else if ((now - bindDoneAt) >= 2000UL) {
    if (bindPhase == 2) toast("Bind timeout");
    gotoScr(MS_BIND, idxBind);
  }
}

// ---- мощность пульта ----
void startTxPwrEdit() {
  editInt = 0;
  for (int i = 0; i < 8; i++) if (TXP_LIST[i] == txPower) editInt = i;
  gotoScr(MS_TXPWR, 0);
}

// ---- описания страниц Debug (листаются CH3) ----
const char* const DESC_RAW[] = {
  "RAW ADC", "Raw values of the", "8 channels from the", "ADS1115 chips.", "Center is about", "13200.", "",
  "ADS8 = time to read", "all 8 channels.", "DZ = dead zone,", "in ADC counts." };
const char* const DESC_RADIO[] = {
  "RADIO", "First line: radio", "settings (freq, SF,", "bandwidth).", "TX/RX = power of", "pult and receiver.", "",
  "RSSI = signal level.", "Closer to 0 is", "better: -63 good,", "-110 bad.", "SNR = signal/noise.", "",
  "Lost = lost packets", "and percent.", "26/s = packets", "per second.", "Age = time since", "last telemetry.", "ID = receiver code." };
const char* const DESC_TELEM[] = {
  "TELEMETRY RAW", "Ip = pulse current", "of each channel, in", "0.1 A (not average).", "P = PWM, -100..100.", "Minus = side B.", "",
  "L = limit reached", "O = overload", "X = open circuit", "Each is a mask: one", "bit per channel.",
  "s = faulty side", "bit: 0=A, 1=B.", "",
  "Vrx = receiver", "battery, volts.", "R = receiver RSSI.", "K = killswitch:", "no, or the reason." };
const char* const DESC_SYS[] = {
  "SYSTEM", "FW: version, date.", "Up: run time.", "Temp: chip temp.", "Heap: free memory.",
  "I2C: devices found.", "Draw a+b: a = render", "time, b = time to", "send the frame, ms." };
const char* const DESC_LOG[] = {
  "LOG", "Last lines of the", "serial log, newest", "at the bottom.", "Same text as on the", "USB serial monitor." };
const char* const DESC_LIM[] = {
  "LIMITS / NOW / OVL", "One row per channel", "channel, then A", "side, then B side.", "Each side has 3", "numbers, in 0.1 A.", "",
  "Lim = current limit", "Now = pulse current", "Ovl = last overload", "from the log", "-- = no data", "",
  "Example row:", "3 30 00 -- 30 31 52", "= CH3. A: lim 3.0,", "now 0, no overload.", "B: lim 3.0, now 3.1", "overload 5.2 A." };
struct DescRef { const char* const* lines; int n; };
const DescRef DESC[NUM_DP] = {
  { DESC_RADIO, (int)(sizeof(DESC_RADIO) / sizeof(DESC_RADIO[0])) },
  { DESC_TELEM, (int)(sizeof(DESC_TELEM) / sizeof(DESC_TELEM[0])) },
  { DESC_LIM,   (int)(sizeof(DESC_LIM)   / sizeof(DESC_LIM[0])) },
  { DESC_RAW,   (int)(sizeof(DESC_RAW)   / sizeof(DESC_RAW[0])) },
  { DESC_SYS,   (int)(sizeof(DESC_SYS)   / sizeof(DESC_SYS[0])) },
  { DESC_LOG,   (int)(sizeof(DESC_LOG)   / sizeof(DESC_LOG[0])) }
};

// порядок пунктов в меню -> внутренний номер пункта (чтобы не переписывать обработчики)
const uint8_t BAT_MAP[9] = { 0, 5, 6, 1, 2, 3, 4, 7, 8 };   // Cal divider, Warn, Crit, Full V, Empty V, Full=now, Empty=low, Curve, Reset

// ---- Wi-Fi сети с пульта: поиск, выбор, ввод названия и пароля ----
void netScanStop() {
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
  pickScanning = false;
}

void netAddBegin() {
  if (netCount() >= NET_MAX) { toast("Full: delete one"); return; }
  pickN = 0;
  pickScanning = true;
  pickT0 = millis();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.scanNetworks(true);
  gotoScr(MS_NETPICK, 0);
}

// результат скана: уникальные видимые названия, сильные сверху (n найденных сетей -> out)
int netCollect(int n, NetFound* out, int maxN) {
  int cnt = 0;
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0 || s.length() > 32) continue;
    int r = WiFi.RSSI(i);
    int dup = -1;
    for (int k = 0; k < cnt; k++) if (strcmp(out[k].ssid, s.c_str()) == 0) dup = k;
    if (dup >= 0) { if (r > out[dup].rssi) out[dup].rssi = (int8_t)r; continue; }
    if (cnt < maxN) {
      snprintf(out[cnt].ssid, sizeof(out[cnt].ssid), "%s", s.c_str());
      out[cnt].rssi = (int8_t)r;
      cnt++;
    }
  }
  for (int a = 0; a < cnt; a++)
    for (int b2 = a + 1; b2 < cnt; b2++)
      if (out[b2].rssi > out[a].rssi) { NetFound t = out[a]; out[a] = out[b2]; out[b2] = t; }
  return cnt;
}

void netPickTick(unsigned long now) {
  if (!pickScanning) return;
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    if ((now - pickT0) < 15000UL) return;
    n = 0;                                              // слишком долго: считаем, что сетей нет
  } else if (n < 0) {
    n = 0;
  }
  pickN = netCollect(n, pickNet, 8);
  netScanStop();
  sel = 0; menuTop = 0;
}

void textBegin(int stage, int maxLen) {
  txtStage = stage;
  txtMax = maxLen;
  txtLen = 0;
  txtBuf[0] = 0;
  txtCur = 0;
  gotoScr(MS_TEXTIN, 0);
}

void textFinish() {
  if (txtStage == 0) {                                   // название скрытой сети
    if (txtLen == 0) { toast("Name is empty"); return; }
    snprintf(txtSsid, sizeof(txtSsid), "%s", txtBuf);
    textBegin(1, 63);
    return;
  }
  if (txtLen > 0 && txtLen < 8) { toast("Min 8 characters"); return; }   // WPA: 8..63, пустой = открытая сеть
  int r = netAdd(txtSsid, txtBuf);
  logLine(r == 0 ? "Wi-Fi network saved" : "Wi-Fi network not saved");
  toast(r == 0 ? "Saved" : (r == 1 ? "No free slot" : "Error"));
  gotoScr(MS_NETS, 0);
  if (r == 0)                                            // сразу проверить подключение
    for (int i = 0; i < NET_MAX; i++)
      if (strcmp(netSsid[i], txtSsid) == 0) ghTestBegin(i, MS_NETS, 0, true);
}

void textStep(int a, int b, int m) {
  const int total = TXT_N + 2;
  if (a) {
    int step = m < 1 ? 1 : (m > 4 ? 4 : m);
    txtCur = ((txtCur + a * step) % total + total) % total;
  }
  if (b > 0) {
    if (txtCur < TXT_N) {
      if (txtLen < txtMax) { txtBuf[txtLen++] = TXT_SET[txtCur]; txtBuf[txtLen] = 0; }
    } else if (txtCur == TXT_N) {
      if (txtLen > 0) txtBuf[--txtLen] = 0;
    } else {
      textFinish();
    }
  } else if (b < 0) {
    if (txtLen > 0) txtBuf[--txtLen] = 0;
    else gotoScr(MS_NETS, idxNets);
  }
}

void menuLabel(int i, char* b, size_t n) {
  static const char* mainL[] = { "Transmitter", "Receiver", "Dino", "Exit" };
  static const char* calL[]  = { "Set Center", "Set Min/Max", "Dead Zone", "Back" };
  static const char* ledM[]  = { "Both", "Red", "Green", "Off" };
  static const char* simN[]  = { "Radio", "Sim link", "Sim lost" };
  static const char* ledL[]  = { "Presets", "Alert events", "Status LEDs", "Test alerts", "Reset alerts", "Back" };
  static const char* rxL[]   = { "Current Limits", "Start Duty", "Max Duty", "Ramps", "Channels On/Off", "Global", "RX Status", "Wi-Fi", "Back" };
  static const char* dbgL[]  = { "Radio", "Telemetry", "Limits", "RAW ADC", "System", "Log", "Back" };
  switch (menuScr) {
    case MS_MAIN: snprintf(b, n, "%s", mainL[i]); break;
    case MS_RXWIFIM: { static const char* rwL[] = { "Phone: Update/Test", "GitHub update", "Back" }; snprintf(b, n, "%s", rwL[i < 3 ? i : 2]); break; }
    case MS_TX:
      switch (i) {
        case TX_CAL:   snprintf(b, n, "Calibrate"); break;
        case TX_CHSET: snprintf(b, n, "Channel Setup"); break;
        case TX_BAT:   snprintf(b, n, "Battery"); break;
        case TX_LED:   snprintf(b, n, "LED & Alerts"); break;
        case TX_SCR:   snprintf(b, n, "Screens"); break;
        case TX_PWR:   snprintf(b, n, "Power"); break;
        case TX_BIND:  snprintf(b, n, "Bind"); break;
        case TX_TRT:   snprintf(b, n, "Telem rate %d/s", telemRate); break;
        case TX_LOG:   snprintf(b, n, "Event Log"); break;
        case TX_DBG:   snprintf(b, n, "Debug"); break;
        case TX_WIFI:  snprintf(b, n, "Wi-Fi"); break;
        case TX_FR:    snprintf(b, n, "Factory Reset"); break;
        case TX_SIM:   snprintf(b, n, "RX: %s", simN[simMode]); break;
        default:       snprintf(b, n, "Back"); break;
      }
      break;
    case MS_CAL:  snprintf(b, n, "%s", calL[i]);  break;
    case MS_CHLIST:
      if (i < NUM_CH) snprintf(b, n, "CH%d %s %s %s", i + 1,
                               chType[i] == T_SW3 ? "Sw3" : "Joy",
                               chRev[i] ? "REV" : "nrm",
                               chEn[i] ? "on" : "OFF");
      else snprintf(b, n, "Back");
      break;
    case MS_CHEDIT: {
      uint8_t ids[8];
      int cnt = chItems(editCh, ids);
      if (i >= cnt) { b[0] = 0; break; }
      switch (ids[i]) {
        case CI_TYPE: snprintf(b, n, "Type: %s", chType[editCh] == T_SW3 ? "Switch3" : "Joystick"); break;
        case CI_REV:  snprintf(b, n, "Reverse: %s", chRev[editCh] ? "ON" : "OFF"); break;
        case CI_EN:   snprintf(b, n, "Enable: %s", chEn[editCh] ? "ON" : "OFF"); break;
        case CI_EXPO: snprintf(b, n, "Expo: %d%%", chExpo[editCh]); break;
        case CI_MAX:  snprintf(b, n, "Max: %d%%", chMaxP[editCh]); break;
        case CI_LED:  snprintf(b, n, "LED: %s", ledM[chLed[editCh]]); break;
        default:      snprintf(b, n, "Back"); break;
      }
      break;
    }
    case MS_BAT:
      switch (i < 9 ? BAT_MAP[i] : i) {
        case 0: snprintf(b, n, "Cal divider"); break;
        case 1: snprintf(b, n, "Full V  %.2f", vFull); break;
        case 2: snprintf(b, n, "Empty V %.2f", vEmpty); break;
        case 3: snprintf(b, n, "Full = now"); break;
        case 4:
          if (vMinSeen < 90.0f) snprintf(b, n, "Empty=low %.2f", vMinSeen);
          else                  snprintf(b, n, "Empty=low --");
          break;
        case 5: snprintf(b, n, "Warn %%: %d", batWarn); break;
        case 6: snprintf(b, n, "Crit %%: %d", batCrit); break;
        case 7: snprintf(b, n, "Curve: %s", batCurve ? "Li-ion" : "Linear"); break;
        case 8: snprintf(b, n, "Reset limits"); break;
        default: snprintf(b, n, "Back"); break;
      }
      break;
    case MS_LED:     snprintf(b, n, "%s", ledL[i < 6 ? i : 5]); break;
    case MS_LEDSTAT:
      if (i == 0)      snprintf(b, n, "Green LEDs: %s", ledGreenOn ? "ON" : "OFF");
      else if (i == 1) snprintf(b, n, "Red LEDs: %s", ledRedOn ? "ON" : "OFF");
      else             snprintf(b, n, "Back");
      break;
    case MS_EVLIST:
      if (i < NUM_EV) snprintf(b, n, "%-12s%s", EV_NAME[i], cfgOf(i).en ? "ON" : "off");
      else            snprintf(b, n, "Back");
      break;
    case MS_EVEDIT: {
      uint8_t ids[8];
      int cnt = evItems(evSel, ids);
      if (i >= cnt) { b[0] = 0; break; }
      EvCfg c = cfgOf(evSel);
      switch (ids[i]) {
        case EI_EN:    snprintf(b, n, "Enable: %s", c.en ? "ON" : "OFF"); break;
        case EI_COLOR: snprintf(b, n, "Color: %s", COL_NAME[c.color]); break;
        case EI_PAT:   snprintf(b, n, "Pattern: %s", PAT_NAME[c.pat]); break;
        case EI_INT:   snprintf(b, n, "Every: %d s", HB_SEC[c.ival]); break;
        case EI_WHERE: snprintf(b, n, "Where: %s", c.whereAll ? "All" : "Channel"); break;
        case EI_SCR:   snprintf(b, n, "Screen: %s", c.scr ? "ON" : "OFF"); break;
        case EI_FIXED: snprintf(b, n, "Fixed: red fast"); break;
        default:       snprintf(b, n, "Back"); break;
      }
      break;
    }
    case MS_PRESET: {
      static const char* pn[] = { "Quiet", "Standard", "Full", "Back" };
      snprintf(b, n, "%s", pn[i]);
      break;
    }
    case MS_RX:
      if (i == RT_DUTY)      snprintf(b, n, "%s", rxCurMode() ? "Start Current" : "Start Duty");
      else if (i == RT_MAXD) snprintf(b, n, "%s", rxCurMode() ? "Max Current" : "Max Duty");
      else                   snprintf(b, n, "%s", rxL[i]);
      break;
    case MS_RXCH:
      if (i < NUM_CH) {
        if (rxTopic == RT_CHEN) snprintf(b, n, "Channel %d %s", i + 1, rxQuery(i, 0, RXP_CHEN) ? "ON" : "OFF");
        else                    snprintf(b, n, "Channel %d", i + 1);
      } else snprintf(b, n, "Back");
      break;
    case MS_RXDIR:
      if (i < 2) {
        char v[12];
        if (rxTopic == RT_RAMP) {
          fmtParam(RXP_RAMPUP + i, rxQuery(rxCh, 0, RXP_RAMPUP + i), v, sizeof(v));
          snprintf(b, n, "%s %s", i ? "Ramp down" : "Ramp up", v);
        } else {
          int p = rxTopicParam(rxTopic);
          fmtParam(p, rxQuery(rxCh, i, p), v, sizeof(v));
          snprintf(b, n, "%s %s", i ? "Retract" : "Extend", v);
        }
      } else snprintf(b, n, "Back");
      break;
    case MS_RXGLOB:
      if (i < 6) {
        char v[12];
        int p = (i == 0) ? RXP_MODE : RXP_OVLDLY + i - 1;
        fmtParam(p, rxQuery(0, 0, p), v, sizeof(v));
        snprintf(b, n, "%-9s %s", RXP[p].name, v);
      } else snprintf(b, n, "Back");
      break;
    case MS_SCREENS:
      if (i < NUM_SCR) {
        uint8_t id = scrOrder[i];
        snprintf(b, n, "%-11s%s%s", SCR_NAME[id], scrEnabled(id) ? "ON" : "off", id == scrStart ? "*" : "");
      } else snprintf(b, n, "Back");
      break;
    case MS_SCRMOVE: {
      uint8_t id = scrOrder[i];
      snprintf(b, n, "%-11s%s", SCR_NAME[id], i == scrSel ? "<<" : "");
      break;
    }
    case MS_SCRITEM: {
      uint8_t id = scrOrder[scrSel];
      switch (i) {
        case 0:  snprintf(b, n, "Enable: %s", scrEnabled(id) ? "ON" : "OFF"); break;
        case 1:  snprintf(b, n, "Move"); break;
        case 2:  snprintf(b, n, "Set as start%s", id == scrStart ? " *" : ""); break;
        default: snprintf(b, n, "Back"); break;
      }
      break;
    }
    case MS_POWER:
      if (i == 0)      snprintf(b, n, "TX pult %ddBm", txPower);
      else if (i == 1) {
        if (rxLinked()) snprintf(b, n, "RX recv %ddBm", (int)rxQuery(0, 0, RXP_PWR));
        else            snprintf(b, n, "RX recv --");
      } else snprintf(b, n, "Back");
      break;
    case MS_BIND: {
      static const char* bl[] = { "Start Bind", "Show ID", "Forget", "Back" };
      snprintf(b, n, "%s", bl[i]);
      break;
    }
    case MS_EVLOG:
      if (i < logN)       logText(logRam[i], b, n);
      else if (i == logN) snprintf(b, n, "Clear log");
      else                snprintf(b, n, "Back");
      break;
    case MS_DBGLIST: snprintf(b, n, "%s", dbgL[i]); break;
    case MS_WIFI: {
      static const char* wl[] = { "Phone update", "GitHub update", "Networks", "Back" };
      snprintf(b, n, "%s", wl[i]);
      break;
    }
    case MS_NETACT: {
      static const char* nl[] = { "Test connection", "Delete", "Back" };
      snprintf(b, n, "%s", nl[i]);
      break;
    }
    case MS_NETS:
      if (i < NET_MAX)       snprintf(b, n, "%d %.14s", i + 1, netSsid[i][0] ? netSsid[i] : "(empty)");
      else if (i == NET_MAX) snprintf(b, n, "Add network");
      else if (i == NET_MAX + 1) snprintf(b, n, "Forget all");
      else                   snprintf(b, n, "Back");
      break;
    case MS_NETPICK:
      if (pickScanning)      snprintf(b, n, "Scanning...");
      else if (i < pickN)    snprintf(b, n, "%.17s", pickNet[i].ssid);
      else if (i == pickN)   snprintf(b, n, "Hidden network");
      else                   snprintf(b, n, "Back");
      break;
    case MS_FRTARGET: {
      static const char* fl[] = { "Transmitter", "Receiver", "Back" };
      snprintf(b, n, "%s", fl[i]);
      break;
    }
    default: b[0] = 0; break;
  }
}

void enterMenu() {
  state = ST_MENU;
  idxMain = 0;
  gotoScr(MS_MAIN, 0);
  logLine("MENU: entered (setup mode, controls = 0)");
}

void goSafe();

// Выход из меню всегда через Safe Start: все включённые каналы должны быть в центре,
// иначе управление сразу пошло бы в приёмник.
void exitMenu() {
  goSafe();
  navReset();
  logLine("MENU: exit -> Safe Start");
}

void startCalCenter();
void startCalRange();

void startBatEdit(int kind) {
  batKind = kind;
  switch (kind) {
    case BE_DIV:   batEditCv = (int)(vbat * 100.0f + 0.5f); break;
    case BE_FULL:  batEditCv = (int)(vFull * 100.0f + 0.5f); break;
    case BE_EMPTY: batEditCv = (int)(vEmpty * 100.0f + 0.5f); break;
    case BE_WARN:  batEditI = batWarn; break;
    case BE_CRIT:  batEditI = batCrit; break;
  }
  gotoScr(MS_BATEDIT, 0);
}

void batEditStep(int a, int m) {
  switch (batKind) {
    case BE_DIV:   batEditCv = constrain(batEditCv + a * m, 300, 900); break;
    case BE_FULL:  batEditCv = constrain(batEditCv + a * m, 650, 900); break;
    case BE_EMPTY: batEditCv = constrain(batEditCv + a * m, 500, 800); break;
    case BE_WARN:  batEditI = constrain(batEditI + a * (m > 3 ? 3 : m), (int)batCrit + 1, 99); break;
    case BE_CRIT:  batEditI = constrain(batEditI + a * (m > 3 ? 3 : m), 1, (int)batWarn - 1); break;
  }
}

void batEditSave() {
  float x = batEditCv / 100.0f;
  switch (batKind) {
    case BE_DIV:
      if (pinV < 0.3f || batUsb) { toast("No battery"); break; }
      {
        float r = x / pinV;                 // реальное напряжение / напряжение на пине
        if (r >= 2.0f && r <= 5.0f) {
          divRatio = r;
          saveBatCfg();
          vbat = pinV * divRatio;
          vbatPct = pctFromV(vbat);
          toast("Divider saved");
          LOGF("Divider -> %.4f (meter %.2f V, pin %.3f V)", divRatio, x, pinV);
        } else {
          toast("Bad value");
        }
      }
      break;
    case BE_FULL:
      if (x >= vEmpty + 0.5f) { vFull = x; saveBatCfg(); toast("Saved"); }
      else toast("Full > Empty");
      break;
    case BE_EMPTY:
      if (x <= vFull - 0.5f) { vEmpty = x; saveBatCfg(); toast("Saved"); }
      else toast("Empty < Full");
      break;
    case BE_WARN:
      batWarn = batEditI; saveBatCfg(); toast("Saved");
      break;
    case BE_CRIT:
      batCrit = batEditI; saveBatCfg(); toast("Saved");
      break;
  }
  vbatPct = pctFromV(vbat);
  gotoScr(MS_BAT, idxBat);
}

// ---- Receiver ----
void enterReceiver() {
  if (!rxLinked())          { toast("RX not connected"); return; }
  if (hlSyncBusy)           { toast("Syncing...");       return; }
  if (!othersCentered())    { toast("Center controls");  return; }
  gotoScr(MS_RX, idxRx);
}

void startRxEdit(int param, int ch, int dir, MenuScr ret) {
  rxParam = param;
  rxCh = ch;
  rxDir = dir;
  rxRetScr = ret;
  rxOrig = rxQuery(ch, dir, param);           // на радио: SetConfig(0xFFFF) -> Ack
  rxCur = rxOrig;
  leverArmed = false;                         // джойстик заработает, когда будет в центре
  gotoScr(MS_RXEDIT, 0);
}

void gotoRxRet() {
  switch (rxRetScr) {
    case MS_RXCH:   gotoScr(MS_RXCH, idxRxCh); break;
    case MS_RXDIR:  gotoScr(MS_RXDIR, idxRxDir); break;
    case MS_RXGLOB: gotoScr(MS_RXGLOB, idxRxGlob); break;
    case MS_POWER:  gotoScr(MS_POWER, idxPower); break;
    default:        gotoScr(MS_RX, idxRx); break;
  }
}

void rxEditStep(int a, int m) {
  const RxParamDef &d = RXP[rxParam];
  if (rxParam == RXP_PWR) {                    // мощность: шаг по списку значений
    int idx = 0;
    for (int i = 0; i < 8; i++) if (TXP_LIST[i] <= rxCur) idx = i;
    idx = constrain(idx + a, 0, 7);
    rxCur = rxSet(rxCh, rxDir, rxParam, TXP_LIST[idx], false);
    return;
  }
  if (d.fmt == F_ONOFF || d.fmt == F_MODE) {
    rxCur = rxSet(rxCh, rxDir, rxParam, a > 0 ? 1 : 0, false);
    return;
  }
  int cap = (((int)d.mx - (int)d.mn) / (int)d.step > 30) ? 10 : 3;
  int k = (m > cap) ? cap : m;
  rxCur = rxSet(rxCh, rxDir, rxParam, rxCur + a * d.step * k, false);   // применить сразу, без записи в память
}

void rxEditSave() {
  rxCur = rxSet(rxCh, rxDir, rxParam, rxCur, true);        // записать в память приёмника
  LOGF("RX saved p%d ch%d d%d v%d", rxParam, rxCh + 1, rxDir, rxCur);
  toast("Saved");
  gotoRxRet();
}

void rxEditCancel() {
  rxSet(rxCh, rxDir, rxParam, rxOrig, false);              // вернуть исходное
  toast("Cancelled");
  gotoRxRet();
}

void menuSelect(int i) {
  switch (menuScr) {
    case MS_MAIN:
      idxMain = i;
      if (i == 0)      gotoScr(MS_TX, idxTx);
      else if (i == 1) enterReceiver();
      else if (i == 2) { gameEnter(); gotoScr(MS_GAME, 0); }
      else             exitMenu();
      break;
    case MS_TX:
      idxTx = i;
      switch (i) {
        case TX_CAL:   gotoScr(MS_CAL, idxCal); break;
        case TX_CHSET: gotoScr(MS_CHLIST, idxChList); break;
        case TX_BAT:   gotoScr(MS_BAT, idxBat); break;
        case TX_LED:   gotoScr(MS_LED, idxLed); break;
        case TX_SCR:   gotoScr(MS_SCREENS, idxScr); break;
        case TX_PWR:   gotoScr(MS_POWER, idxPower); break;
        case TX_BIND:  gotoScr(MS_BIND, idxBind); break;
        case TX_TRT:   editInt = telemRate; gotoScr(MS_TRATE, 0); break;
        case TX_LOG:   gotoScr(MS_EVLOG, idxEvLog); break;
        case TX_DBG:   gotoScr(MS_DBGLIST, idxDbg); break;
        case TX_WIFI:  gotoScr(MS_WIFI, idxWifi); break;
        case TX_FR:    startPin(); break;
        case TX_SIM:
          simMode = (SimMode)((simMode + 1) % 3);      // откат и журнал делает eventsTick
          LOGF("Sim RX -> %d", simMode);
          break;
        default:       gotoScr(MS_MAIN, idxMain); break;
      }
      break;
    case MS_CAL:
      idxCal = i;
      if (i == 0)      startCalCenter();
      else if (i == 1) startCalRange();
      else if (i == 2) { dzEdit = deadzone; gotoScr(MS_DZ, 0); }
      else             gotoScr(MS_TX, idxTx);
      break;
    case MS_CHLIST:
      idxChList = i;
      if (i < NUM_CH) { editCh = i; idxChEdit = 0; gotoScr(MS_CHEDIT, 0); }
      else            gotoScr(MS_TX, idxTx);
      break;
    case MS_CHEDIT: {
      uint8_t ids[8];
      int cnt = chItems(editCh, ids);
      if (i >= cnt) break;
      idxChEdit = i;
      switch (ids[i]) {
        case CI_TYPE:
          chType[editCh] = (chType[editCh] == T_JOY) ? T_SW3 : T_JOY;
          saveChannelCfg(editCh);
          LOGF("CH%d type -> %s", editCh + 1, chType[editCh] == T_SW3 ? "Switch3" : "Joystick");
          break;
        case CI_REV:
          chRev[editCh] = !chRev[editCh];
          saveChannelCfg(editCh);
          LOGF("CH%d reverse -> %s", editCh + 1, chRev[editCh] ? "ON" : "OFF");
          break;
        case CI_EN:
          chEn[editCh] = !chEn[editCh];
          saveChannelCfg(editCh);
          LOGF("CH%d enable -> %s", editCh + 1, chEn[editCh] ? "ON" : "OFF");
          break;
        case CI_EXPO:
          editInt = chExpo[editCh];
          gotoScr(MS_EXPO, 0);
          break;
        case CI_MAX:
          editInt = chMaxP[editCh];
          gotoScr(MS_MAXV, 0);
          break;
        case CI_LED:
          chLed[editCh] = (chLed[editCh] + 1) % 4;
          saveChannelCfg(editCh);
          LOGF("CH%d led mode -> %d", editCh + 1, chLed[editCh]);
          break;
        default:
          gotoScr(MS_CHLIST, idxChList);
          break;
      }
      break;
    }
    case MS_BAT:
      idxBat = i;
      switch (i < 9 ? BAT_MAP[i] : i) {
        case 0: startBatEdit(BE_DIV); break;
        case 1: startBatEdit(BE_FULL); break;
        case 2: startBatEdit(BE_EMPTY); break;
        case 3:
          if (batUsb || vbat < 5.0f) toast("No battery");
          else if (vbat < vEmpty + 0.5f) toast("Too low");
          else { vFull = vbat; saveBatCfg(); vbatPct = pctFromV(vbat); toast("Full set"); }
          break;
        case 4:
          if (vMinSeen > 90.0f) toast("No data yet");
          else if (vMinSeen > vFull - 0.5f) toast("Too high");
          else { vEmpty = vMinSeen; saveBatCfg(); vbatPct = pctFromV(vbat); toast("Empty set"); }
          break;
        case 5: startBatEdit(BE_WARN); break;
        case 6: startBatEdit(BE_CRIT); break;
        case 7: batCurve = batCurve ? 0 : 1; saveBatCfg(); vbatPct = pctFromV(vbat); break;
        case 8:
          vFull = DEF_VFULL; vEmpty = DEF_VEMPTY; batWarn = DEF_WARN; batCrit = DEF_CRIT;
          batCurve = 1; vMinSeen = 99.0f;
          saveBatCfg(); saveBatMin();
          vbatPct = pctFromV(vbat);
          toast("Limits reset");
          break;
        default: gotoScr(MS_TX, idxTx); break;
      }
      break;
    case MS_LED:
      idxLed = i;
      switch (i) {
        case 0: gotoScr(MS_PRESET, idxPreset); break;
        case 1: evFromLed = false; gotoScr(MS_EVLIST, idxEvList); break;
        case 2: gotoScr(MS_LEDSTAT, 0); break;
        case 3: testEv = 0; gotoScr(MS_TEST, 0); break;
        case 4:
          for (int e = 0; e < NUM_EV; e++) ev[e] = DEF_EV[e];
          ledGreenOn = true; ledRedOn = true;
          saveAllEv(); saveLedCfg();
          toast("Alerts reset");
          break;
        default: gotoScr(MS_TX, idxTx); break;
      }
      break;
    case MS_LEDSTAT:
      if (i == 0)      { ledGreenOn = !ledGreenOn; saveLedCfg(); toast("Saved"); }
      else if (i == 1) { ledRedOn = !ledRedOn; saveLedCfg(); toast("Saved"); }
      else             gotoScr(MS_LED, idxLed);
      break;
    case MS_EVLIST:
      idxEvList = i;
      if (i < NUM_EV) { evSel = i; evFromLed = false; idxEvEdit = 0; gotoScr(MS_EVEDIT, 0); }
      else            gotoScr(MS_LED, idxLed);
      break;
    case MS_EVEDIT: {
      uint8_t ids[8];
      int cnt = evItems(evSel, ids);
      if (i >= cnt) break;
      idxEvEdit = i;
      EvCfg &c = ev[evSel];
      switch (ids[i]) {
        case EI_EN:    c.en = !c.en; saveEv(evSel); break;
        case EI_COLOR: c.color = (c.color + 1) % 3; saveEv(evSel); break;
        case EI_PAT:   c.pat = (c.pat + 1) % 6; saveEv(evSel); break;
        case EI_INT:   c.ival = (c.ival + 1) % 4; saveEv(evSel); break;
        case EI_WHERE: c.whereAll = !c.whereAll; saveEv(evSel); break;
        case EI_SCR:   c.scr = !c.scr; saveEv(evSel); break;
        case EI_FIXED: break;
        default:
          if (evFromLed) gotoScr(MS_LED, idxLed);
          else           gotoScr(MS_EVLIST, idxEvList);
          break;
      }
      break;
    }
    case MS_PRESET:
      idxPreset = i;
      if (i < 3) { presetApply(i); toast("Preset applied"); }
      else       gotoScr(MS_LED, idxLed);
      break;
    case MS_RX:
      idxRx = i;
      rxTopic = i;
      if (i == RT_GLOB)      gotoScr(MS_RXGLOB, idxRxGlob);
      else if (i == RT_STAT) gotoScr(MS_RXSTAT, 0);
      else if (i == RT_WIFI) gotoScr(MS_RXWIFIM, idxRxWifi);
      else if (i == RT_BACK) gotoScr(MS_MAIN, idxMain);
      else                   gotoScr(MS_RXCH, idxRxCh);
      break;
    case MS_RXWIFIM:
      idxRxWifi = i;
      if (i == RW_BACK)           gotoScr(MS_RX, idxRx);
      else if (!rxLinked())       toast("RX not connected");
      else if (hlSyncBusy)        toast("Syncing...");
      else if (!othersCentered()) toast("Center controls");
      else if (i == RW_GH && netCount() == 0) toast("No saved network");
      else                        rxWifiBegin(i == RW_GH ? 1 : 0);
      break;
    case MS_RXCH:
      idxRxCh = i;
      if (i < NUM_CH) {
        rxCh = i;
        if (rxTopic == RT_CHEN) startRxEdit(RXP_CHEN, rxCh, 0, MS_RXCH);
        else                    gotoScr(MS_RXDIR, idxRxDir);
      } else gotoScr(MS_RX, idxRx);
      break;
    case MS_RXDIR:
      idxRxDir = i;
      if (i < 2) {
        if (rxTopic == RT_RAMP) startRxEdit(RXP_RAMPUP + i, rxCh, 0, MS_RXDIR);
        else                    startRxEdit(rxTopicParam(rxTopic), rxCh, i, MS_RXDIR);
      } else gotoScr(MS_RXCH, idxRxCh);
      break;
    case MS_RXGLOB:
      idxRxGlob = i;
      if (i < 6) startRxEdit((i == 0) ? RXP_MODE : RXP_OVLDLY + i - 1, 0, 0, MS_RXGLOB);
      else       gotoScr(MS_RX, idxRx);
      break;
    case MS_SCREENS:
      idxScr = i;
      if (i < NUM_SCR) { scrSel = i; gotoScr(MS_SCRITEM, 0); }
      else             gotoScr(MS_TX, idxTx);
      break;
    case MS_SCRITEM: {
      uint8_t id = scrOrder[scrSel];
      switch (i) {
        case 0: scrToggle(id); break;
        case 1: gotoScr(MS_SCRMOVE, scrSel); break;
        case 2: scrStart = id; scrEn |= (uint8_t)(1 << id); saveScreens(); toast("Start set"); break;
        default: gotoScr(MS_SCREENS, idxScr); break;
      }
      break;
    }
    case MS_POWER:
      idxPower = i;
      if (i == 0)      startTxPwrEdit();
      else if (i == 1) {
        if (!rxLinked()) toast("RX not connected");
        else if (hlSyncBusy) toast("Syncing...");
        else             startRxEdit(RXP_PWR, 0, 0, MS_POWER);
      } else gotoScr(MS_TX, idxTx);
      break;
    case MS_BIND:
      idxBind = i;
      if (i == 0)      startBind();
      else if (i == 1) gotoScr(MS_BINDID, 0);
      else if (i == 2) startConfirm(CA_FORGET, MS_BIND, idxBind);
      else             gotoScr(MS_TX, idxTx);
      break;
    case MS_EVLOG:
      idxEvLog = i;
      if (i == logN)      startConfirm(CA_CLEARLOG, MS_EVLOG, idxEvLog);
      else if (i > logN)  gotoScr(MS_TX, idxTx);
      break;
    case MS_DBGLIST:
      idxDbg = i;
      if (i < NUM_DP) { dbgPage = i; dbgTop = 0; gotoScr(MS_DBGDESC, 0); }
      else            gotoScr(MS_TX, idxTx);
      break;
    case MS_WIFI:
      idxWifi = i;
      if (i == WF_PHONE)     wifiBegin();
      else if (i == WF_GH)   ghBegin();
      else if (i == WF_NETS) gotoScr(MS_NETS, idxNets);
      else                   gotoScr(MS_TX, idxTx);
      break;
    case MS_NETACT:
      if (i == 0)      ghTestBegin(netSlot, MS_NETACT, 0, false);
      else if (i == 1) startConfirm(CA_NETDEL, MS_NETS, netSlot);
      else             gotoScr(MS_NETS, idxNets);
      break;
    case MS_NETS:
      idxNets = i;
      if (i < NET_MAX) {
        if (netSsid[i][0]) { netSlot = i; gotoScr(MS_NETACT, 0); }
        else               netAddBegin();
      }
      else if (i == NET_MAX)     netAddBegin();
      else if (i == NET_MAX + 1) {
        if (netCount() == 0) toast("Nothing saved");
        else                 startConfirm(CA_NETALL, MS_NETS, i);
      }
      else gotoScr(MS_WIFI, idxWifi);
      break;
    case MS_NETPICK:
      if (pickScanning) break;
      if (i < pickN) {
        snprintf(txtSsid, sizeof(txtSsid), "%s", pickNet[i].ssid);
        textBegin(1, 63);
      } else if (i == pickN) {
        textBegin(0, 32);
      } else {
        gotoScr(MS_NETS, idxNets);
      }
      break;
    case MS_FRTARGET:
      idxFr = i;
      if (i == 0)      startConfirm(CA_FR_TX, MS_FRTARGET, idxFr);
      else if (i == 1) {
        if (!rxLinked())            toast("RX not connected");
        else if (hlSyncBusy)        toast("Syncing...");
        else if (!othersCentered()) toast("Center controls");
        else                        startConfirm(CA_FR_RX, MS_FRTARGET, idxFr);
      } else gotoScr(MS_TX, idxTx);
      break;
    default:
      break;
  }
}

void menuBack() {
  switch (menuScr) {
    case MS_MAIN:     exitMenu(); break;
    case MS_TX:       gotoScr(MS_MAIN, idxMain); break;
    case MS_CAL:
    case MS_CHLIST:
    case MS_BAT:
    case MS_LED:
    case MS_SCREENS:
    case MS_POWER:
    case MS_BIND:
    case MS_EVLOG:
    case MS_DBGLIST:
    case MS_FRTARGET:
    case MS_WIFI:
    case MS_PIN:      gotoScr(MS_TX, idxTx); break;
    case MS_NETS:     gotoScr(MS_WIFI, idxWifi); break;
    case MS_NETACT:   gotoScr(MS_NETS, idxNets); break;
    case MS_NETPICK:  netScanStop(); gotoScr(MS_NETS, idxNets); break;
    case MS_TEXTIN:   gotoScr(MS_NETS, idxNets); break;
    case MS_CHEDIT:   gotoScr(MS_CHLIST, idxChList); break;
    case MS_DZ:       gotoScr(MS_CAL, idxCal); break;
    case MS_BATEDIT:  gotoScr(MS_BAT, idxBat); break;
    case MS_LEDSTAT:
    case MS_EVLIST:
    case MS_PRESET:
    case MS_TEST:     gotoScr(MS_LED, idxLed); break;
    case MS_EVEDIT:   if (evFromLed) gotoScr(MS_LED, idxLed); else gotoScr(MS_EVLIST, idxEvList); break;
    case MS_EXPO:
    case MS_MAXV:     gotoScr(MS_CHEDIT, idxChEdit); break;
    case MS_SCRITEM:  gotoScr(MS_SCREENS, idxScr); break;
    case MS_SCRMOVE:  saveScreens(); gotoScr(MS_SCRITEM, 0); break;
    case MS_TXPWR:    gotoScr(MS_POWER, idxPower); break;
    case MS_TRATE:    gotoScr(MS_TX, idxTx); break;
    case MS_BINDRUN:  hlBindOn = false; gotoScr(MS_BIND, idxBind); break;
    case MS_BINDID:   gotoScr(MS_BIND, idxBind); break;
    case MS_DBGDESC:
    case MS_DBGPAGE:  gotoScr(MS_DBGLIST, idxDbg); break;
    case MS_CONFIRM:  gotoScr(confBack, confIdx); break;
    case MS_RX:       gotoScr(MS_MAIN, idxMain); break;
    case MS_RXCH:     gotoScr(MS_RX, idxRx); break;
    case MS_RXDIR:    gotoScr(MS_RXCH, idxRxCh); break;
    case MS_RXGLOB:   gotoScr(MS_RX, idxRx); break;
    case MS_RXEDIT:   rxEditCancel(); break;
    case MS_RXSTAT:   gotoScr(MS_RX, idxRx); break;
    case MS_RXWIFI:   if (rxWifiSt == 5 || (simMode == SIM_RADIO && (rxWifiSt == HLW_PH_UP || rxWifiSt == HLW_GH_DL || rxWifiSt == HLW_GH_VERIFY))) break;   // во время загрузки прошивки выход заблокирован
                      if (simMode == SIM_RADIO) hlCfgPut(HLO_CMD, HLC_WIFI_OFF, 0, 0, 0, false, 0, 0);
                      rxWifiSt = 0; gotoScr(MS_RXWIFIM, idxRxWifi); break;
    case MS_RXWIFIM:  gotoScr(MS_RX, idxRx); break;
    case MS_GAME:     gotoScr(MS_MAIN, idxMain); break;
  }
}

void navStep(unsigned long now) {
  bool numeric = (menuScr == MS_DZ || menuScr == MS_EXPO || menuScr == MS_MAXV ||
                  menuScr == MS_BATEDIT || menuScr == MS_RXEDIT || menuScr == MS_TXPWR || menuScr == MS_TRATE ||
                  menuScr == MS_TEXTIN);
  int a = navPoll(MENU_NAV_CH, navA, now, numeric);
  int b = navPoll(MENU_ENTER_CH, navB, now, false);

  if (numeric) {
    int m = navMult(navA, now);
    if (b < 0 && menuScr != MS_TEXTIN) { menuBack(); return; }
    switch (menuScr) {
      case MS_TEXTIN:
        textStep(a, b, m);
        break;
      case MS_DZ:
        if (a) dzEdit = constrain(dzEdit + a * DZ_STEP * (m > 6 ? 6 : m), DZ_MIN, DZ_MAX);
        if (b > 0) {
          deadzone = dzEdit;
          saveDeadzone();
          LOGF("Dead zone saved: %d", deadzone);
          toast("Saved");
          gotoScr(MS_CAL, idxCal);
        }
        break;
      case MS_EXPO:
        if (a) editInt = constrain(editInt + a * EXPO_STEP * (m > 3 ? 3 : m), 0, 100);
        if (b > 0) {
          chExpo[editCh] = editInt;
          saveChannelCfg(editCh);
          LOGF("CH%d expo -> %d%%", editCh + 1, editInt);
          toast("Saved");
          gotoScr(MS_CHEDIT, idxChEdit);
        }
        break;
      case MS_MAXV:
        if (a) editInt = constrain(editInt + a * MAXP_STEP * (m > 3 ? 3 : m), MAXP_MIN, 100);
        if (b > 0) {
          chMaxP[editCh] = editInt;
          saveChannelCfg(editCh);
          LOGF("CH%d max -> %d%%", editCh + 1, editInt);
          toast("Saved");
          gotoScr(MS_CHEDIT, idxChEdit);
        }
        break;
      case MS_BATEDIT:
        if (a) batEditStep(a, m);
        if (b > 0) batEditSave();
        break;
      case MS_RXEDIT:
        if (a) rxEditStep(a, m);
        if (b > 0) rxEditSave();
        break;
      case MS_TXPWR:
        if (a) editInt = constrain(editInt + a, 0, 7);
        if (b > 0) {
          txPower = TXP_LIST[editInt];
          saveRadio();
          hlSetPower();
          LOGF("TX power -> %d dBm", txPower);
          toast("Saved");
          gotoScr(MS_POWER, idxPower);
        }
        break;
      case MS_TRATE:
        if (a) editInt = constrain(editInt + a, TELEM_RATE_MIN, TELEM_RATE_MAX);
        if (b > 0) {
          telemRate = editInt;
          saveRadio();
          LOGF("Telemetry rate -> %d/s", telemRate);
          toast("Saved");
          gotoScr(MS_TX, idxTx);
        }
        break;
      default:
        break;
    }
    return;
  }

  switch (menuScr) {
    case MS_DBGPAGE:
    case MS_RXSTAT:
    case MS_BINDID:
      if (b < 0) menuBack();                     // нечего подтверждать: выходит только "-"
      return;
    case MS_RXWIFI:
      if (b > 0 && simMode == SIM_RADIO && rxWifiSt == HLW_GH_ASK) { hlCfgPut(HLO_CMD, HLC_WIFI_YES, 0, 0, 1, false, 0, 0); toast("Install sent"); }
      if (b < 0) menuBack();
      return;
    case MS_GAME:
      gameStep(now, b);
      return;
    case MS_BINDRUN:
      if (b < 0) menuBack();
      return;
    case MS_TEST:
      if (a) testEv = (testEv + a + NUM_EV) % NUM_EV;
      if (b < 0) menuBack();
      return;
    case MS_DBGDESC: {
      int total = DESC[dbgPage].n;
      if (a > 0 && dbgTop + 6 < total) dbgTop += 3;
      if (a < 0) dbgTop = (dbgTop >= 3) ? dbgTop - 3 : 0;
      if (b > 0) gotoScr(MS_DBGPAGE, 0);
      else if (b < 0) menuBack();
      return;
    }
    case MS_PIN:
      pinStep(a, b);
      return;
    case MS_CONFIRM:
      if (b > 0) doConfirm();
      else if (b < 0) menuBack();
      return;
    case MS_SCRMOVE:
      if (a) { scrMove(a > 0 ? 1 : -1); sel = scrSel; adjustTop(); }
      if (b) menuBack();
      return;
    default:
      break;
  }

  int cnt = menuCount();
  if (a && cnt > 0) { sel = (sel + a + cnt) % cnt; adjustTop(); }
  if (b > 0) menuSelect(sel);
  else if (b < 0) menuBack();
}

// ============ КАЛИБРОВКА ============
unsigned long calStart = 0, calResMs = 3000;
int  calPhase = 0, calCount = 0, calFailCount = 0, calKind = 0;
long calSum[NUM_CH];
int16_t mnT[NUM_CH], mxT[NUM_CH];
bool calChOk[NUM_CH];

void startCalCenter() {
  state = ST_CAL_C;
  calPhase = 0;
  calStart = millis();
  logLine("CAL: set all controls to center, wait 5 s");
}

void startCalRange() {
  for (int i = 0; i < NUM_CH; i++) { mnT[i] = 32767; mxT[i] = -32768; }
  state = ST_CAL_R;
  calStart = millis();
  logLine("CAL: move all controls to both extremes (10 s)");
}

void cancelCal() {
  logLine("CAL: cancelled");
  state = ST_MENU;
  gotoScr(MS_CAL, idxCal);
}

void calCenterTick(unsigned long now) {
  if (calPhase == 0) {
    if (now - calStart >= CAL_COUNT_MS) {
      calPhase = 1;
      calStart = now;
      calCount = 0;
      for (int i = 0; i < NUM_CH; i++) calSum[i] = 0;
    }
  } else {
    for (int i = 0; i < NUM_CH; i++) calSum[i] += raw[i];
    calCount++;
    if ((now - calStart) >= CAL_AVG_MS && calCount > 0) {
      for (int i = 0; i < NUM_CH; i++) {
        cenV[i] = (int16_t)(calSum[i] / calCount);
        saveCenter(i);
      }
      calKind = 0;
      calFailCount = 0;
      calResMs = 2500;
      calStart = now;
      state = ST_CAL_RES;
    }
  }
}

void calRangeTick(unsigned long now) {
  for (int i = 0; i < NUM_CH; i++) {
    if (raw[i] < mnT[i]) mnT[i] = raw[i];
    if (raw[i] > mxT[i]) mxT[i] = raw[i];
  }
  if ((now - calStart) >= CAL_RANGE_MS) {
    calFailCount = 0;
    for (int i = 0; i < NUM_CH; i++) {
      bool ok = adsOk[CH[i].ads] &&
                (cenV[i] - mnT[i]) >= MIN_SIDE &&
                (mxT[i] - cenV[i]) >= MIN_SIDE;
      calChOk[i] = ok;
      if (ok) {
        mnV[i] = mnT[i];
        mxV[i] = mxT[i];
        saveRange(i);
        LOGF("CAL: CH%d ok  min=%d max=%d", i + 1, mnV[i], mxV[i]);
      } else {
        calFailCount++;
        LOGF("CAL: CH%d NOT MOVED  min=%d max=%d (old range kept)", i + 1, mnT[i], mxT[i]);
      }
    }
    calKind = 1;
    calResMs = calFailCount ? 5000 : 3000;
    calStart = now;
    state = ST_CAL_RES;
  }
}

// ============ ПЕРЕХОДЫ БЕЗОПАСНОСТИ ============
void goEstop() {
  state = ST_ESTOP;
  logEvent(LT_ESTOP, 0, 0, 0);
  navReset();
  rxRevertAll();
  rxWifiSt = 0;                          // Wi-Fi приёмника (и режим Test) закрывается
  logLine("STATE: ESTOP");
}

void goSafe() {
  state = ST_SAFE;
  safeSince = 0;
  logLine("STATE: SAFE START (set all controls to center)");
}

// ============ WI-FI UPDATE (обновление прошивки с телефона) ============
void drawScreen();
void oledStep();
bool oledBusy();

// Метка прошивки: страница и пульт ищут её внутри файла, так определяется, для какого устройства файл
static const char FW_TAG[] __attribute__((used)) = "FWTAG:" PROJECT_NAME ":" FW_VERSION ";";

static const char WIFI_PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>FORESTER</title><style>
:root{--bg:#f1f2ef;--c:#fff;--fg:#14181a;--m:#586166;--l:#d6dad5;--a:#1f5d3a;--af:#fff;--ok:#1f5d3a;--w:#a8480f;--b:#b3261e;--t:#e2e5e0}
@media(prefers-color-scheme:dark){:root{--bg:#0f1213;--c:#171b1d;--fg:#eceeed;--m:#9aa3a8;--l:#2a3033;--a:#5fbf86;--af:#07130c;--ok:#5fbf86;--w:#e5904f;--b:#ff8d84;--t:#262c2f;color-scheme:dark}}
*{box-sizing:border-box}body{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:16px/1.4 system-ui,sans-serif}
.w{max-width:460px;margin:0 auto;display:flex;flex-direction:column;gap:12px}
.top{display:flex;justify-content:space-between;align-items:center}
.pill{background:#000;color:#fff;border-radius:999px;padding:5px 16px;font-weight:800;letter-spacing:.06em}
.lg{background:none;border:1px solid var(--l);color:var(--m);border-radius:8px;padding:6px 10px;font-size:13px}
h1{font-size:24px;margin:2px 0}
.cd{background:var(--c);border:1px solid var(--l);border-radius:10px;padding:14px 16px;display:flex;flex-direction:column;gap:10px}
h2{margin:0;font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--m)}
.r{display:flex;justify-content:space-between;gap:12px;font-size:14px}.r b{font-family:ui-monospace,monospace;font-weight:500;text-align:right;overflow-wrap:anywhere}
.pk{display:block;border:2px dashed var(--l);border-radius:10px;padding:18px 12px;text-align:center;cursor:pointer}
.pk span{display:block;color:var(--m);font-size:14px}
input[type=file]{position:absolute;width:1px;height:1px;opacity:0}
button.m{background:var(--a);color:var(--af);border:0;border-radius:10px;padding:14px;font:700 16px system-ui;min-height:48px}
button.m:disabled{opacity:.35}
.bar{height:14px;background:var(--t);border-radius:7px;overflow:hidden}.bar i{display:block;height:100%;width:0;background:var(--a)}
.ok{color:var(--ok)}.bad{color:var(--b)}.wn{color:var(--w)}
.ms{border:1px solid var(--l);border-radius:8px;padding:10px 12px;font-size:14px}
.in{border:1px solid var(--l);border-radius:8px;padding:12px;background:var(--bg);color:var(--fg);font:16px system-ui;width:100%}
.sb{background:none;border:1px solid var(--l);color:var(--fg);border-radius:8px;padding:8px 10px;font-size:14px;text-align:left}
#lst{display:flex;flex-direction:column;gap:6px}
[hidden]{display:none!important}
</style></head><body><div class="w">
<div class="top"><span class="pill">FORESTER</span><button class="lg" id="lg">RU/EN</button></div>
<h1 id="h1"></h1>
<div class="cd"><h2 id="t0"></h2>
<div class="r"><span id="k1"></span><b id="v1">-</b></div>
<div class="r"><span id="k2"></span><b id="v2">-</b></div>
<div class="r"><span id="k3"></span><b id="v3">-</b></div></div>
<div class="cd" id="c1"><h2 id="t1"></h2>
<label class="pk" for="f"><b id="pt"></b><span id="ph"></span></label>
<input type="file" id="f" accept=".bin">
<div id="ck" hidden></div>
<button class="m" id="go" disabled></button></div>
<div class="cd" id="c4"><h2 id="t4"></h2><div class="ms" id="hn"></div><div id="sv"></div>
<button class="m" id="sc"></button><div id="lst"></div>
<input class="in" id="ss" maxlength="32" autocomplete="off"><input class="in" id="pw" maxlength="63" autocomplete="off">
<button class="m" id="sa"></button><div class="ms" id="sm" hidden></div></div>
<div class="cd" id="c2" hidden><h2 id="t2"></h2><div class="bar"><i id="b"></i></div>
<div class="r"><span id="st"></span><b id="pc">0%</b></div><div class="ms" id="wm"></div></div>
<div class="cd" id="c3" hidden><div class="ms" id="rs"></div><button class="m" id="bk"></button></div>
</div><script>
var T={en:{h1:'Firmware update',t0:'This device',k1:'Device',k2:'Firmware',k3:'Network',t1:'New firmware',pt:'Choose .bin file',ph:'Sketch > Export Compiled Binary',
ck1:'Firmware for',ck2:'Size',ck3:'Version',go:'Install',t2:'Installing',up:'Uploading',vf:'Verifying and installing',wm:'Do not switch off the device and do not leave the network.',
ok:'Installed. The device is restarting, close this page.',wr:'Wrong firmware: this file is not for this device. Nothing was changed.',er:'Error: ',bk:'Back',nf:'Not a firmware file',t4:'Wi-Fi for GitHub update',n0:'No saved networks',sc:'Scan networks',scn:'Scanning...',sn:'Network name',pw:'Password (empty = open)',sa:'Save',sd:'Saved',fl:'List is full (3). Delete one.',dl:'Delete',e2:'Error',hn:'Check the connection on the pult: Wi-Fi > Networks > select the network > Test connection.'},
ru:{h1:'Обновление прошивки',t0:'Это устройство',k1:'Устройство',k2:'Прошивка',k3:'Сеть',t1:'Новая прошивка',pt:'Выбрать файл .bin',ph:'Скетч > Экспорт бинарного файла',
ck1:'Прошивка для',ck2:'Размер',ck3:'Версия',go:'Установить',t2:'Установка',up:'Загрузка',vf:'Проверка и установка',wm:'Не выключайте устройство и не отключайтесь от сети.',
ok:'Установлено. Устройство перезагружается, закройте страницу.',wr:'Не та прошивка: файл не для этого устройства. Ничего не изменено.',er:'Ошибка: ',bk:'Назад',nf:'Это не файл прошивки',t4:'Wi-Fi для обновления с GitHub',n0:'Нет сохранённых сетей',sc:'Найти сети',scn:'Поиск...',sn:'Название сети',pw:'Пароль (пусто = открытая)',sa:'Сохранить',sd:'Сохранено',fl:'Список полон (3). Удалите одну.',dl:'Удалить',e2:'Ошибка',hn:'Проверить подключение можно на пульте: Wi-Fi > Networks > выбрать сеть > Test connection.'}};
var $=function(i){return document.getElementById(i)},lang=(navigator.language||'en').slice(0,2)=='ru'?'ru':'en',D={dev:'',name:'-',ver:'-',net:'-'},F=null;
function tx(k){return T[lang][k]}
function ui(){var m={h1:'h1',t0:'t0',k1:'k1',k2:'k2',k3:'k3',t1:'t1',pt:'pt',ph:'ph',go:'go',t2:'t2',wm:'wm',bk:'bk',t4:'t4',sc:'sc',sa:'sa',hn:'hn'};for(var k in m)$(m[k]).textContent=tx(k);$('ss').placeholder=tx('sn');$('pw').placeholder=tx('pw');ld();$('v1').textContent=D.name;$('v2').textContent=D.ver;$('v3').textContent=D.net;if(F)chk()}
$('lg').onclick=function(){lang=lang=='en'?'ru':'en';ui()};
function tag(u,d){var p='FWTAG:'+d+':',n=p.length;for(var i=0;i<u.length-n-2;i++){if(u[i]!==70)continue;var j=0;while(j<n&&u[i+j]===p.charCodeAt(j))j++;if(j===n&&u[i+n]>=48&&u[i+n]<=57){var s='';for(var k=i+n;k<i+n+12&&u[k]!==59&&u[k]!==0;k++)s+=String.fromCharCode(u[k]);return s}}return''}
function chk(){var c=$('ck'),o='';c.hidden=false;if(F.v){o='<div class="r"><span>'+tx('ck1')+'</span><b class="ok">'+D.name+' &#10003;</b></div><div class="r"><span>'+tx('ck2')+'</span><b>'+Math.round(F.f.size/1024)+' KB</b></div><div class="r"><span>'+tx('ck3')+'</span><b class="wn">'+D.ver+' &rarr; '+F.v+'</b></div>'}else o='<div class="ms bad">'+tx(F.nf?'nf':'wr')+'</div>';c.innerHTML=o;$('go').disabled=!F.v}
$('f').onchange=function(){var f=this.files[0];if(!f)return;$('pt').textContent=f.name;var r=new FileReader();r.onload=function(){var u=new Uint8Array(r.result),img=u[0]===233;F={f:f,v:img?tag(u,D.dev):'',nf:!img};chk()};r.readAsArrayBuffer(f)};
function show(a){$('c1').hidden=a!=1;$('c2').hidden=a!=2;$('c3').hidden=a!=3}
function fin(ok,m){show(3);$('rs').className='ms '+(ok?'ok':'bad');$('rs').textContent=m;$('bk').hidden=ok}
$('bk').onclick=function(){show(1)};
$('go').onclick=function(){show(2);$('st').textContent=tx('up');var x=new XMLHttpRequest(),d=new FormData();d.append('f',F.f,'fw.bin');x.open('POST','/update?size='+F.f.size);
x.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);$('b').style.width=p+'%';$('pc').textContent=p+'%';if(p>=100)$('st').textContent=tx('vf')}};
x.onload=function(){var r;try{r=JSON.parse(x.responseText)}catch(e){r={ok:false,err:'?'}}fin(r.ok,r.ok?tx('ok'):(r.err==='wrong'?tx('wr'):tx('er')+r.err))};
x.onerror=function(){fin(false,tx('er')+'connection')};x.send(d)};
function post(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b}).then(function(r){return r.json()})}
function ld(){fetch('/nets').then(function(r){return r.json()}).then(function(j){var s=$('sv');s.textContent='';if(!j.n.length){s.textContent=tx('n0');return}j.n.forEach(function(o){var d=document.createElement('div'),a=document.createElement('b'),b=document.createElement('button');d.className='r';a.textContent=o.s;b.className='sb';b.textContent=tx('dl');b.onclick=function(){post('/netdel','i='+o.i).then(ld)};d.appendChild(a);d.appendChild(b);s.appendChild(d)})}).catch(function(){})}
function msg(t){var m=$('sm');m.hidden=!t;m.textContent=t}
function poll(){fetch('/scanres').then(function(r){return r.json()}).then(function(j){if(j.run){setTimeout(poll,1000);return}$('sc').disabled=false;msg('');var l=$('lst');l.textContent='';j.l.forEach(function(o){var b=document.createElement('button');b.className='sb';b.textContent=o.s+'   '+o.r+' dBm';b.onclick=function(){$('ss').value=o.s;$('pw').focus()};l.appendChild(b)})}).catch(function(){setTimeout(poll,1500)})}
$('sc').onclick=function(){$('sc').disabled=true;msg(tx('scn'));fetch('/scan').then(poll).catch(function(){setTimeout(poll,1500)})};
$('sa').onclick=function(){var s=$('ss').value;if(!s)return;post('/netadd','s='+encodeURIComponent(s)+'&p='+encodeURIComponent($('pw').value)).then(function(j){msg(j.ok?tx('sd'):(j.err==='full'?tx('fl'):tx('e2')));if(j.ok){$('pw').value='';ld()}}).catch(function(){msg(tx('e2'))})};
ui();show(1);fetch('/info').then(function(r){return r.json()}).then(function(j){D=j;ui()}).catch(function(){});
</script></body></html>
)HTML";

WebServer server(80);
static char    scanPre[24];
static size_t  scanPreLen = 0;
static uint8_t scanBuf[1536 + 64];
static uint8_t scanTail[48];
static size_t  scanTailLen = 0;
static bool    upChecked = false, upFound = false;
static bool    wifiRoutes = false;

static void upFail(const char* msg) {
  snprintf(upErr, sizeof(upErr), "%s", msg);
  Update.abort();
}

// Ищет в потоке байт метку "FWTAG:pult:<версия>;" (может оказаться на стыке двух блоков)
static void upScan(const uint8_t* d, size_t n, bool last) {
  if (upFound) return;
  const size_t need = scanPreLen + 12;
  size_t carry = scanTailLen;
  if (carry + n > sizeof(scanBuf)) n = sizeof(scanBuf) - carry;
  memcpy(scanBuf, scanTail, carry);
  memcpy(scanBuf + carry, d, n);
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

static void wifiServiceDisplay() {            // экран и страницы остаются живыми, пока идёт приём файла
  oledStep();
  static unsigned long lastD = 0;
  unsigned long now = millis();
  if (okOled && !oledBusy() && (now - lastD) >= DISP_PERIOD_MS) { lastD = now; drawScreen(); }
}

static void wifiRoot() {
  wifiAct = millis();
  server.send_P(200, "text/html; charset=utf-8", WIFI_PAGE);
}

static void wifiInfo() {            // запрашивает скрипт страницы: страница реально открылась
  wifiPage = true;
  wifiAct = millis();
  char b[160];
  snprintf(b, sizeof(b), "{\"dev\":\"%s\",\"name\":\"PULT\",\"ver\":\"%s\",\"net\":\"%s\"}", PROJECT_NAME, FW_VERSION, wifiSsid);
  server.send(200, "application/json", b);
}

// Проверки интернета: отвечаем "интернет есть", чтобы телефон не отключался от сети без интернета
static void wifiOk204() { server.send(204, "text/plain", ""); }
static void wifiOkApple() { server.send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"); }
static void wifiOkMsft() { server.send(200, "text/plain", "Microsoft Connect Test"); }
static void wifiOkNcsi() { server.send(200, "text/plain", "Microsoft NCSI"); }

static void jsonEsc(const char* in, char* out, size_t n) {   // экранирование для JSON-строки
  size_t k = 0;
  for (; *in && k + 7 < n; in++) {
    unsigned char c = (unsigned char)*in;
    if (c == '"' || c == '\\') { out[k++] = '\\'; out[k++] = (char)c; }
    else if (c >= 0x20)        { out[k++] = (char)c; }
  }
  out[k] = 0;
}

static void wifiNets() {                       // GET /nets: сохранённые сети
  wifiAct = millis();
  String j = "{\"max\":" + String(NET_MAX) + ",\"n\":[";
  bool first = true;
  for (int i = 0; i < NET_MAX; i++) {
    if (!netSsid[i][0]) continue;
    char e[80];
    jsonEsc(netSsid[i], e, sizeof(e));
    if (!first) j += ",";
    first = false;
    j += "{\"i\":" + String(i) + ",\"s\":\"" + e + "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j.c_str());
}

static void wifiScanStart() {                  // GET /scan: поиск сетей (фоновый)
  wifiAct = millis();
  WiFi.scanDelete();
  WiFi.scanNetworks(true);
  server.send(200, "application/json", "{\"run\":true}");
}

static void wifiScanRes() {                    // GET /scanres: результат поиска
  wifiAct = millis();
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) { server.send(200, "application/json", "{\"run\":true}"); return; }
  NetFound f[12];
  int c = (n > 0) ? netCollect(n, f, 12) : 0;
  WiFi.scanDelete();
  String j = "{\"run\":false,\"l\":[";
  for (int i = 0; i < c; i++) {
    char e[80];
    jsonEsc(f[i].ssid, e, sizeof(e));
    if (i) j += ",";
    j += "{\"s\":\"" + String(e) + "\",\"r\":" + String((int)f[i].rssi) + "}";
  }
  j += "]}";
  server.send(200, "application/json", j.c_str());
}

static void wifiNetAdd() {                     // POST /netadd (s, p)
  wifiAct = millis();
  int r = netAdd(server.arg("s").c_str(), server.arg("p").c_str());
  if (r == 0) server.send(200, "application/json", "{\"ok\":true}");
  else        server.send(200, "application/json", r == 1 ? "{\"ok\":false,\"err\":\"full\"}" : "{\"ok\":false,\"err\":\"bad\"}");
}

static void wifiNetDel() {                     // POST /netdel (i)
  wifiAct = millis();
  netDel((int)server.arg("i").toInt());
  server.send(200, "application/json", "{\"ok\":true}");
}

static void wifiRedirect() {                  // проверки интернета телефоном -> переход на страницу
  server.sendHeader("Location", "http://" WIFI_ADDR "/", true);
  server.send(302, "text/plain", "");
}

static void wifiUpload() {
  HTTPUpload& up = server.upload();
  wifiAct = millis();
  if (up.status == UPLOAD_FILE_START) {
    upActive = true; upOk = false; upChecked = false; upFound = false;
    upErr[0] = 0; upVer[0] = 0; upDone = 0; scanTailLen = 0;
    upTotal = (size_t)server.arg("size").toInt();
    logLine("Wi-Fi Update: upload started");
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
    wifiServiceDisplay();
  } else if (up.status == UPLOAD_FILE_END) {
    if (!upErr[0]) {
      upScan(nullptr, 0, true);
      if (!upChecked)       upFail("empty file");
      else if (!upFound)    { upFail("wrong"); }
      else if (Update.end(true)) { upOk = true; }
      else                  upFail(Update.errorString());
    }
    upActive = false;
    LOGF("Wi-Fi Update: upload end, %s %s", upOk ? "OK" : "FAIL", upOk ? upVer : upErr);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (upChecked) Update.abort();
    upActive = false;
    snprintf(upErr, sizeof(upErr), "aborted");
  }
}

static void wifiUpdateDone() {
  wifiAct = millis();
  char b[96];
  if (upOk) {
    server.send(200, "application/json", "{\"ok\":true}");
    restartAt = millis() + 2500UL;
  } else {
    snprintf(b, sizeof(b), "{\"ok\":false,\"err\":\"%s\"}", upErr[0] ? upErr : "unknown");
    server.send(200, "application/json", b);
  }
}

void wifiBegin() {
  if (batLevel >= 1 && !batUsb) { toast("Battery low"); return; }
  uint64_t mac = ESP.getEfuseMac();
  snprintf(wifiSsid, sizeof(wifiSsid), "PULT-%02X%02X", (unsigned)((mac >> 32) & 0xFF), (unsigned)((mac >> 40) & 0xFF));
  snprintf(scanPre, sizeof(scanPre), "FWTAG:%s:", PROJECT_NAME);
  scanPreLen = strlen(scanPre);
  WiFi.mode(WIFI_AP_STA);                       // STA нужен только для поиска сетей на странице
  WiFi.softAP(wifiSsid, WIFI_PASS);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  if (!wifiRoutes) {
    server.on("/", HTTP_GET, wifiRoot);
    server.on("/info", HTTP_GET, wifiInfo);
    server.on("/update", HTTP_POST, wifiUpdateDone, wifiUpload);
    server.on("/nets", HTTP_GET, wifiNets);
    server.on("/scan", HTTP_GET, wifiScanStart);
    server.on("/scanres", HTTP_GET, wifiScanRes);
    server.on("/netadd", HTTP_POST, wifiNetAdd);
    server.on("/netdel", HTTP_POST, wifiNetDel);
    server.on("/generate_204", HTTP_GET, wifiOk204);
    server.on("/gen_204", HTTP_GET, wifiOk204);
    server.on("/hotspot-detect.html", HTTP_GET, wifiOkApple);
    server.on("/library/test/success.html", HTTP_GET, wifiOkApple);
    server.on("/connecttest.txt", HTTP_GET, wifiOkMsft);
    server.on("/ncsi.txt", HTTP_GET, wifiOkNcsi);
    server.onNotFound(wifiRedirect);
    wifiRoutes = true;
  }
  server.begin();
  wifiUp = true; wifiEver = false; wifiPage = false;
  upActive = false; upOk = false; upErr[0] = 0; upVer[0] = 0;
  wifiT0 = millis(); wifiAct = wifiT0;
  state = ST_WIFI;
  navReset();
  LOGF("Wi-Fi Update: on, net %s, pass %s, %s", wifiSsid, WIFI_PASS, WIFI_ADDR);
}

void wifiStop(const char* why) {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiUp = false;
  state = ST_MENU;
  gotoScr(MS_WIFI, idxWifi);
  navReset();
  LOGF("Wi-Fi Update: off (%s)", why);
  char tb[20]; snprintf(tb, sizeof(tb), "Wi-Fi off: %.8s", why); toast(tb);
  if (estop.state) goEstop();
}

void wifiTick(unsigned long now) {
  if (state != ST_WIFI) return;
  server.handleClient();
  now = millis();                                           // handleClient мог обновить wifiAct позже, чем "now" из начала цикла
  if (upActive) { wifiAct = now; return; }                 // во время приёма файла выхода и таймаутов нет
  if (navPoll(MENU_ENTER_CH, navB, now, false) < 0) { wifiStop("lever"); return; }   // CH6 "-" = выход
  if (estop.state) { wifiStop("E-Stop"); return; }
  if (WiFi.softAPgetStationNum() > 0 && !wifiEver) { wifiEver = true; wifiAct = now; }
  if (!wifiEver) {
    if ((long)(now - wifiT0) >= (long)WIFI_NOCONN_MS) wifiStop("no phone");
  } else if ((long)(now - wifiAct) >= (long)WIFI_IDLE_MS) {
    wifiStop("idle");
  }
}

// ============ ЗАГРУЗКА ПРОШИВКИ С GITHUB ============
// Пульт подключается к сохранённой сети, читает описание последнего релиза (имя файла, размер, SHA-256),
// после подтверждения (CH6) скачивает файл прямо в раздел обновления и проверяет его.
// Шифрование: сертификат сервера пока не проверяется (setInsecure), целостность файла держится на SHA-256
// из ответа GitHub и метке проекта внутри файла.
static uint8_t ghBuf[1024];
static bool    ghBegan = false;      // Update.begin уже вызван

static void ghFrame() {              // нарисовать и вытолкнуть кадр перед долгим блокирующим шагом
  if (!okOled) return;
  unsigned long t0 = millis();
  drawScreen();
  while (oledBusy() && (millis() - t0) < 300) oledStep();
}

static void ghWifiEvt(arduino_event_id_t ev, arduino_event_info_t info) {
  if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) ghReason = info.wifi_sta_disconnected.reason;
}
static bool ghAuthReason(int r) { return r == 15 || r == 202 || r == 204; }
static const char* ghReasonText(int r) {
  if (ghAuthReason(r)) return "Wrong password";
  if (r == 201)        return "Network not found";
  if (r == 203 || r == 5 || r == 4) return "Router refused";
  if (r == 200 || r == 8 || r == 1 || r == 2) return "Signal lost";
  return "Can't connect to";
}
char ghErrInfo[22] = "";                  // код для диагностики: "http 404", "update 12"
static void ghFail(const char* m, const char* m2 = "", const char* info = "") {
  ghErrCode = 0;
  snprintf(ghErr, sizeof(ghErr), "%s", m);
  snprintf(ghErr2, sizeof(ghErr2), "%s", m2);
  snprintf(ghErrInfo, sizeof(ghErrInfo), "%s", info);
  if (ghBegan) { Update.abort(); ghBegan = false; }
  WiFi.disconnect();
  ghStage = G_ERR;
  ghT0 = millis();
  LOGF("GitHub: %s %s", m, m2);
}

void ghStop(const char* why) {
  if (ghBegan) { Update.abort(); ghBegan = false; }
  WiFi.scanDelete();
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  state = ST_MENU;
  gotoScr(ghBackScr, ghBackIdx);
  navReset();
  LOGF("GitHub Update: off (%s)", why);
  if (strcmp(why, "declined") == 0) toast("Cancelled");
  else { char tb[20]; snprintf(tb, sizeof(tb), "Wi-Fi off: %.8s", why); toast(tb); }
  if (estop.state) goEstop();
}

static void ghStart(int testSlot, MenuScr back, int idx, bool fresh) {
  if (batLevel >= 1 && !batUsb) { toast("Battery low"); return; }
  ghTestSlot = testSlot; ghFresh = fresh; ghBackScr = back; ghBackIdx = idx;
  state = ST_GH;
  ghErr[0] = ghErr2[0] = 0; ghNet[0] = 0; ghVer[0] = 0; ghSha[0] = 0; ghUrl[0] = 0;
  ghSize = ghDone = 0; ghBegan = false;
  ghT0 = millis();
  navReset();
  if (testSlot >= 0) snprintf(ghNet, sizeof(ghNet), "%s", netSsid[testSlot]);
  if (netCount() == 0) { ghStage = G_ERR; snprintf(ghErr, sizeof(ghErr), "No saved network"); snprintf(ghErr2, sizeof(ghErr2), "Add: Wi-Fi Networks"); return; }
  ghStage = G_SCAN;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.scanNetworks(true);
  LOGF(testSlot >= 0 ? "Wi-Fi test: start" : "GitHub Update: start");
}

void ghBegin() { ghStart(-1, MS_WIFI, idxWifi, false); }
void ghTestBegin(int slot, MenuScr back, int idx, bool fresh) { ghStart(slot, back, idx, fresh); }

// ответ GitHub: ищем файл "pult_<версия>.bin" (не *_full.bin), берём размер, SHA-256 и адрес скачивания
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
    if (body.indexOf('"', dg + 17) != dg + 17 + 64) continue;                      // сумма ровно 64 символа
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
  ghFrame();
  WiFiClientSecure cl;
  cl.setInsecure();
  HTTPClient http;
  http.setUserAgent("FORESTER-pult");
  http.addHeader("Accept", "application/vnd.github+json");
  http.setTimeout(10000);
  if (!http.begin(cl, "https://api.github.com/repos/" GH_REPO "/releases/tags/" GH_TAG)) { ghFail("GitHub not reachable"); return; }
  int code = http.GET();
  char hi[22]; snprintf(hi, sizeof(hi), "http %d", code);
  if (code <= 0)                 { http.end(); ghFail("GitHub not reachable", "", hi); return; }
  if (code == 404)               { http.end(); ghFail("Release not found", "Is the repo public?", hi); return; }
  if (code == 403 || code == 429){ http.end(); ghFail("GitHub busy", "Try again later", hi); return; }
  if (code != 200)               { http.end(); char t[24]; snprintf(t, sizeof(t), "GitHub error %d", code); ghFail(t, "", hi); return; }
  String body = http.getString();
  http.end();
  if (!ghParse(body)) { ghFail("No firmware found", "in the release"); return; }
  LOGF("GitHub: found %s %u B", ghVer, (unsigned)ghSize);
  ghStage = G_ASK;
  ghT0 = millis();
  navReset();
}

static bool ghChunk(const uint8_t* b, size_t n, mbedtls_sha256_context* sha) {
  upScan(b, n, false);
  mbedtls_sha256_update(sha, b, n);
  if (Update.write((uint8_t*)b, n) != n) { { char ui[22]; snprintf(ui, sizeof(ui), "update %d", (int)Update.getError()); ghFail("Write error", Update.errorString(), ui); } return false; }
  ghDone += n;
  return true;
}

static void ghDownload() {
  ghStage = G_DL;
  ghDone = 0;
  upFound = false; upVer[0] = 0; scanTailLen = 0;
  snprintf(scanPre, sizeof(scanPre), "FWTAG:%s:", PROJECT_NAME);
  scanPreLen = strlen(scanPre);
  ghFrame();
  WiFiClientSecure cl;
  cl.setInsecure();
  HTTPClient http;
  http.setUserAgent("FORESTER-pult");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setTimeout(15000);
  if (!http.begin(cl, ghUrl)) { ghFail("Download failed"); return; }
  int code = http.GET();
  if (code != 200) { http.end(); char t[24], hi[22]; snprintf(t, sizeof(t), "Download error %d", code); snprintf(hi, sizeof(hi), "http %d", code); ghFail(code <= 0 ? "Download failed" : t, "", hi); return; }
  int len = http.getSize();
  if (len > 0 && (size_t)len != ghSize) { http.end(); ghFail("Size mismatch"); return; }
  WiFiClient* st = http.getStreamPtr();
  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  mbedtls_sha256_starts(&sha, 0);
  bool ok = true;
  // первые 64 байта: проверка заголовка образа ESP32-S3, затем начало записи
  uint8_t hdr[64];
  if (st->readBytes(hdr, sizeof(hdr)) != sizeof(hdr)) { ok = false; ghFail("Download failed"); }
  else if (hdr[0] != 0xE9 || (hdr[12] | (hdr[13] << 8)) != 9) { ok = false; ghFail("Not an ESP32-S3 image"); }
  else if (!Update.begin(ghSize)) { ok = false; char ui[22]; snprintf(ui, sizeof(ui), "update %d", (int)Update.getError()); ghFail("Update error", Update.errorString(), ui); }
  else {
    ghBegan = true;
    ok = ghChunk(hdr, sizeof(hdr), &sha);
  }
  unsigned long last = millis();
  while (ok && ghDone < ghSize) {
    size_t av = st->available();
    if (av == 0) {
      if (!http.connected() || (millis() - last) > 15000UL) { ok = false; ghFail("Download failed"); break; }
      wifiServiceDisplay();
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
    wifiServiceDisplay();
  }
  http.end();
  uint8_t dig[32];
  mbedtls_sha256_finish(&sha, dig);
  mbedtls_sha256_free(&sha);
  if (!ok) return;
  ghFrame();                                                  // показать "Verifying"
  upScan(nullptr, 0, true);
  char hex[65];
  for (int i = 0; i < 32; i++) snprintf(hex + i * 2, 3, "%02x", dig[i]);
  if (ghDone != ghSize)                       { ghFail("Download failed"); return; }
  if (strcasecmp(hex, ghSha) != 0)            { ghFail("Checksum error", "Nothing changed"); return; }
  if (!upFound)                               { ghFail("Wrong firmware", "Nothing changed"); return; }
  if (!Update.end(true))                      { char ui[22]; snprintf(ui, sizeof(ui), "update %d", (int)Update.getError()); ghFail("Update error", Update.errorString(), ui); return; }
  ghBegan = false;
  ghStage = G_OK;
  restartAt = millis() + 2500UL;
  LOGF("GitHub Update: installed %s", upVer);
}

void ghTick(unsigned long now) {
  if (state != ST_GH) return;
  if (estop.state && ghStage != G_DL && ghStage != G_OK) { ghStop("E-Stop"); return; }
  int nb = (ghStage == G_DL || ghStage == G_OK || ghStage == G_API) ? 0 : navPoll(MENU_ENTER_CH, navB, now, false);
  if (nb < 0 && ghStage != G_ASK) { ghStop(ghStage == G_SCAN || ghStage == G_CONN ? "cancel" : "exit"); return; }   // CH6 = выход / отмена
  switch (ghStage) {
    case G_SCAN: {
      int n = WiFi.scanComplete();
      if (n == WIFI_SCAN_RUNNING) { if ((now - ghT0) > 15000UL) ghFail("Scan failed"); break; }
      if (n < 0) { ghFail("Scan failed"); break; }
      int best = -1, bestR = -1000;
      for (int i = 0; i < n; i++) {
        String sd = WiFi.SSID(i);
        int r = WiFi.RSSI(i);
        for (int k = 0; k < NET_MAX; k++) {
          if (ghTestSlot >= 0 && k != ghTestSlot) continue;
          if (netSsid[k][0] && sd == netSsid[k] && r > bestR) { best = k; bestR = r; }
        }
      }
      WiFi.scanDelete();
      if (best < 0) {
        if (ghTestSlot >= 0) ghFail("Network not in range", ghFresh ? "Saved, not tested" : netSsid[ghTestSlot]);
        else                 ghFail("No saved network", "in range");
        break;
      }
      snprintf(ghNet, sizeof(ghNet), "%s", netSsid[best]);
      { static bool evSet = false;
        if (!evSet) { WiFi.onEvent(ghWifiEvt, ARDUINO_EVENT_WIFI_STA_DISCONNECTED); evSet = true; } }
      ghReason = 0;
      ghPassLen = strlen(netPass[best]);
      WiFi.begin(netSsid[best], netPass[best][0] ? netPass[best] : nullptr);
      ghStage = G_CONN;
      ghT0 = now;
      break;
    }
    case G_CONN: {
      wl_status_t st = WiFi.status();
      if (st == WL_CONNECTED) {
        if (ghTestSlot >= 0) {
          ghRssi = WiFi.RSSI();
          snprintf(ghIp, sizeof(ghIp), "%s", WiFi.localIP().toString().c_str());
          ghStage = G_TEST;
          LOGF("Wi-Fi test: connected %d dBm", ghRssi);
        } else {
          ghStage = G_API;
        }
        ghT0 = now;
      }
      else if (st == WL_CONNECT_FAILED || ghAuthReason(ghReason)) {
        int r = ghReason;
        ghFail("Wrong password", ghNet);
        ghErrCode = r ? r : 15;
      }
      else if ((now - ghT0) > 30000UL) {
        int r = ghReason;
        ghFail(ghReasonText(r), ghNet);
        ghErrCode = r;
      }
      break;
    }
    case G_API:
      ghCheck();                                 // блокирующий шаг: экран нарисован внутри
      break;
    case G_ASK: {
      int b = nb;
      if (b > 0)       ghDownload();
      else if (b < 0)  ghStop("declined");
      else if ((now - ghT0) > 120000UL) ghStop("timeout");
      break;
    }
    case G_DL:
    case G_OK:
      break;
    case G_TEST:
      if ((now - ghT0) > 30000UL) ghStop("timeout");
      break;
    case G_ERR:
      if ((now - ghT0) > 30000UL) ghStop("timeout");
      break;
  }
}

// Wi-Fi приёмника: с радио пульт отправит команду, приёмник ответит суффиксом имени сети.
// Пока радио нет, ответ имитируется.
void rxWifiBegin(int mode) {
  rxWifiMode = mode;
  rxWifiSt = 1;
  rxWifiAt = millis();
  rxWifiSsid[0] = 0;
  if (simMode == SIM_RADIO) {                  // настоящая команда: приёмник сам открывает Wi-Fi, состояние берём из телеметрии
    hlRxVer[0] = 0; hlWifiAck = false; hlWifiAckSt = 0;
    if (mode == 1) {
      for (int i = 0; i < NET_MAX; i++) if (netSsid[i][0]) hlCfgPut(HLO_NET, 0, (uint8_t)i, 0, 0, false, 0, 0);
      hlCfgPut(HLO_CMD, HLC_WIFI_GH, 0, 0, 0, false, 0, 0);
    } else {
      hlCfgPut(HLO_CMD, HLC_WIFI_PHONE, 0, 0, 0, false, 0, 0);
    }
    gotoScr(MS_RXWIFI, 0);
    logLine(mode ? "RX GitHub update: requested" : "RX Wi-Fi (Update/Test): requested");
    return;
  }
  gotoScr(MS_RXWIFI, 0);
  logLine(mode ? "RX GitHub update: requested (simulated)" : "RX Wi-Fi Update: requested (simulated)");
}

// Phone (Update/Test), имитация статусов из телеметрии приёмника (с радио придут от него):
// 2 ждём телефон, 3 телефон подключён (вкладка Update), 4 Test active (вкладка Test подтверждена), 5 Uploading n %, 6 Installed.
// Пока Test active, пульт управление не отправляет (нули), E-Stop закрывает Wi-Fi.
int rxUpPct = 0;
void rxWifiTick(unsigned long now) {
  if (simMode == SIM_RADIO) {
    if (rxWifiSt == 1) {
      if (hlWifiAck && hlWifiAckSt != 0) {
        toast(hlWifiAckSt == 255 ? "RX no answer" : "RX refused");
        rxWifiSt = 0; gotoScr(MS_RXWIFIM, idxRxWifi);
      } else if (telValid && tel.wifi != 0) {
        rxWifiSt = tel.wifi; rxWifiAt = now;
      } else if ((now - rxWifiAt) >= 5000UL) {
        toast("RX no answer");
        rxWifiSt = 0; gotoScr(MS_RXWIFIM, idxRxWifi);
      }
    } else if (rxWifiSt >= 2) {
      if (telValid && tel.wifi != 0) rxWifiSt = tel.wifi;
      else if (telValid && tel.wifi == 0) {            // приёмник закрыл Wi-Fi (таймаут, ошибка, E-Stop, перезагрузка)
        rxWifiSt = 0; toast("RX Wi-Fi closed");
        gotoScr(MS_RXWIFIM, idxRxWifi);
      }
    }
    return;
  }
  if (rxWifiSt == 1 && (now - rxWifiAt) >= 800) {
    rxWifiSt = 2; rxWifiAt = now;
    snprintf(rxWifiSsid, sizeof(rxWifiSsid), "RX-SIM1");
  } else if (rxWifiMode == 1 && rxWifiSt >= 2 && rxWifiSt < 5 && (now - rxWifiAt) >= 1200) {
    rxWifiSt++; rxWifiAt = now;                  // имитация этапов: сети -> подключение -> проверка GitHub -> конец
  } else if (rxWifiMode == 0 && rxWifiSt >= 2 && rxWifiSt <= 3 && (now - rxWifiAt) >= 4000) {
    rxWifiSt++; rxWifiAt = now; rxUpPct = 0;     // имитация: ждём телефон -> подключён -> Test active
  } else if (rxWifiMode == 0 && rxWifiSt == 4 && (now - rxWifiAt) >= 10000) {
    rxWifiSt = 5; rxWifiAt = now; rxUpPct = 0;   // имитация: загрузка прошивки
  } else if (rxWifiMode == 0 && rxWifiSt == 5) {
    rxUpPct = (int)((now - rxWifiAt) / 40UL);
    if (rxUpPct >= 100) { rxUpPct = 100; rxWifiSt = 6; rxWifiAt = now; }
  } else if (rxWifiSt >= 2 && (now - rxWifiAt) >= (rxWifiMode ? 20000UL : (rxWifiSt == 6 ? 5000UL : WIFI_NOCONN_MS))) {
    rxWifiSt = 0;
    toast("RX Wi-Fi closed");
    gotoScr(MS_RXWIFIM, idxRxWifi);
  }
}

// ============ СВЕТОДИОДЫ И СОБЫТИЯ ============
int lastA = -1, lastB = -1;

void setLedBit(uint8_t pin, uint8_t &a, uint8_t &b) {
  if (pin < 8) a |= (1 << pin);
  else         b |= (1 << (pin - 8));
}

uint8_t colorMask(uint8_t c) { return c == COL_RED ? 1 : (c == COL_GREEN ? 2 : 3); }

bool patOwns(uint8_t pat) {   // непрерывные ритмы занимают светодиод целиком, вспышки лишь накладываются
  return pat == PAT_STEADY || pat == PAT_SLOW || pat == PAT_FAST || pat == PAT_ALT;
}

uint8_t patMask(const EvCfg &c, unsigned long t) {
  if (c.pat == PAT_ALT) return ((t % 1000) < 500) ? 1 : 3;
  bool on;
  switch (c.pat) {
    case PAT_SLOW:   on = (t % 1000) < 500; break;
    case PAT_FAST:   on = (t % 400) < 200; break;
    case PAT_PULSE:  on = (t % 2000) < 100; break;
    case PAT_DOUBLE: { unsigned long p = t % 2000; on = (p < 100) || (p >= 250 && p < 350); break; }
    default:         on = true; break;
  }
  return on ? colorMask(c.color) : 0;
}

// Маска события в момент t. Для «связь восстановлена» t = время с момента события.
uint8_t evMask(int e, unsigned long t) {
  EvCfg c = cfgOf(e);
  if (e == EV_HEART) {
    unsigned long period = HB_SEC[c.ival] * 1000UL;
    if ((t % period) >= HB_FLASH_MS) return 0;
    // зелёная вспышка на уже зелёном светодиоде не видна: тогда делаем короткое гашение
    if (c.color == COL_GREEN && ledGreenOn) return MASK_DARK;
    return colorMask(c.color);
  }
  return patMask(c, t);
}

bool evAllAct[NUM_EV];
uint8_t evChAct[NUM_EV];

// Какие события активны сейчас. Источники от приёмника появятся вместе с радио.
void collectEvents(unsigned long now) {
  for (int e = 0; e < NUM_EV; e++) { evAllAct[e] = false; evChAct[e] = 0; }
  evAllAct[EV_ESTOP]    = (state == ST_ESTOP);
  evAllAct[EV_NOLINK]   = (simMode == SIM_LOST) || (simMode == SIM_RADIO && !hlUp && (now - hlBootAt) > 4000UL);
  evAllAct[EV_RESTORED] = (restoredAt != 0 && (now - restoredAt) < RESTORED_MS);
  evAllAct[EV_HEART]    = (simMode == SIM_LINK) || (simMode == SIM_RADIO && hlUp);
  evAllAct[EV_BATTTX]   = (batLevel >= 1);
  if (state == ST_SAFE) {
    for (int i = 0; i < NUM_CH; i++) if (chBad(i)) evChAct[EV_NOTCTR] |= (1 << i);
  }
  if (telValid) {
    evChAct[EV_OVERLOAD] |= tel.mOvl;
    evChAct[EV_OPEN]     |= tel.mOpen;
    evChAct[EV_LIMIT]    |= tel.mLim;
  }
  if (telValid) {
    evAllAct[EV_RXALARM] = tel.kill && tel.reason == 1;
    evAllAct[EV_RXFAULT] = tel.kill && tel.reason == 2;
    evAllAct[EV_BATTRX]  = tel.vrx != 0 && tel.vrx < HL_RXBATT_LOW;
  }
}

// ============ ЭКРАН ТРЕВОГ (только рабочий режим) ============
// Событие с включёнными Enable и Screen заменяет экран крупной надписью, пока оно активно (не менее
// ALERT_MIN_MS). Несколько событий показываются по очереди, порядок: от самого серьёзного.
#define AL_SLOTS 9                       // 0..7: канал, 8: событие «на всё»
#define AL_ALL   8
static const uint8_t AL_ORDER[] = { EV_RXALARM, EV_NOLINK, EV_RXFAULT, EV_OVERLOAD, EV_OPEN, EV_LIMIT, EV_BATTRX, EV_BATTTX, EV_RESTORED };
#define AL_N (int)(sizeof(AL_ORDER) / sizeof(AL_ORDER[0]))
unsigned long alHold[NUM_EV][AL_SLOTS];
bool alPrev[NUM_EV][AL_SLOTS];
int  alShown = -1;                       // e * 16 + слот, -1: тревоги нет
int  alIdx = 0, alCnt = 0;               // номер показанной и сколько всего
unsigned long alShownAt = 0;

bool alPerChannel(int e) { return e == EV_OVERLOAD || e == EV_OPEN || e == EV_LIMIT; }

void alertTick(unsigned long now) {
  if (state != ST_RUN) { alShown = -1; alCnt = 0; return; }
  collectEvents(now);
  uint8_t list[AL_N * AL_SLOTS];
  int n = 0;
  for (int k = 0; k < AL_N; k++) {
    int e = AL_ORDER[k];
    EvCfg c = cfgOf(e);
    bool on = c.en && c.scr;
    for (int sl = 0; sl < AL_SLOTS; sl++) {
      bool act;
      if (alPerChannel(e)) act = (sl < NUM_CH) && ((evChAct[e] >> sl) & 1);
      else                 act = (sl == AL_ALL) && evAllAct[e];
      if (act && !alPrev[e][sl]) alHold[e][sl] = now + ALERT_MIN_MS;
      alPrev[e][sl] = act;
      if (on && (act || (long)(alHold[e][sl] - now) > 0)) list[n++] = (uint8_t)(e * 16 + sl);
    }
  }
  alCnt = n;
  if (n == 0) { alShown = -1; return; }
  int idx = -1;
  for (int i = 0; i < n; i++) if (list[i] == alShown) idx = i;
  if (idx < 0) {                          // первое показание или текущее ушло
    alShown = list[0]; alShownAt = now; alIdx = 0;
  } else if (n > 1 && (now - alShownAt) >= ALERT_SLOT_MS) {
    idx = (idx + 1) % n;
    alShown = list[idx]; alShownAt = now; alIdx = idx;
  } else {
    alIdx = idx;
  }
}

void applyMask(uint8_t* m, int i, uint8_t mask, bool owns) {
  if (mask == MASK_DARK) { m[i] = 0; return; }
  if (owns) m[i] = mask;
  else if (mask) m[i] = mask;
}

// Слои (снизу вверх): обычный статус -> события от низкого приоритета к высокому.
// Правила: E-Stop и системные события (0..3) идут на все 8 светодиодов, остальные только
// на включённые каналы. Глобальные выключатели цвета и режим LED канала действуют на обычный статус.
void updateLeds() {
  if (!okMcp) return;
  unsigned long now = millis();
  uint8_t m[NUM_CH];

  int previewEv = -1;
  if (state == ST_MENU && menuScr == MS_EVEDIT) previewEv = evSel;
  else if (state == ST_MENU && menuScr == MS_TEST) previewEv = testEv;

  if (previewEv >= 0) {
    // предпросмотр: событие показывается на всех светодиодах независимо от Enable
    unsigned long t = (previewEv == EV_RESTORED) ? (now % 3000) : now;
    uint8_t mk = evMask(previewEv, t);
    for (int i = 0; i < NUM_CH; i++) m[i] = (mk == MASK_DARK) ? 0 : mk;
  } else {
    collectEvents(now);
    for (int i = 0; i < NUM_CH; i++) {
      m[i] = 0;
      if (!chEn[i]) continue;
      uint8_t md = chLed[i];
      if (md == LED_OFF) continue;
      if (isMoved(i)) {
        if (ledRedOn && (md == LED_BOTH || md == LED_RED)) m[i] = 1;
      } else {
        if (ledGreenOn && (md == LED_BOTH || md == LED_GREEN)) m[i] = 2;
      }
    }
    for (int e = NUM_EV - 1; e >= 0; e--) {
      EvCfg c = cfgOf(e);
      if (!c.en) continue;
      bool owns = (e == EV_HEART) ? false : patOwns(c.pat);
      bool isAllEvent = (e < EV_OVERLOAD) || (e > EV_NOTCTR);
      if (isAllEvent) {
        if (!evAllAct[e]) continue;
        unsigned long t = (e == EV_RESTORED) ? (now - restoredAt) : now;
        uint8_t mk = evMask(e, t);
        bool incDisabled = (e <= EV_RXFAULT);
        for (int i = 0; i < NUM_CH; i++) {
          if (!chEn[i] && !incDisabled) continue;
          applyMask(m, i, mk, owns);
        }
      } else {
        uint8_t act = evChAct[e];
        if (!act) continue;
        uint8_t mk = evMask(e, now);
        for (int i = 0; i < NUM_CH; i++) {
          if (!chEn[i]) continue;
          bool hit = c.whereAll ? true : ((act >> i) & 1);
          if (hit) applyMask(m, i, mk, owns);
        }
      }
    }
  }

  if (gameFlashUntil && (long)(gameFlashUntil - now) > 0) {      // проигрыш в игре: все красные один раз
    for (int i = 0; i < NUM_CH; i++) m[i] = 1;
  } else if (gameFlashUntil) {
    gameFlashUntil = 0;
  }
  uint8_t a = 0, b = 0;
  for (int i = 0; i < NUM_CH; i++) {
    if (m[i] & 1) setLedBit(CH[i].redPin, a, b);
    if (m[i] & 2) setLedBit(CH[i].greenPin, a, b);
  }
  if ((int)a != lastA || (int)b != lastB) {
    mcp.writeGPIOA(a);
    mcp.writeGPIOB(b);
    lastA = a; lastB = b;
  }
}

// ============ УПРАВЛЕНИЕ (то, что пойдёт в Control-пакет) ============
// Рабочий режим: значения включённых каналов. Любое меню, калибровка, Safe Start, E-Stop: нули.
// Исключение: экран тестирования Receiver, там тестовый джойстик работает на выбранную секцию.
void buildControl() {
  for (int i = 0; i < NUM_CH; i++) ctl[i] = 0;
  if (state == ST_RUN) {
    for (int i = 0; i < NUM_CH; i++) ctl[i] = chEn[i] ? val(i) : 0;
  } else if (state == ST_MENU && menuScr == MS_RXEDIT && rxLinked() &&
             rxLeverParam(rxParam)) {
    if (!leverArmed && !isMoved(TEST_LEVER_CH)) leverArmed = true;
    if (leverArmed) ctl[rxCh] = val(TEST_LEVER_CH);
  }
}

// ============ ЭКРАНЫ ============
void drawTick(int cx, int cy, int r) {
  display.drawCircle(cx, cy, r, SH110X_WHITE);
  display.drawCircle(cx, cy, r - 1, SH110X_WHITE);
  display.drawLine(cx - 5, cy, cx - 2, cy + 4, SH110X_WHITE);
  display.drawLine(cx - 5, cy + 1, cx - 2, cy + 5, SH110X_WHITE);
  display.drawLine(cx - 2, cy + 4, cx + 5, cy - 4, SH110X_WHITE);
  display.drawLine(cx - 2, cy + 5, cx + 5, cy - 3, SH110X_WHITE);
}

void drawCross(int cx, int cy, int r) {
  display.drawCircle(cx, cy, r, SH110X_WHITE);
  display.drawCircle(cx, cy, r - 1, SH110X_WHITE);
  int d = r / 2 + 1;
  display.drawLine(cx - d, cy - d, cx + d, cy + d, SH110X_WHITE);
  display.drawLine(cx - d + 1, cy - d, cx + d + 1, cy + d, SH110X_WHITE);
  display.drawLine(cx - d, cy + d, cx + d, cy - d, SH110X_WHITE);
  display.drawLine(cx - d + 1, cy + d, cx + d + 1, cy - d, SH110X_WHITE);
}

void startScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
}

// ---- постраничный вывод на OLED (SH1106) ----
// Кадр рисуется в буфер библиотеки, затем копируется и уходит в экран по одной странице (8 строк)
// за проход цикла. Цикл не замирает, шина остаётся на частоте I2C_CLOCK.
static uint8_t oledFrame[SCREEN_W * SCREEN_H / 8];
static int8_t  oledPage = -1;              // -1: передача не идёт
static unsigned long oledT0 = 0, oledFlushUs = 0;
#define OLED_COL_OFFSET 2                  // SH1106: 132 колонки в памяти, видимые начинаются со второй
#define OLED_CHUNK      64                 // байт данных в одной передаче

void oledCommit() {
  memcpy(oledFrame, display.getBuffer(), sizeof(oledFrame));
  oledPage = 0;
  oledT0 = micros();
}
bool oledBusy() { return oledPage >= 0; }

void oledStep() {
  if (oledPage < 0) return;
  uint8_t p = (uint8_t)oledPage;
  Wire.beginTransmission(OLED_ADDR);
  Wire.write((uint8_t)0x00);                                   // команды
  Wire.write((uint8_t)(0xB0 + p));                             // страница
  Wire.write((uint8_t)(OLED_COL_OFFSET & 0x0F));               // колонка, младшая часть
  Wire.write((uint8_t)(0x10 | (OLED_COL_OFFSET >> 4)));        // колонка, старшая часть
  Wire.endTransmission();
  const uint8_t* src = oledFrame + (uint16_t)p * SCREEN_W;
  for (int c = 0; c < SCREEN_W; c += OLED_CHUNK) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write((uint8_t)0x40);                                 // данные
    Wire.write(src + c, (size_t)OLED_CHUNK);
    Wire.endTransmission();
  }
  if (++oledPage >= SCREEN_H / 8) {
    oledPage = -1;
    oledFlushUs = micros() - oledT0;
  }
}

// ---- заставка FORESTER ----
#define LOGO_W 124
#define LOGO_H 27
static const uint8_t LOGO_BMP[] PROGMEM = {
  0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00,
  0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00,
  0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
  0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
  0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
  0x40, 0x0F, 0xFC, 0x3F, 0x83, 0xFF, 0x0F, 0xFC, 0x7F, 0x0F, 0xFC, 0xFF, 0xCF, 0xFC, 0x00, 0x20,
  0x40, 0x0F, 0xFC, 0x7F, 0xC3, 0xFF, 0x8F, 0xF8, 0xFF, 0x8F, 0xFC, 0xFF, 0xCF, 0xFE, 0x00, 0x20,
  0x40, 0x0E, 0x00, 0xF0, 0xE3, 0x83, 0x8C, 0x00, 0xE1, 0xC0, 0xC0, 0xE0, 0x0E, 0x0E, 0x00, 0x20,
  0x80, 0x0E, 0x00, 0xE0, 0x73, 0x81, 0x8C, 0x00, 0xC0, 0xC0, 0xC0, 0xE0, 0x0E, 0x06, 0x00, 0x10,
  0x80, 0x0E, 0x00, 0xC0, 0x73, 0x83, 0x8C, 0x00, 0xE0, 0x00, 0xC0, 0xE0, 0x0E, 0x0E, 0x00, 0x10,
  0x80, 0x0F, 0x01, 0xC0, 0x73, 0x87, 0x8F, 0xF8, 0xFE, 0x00, 0xC0, 0xFF, 0x8E, 0x1E, 0x00, 0x10,
  0x80, 0x0F, 0xF9, 0xC0, 0x73, 0xFF, 0x0F, 0xF8, 0x3F, 0x80, 0xC0, 0xFF, 0x8F, 0xFC, 0x00, 0x10,
  0x80, 0x0F, 0xF9, 0xC0, 0x73, 0xFE, 0x0E, 0x00, 0x07, 0xC0, 0xC0, 0xE0, 0x0F, 0xF8, 0x00, 0x10,
  0x80, 0x0E, 0x00, 0xC0, 0x73, 0x87, 0x0C, 0x00, 0x01, 0xC0, 0xC0, 0xC0, 0x0E, 0x1C, 0x00, 0x10,
  0x80, 0x0E, 0x00, 0xE0, 0x73, 0x83, 0x0C, 0x01, 0xC0, 0xC0, 0xC0, 0xE0, 0x0E, 0x0C, 0x00, 0x10,
  0x40, 0x0E, 0x00, 0xF0, 0xE3, 0x83, 0x8C, 0x00, 0xE1, 0xC0, 0xC0, 0xE0, 0x0E, 0x0E, 0x00, 0x20,
  0x40, 0x0E, 0x00, 0x7F, 0xC3, 0x83, 0x8F, 0xFC, 0xFF, 0xC0, 0xC0, 0xFF, 0xCE, 0x0E, 0x00, 0x20,
  0x40, 0x0E, 0x00, 0x3F, 0x83, 0x81, 0xCF, 0xFC, 0x7F, 0x80, 0xC0, 0xFF, 0xCE, 0x07, 0x00, 0x20,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
  0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
  0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
  0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
  0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00,
  0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00,
};

void drawSplash() {
  startScreen();
  display.drawBitmap((SCREEN_W - LOGO_W) / 2, 10, LOGO_BMP, LOGO_W, LOGO_H, SH110X_WHITE);
  display.setCursor((SCREEN_W - 18 * 6) / 2, 44);
  display.print("FORESTRY EQUIPMENT");
  oledCommit();
}

void drawBoot() {
  startScreen();
  display.setCursor(0, 0);   display.print("BOOT CHECK");
  display.setCursor(0, 14);  display.printf("MCP23017: %s", okMcp ? "ok" : "FAIL");
  display.setCursor(0, 24);  display.printf("ADS1 0x48: %s", adsOk[0] ? "ok" : "FAIL");
  display.setCursor(0, 34);  display.printf("ADS2 0x49: %s", adsOk[1] ? "ok" : "FAIL");
  display.setCursor(0, 44);  display.print("Config loaded");
  drawBattery();
  oledCommit();
}

void drawSafe() {
  startScreen();
  display.setCursor(0, 0);   display.print("SAFE START");
  display.setCursor(0, 10);  display.print("Center all controls");

  bool any = false;
  bool adsFail = false;
  display.setCursor(0, 24);
  display.print("Not centered:");
  display.setCursor(0, 34);
  for (int i = 0; i < NUM_CH; i++) {
    if (chBad(i)) {
      display.printf("%d ", i + 1);
      any = true;
      if (!adsOk[CH[i].ads]) adsFail = true;
    }
  }
  if (!any) display.print("none, hold still");
  if (adsFail) { display.setCursor(0, 44); display.print("ADS error (check I2C)"); }

  drawHintText("hold MENU = menu", 56);
  drawBattery();
  oledCommit();
}

void drawRun1() {
  startScreen();
  display.setTextSize(2);
  display.setCursor(2, 6);   display.print("LINK");
  if (rxLinked()) drawTick(92, 14, 9); else drawCross(92, 14, 9);

  display.drawLine(0, 28, 108, 28, SH110X_WHITE);

  display.setCursor(2, 34);  display.print("RX");
  display.setTextSize(1);
  display.setCursor(32, 38);
  if (simMode == SIM_RADIO) {
    if (!hlInit)             display.print("RADIO ERROR");
    else if (!hlUp)          display.print("NO SIGNAL");
    else if (hlSyncBusy)     display.printf("SYNC %d%%", hlSyncPct());
    else if (tel.kill)       display.print("STOPPED");
    else                     display.print("READY");
  }
  else if (simMode == SIM_LINK) display.print("SIM: READY");
  else                          display.print("NO SIGNAL");
  if (rxLinked() && !hlSyncBusy && !(simMode == SIM_RADIO && tel.kill)) drawTick(92, 42, 9); else drawCross(92, 42, 9);

  drawHintText("hold MENU = menu", 56);
  drawBattery();
  oledCommit();
}

void printChan(int i) {
  if (!chEn[i])               display.printf("%d: off", i + 1);
  else if (!adsOk[CH[i].ads]) display.printf("%d: ERR", i + 1);
  else                        display.printf("%d:%4d", i + 1, val(i));
}

void drawRun2() {
  startScreen();
  display.setCursor(0, 0);
  display.print("TX CHANNELS");
  for (int r = 0; r < 4; r++) {
    display.setCursor(0, 11 + r * 10);
    printChan(r);
    display.setCursor(60, 11 + r * 10);
    printChan(r + 4);
  }
  drawHintText("hold MENU = menu", 56);
  drawBattery();
  oledCommit();
}

// RX CHANNELS: по 9 символов на канал: номер, сторона A/B, ток в импульсе (А), ШИМ со знаком, пометка ! L O
void drawRxChannels() {
  startScreen();
  display.setCursor(0, 0);
  display.print("RX CHANNELS Ipulse");
  for (int r = 0; r < 4; r++) {
    for (int col = 0; col < 2; col++) {
      int c = r + col * 4;
      char buf[16];
      if (!telValid) {
        snprintf(buf, sizeof(buf), "%d    --", c + 1);
      } else {
        int pwm = tel.pwm[c];
        bool ovl = (tel.mOvl >> c) & 1;
        char side = (pwm != 0 || ovl) ? (((tel.side >> c) & 1) ? 'B' : 'A') : ' ';
        char fl = ovl ? '!' : (((tel.mLim >> c) & 1) ? 'L' : (((tel.mOpen >> c) & 1) ? 'O' : ' '));
        int ci = tel.cur[c];
        if (ci > 99) ci = 99;
        snprintf(buf, sizeof(buf), "%d%c%d.%d%+3d%c", c + 1, side, ci / 10, ci % 10, pwm, fl);
      }
      display.setCursor(col * 60, 11 + r * 10);
      display.print(buf);
    }
  }
  drawHintText("!=Ovl L=Lim O=Open", 56);
  drawBattery();
  oledCommit();
}

void drawLinkScreen() {
  startScreen();
  display.setCursor(0, 0);   display.print("LINK");
  display.setCursor(0, 10);
  if (simMode == SIM_RADIO) {
    if (!hlInit)         display.print("Status: RADIO ERROR");
    else if (!hlUp)      display.print("Status: NO SIGNAL");
    else if (hlSyncBusy) display.printf("Sync: reading %d%%", hlSyncPct());
    else                 display.print("Status: OK");
  }
  else if (simMode == SIM_LINK) display.print("Status: OK");
  else                          display.print("Status: NO SIGNAL");
  display.setCursor(0, 19);
  if (simMode == SIM_RADIO && telValid) display.printf("RSSI%d/%d SNR%d/%d", rfRssi, (int)tel.rssi, (rfSnr10 + (rfSnr10 < 0 ? -5 : 5)) / 10, (int)tel.snr);
  else if (telValid) display.printf("RSSI%d SNR%.1f", rfRssi, rfSnr10 / 10.0f);
  else          display.print("RSSI-- SNR--");
  display.setCursor(0, 28);
  if (telValid) {
    uint32_t tot = rfRx + rfLost;
    display.printf("Lost %lu (%.1f%%)", (unsigned long)rfLost, tot ? rfLost * 100.0f / tot : 0.0f);
  } else display.print("Lost --");
  display.setCursor(0, 37);
  if (telValid && simMode == SIM_RADIO) display.printf("%d/s Age%.1f RTT%d", rfRate, rfAgeMs / 1000.0f, hlRtt);
  else if (telValid) display.printf("%d/s Age %.1fs", rfRate, rfAgeMs / 1000.0f);
  else          display.print("-- /s  Age --");
  display.setCursor(0, 46);
  if (telValid) display.printf("ID %04X TX%d RX%d", rxId, txPower, (int)rxQuery(0, 0, RXP_PWR));
  else          display.printf("TX%ddBm", txPower);
  drawHintText("MENU = next screen", 56);
  drawBattery();
  oledCommit();
}

void drawPowerScreen() {
  static const char* reasons[4] = { "pult button", "RX button", "RX fault", "link lost" };
  startScreen();
  display.setCursor(0, 0);   display.print("POWER");
  display.setCursor(0, 11);
  if (batUsb) display.print("Pult: USB");
  else        display.printf("Pult %.1fV %d%%", vbat, vbatPct);
  display.setCursor(0, 21);
  if (telValid) display.printf("RX   %.1fV", tel.vrx / 10.0f);
  else          display.print("RX   --");
  display.setCursor(0, 31);
  if (telValid) {
    int sum = 0;
    for (int c = 0; c < NUM_CH; c++) sum += tel.cur[c];
    display.printf("Ipulse sum %d.%dA", sum / 10, sum % 10);
  } else display.print("Ipulse sum --");
  display.setCursor(0, 41);
  if (!telValid)       display.print("Kill: --");
  else if (!tel.kill)  display.print("Kill: no");
  else                 display.printf("Kill: %s", reasons[tel.reason & 3]);
  drawHintText("MENU = next screen", 56);
  drawBattery();
  oledCommit();
}

void drawLogScreen() {
  startScreen();
  display.setCursor(0, 0);   display.print("EVENT LOG");
  char t[28];
  if (logN == 0) {
    display.setCursor(0, 12);  display.print("(no events)");
  }
  for (int r = 0; r < LOG_SHOW && r < logN; r++) {
    logText(logRam[r], t, sizeof(t));
    display.setCursor(0, 11 + r * 9);
    display.print(t);
  }
  drawHintText("MENU = next screen", 56);
  drawBattery();
  oledCommit();
}

void drawEstop() {
  startScreen();
  display.setTextSize(2);
  display.setCursor(0, 4);   display.print("EMERGENCY");
  display.setCursor(0, 22);  display.print("STOP");
  display.setTextSize(1);
  display.setCursor(0, 46);  display.print("Release button");
  display.setCursor(0, 56);  display.print("then center all");
  drawBattery();
  oledCommit();
}

void drawAlert() {
  int e = alShown >> 4, sl = alShown & 15;
  const char* title = "";
  const char* hint = "";
  char l1[28] = "", l2[28] = "";
  char side = ((tel.side >> (sl & 7)) & 1) ? 'B' : 'A';
  int cur = (sl < NUM_CH) ? tel.cur[sl] : 0;
  switch (e) {
    case EV_RXALARM:  title = "RX ALARM";  snprintf(l1, sizeof(l1), "Alarm button on RX"); snprintf(l2, sizeof(l2), "Outputs stopped"); break;
    case EV_NOLINK:   title = "NO LINK";   snprintf(l1, sizeof(l1), "No signal from RX"); snprintf(l2, sizeof(l2), "Controls locked"); break;
    case EV_RXFAULT:  title = "RX FAULT";  snprintf(l1, sizeof(l1), "Receiver fault"); snprintf(l2, sizeof(l2), "Check receiver"); break;
    case EV_OVERLOAD: title = "OVERLOAD";  snprintf(l1, sizeof(l1), "CH%d %c pulse %d.%dA", sl + 1, side, cur / 10, cur % 10); hint = "Center lever to reset"; break;
    case EV_OPEN:     title = "OPEN COIL"; snprintf(l1, sizeof(l1), "CH%d %c", sl + 1, side); hint = "Check solenoid wiring"; break;
    case EV_LIMIT:    title = "CUR LIMIT"; snprintf(l1, sizeof(l1), "CH%d %c pulse %d.%dA", sl + 1, side, cur / 10, cur % 10); hint = "Current is limited"; break;
    case EV_RESTORED: title = "LINK BACK"; snprintf(l1, sizeof(l1), "Link restored"); break;
    case EV_BATTTX:   title = "TX BATT";   snprintf(l1, sizeof(l1), "Pult battery %d%%", vbatPct); hint = batLevel >= 2 ? "Critical, charge now" : "Charge soon"; break;
    case EV_BATTRX:   title = "RX BATT";   snprintf(l1, sizeof(l1), "Receiver supply low"); break;
    default: break;
  }
  startScreen();
  display.fillRect(0, 0, SCREEN_W, 18, SH110X_WHITE);
  display.setTextColor(SH110X_BLACK);
  display.setTextSize(2);
  display.setCursor(2, 2);   display.print(title);
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 24);  display.print(l1);
  display.setCursor(0, 36);  display.print(l2);
  if (alCnt > 1) { display.setCursor(104, 36); display.printf("%d/%d", alIdx + 1, alCnt); }
  display.setCursor(0, 54);  display.print(hint);
  oledCommit();
}

static void exitHint(const char* what, int y) {     // "MENU or 6 = exit": выход кнопкой меню или рычагом CH6
  char h[24];
  snprintf(h, sizeof(h), "MENU or %d = %s", MENU_ENTER_CH + 1, what);
  drawHintText(h, y);
}

void drawWifi() {
  startScreen();
  display.setCursor(0, 0);   display.print("WI-FI UPDATE");
  unsigned long now = millis();
  if (upActive) {
    int pct = upTotal ? (int)((upDone * 100UL) / upTotal) : 0;
    if (pct > 99) pct = 99;
    display.setCursor(0, 14);  display.print("Receiving firmware");
    display.drawRect(0, 28, PBAR_W, 12, SH110X_WHITE);   // уже значка батареи (x от 116)
    display.fillRect(2, 30, (PBAR_W - 4) * pct / 100, 8, SH110X_WHITE);
    display.setCursor(0, 46);  display.printf("%d%%", pct);
    display.setCursor(0, 56);  display.print("do not switch off");
  } else if (upOk) {
    display.setCursor(0, 14);  display.print("Installed");
    display.setCursor(0, 26);  display.printf("version %s", upVer);
    display.setCursor(0, 42);  display.print("Restarting...");
  } else if (upErr[0]) {
    display.setCursor(0, 14);  display.print("Update failed");
    display.setCursor(0, 26);  display.print(upErr);
    exitHint("exit", 56);
  } else if (!wifiPage) {
    display.setCursor(0, 12);  display.printf("Net:  %s", wifiSsid);
    display.setCursor(0, 22);  display.print("Pass: " WIFI_PASS);
    display.setCursor(0, 32);  display.print("Open: " WIFI_ADDR);
    display.setCursor(0, 44);  display.print(wifiEver ? "Phone connected" : "Waiting for phone");
    long left = (long)((wifiEver ? WIFI_IDLE_MS : WIFI_NOCONN_MS) - (now - (wifiEver ? wifiAct : wifiT0)) ) / 1000;
    if (left < 0) left = 0;
    display.setCursor(0, 56);  display.printf("MENU/%d=exit %ld:%02ld", MENU_ENTER_CH + 1, left / 60, left % 60);
  } else {
    long left = (long)(WIFI_IDLE_MS - (now - wifiAct)) / 1000;
    if (left < 0) left = 0;
    display.setCursor(0, 14);  display.print("Page open");
    display.setCursor(0, 26);  display.print("Choose the file on the phone");
    display.setCursor(0, 44);  display.printf("idle exit %ld:%02ld", left / 60, left % 60);
    exitHint("exit", 56);
  }
  drawBattery();
  oledCommit();
}

static const char* ghErrText(int c) {
  switch (c) {
    case 1: return "No network in range";
    case 2: return "Wrong password";
    case 3: return "GitHub not reachable";
    case 4: return "Release not found";
    case 5: return "Download failed";
    case 6: return "Checksum error";
    case 7: return "Wrong firmware";
    case 8: return "Update error";
    case 9: return "GitHub busy";
    default: return "Receiver error";
  }
}

// Wi-Fi приёмника по радио: состояние приходит в телеметрии (tel.wifi / tel.wifiArg), имя сети и версия в пакетах INFO
void drawRxWifiRadio() {
  int w = rxWifiSt;
  int pct = constrain((int)tel.wifiArg, 0, 100);
  if (w == 1) {
    display.setCursor(0, 20);  display.print("Asking receiver...");
  } else if (w == HLW_PH_WAIT) {
    display.setCursor(0, 12);  display.printf("Net:  %s", rxWifiSsid);
    display.setCursor(0, 22);  display.print("Pass: " WIFI_PASS);
    display.setCursor(0, 32);  display.print("Open: " WIFI_ADDR);
    display.setCursor(0, 42);  display.print("Tabs: Update, Test");
  } else if (w == HLW_PH_CONN) {
    display.setCursor(0, 14);  display.print("Phone connected");
    display.setCursor(0, 26);  display.print("Tab: Update");
    display.setCursor(0, 40);  display.print("Receiver outputs off");
  } else if (w == HLW_PH_TEST) {
    display.setCursor(0, 12);  display.print("TEST ACTIVE");
    display.setCursor(0, 24);  display.print("RX outputs LIVE");
    display.setCursor(0, 34);  display.print("E-Stop works");
    display.setCursor(0, 44);  display.print("Pult sends no control");
  } else if (w == HLW_PH_UP || w == HLW_GH_DL || w == HLW_GH_VERIFY) {
    display.setCursor(0, 14);
    if (w == HLW_GH_VERIFY) display.print("Verifying...");
    else if (w == HLW_GH_DL) display.printf("Downloading %d%%", pct);
    else display.printf("Uploading %d%%", pct);
    display.drawRect(0, 26, PBAR_W, 12, SH110X_WHITE);
    display.fillRect(2, 28, (PBAR_W - 4) * pct / 100, 8, SH110X_WHITE);
    display.setCursor(0, 44);  display.print("Do not switch off");
  } else if (w == HLW_PH_DONE || w == HLW_GH_DONE) {
    display.setCursor(0, 14);  display.print("Installed");
    display.setCursor(0, 26);  display.print("Receiver restarts");
  } else if (w == HLW_GH_CONN) {
    display.setCursor(0, 16);  display.printf("%d saved networks", netCount());
    display.setCursor(0, 28);  display.print("Receiver connecting");
    display.setCursor(0, 40);  display.print("Receiver outputs off");
  } else if (w == HLW_GH_CHECK) {
    display.setCursor(0, 28);  display.print("Receiver checks GitHub");
    display.setCursor(0, 40);  display.print("Receiver outputs off");
  } else if (w == HLW_GH_ASK) {
    display.setCursor(0, 12);  display.print("Found on GitHub:");
    display.setCursor(0, 24);  display.printf("%.21s", hlRxVer);
    display.setCursor(0, 38);  display.print("6 + = install");
    display.setCursor(0, 47);  display.print("6 - / MENU = cancel");
  } else if (w == HLW_ERR) {
    display.setCursor(0, 14);  display.print("Receiver Wi-Fi error");
    display.setCursor(0, 26);  display.printf("%.21s", ghErrText(tel.wifiArg));
    display.setCursor(0, 38);  display.printf("code %d", (int)tel.wifiArg);
  } else {
    display.setCursor(0, 20);  display.printf("State %d", w);
  }
}

void drawRxWifi() {
  startScreen();
  display.setCursor(0, 0);   display.print(rxWifiMode ? "RECEIVER GITHUB" : "RECEIVER PHONE");
  if (simMode == SIM_RADIO) {
    drawRxWifiRadio();
    bool busy = (rxWifiSt == HLW_PH_UP || rxWifiSt == HLW_GH_DL || rxWifiSt == HLW_GH_VERIFY);
    if (busy) { drawBattery(); oledCommit(); return; }
    if (rxWifiSt != HLW_GH_ASK) exitHint("exit", 56);
    drawBattery();
    oledCommit();
    return;
  }
  if (rxWifiSt == 1) {
    display.setCursor(0, 20);  display.print("Asking receiver...");
  } else if (rxWifiMode == 1) {
    static const char* st[] = { "", "", "Sending networks", "Receiver connecting", "Receiver checks GitHub", "Simulation: no radio" };
    display.setCursor(0, 16);  display.printf("%d saved networks", netCount());
    display.setCursor(0, 28);  display.print(st[rxWifiSt < 6 ? rxWifiSt : 5]);
    display.setCursor(0, 40);  display.print(rxWifiSt >= 5 ? "works with radio 1.0" : "Receiver outputs off");
  } else if (rxWifiSt == 3) {
    display.setCursor(0, 14);  display.print("Phone connected");
    display.setCursor(0, 26);  display.print("Tab: Update");
    display.setCursor(0, 40);  display.print("Receiver outputs off");
  } else if (rxWifiSt == 4) {
    display.setCursor(0, 12);  display.print("TEST ACTIVE");
    display.setCursor(0, 24);  display.print("RX outputs LIVE");
    display.setCursor(0, 34);  display.print("E-Stop works");
    display.setCursor(0, 44);  display.print("Pult sends no control");
  } else if (rxWifiSt == 5) {
    display.setCursor(0, 14);  display.printf("Uploading %d%%", rxUpPct);
    display.drawRect(0, 26, PBAR_W, 12, SH110X_WHITE);
    display.fillRect(2, 28, (PBAR_W - 4) * rxUpPct / 100, 8, SH110X_WHITE);
    display.setCursor(0, 44);  display.print("Do not switch off");
  } else if (rxWifiSt == 6) {
    display.setCursor(0, 14);  display.print("Installed");
    display.setCursor(0, 26);  display.print("Receiver restarts");
    display.setCursor(0, 40);  display.print("Simulation: no radio");
  } else {
    display.setCursor(0, 12);  display.printf("Net:  %s", rxWifiSsid);
    display.setCursor(0, 22);  display.print("Pass: " WIFI_PASS);
    display.setCursor(0, 32);  display.print("Open: " WIFI_ADDR);
    display.setCursor(0, 42);  display.print("Tabs: Update, Test");
  }
  if (rxWifiSt == 5) { drawBattery(); oledCommit(); return; }     // во время загрузки выхода нет
  exitHint("exit", 56);
  drawBattery();
  oledCommit();
}

// ============ ИГРА DINO ============
// Запускается из главного меню (выходы приёмника в нуле). CH3: вверх прыжок, вниз присесть (знак меняется GAME_JUMP_SIGN).
// CH6: "+" старт и повтор, "-" выход в любой момент; MENU тоже выходит. Индикация: только один раз все красные при проигрыше.
#define GAME_JUMP_SIGN  (-1)
#define GAME_GROUND     58
#define GAME_G          1500.0f
#define GAME_JUMP_V     330.0f
enum GameSt { GS_READY, GS_PLAY, GS_OVER };
const int GOBS_N = 3;
GObs gObs[GOBS_N];
uint8_t  gameSt = GS_READY;
float    gSpeed = 80, gH = 0, gV = 0, gDist = 0, gNext = 0;
bool     gDuck = false;
uint16_t gScore = 0, gHi = 0;
unsigned long gLast = 0, gT0 = 0;
uint32_t gRnd = 12345;

bool gameOn() { return state == ST_MENU && menuScr == MS_GAME; }

static uint32_t gameRand(uint32_t n) { gRnd = gRnd * 1664525UL + 1013904223UL; return (gRnd >> 8) % n; }

void gameEnter() {
  prefs.begin("game", true);
  gHi = prefs.getUShort("hi", 0);
  prefs.end();
  gameSt = GS_READY;
  gLast = millis();
  gRnd ^= millis();
}

static void gameBegin(unsigned long now) {
  gameSt = GS_PLAY;
  gSpeed = 80; gH = 0; gV = 0; gDist = 0; gScore = 0; gDuck = false;
  gNext = 90;
  for (int i = 0; i < GOBS_N; i++) gObs[i].on = false;
  gLast = now; gT0 = now;
}

static void gameBox(const GObs& o, float& w, float& base, float& h) {
  switch (o.kind) {
    case 0:  w = 6;  base = 0;  h = 12; break;
    case 1:  w = 8;  base = 0;  h = 18; break;
    case 2:  w = 16; base = 0;  h = 12; break;
    case 3:  w = 14; base = 11; h = 8;  break;
    default: w = 14; base = 26; h = 8;  break;
  }
}

static bool gameHit() {
  float dw = gDuck ? 16 : 12, dh = gDuck ? 9 : 16;
  float dx0 = 12, dx1 = 10 + dw - 2, dy0 = gH, dy1 = gH + dh - 2;
  for (int i = 0; i < GOBS_N; i++) {
    if (!gObs[i].on) continue;
    float w, base, h; gameBox(gObs[i], w, base, h);
    float ox0 = gObs[i].x + 2, ox1 = gObs[i].x + w - 2, oy0 = base, oy1 = base + h - 2;
    if (ox0 < dx1 && ox1 > dx0 && oy0 < dy1 && oy1 > dy0) return true;
  }
  return false;
}

void gameStep(unsigned long now, int b) {
  if (b < 0) { menuBack(); return; }                         // CH6 "-" = выход в любой момент
  if (gameSt != GS_PLAY) {
    if (b > 0) gameBegin(now);
    gLast = now;
    return;
  }
  unsigned long el = now - gLast;
  if (el < 15) return;
  if (el > 100) el = 100;
  gLast = now;
  float dt = el / 1000.0f;
  int lev = valBase(MENU_NAV_CH) * GAME_JUMP_SIGN;           // > 0 вверх
  bool up = lev >= NAV_ON, down = lev <= -NAV_ON;
  bool air = gH > 0.01f || gV > 0.01f;
  if (up && !air) { gV = GAME_JUMP_V; air = true; }
  gDuck = down && !air;
  if (air) {
    gV -= GAME_G * (down ? 2.0f : 1.0f) * dt;
    gH += gV * dt;
    if (gH <= 0) { gH = 0; gV = 0; }
  }
  gSpeed = 80.0f + (now - gT0) / 1000.0f * 4.0f;
  if (gSpeed > 220) gSpeed = 220;
  gDist += gSpeed * dt;
  gScore = (uint16_t)(gDist / 8.0f);
  gNext -= gSpeed * dt;
  for (int i = 0; i < GOBS_N; i++) {
    if (!gObs[i].on) continue;
    gObs[i].x -= gSpeed * dt;
    if (gObs[i].x < -20) gObs[i].on = false;
  }
  if (gNext <= 0) {
    for (int i = 0; i < GOBS_N; i++) if (!gObs[i].on) {
      uint8_t k = gameRand(3);
      if (gScore >= 200 && gameRand(3) == 0) k = 3 + gameRand(2);
      gObs[i].kind = k; gObs[i].x = 128; gObs[i].on = true;
      break;
    }
    gNext = gSpeed * 0.5f + 30 + gameRand(60);
  }
  if (gameHit()) {
    gameSt = GS_OVER;
    gameFlashUntil = now + 300;
    if (gScore > gHi) {
      gHi = gScore;
      prefs.begin("game", false);
      prefs.putUShort("hi", gHi);
      prefs.end();
    }
    LOGF("Dino: game over, score %u", (unsigned)gScore);
  }
}

static void gDrawDino(int x, int yb, bool duck, bool step) {
  const uint16_t W = SH110X_WHITE, K = SH110X_BLACK;
  if (!duck) {
    int y = yb - 16;
    display.fillRect(x + 6, y, 8, 6, W);
    display.drawPixel(x + 11, y + 1, K);
    display.fillRect(x + 2, y + 6, 9, 6, W);
    display.fillRect(x, y + 7, 3, 3, W);
    display.fillRect(x + 10, y + 8, 3, 2, W);
    display.fillRect(x + 3, y + 12, 2, step ? 4 : 2, W);
    display.fillRect(x + 8, y + 12, 2, step ? 2 : 4, W);
  } else {
    int y = yb - 9;
    display.fillRect(x + 8, y, 9, 5, W);
    display.drawPixel(x + 13, y + 1, K);
    display.fillRect(x, y + 2, 10, 5, W);
    display.fillRect(x + 2, y + 7, 2, step ? 2 : 1, W);
    display.fillRect(x + 7, y + 7, 2, step ? 1 : 2, W);
  }
}

static void gDrawObs(const GObs& o, int yb, bool flap) {
  const uint16_t W = SH110X_WHITE;
  int x = (int)o.x;
  switch (o.kind) {
    case 0: display.fillRect(x + 2, yb - 12, 3, 12, W); display.fillRect(x, yb - 8, 2, 3, W); display.fillRect(x + 5, yb - 9, 2, 3, W); break;
    case 1: display.fillRect(x + 3, yb - 18, 3, 18, W); display.fillRect(x, yb - 12, 3, 3, W); display.fillRect(x + 6, yb - 13, 2, 3, W); break;
    case 2: display.fillRect(x + 2, yb - 12, 3, 12, W); display.fillRect(x + 10, yb - 10, 3, 10, W); display.fillRect(x, yb - 8, 2, 3, W); display.fillRect(x + 13, yb - 7, 2, 3, W); break;
    default: {
      int by = yb - (o.kind == 3 ? 19 : 34);
      display.fillRect(x + 3, by + 3, 8, 3, W);
      display.fillRect(x, by + 4, 3, 1, W);
      display.fillRect(x + 11, by + 2, 3, 2, W);
      if (flap) display.fillRect(x + 5, by, 4, 3, W); else display.fillRect(x + 5, by + 6, 4, 2, W);
    }
  }
}

void drawGame() {
  startScreen();
  unsigned long now = millis();
  bool step = ((now / 140) & 1) != 0;
  display.drawFastHLine(0, GAME_GROUND, SCREEN_W, SH110X_WHITE);
  for (int i = 0; i < 6; i++) {                                  // бугорки земли едут назад
    int gx = (int)(((i * 23) - (long)(gDist * 1.0f)) % 140);
    if (gx < 0) gx += 140;
    display.drawPixel(gx - 6, GAME_GROUND + 2 + (i & 1), SH110X_WHITE);
  }
  int dy = GAME_GROUND - (int)gH;
  gDrawDino(10, dy, gDuck, gameSt == GS_PLAY && gH <= 0 && step);
  for (int i = 0; i < GOBS_N; i++) if (gObs[i].on) gDrawObs(gObs[i], GAME_GROUND, step);
  display.setCursor(50, 0);
  display.printf("HI%05u %05u", (unsigned)gHi, (unsigned)gScore);
  if (gameSt == GS_READY) {
    display.setCursor(30, 18); display.print("DINO");
    display.setCursor(0, 30);  display.printf("%d+ start  %d- exit", MENU_ENTER_CH + 1, MENU_ENTER_CH + 1);
  } else if (gameSt == GS_OVER) {
    display.setCursor(30, 18); display.print("GAME OVER");
    display.setCursor(0, 30);  display.printf("%d+ again  %d- exit", MENU_ENTER_CH + 1, MENU_ENTER_CH + 1);
  }
  oledCommit();
}

void hintLine(const char* what);
void drawGh() {
  startScreen();
  display.setCursor(0, 0);   display.print(ghTestSlot >= 0 ? "WI-FI TEST" : "GITHUB UPDATE");
  switch (ghStage) {
    case G_TEST:
      display.setCursor(0, 14);  display.print(ghFresh ? "Saved and connected" : "Connected");
      display.setCursor(0, 26);  display.printf("%.21s", ghNet);
      display.setCursor(0, 36);  display.printf("%d dBm  pass:%d", ghRssi, ghPassLen);
      display.setCursor(0, 46);  display.printf("IP %s", ghIp);
      exitHint("exit", 56);
      break;
    case G_SCAN:
      display.setCursor(0, 16);  display.print("Looking for networks");
      display.setCursor(0, 28);  display.print("saved in the pult...");
      exitHint("cancel", 56);
      break;
    case G_CONN:
      display.setCursor(0, 16);  display.print("Connecting to");
      display.setCursor(0, 28);  display.printf("%.21s", ghNet);
      exitHint("cancel", 56);
      break;
    case G_API:
      display.setCursor(0, 16);  display.printf("%.21s", ghNet);
      display.setCursor(0, 28);  display.print("Connected");
      display.setCursor(0, 40);  display.print("Checking GitHub...");
      break;
    case G_ASK:
      display.setCursor(0, 14);  display.printf("%s %s", PROJECT_NAME, ghVer);
      display.setCursor(0, 24);  display.printf("now %s", FW_VERSION);
      display.setCursor(0, 34);  display.printf("%u KB  %s", (unsigned)(ghSize / 1024), strcmp(ghVer, FW_VERSION) == 0 ? "SAME" : "new");
      hintLine("%d:- %d:yes/no");
      break;
    case G_DL: {
      bool ver = ghSize && ghDone >= ghSize;
      int pct = ghSize ? (int)((uint64_t)ghDone * 100 / ghSize) : 0;
      display.setCursor(0, 14);  display.print(ver ? "Verifying..." : "Downloading");
      display.drawRect(0, 26, PBAR_W, 12, SH110X_WHITE);
      display.fillRect(2, 28, (PBAR_W - 4) * pct / 100, 8, SH110X_WHITE);
      display.setCursor(0, 42);  display.printf("%d%% %u/%u KB", pct, (unsigned)(ghDone / 1024), (unsigned)(ghSize / 1024));
      display.setCursor(0, 56);  display.print("do not switch off");
      break;
    }
    case G_OK:
      display.setCursor(0, 14);  display.print("Installed");
      display.setCursor(0, 26);  display.printf("version %s", upVer);
      display.setCursor(0, 42);  display.print("Restarting...");
      break;
    case G_ERR:
      display.setCursor(0, 14);  display.print(ghErr);
      display.setCursor(0, 26);  display.printf("%.21s", ghErr2);
      if (ghErrCode > 0) { display.setCursor(0, 38); display.printf("code %d  pass:%d", ghErrCode, ghPassLen); }
      else if (ghErrInfo[0]) { display.setCursor(0, 38); display.printf("code: %s", ghErrInfo); }
      exitHint("exit", 56);
      break;
  }
  drawBattery();
  oledCommit();
}

void hintLine(const char* what) {
  char h[24];
  snprintf(h, sizeof(h), what, MENU_NAV_CH + 1, MENU_ENTER_CH + 1);
  drawHintText(h, 54);
}

void drawMenu() {
  startScreen();
  display.setCursor(0, 0);
  switch (menuScr) {
    case MS_MAIN:    display.print("MENU"); break;
    case MS_TX:      display.print("TRANSMITTER"); break;
    case MS_CAL:     display.print("CALIBRATE"); break;
    case MS_CHLIST:  display.print("CHANNEL SETUP"); break;
    case MS_CHEDIT:  display.printf("CH%d now:%+4d", editCh + 1, val(editCh)); break;
    case MS_BAT:
      if (batUsb) display.print("BATTERY (USB)");
      else        display.printf("BATTERY %.2fV", vbat);
      break;
    case MS_LED:     display.print("LED & ALERTS"); break;
    case MS_LEDSTAT: display.print("STATUS LEDS"); break;
    case MS_EVLIST:  display.print("ALERT EVENTS"); break;
    case MS_EVEDIT:
      display.print(EV_NAME[evSel]);
      if (evSel == EV_RXALARM || evSel == EV_NOLINK) display.print(" (fixed ON)");
      break;
    case MS_PRESET:  display.printf("PRESET: %s", presetName()); break;
    case MS_SCREENS: display.print("SCREENS (*=start)"); break;
    case MS_SCRITEM: display.print(SCR_NAME[scrOrder[scrSel]]); break;
    case MS_SCRMOVE: display.print("MOVE: 3 up/down"); break;
    case MS_POWER:   display.print("POWER"); break;
    case MS_BIND:    display.print("BIND"); break;
    case MS_EVLOG:   display.printf("EVENT LOG %d/%d", logN, LOG_MAX); break;
    case MS_DBGLIST: display.print("DEBUG"); break;
    case MS_FRTARGET: display.print("FACTORY RESET"); break;
    case MS_WIFI:    display.print("WI-FI"); break;
    case MS_NETACT:  display.printf("%.21s", netSsid[netSlot < NET_MAX ? netSlot : 0]); break;
    case MS_NETS:    display.printf("NETWORKS %d/%d", netCount(), NET_MAX); break;
    case MS_NETPICK: display.print("SELECT NETWORK"); break;
    case MS_RX:      display.print("RECEIVER"); break;
    case MS_RXWIFIM: display.print("RECEIVER WI-FI"); break;
    case MS_RXCH: {
      static const char* tt[] = { "CURRENT LIMITS", "START DUTY", "MAX DUTY", "RAMPS", "CHANNELS ON/OFF" };
      static const char* ti[] = { "CURRENT LIMITS", "START CURRENT", "MAX CURRENT", "RAMPS", "CHANNELS ON/OFF" };
      display.print((rxCurMode() ? ti : tt)[rxTopic < 5 ? rxTopic : 0]);
      break;
    }
    case MS_RXDIR: {
      static const char* dt[] = { "LIMIT", "DUTY", "MAX DUTY", "RAMPS" };
      static const char* di[] = { "LIMIT", "START I", "MAX I", "RAMPS" };
      display.printf("CH%d %s", rxCh + 1, (rxCurMode() ? di : dt)[rxTopic < 4 ? rxTopic : 0]);
      break;
    }
    case MS_RXGLOB:  display.print("RX GLOBAL"); break;
    default: break;
  }
  int cnt = menuCount();
  char lab[24];
  for (int r = 0; r < MENU_ROWS; r++) {
    int idx = menuTop + r;
    if (idx >= cnt) break;
    int y = 11 + r * 10;
    menuLabel(idx, lab, sizeof(lab));
    bool inactive = !rxLinked() && ((menuScr == MS_MAIN && idx == 1) || (menuScr == MS_POWER && idx == 1));   // без связи
    if (idx == sel) {
      display.fillRect(0, y - 1, 112, 10, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.setTextColor(SH110X_WHITE);
    }
    display.setCursor(2, y);
    display.print(lab);
    if (inactive) {   // зачёркнуто = недоступно
      display.drawLine(1, y + 3, 2 + 6 * (int)strlen(lab), y + 3, idx == sel ? SH110X_BLACK : SH110X_WHITE);
    }
  }
  hintLine("%d:move %d:ok/back");
  drawBattery();
  oledCommit();
}

void drawDz() {
  startScreen();
  display.setCursor(0, 0);   display.print("DEAD ZONE");
  display.setTextSize(2);
  display.setCursor(0, 14);  display.printf("%d", dzEdit);
  display.setTextSize(1);
  display.setCursor(0, 36);  display.printf("ADC counts, was %d", deadzone);
  hintLine("%d:+/- %d:ok/cancel");
  drawBattery();
  oledCommit();
}

// рамка с кривой отклика и точкой текущего положения стика
void drawCurveBox(int x0, int y0, int sz, int expo, int maxp, int stick) {
  display.drawRect(x0, y0, sz, sz, SH110X_WHITE);
  display.drawLine(x0 + sz / 2, y0, x0 + sz / 2, y0 + 2, SH110X_WHITE);
  display.drawLine(x0 + sz / 2, y0 + sz - 3, x0 + sz / 2, y0 + sz - 1, SH110X_WHITE);
  display.drawLine(x0, y0 + sz / 2, x0 + 2, y0 + sz / 2, SH110X_WHITE);
  display.drawLine(x0 + sz - 3, y0 + sz / 2, x0 + sz - 1, y0 + sz / 2, SH110X_WHITE);
  int span = sz - 5;
  int px = 0, py = 0;
  for (int k = 0; k <= 20; k++) {
    int v = -100 + k * 10;
    int o = applyCurve(v, expo, maxp);
    int cx = x0 + 2 + (v + 100) * span / 200;
    int cy = y0 + sz - 3 - (o + 100) * span / 200;
    if (k > 0) display.drawLine(px, py, cx, cy, SH110X_WHITE);
    px = cx; py = cy;
  }
  int o = applyCurve(stick, expo, maxp);
  int dx = x0 + 2 + (stick + 100) * span / 200;
  int dy = y0 + sz - 3 - (o + 100) * span / 200;
  display.fillRect(dx - 1, dy - 1, 3, 3, SH110X_WHITE);
}

void drawCurveEdit() {
  startScreen();
  bool isExpo = (menuScr == MS_EXPO);
  int expo = isExpo ? editInt : chExpo[editCh];
  int mx   = isExpo ? chMaxP[editCh] : editInt;
  int was  = isExpo ? chExpo[editCh] : chMaxP[editCh];
  int stick = valBase(editCh);
  display.setCursor(0, 0);   display.printf("CH%d %s", editCh + 1, isExpo ? "EXPO" : "MAX");
  display.setTextSize(2);
  display.setCursor(0, 14);  display.printf("%d%%", editInt);
  display.setTextSize(1);
  display.setCursor(0, 36);  display.printf("was %d%%", was);
  display.setCursor(0, 45);  display.printf("out %+d", applyCurve(stick, expo, mx));
  drawCurveBox(70, 10, 40, expo, mx, stick);
  hintLine("%d:+/- %d:ok/cancel");
  drawBattery();
  oledCommit();
}

void drawBatEdit() {
  static const char* titles[] = { "DIVIDER CAL", "FULL VOLTAGE", "EMPTY VOLTAGE", "WARN LEVEL", "CRIT LEVEL" };
  startScreen();
  display.setCursor(0, 0);   display.print(titles[batKind]);
  display.setTextSize(2);
  display.setCursor(0, 14);
  if (batKind == BE_WARN || batKind == BE_CRIT) display.printf("%d%%", batEditI);
  else                                          display.printf("%.2fV", batEditCv / 100.0f);
  display.setTextSize(1);
  switch (batKind) {
    case BE_DIV:
      display.setCursor(0, 34);  display.print("Type meter reading");
      display.setCursor(0, 44);  display.printf("Pult shows %.2fV", vbat);
      break;
    case BE_FULL:
      display.setCursor(0, 34);  display.print("100% at this V");
      display.setCursor(0, 44);  display.printf("Now %.2fV", vbat);
      break;
    case BE_EMPTY:
      display.setCursor(0, 34);  display.print("0% at this V");
      display.setCursor(0, 44);  display.printf("Now %.2fV", vbat);
      break;
    case BE_WARN:
    case BE_CRIT:
      display.setCursor(0, 34);  display.print("Alert at or below");
      display.setCursor(0, 44);  display.printf("Now %d%%", vbatPct);
      break;
  }
  hintLine("%d:+/- %d:ok/cancel");
  drawBattery();
  oledCommit();
}

void drawTest() {
  EvCfg c = cfgOf(testEv);
  startScreen();
  display.setCursor(0, 0);   display.printf("TEST ALERTS %d/%d", testEv + 1, NUM_EV);
  display.setTextSize(2);
  display.setCursor(0, 14);  display.print(EV_NAME[testEv]);
  display.setTextSize(1);
  display.setCursor(0, 36);
  if (testEv == EV_HEART) display.printf("%s every %ds", COL_NAME[c.color], HB_SEC[c.ival]);
  else                    display.printf("%s %s", COL_NAME[c.color], PAT_NAME[c.pat]);
  display.setCursor(0, 45);  display.print(c.en ? "enabled" : "disabled");
  hintLine("%d:next %d:exit");
  drawBattery();
  oledCommit();
}

void drawRxEdit() {
  const RxParamDef &d = RXP[rxParam];
  char t[28], v[14], w[14];
  startScreen();
  if (d.scope == PS_SOL)       snprintf(t, sizeof(t), "CH%d %s %s", rxCh + 1, rxDir ? "RET" : "EXT", d.name);
  else if (d.scope == PS_CH)   snprintf(t, sizeof(t), "CH%d %s", rxCh + 1, d.name);
  else                         snprintf(t, sizeof(t), "%s", d.name);
  display.setCursor(0, 0);   display.print(t);
  fmtParam(rxParam, rxCur, v, sizeof(v));
  display.setTextSize(2);
  display.setCursor(0, 11);  display.print(v);
  display.setTextSize(1);
  fmtParam(rxParam, rxOrig, w, sizeof(w));
  display.setCursor(0, 30);  display.printf("was %s (RX ack)", w);
  display.setCursor(0, 40);
  if (rxLeverParam(rxParam)) {
    if (!leverArmed) display.printf("Center lever CH%d!", TEST_LEVER_CH + 1);
    else             display.printf("Lever CH%d: %+d", TEST_LEVER_CH + 1, ctl[rxCh]);
  } else if (rxParam == RXP_PWR && rxCur > 14) {
    display.print("check local limits");
  } else if (rxParam == RXP_PWMFREQ) {
    display.print("all channels");
  } else if (rxParam == RXP_MODE) {
    display.print("PWM=open, CURRENT=PI");
  }
  hintLine("%d:+/- %d:ok/cancel");
  drawBattery();
  oledCommit();
}

void drawRxStat() {
  startScreen();
  display.setCursor(0, 0);   display.print(simMode == SIM_RADIO ? "RX STATUS" : "RX STATUS (sim)");
  display.setCursor(0, 11);  display.printf("Link: %s", rxLinked() ? "OK" : "LOST");
  display.setCursor(0, 21);
  if (telValid) display.printf("VBAT-RX: %.1fV", tel.vrx / 10.0f);
  else          display.print("VBAT-RX: --");
  display.setCursor(0, 31);
  if (telValid) display.printf("RSSI: %d dBm", rfRssi);
  else          display.print("RSSI: --");
  display.setCursor(0, 41);  display.printf("Lost pkts: %lu", (unsigned long)rfLost);
  display.setCursor(0, 51);
  if (rxCfgKnown) display.printf("Control: %s", rxModeCache ? "CURRENT" : "PWM");
  else            display.print("Control: --");
  hintLine("%d:back");
  drawBattery();
  oledCommit();
}

void drawDbgDesc() {
  startScreen();
  const DescRef &d = DESC[dbgPage];
  for (int r = 0; r < 6; r++) {
    int idx = dbgTop + r;
    if (idx >= d.n) break;
    display.setCursor(0, r * 9);
    display.print(d.lines[idx]);
  }
  char h[24];
  snprintf(h, sizeof(h), "%d:scroll %d:open", MENU_NAV_CH + 1, MENU_ENTER_CH + 1);
  drawHintText(h, 56);
  drawBattery();
  oledCommit();
}

// последний ток перегрузки/КЗ для соленоида из журнала, -1 если не было
int lastOvl(int ch, int side) {
  for (int i = 0; i < logN; i++) {
    const LogEv &e = logRam[i];
    if ((e.type == LT_OVL || e.type == LT_SHORT) && e.ch == ch && e.side == side) return e.dA;
  }
  return -1;
}

void drawDbgPage() {
  startScreen();
  char h[16];
  snprintf(h, sizeof(h), "%d:back", MENU_ENTER_CH + 1);
  switch (dbgPage) {
    case DP_RAW:
      display.setCursor(0, 0);   display.print("RAW ADC 4/6");
      for (int r = 0; r < 4; r++) {
        display.setCursor(0, 10 + r * 9);
        display.printf("%d:%5d", r + 1, raw[r]);
        display.setCursor(58, 10 + r * 9);
        display.printf("%d:%5d", r + 5, raw[r + 4]);
      }
      display.setCursor(0, 46);
      display.printf("ADS8 %lums DZ%d", readUs / 1000, deadzone);
      drawHintText(h, 56);
      drawBattery();
      break;
    case DP_RADIO:
      display.setCursor(0, 0);   display.print("RADIO 1/6");
      display.setCursor(0, 10);  display.print(RF_FREQ_TXT);
      display.setCursor(0, 19);
      if (telValid) display.printf("TX%ddBm RX%ddBm", txPower, (int)rxQuery(0, 0, RXP_PWR));
      else          display.printf("TX%ddBm RX--", txPower);
      display.setCursor(0, 28);
      if (telValid && simMode == SIM_RADIO) display.printf("RSSI%d/%d SNR%d/%d", rfRssi, (int)tel.rssi, (rfSnr10 + (rfSnr10 < 0 ? -5 : 5)) / 10, (int)tel.snr);
      else if (telValid) display.printf("RSSI%d SNR%.1f", rfRssi, rfSnr10 / 10.0f);
      else          display.print("RSSI-- SNR--");
      display.setCursor(0, 37);
      if (telValid) {
        uint32_t tot = rfRx + rfLost;
        display.printf("Lost %lu (%.1f%%) %d/s", (unsigned long)rfLost, tot ? rfLost * 100.0f / tot : 0.0f, rfRate);
      } else display.print("Lost --");
      display.setCursor(0, 46);
      if (telValid) display.printf("Age %.1fs ID %04X", rfAgeMs / 1000.0f, rxId);
      else          display.print("Age -- ID ----");
      drawHintText(h, 56);
      drawBattery();
      break;
    case DP_TELEM:
      display.setCursor(0, 0);   display.print("TELEMETRY 2/6");
      if (!telValid) {
        display.setCursor(0, 20);  display.print("no telemetry");
      } else {
        display.setCursor(0, 10);
        display.print("Ip");
        for (int c = 0; c < NUM_CH; c++) display.printf("%d ", tel.cur[c]);
        display.setCursor(0, 19);
        display.print("P");
        for (int c = 0; c < NUM_CH; c++) display.printf("%d ", tel.pwm[c]);
        display.setCursor(0, 28);
        display.printf("L%02X O%02X X%02X s%02X", tel.mLim, tel.mOvl, tel.mOpen, tel.side);
        display.setCursor(0, 37);
        display.printf("Vrx%.1f R%d K:%s", tel.vrx / 10.0f, tel.rssi, tel.kill ? "YES" : "no");
      }
      drawHintText(h, 56);
      drawBattery();
      break;
    case DP_SYS: {
      unsigned long sec = millis() / 1000UL;
      display.setCursor(0, 0);   display.print("SYSTEM 5/6");
      display.setCursor(0, 10);  display.printf("FW %s %s", FW_VERSION, __DATE__);
      display.setCursor(0, 19);  display.printf("Up %02lu:%02lu:%02lu", sec / 3600, (sec / 60) % 60, sec % 60);
      display.setCursor(0, 28);  display.printf("Temp %dC", (int)temperatureRead());
      display.setCursor(0, 37);  display.printf("Heap %uk free", (unsigned)(ESP.getFreeHeap() / 1024));
      display.setCursor(0, 46);  display.printf("I2C %d/4 Draw %lu+%lu", (int)okOled + (int)okMcp + (int)adsOk[0] + (int)adsOk[1], dispUs / 1000, oledFlushUs / 1000);
      drawHintText(h, 56);
      drawBattery();
      break;
    }
    case DP_LOG: {
      display.setCursor(0, 0);   display.print("LOG 6/6");
      int total = slogCount < SLOG_LINES ? slogCount : SLOG_LINES;
      int first = slogCount - total;
      int rows = total < 5 ? total : 5;
      for (int r = 0; r < rows; r++) {
        int k = first + (total - rows) + r;
        display.setCursor(0, 10 + r * 9);
        display.print(slog[k % SLOG_LINES]);
      }
      drawHintText(h, 56);
      drawBattery();
      break;
    }
    case DP_LIM:
      // таблица без заголовка и подсказки: 8 строк, канал, A(лимит, ток, перегрузка), B(лимит, ток, перегрузка)
      for (int c = 0; c < NUM_CH; c++) {
        char buf[28], ovl[2][8];
        for (int sd = 0; sd < 2; sd++) {
          int o = lastOvl(c, sd);
          if (o < 0) snprintf(ovl[sd], sizeof(ovl[sd]), "--");
          else       snprintf(ovl[sd], sizeof(ovl[sd]), "%02d", o > 99 ? 99 : o);
        }
        if (telValid) {
          int lA = rxQuery(c, 0, RXP_LIMIT), lB = rxQuery(c, 1, RXP_LIMIT);
          int nA = (tel.pwm[c] > 0) ? tel.cur[c] : 0;
          int nB = (tel.pwm[c] < 0) ? tel.cur[c] : 0;
          snprintf(buf, sizeof(buf), "%d %02d %02d %s %02d %02d %s", c + 1,
                   lA > 99 ? 99 : lA, nA > 99 ? 99 : nA, ovl[0],
                   lB > 99 ? 99 : lB, nB > 99 ? 99 : nB, ovl[1]);
        } else {
          snprintf(buf, sizeof(buf), "%d -- -- %s -- -- %s", c + 1, ovl[0], ovl[1]);
        }
        display.setCursor(0, c * 8);
        display.print(buf);
      }
      break;
  }
  oledCommit();
}

void drawPinScreen() {
  startScreen();
  display.setCursor(0, 0);   display.print("FACTORY RESET");
  display.setCursor(0, 11);  display.print("Enter code:");
  display.setTextSize(2);
  for (int i = 0; i < 4; i++) {
    display.setCursor(8 + i * 22, 24);
    display.printf("%d", pinDigit[i]);
    if (i == pinPos) {
      display.drawLine(8 + i * 22, 42, 8 + i * 22 + 11, 42, SH110X_WHITE);
      display.drawLine(8 + i * 22, 43, 8 + i * 22 + 11, 43, SH110X_WHITE);
    }
  }
  display.setTextSize(1);
  hintLine("%d:digit %d:next/back");
  drawBattery();
  oledCommit();
}

void drawText() {
  startScreen();
  display.setCursor(0, 0);
  if (txtStage == 0) display.print("HIDDEN NETWORK NAME");
  else               display.printf("PASS: %.14s", txtSsid);
  // введённый текст: последние 20 символов и курсор
  char line[24];
  int from = txtLen > 20 ? txtLen - 20 : 0;
  snprintf(line, sizeof(line), "%s_", txtBuf + from);
  display.setCursor(0, 14);  display.print(line);
  display.setCursor(104, 0); display.printf("%d", txtLen);
  // выбранный символ крупно, соседние мелко
  const int total = TXT_N + 2;
  for (int k = -2; k <= 2; k++) {
    int idx = ((txtCur + k) % total + total) % total;
    char t[5];
    if (idx < TXT_N)       snprintf(t, sizeof(t), "%c", TXT_SET[idx] == ' ' ? '_' : TXT_SET[idx]);
    else if (idx == TXT_N) snprintf(t, sizeof(t), "DEL");
    else                   snprintf(t, sizeof(t), "OK");
    if (k == 0) {
      display.setTextSize(2);
      display.fillRect(46, 28, 36, 18, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
      display.setCursor(64 - (int)strlen(t) * 6, 30); display.print(t);
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);
    } else {
      display.setCursor(64 + k * 22 - (k > 0 ? 0 : 0) - (int)strlen(t) * 3 + (k > 0 ? 6 : -6), 34);
      display.print(t);
    }
  }
  hintLine("%d:char %d:ok/del");
  drawBattery();
  oledCommit();
}

void drawConfirm() {
  startScreen();
  switch (confAct) {
    case CA_CLEARLOG:
      display.setCursor(0, 0);   display.print("CLEAR LOG?");
      display.setCursor(0, 14);  display.print("Erases the event log");
      display.setCursor(0, 24);  display.print("(RAM and saved)");
      break;
    case CA_FORGET:
      display.setCursor(0, 0);   display.print("FORGET BIND?");
      display.setCursor(0, 14);  display.print("Back to default ID.");
      display.setCursor(0, 24);  display.print("Bind again later.");
      break;
    case CA_FR_TX:
      display.setCursor(0, 0);   display.print("RESET PULT?");
      if (restartAt) {
        display.setTextSize(2);
        display.setCursor(0, 20); display.print("Resetting");
        display.setTextSize(1);
      } else {
        display.setCursor(0, 14);  display.print("Erase ALL settings.");
        display.setCursor(0, 24);  display.print("Bind ID is kept.");
        display.setCursor(0, 34);  display.print("Pult will restart.");
      }
      break;
    case CA_NETDEL:
      display.setCursor(0, 0);   display.print("DELETE NETWORK?");
      display.setCursor(0, 14);  display.printf("%.21s", netSsid[confIdx < NET_MAX ? confIdx : 0]);
      display.setCursor(0, 24);  display.print("Name and password");
      display.setCursor(0, 34);  display.print("are erased.");
      break;
    case CA_NETALL:
      display.setCursor(0, 0);   display.print("FORGET ALL WI-FI?");
      display.setCursor(0, 14);  display.print("All saved networks");
      display.setCursor(0, 24);  display.print("are erased.");
      break;
    case CA_FR_RX:
      display.setCursor(0, 0);   display.print("RESET RECEIVER?");
      display.setCursor(0, 14);  display.print("Receiver settings");
      display.setCursor(0, 24);  display.print("to defaults.");
      display.setCursor(0, 34);  display.print("Bind ID is kept.");
      break;
  }
  if (!restartAt) hintLine("%d:- %d:yes/no");
  drawBattery();
  oledCommit();
}

void drawBindRun() {
  startScreen();
  display.setCursor(0, 0);   display.print("BIND MODE");
  if (bindPhase == 0) {
    int left = (int)((BIND_TIMEOUT_MS - (millis() - bindStart) + 999) / 1000);
    if (left < 0) left = 0;
    display.setCursor(0, 11);  display.print("Hold PRG on RX ~3s");
    display.setCursor(0, 21);  display.printf("Waiting... %ds", left);
    display.setCursor(0, 31);  display.print("Keep pult close");
    if (simMode == SIM_RADIO) {
      display.setCursor(0, 41);  display.print("Sending new pair ID");
    } else {
      display.setCursor(0, 41);  display.print("Sim mode: LINK");
      display.setCursor(0, 50);  display.print("fakes success");
    }
  } else if (bindPhase == 1) {
    display.setTextSize(2);
    display.setCursor(0, 16);  display.print("BOUND!");
    display.setTextSize(1);
    display.setCursor(0, 40);  display.printf("ID %04X", (unsigned)(bindId & 0xFFFF));
  } else {
    display.setTextSize(2);
    display.setCursor(0, 16);  display.print("TIMEOUT");
    display.setTextSize(1);
  }
  drawBattery();
  oledCommit();
}

void drawBindId() {
  startScreen();
  display.setCursor(0, 0);   display.print("BIND ID");
  display.setCursor(0, 12);  display.print("Pair ID:");
  display.setTextSize(2);
  display.setCursor(0, 22);  display.printf("%08lX", (unsigned long)bindId);
  display.setTextSize(1);
  display.setCursor(0, 42);  display.printf("Short: %04X", (unsigned)(bindId & 0xFFFF));
  display.setCursor(0, 51);  display.print(bindId == DEF_BIND_ID ? "Default ID" : "Custom (bound)");
  drawBattery();
  oledCommit();
}

void drawTxPwr() {
  startScreen();
  display.setCursor(0, 0);   display.print("TX POWER (pult)");
  display.setTextSize(2);
  display.setCursor(0, 12);  display.printf("%ddBm", TXP_LIST[editInt]);
  display.setTextSize(1);
  display.setCursor(0, 33);  display.printf("about %d mW", TXP_MW[editInt]);
  display.setCursor(0, 42);
  if (TXP_LIST[editInt] > 14) display.print("check local limits");
  else                        display.printf("was %ddBm", txPower);
  hintLine("%d:+/- %d:ok/cancel");
  drawBattery();
  oledCommit();
}

void drawTRate() {
  startScreen();
  display.setCursor(0, 0);   display.print("TELEMETRY RATE");
  display.setTextSize(2);
  display.setCursor(0, 12);  display.printf("%d/s", editInt);
  display.setTextSize(1);
  display.setCursor(0, 33);  display.print("RX replies on request");
  display.setCursor(0, 42);  display.printf("was %d/s, default %d/s", telemRate, TELEM_RATE_DEF);
  hintLine("%d:+/- %d:ok/cancel");
  drawBattery();
  oledCommit();
}

void drawCal() {
  startScreen();
  if (state == ST_CAL_C && calPhase == 0) {
    int left = (int)((CAL_COUNT_MS - (millis() - calStart) + 999) / 1000);
    if (left < 0) left = 0;
    display.setCursor(0, 0);   display.print("CALIBRATE CENTER");
    display.setCursor(0, 12);  display.print("Set ALL controls");
    display.setCursor(0, 22);  display.print("to CENTER");
    display.setTextSize(2);
    display.setCursor(0, 38);  display.printf("In %ds", left);
    display.setTextSize(1);
  } else if (state == ST_CAL_C) {
    display.setCursor(0, 0);   display.print("CALIBRATE CENTER");
    display.setCursor(0, 20);  display.print("Reading center...");
    display.setCursor(0, 32);  display.print("Do not touch!");
  } else if (state == ST_CAL_R) {
    int left = (int)((CAL_RANGE_MS - (millis() - calStart) + 999) / 1000);
    if (left < 0) left = 0;
    display.setCursor(0, 0);   display.print("CALIBRATE RANGE");
    display.setCursor(0, 12);  display.print("Move every control");
    display.setCursor(0, 22);  display.print("to both extremes");
    display.setTextSize(2);
    display.setCursor(0, 38);  display.printf("Left: %ds", left);
    display.setTextSize(1);
  } else if (calKind == 0) {
    display.setCursor(0, 0);   display.print("CENTER SAVED");
    display.setCursor(0, 16);  display.print("Center position");
    display.setCursor(0, 26);  display.print("saved to memory");
  } else if (calFailCount == 0) {
    display.setCursor(0, 0);   display.print("CALIBRATION OK");
    display.setCursor(0, 16);  display.print("Range saved");
    display.setCursor(0, 26);  display.print("for all channels");
  } else {
    display.setCursor(0, 0);   display.print("CALIBRATION FAILED");
    display.setCursor(0, 14);  display.print("Not moved: CH");
    display.setCursor(0, 24);
    for (int i = 0; i < NUM_CH; i++) if (!calChOk[i]) display.printf("%d ", i + 1);
    display.setCursor(0, 38);  display.print("Others were saved");
    display.setCursor(0, 48);  display.print("Old range kept");
  }
  if (state == ST_CAL_C || state == ST_CAL_R) {
    display.setCursor(0, 56);  display.print("MENU = cancel");
  }
  drawBattery();
  oledCommit();
}

void drawScreen() {
  switch (state) {
    case ST_BOOT:    if ((millis() - bootStart) < SPLASH_MS) drawSplash(); else drawBoot(); break;
    case ST_SAFE:    drawSafe(); break;
    case ST_RUN:
      if (alShown >= 0) { drawAlert(); break; }
      switch (screenNo) {
        case SC_TX:    drawRun2(); break;
        case SC_RX:    drawRxChannels(); break;
        case SC_LINK:  drawLinkScreen(); break;
        case SC_POWER: drawPowerScreen(); break;
        case SC_LOG:   drawLogScreen(); break;
        default:       drawRun1(); break;
      }
      break;
    case ST_MENU:
      switch (menuScr) {
        case MS_DZ:      drawDz(); break;
        case MS_EXPO:
        case MS_MAXV:    drawCurveEdit(); break;
        case MS_BATEDIT: drawBatEdit(); break;
        case MS_TEST:    drawTest(); break;
        case MS_RXEDIT:  drawRxEdit(); break;
        case MS_RXSTAT:  drawRxStat(); break;
        case MS_RXWIFI:  drawRxWifi(); break;
        case MS_DBGDESC: drawDbgDesc(); break;
        case MS_DBGPAGE: drawDbgPage(); break;
        case MS_PIN:     drawPinScreen(); break;
        case MS_CONFIRM: drawConfirm(); break;
        case MS_TEXTIN:  drawText(); break;
        case MS_GAME:    drawGame(); break;
        case MS_BINDRUN: drawBindRun(); break;
        case MS_BINDID:  drawBindId(); break;
        case MS_TXPWR:   drawTxPwr(); break;
        case MS_TRATE:   drawTRate(); break;
        default:         drawMenu(); break;
      }
      break;
    case ST_CAL_C:
    case ST_CAL_R:
    case ST_CAL_RES: drawCal(); break;
    case ST_ESTOP:   drawEstop(); break;
    case ST_WIFI:    drawWifi(); break;
    case ST_GH:      drawGh(); break;
  }
}

// ============ ЛОГИКА КАЖДОГО ТАКТА ОПРОСА ============
void stepLogic(unsigned long now) {
  // связь появилась (вернулась) в рабочем режиме: как при включении, нули, пока все рычаги не в центре
  static bool prevLinked = false;
  bool lk = rxLinked();
  if (lk && !prevLinked && state == ST_RUN) {
    logLine("LINK back -> SAFE START");
    goSafe();
  }
  prevLinked = lk;
  switch (state) {
    case ST_SAFE: {
      bool ok = !hlSyncBusy;                                   // пока идёт синхронизация с приёмником, управление не передаётся
      for (int i = 0; i < NUM_CH; i++) if (chBad(i)) ok = false;
      if (ok) {
        if (safeSince == 0) safeSince = now;
        else if (now - safeSince >= SAFE_HOLD_MS) {
          state = ST_RUN;
          screenNo = scrStart;
          logLine("SAFE START OK -> RUN");
        }
      } else {
        safeSince = 0;
      }
      break;
    }
    case ST_MENU:
      // связь пропала внутри раздела Receiver: несохранённое откатывается, выход на верхний уровень
      if (isRxScreen() && !rxLinked()) {
        rxRevertAll();
        gotoScr(MS_MAIN, idxMain);
        toast("Link lost");
        logLine("RX menu: link lost, reverted unsaved");
        break;
      }
      if (menuScr == MS_BINDRUN) bindTick(now);
      if (menuScr == MS_RXWIFI) rxWifiTick(now);
      if (menuScr == MS_NETPICK) netPickTick(now);
      navStep(now);
      break;
    case ST_CAL_C:   calCenterTick(now); break;
    case ST_CAL_R:   calRangeTick(now); break;
    case ST_CAL_RES:
      if (now - calStart >= calResMs) {
        state = ST_MENU;
        gotoScr(MS_CAL, idxCal);
      }
      break;
    default:
      break;
  }
}

void handleMenuButton() {
  if (estop.state) return;
  switch (state) {
    case ST_RUN:
      if (btnMenu.shortEv) {
        screenNo = scrNext((uint8_t)screenNo);
        LOGF("SCREEN -> %s", SCR_NAME[screenNo]);
      }
      if (btnMenu.longEv) enterMenu();
      break;
    case ST_SAFE:
      if (btnMenu.longEv) enterMenu();
      break;
    case ST_MENU:
      if (btnMenu.shortEv) menuBack();
      break;
    case ST_WIFI:
      if (btnMenu.shortEv && !upActive) wifiStop("button");
      break;
    case ST_GH:
      if (btnMenu.shortEv && ghStage != G_DL && ghStage != G_OK) ghStop(ghStage == G_ASK ? "declined" : "button");
      break;
    case ST_CAL_C:
    case ST_CAL_R:
      if (btnMenu.shortEv) cancelCal();
      break;
    default:
      break;
  }
}

// ============ I2C ============
void i2cScan() {
  logLine("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      LOGF("  found 0x%02X", addr);
      found++;
    }
  }
  LOGF("  total: %d (expected 4: 0x20 0x3C 0x48 0x49)", found);
}

// ============ SETUP ============
void setup() {
  LOGF("=== PULT FW %s (radio) ===", FW_VERSION);
  LOGF("%s", FW_TAG);                          // ссылка на метку: без неё линкер выбросит её из файла

  pinMode(BTN_MENU_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);   // внешний 10к к 3.3В тоже должен стоять
  analogReadResolution(12);
  initButton(btnMenu);
  initButton(estop);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);
  i2cScan();

  okOled = display.begin(OLED_ADDR, true);
  LOGF("OLED: %s", okOled ? "ok" : "FAIL");

  okMcp = mcp.begin_I2C(MCP_ADDR, &Wire);
  LOGF("MCP23017: %s", okMcp ? "ok" : "FAIL");
  if (okMcp) {
    for (int i = 0; i < 16; i++) mcp.pinMode(i, OUTPUT);
    mcp.writeGPIOA(0);
    mcp.writeGPIOB(0);
  }

  adsOk[0] = ads1.begin(ADS1_ADDR, &Wire);
  adsOk[1] = ads2.begin(ADS2_ADDR, &Wire);
  LOGF("ADS1 0x48: %s", adsOk[0] ? "ok" : "FAIL");
  LOGF("ADS2 0x49: %s", adsOk[1] ? "ok" : "FAIL");
  for (int i = 0; i < 2; i++) {
    if (adsOk[i]) {
      adsBank[i]->setGain(GAIN_ONE);
      adsBank[i]->setDataRate(RATE_ADS1115_860SPS);
    }
  }

  loadConfig();
  netLoad();
  simInit();
  hlMirrorLoad();                              // копия настроек приёмника из памяти (до первой синхронизации только для просмотра)
  for (int i = 0; i < NUM_CH; i++) { raw[i] = cenV[i]; ctl[i] = 0; }

  batTick(millis());
  hlBegin();                                   // радио SX1262
  bootStart = millis();
  state = ST_BOOT;
}

// ============ LOOP ============
void loop() {
  unsigned long now = millis();

  updateButton(btnMenu);
  updateButton(estop);

  if (estop.changed) {
    LOGF("E-STOP: %s", estop.state ? "ACTIVE (pressed or wire break)" : "released");
    if (estop.state) { if (state != ST_WIFI && state != ST_GH) goEstop(); }   // в режимах Wi-Fi это делают wifiTick / ghTick
    else if (state == ST_ESTOP) goSafe();
  }
  if (btnMenu.changed) {
    LOGF("MENU button: %s", btnMenu.state ? "pressed" : "released");
  }

  if (state == ST_BOOT && (now - bootStart) >= (SPLASH_MS + BOOT_MS)) {
    if (estop.state) goEstop();
    else goSafe();
  }

  handleMenuButton();
  wifiTick(now);
  ghTick(now);
  hlStep(now);                                 // радио: на каждом проходе цикла

  static unsigned long lastRead = 0;
  if ((now - lastRead) >= READ_PERIOD_MS) {
    lastRead = now;
    readAll();
    stepLogic(now);
    buildControl();
    simTelemetry(now);
    eventsTick(now);
    updateLeds();
  } else {
    updateLeds();   // чтобы ритмы мигания не зависели от периода опроса
  }
  alertTick(now);

  batTick(now);

  if (restartAt && (long)(now - restartAt) >= 0) ESP.restart();   // после Factory Reset

  oledStep();                                  // одна страница кадра за проход цикла
  static unsigned long lastDisp = 0;
  if (okOled && !oledBusy() && (now - lastDisp) >= (gameOn() ? GAME_FRAME_MS : DISP_PERIOD_MS)) {
    lastDisp = now;
    unsigned long t0 = micros();
    drawScreen();
    dispUs = micros() - t0;
  }

}
