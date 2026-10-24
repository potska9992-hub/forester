// ============================================================
// Приёмник: ТЕСТ РАДИО 0.1 (Heltec Wireless Stick Lite V3, SX1262, 868 МГц)
// Только проверка радиоканала с пультом 0.3.1. Реле, драйверы и PCA9685 здесь не используются.
//  - ловит пакеты PING от пульта, мигает светодиодом платы, пишет в Serial: номер, RSSI, SNR, потери
//  - отвечает PONG (в нём RSSI/SNR, с которыми приёмник услышал пульт) ТОЛЬКО после нажатия USER:
//    сначала подключите антенну, потом нажмите USER. Повторное нажатие выключает ответы.
//  - мощность ответов маленькая (TX_POWER_DBM). Без antenna передавать нельзя!
// Arduino IDE: плата "Heltec Wireless Stick Lite (V3)", Tools -> USB CDC On Boot: Enabled,
//              библиотека RadioLib (Library Manager). Serial Monitor 115200.
// ============================================================
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#define PROJECT_NAME  "receiver"
#define FW_VERSION    "0.1"
static const char FW_TAG[] __attribute__((used)) = "FWTAG:" PROJECT_NAME ":" FW_VERSION ";";

// Пины Heltec Wireless Stick Lite V3
#define PIN_NSS   8
#define PIN_SCK   9
#define PIN_MOSI  10
#define PIN_MISO  11
#define PIN_RST   12
#define PIN_BUSY  13
#define PIN_DIO1  14
#define PIN_LED   35      // белый светодиод платы, HIGH = горит
#define PIN_USER  0       // кнопка USER, LOW = нажата

// Параметры радио (одинаковые в пульте и приёмнике)
#define RF_FREQ_MHZ   868.0
#define RF_BW_KHZ     125.0
#define RF_SF         7
#define RF_CR         5          // 4/5
#define RF_SYNC       0x12       // частная сеть
#define RF_PREAMBLE   8
#define TX_POWER_DBM  2          // мощность ответов, дБм (меняйте здесь, максимум 14)
#define TCXO_VOLTAGE  1.8f       // если радио не запустится, прошивка сама попробует без TCXO

#define PKT_MAGIC0  0xA5
#define PKT_MAGIC1  0x5A
#define PKT_PING    1
#define PKT_PONG    2
#define PKT_BIND    0x42         // метка пары для теста

struct __attribute__((packed)) Pkt {
  uint8_t  m0, m1, type;
  uint16_t seq;
  int8_t   rssi;                 // в PONG: RSSI, с которым приёмник услышал PING
  int8_t   snr;                  // в PONG: SNR того же PING
  uint8_t  bind;
  uint8_t  crc;                  // XOR всех байтов до этого
};

SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY);

volatile bool irqFlag = false;
void IRAM_ATTR onRadioIrq() { irqFlag = true; }

bool armed = false;              // ответы разрешены (антенна подтверждена кнопкой USER)
bool txBusy = false;
bool replyPending = false;
unsigned long replyAt = 0, ledOffAt = 0, statAt = 0, btnAt = 0;
bool btnPrev = false;
Pkt reply;

uint32_t rxOk = 0, rxBad = 0, rxLost = 0, txCount = 0;
int32_t  rssiSum = 0;
uint16_t lastSeq = 0;
bool     haveSeq = false;

static uint8_t pktCrc(const Pkt& p) {
  const uint8_t* b = (const uint8_t*)&p;
  uint8_t c = 0;
  for (size_t i = 0; i < sizeof(Pkt) - 1; i++) c ^= b[i];
  return c;
}

static void ledFlash(unsigned long ms) {
  digitalWrite(PIN_LED, HIGH);
  ledOffAt = millis() + ms;
}

static void radioFail(int state) {            // радио не запустилось: быстро мигаем и пишем причину
  Serial.printf("RADIO INIT FAILED, code %d\n", state);
  unsigned long last = 0;
  bool on = false;
  for (;;) {
    unsigned long now = millis();
    if (now - last >= 100) { last = now; on = !on; digitalWrite(PIN_LED, on); }
    yield();
  }
}

void handleRx() {
  size_t len = radio.getPacketLength();
  uint8_t buf[32];
  if (len > sizeof(buf)) len = sizeof(buf);
  int st = radio.readData(buf, len);
  if (st != RADIOLIB_ERR_NONE) { rxBad++; Serial.printf("RX bad packet (code %d)\n", st); return; }
  float rssi = radio.getRSSI();
  float snr = radio.getSNR();
  if (len != sizeof(Pkt)) { rxBad++; return; }
  Pkt p;
  memcpy(&p, buf, sizeof(Pkt));
  if (p.m0 != PKT_MAGIC0 || p.m1 != PKT_MAGIC1 || p.bind != PKT_BIND || p.crc != pktCrc(p)) { rxBad++; return; }
  if (p.type != PKT_PING) return;

  if (haveSeq && p.seq < lastSeq) { rxOk = 0; rxLost = 0; rssiSum = 0; haveSeq = false; }   // пульт перезапустился
  if (haveSeq && p.seq > (uint16_t)(lastSeq + 1)) rxLost += (uint16_t)(p.seq - lastSeq - 1);
  lastSeq = p.seq; haveSeq = true;
  rxOk++;
  rssiSum += (int32_t)lroundf(rssi);
  ledFlash(30);
  Serial.printf("PING #%u  RSSI %.0f dBm  SNR %.1f dB\n", (unsigned)p.seq, rssi, snr);

  if (armed) {
    reply.m0 = PKT_MAGIC0; reply.m1 = PKT_MAGIC1; reply.type = PKT_PONG; reply.seq = p.seq;
    reply.rssi = (int8_t)constrain((int)lroundf(rssi), -127, 0);
    reply.snr = (int8_t)constrain((int)lroundf(snr), -127, 127);
    reply.bind = PKT_BIND;
    reply.crc = pktCrc(reply);
    replyPending = true;
    replyAt = millis() + 15;                  // пауза, чтобы пульт успел переключиться на приём
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_USER, INPUT_PULLUP);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { yield(); }
  Serial.printf("\nRECEIVER RADIO TEST %s\n", FW_VERSION);
  Serial.println(FW_TAG);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  int state = radio.begin(RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, RF_CR, RF_SYNC, TX_POWER_DBM, RF_PREAMBLE, TCXO_VOLTAGE);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("begin with TCXO failed (%d), trying without TCXO\n", state);
    state = radio.begin(RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, RF_CR, RF_SYNC, TX_POWER_DBM, RF_PREAMBLE, 0.0f);
  }
  if (state != RADIOLIB_ERR_NONE) radioFail(state);
  radio.setDio2AsRfSwitch(true);
  radio.setDio1Action(onRadioIrq);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) radioFail(state);

  Serial.printf("Radio OK: %.1f MHz, BW %.0f, SF%d, TX power %d dBm\n", RF_FREQ_MHZ, RF_BW_KHZ, RF_SF, TX_POWER_DBM);
  Serial.println("Listening. Replies are OFF.");
  Serial.println("Connect the antenna, then press USER to enable replies (press again to disable).");
  ledFlash(300);
}

void loop() {
  unsigned long now = millis();

  // кнопка USER: включить/выключить ответы
  bool btn = (digitalRead(PIN_USER) == LOW);
  if (btn && !btnPrev && (now - btnAt) > 250) {
    btnAt = now;
    armed = !armed;
    if (!armed) replyPending = false;
    Serial.println(armed ? "Replies ON (antenna must be connected)" : "Replies OFF");
    ledFlash(armed ? 600 : 100);
  }
  btnPrev = btn;

  // прерывание радио: окончание передачи или пришёл пакет
  if (irqFlag) {
    irqFlag = false;
    if (txBusy) {
      radio.finishTransmit();
      txBusy = false;
      radio.startReceive();
    } else {
      handleRx();
    }
  }

  // отправка ответа
  if (replyPending && !txBusy && (long)(now - replyAt) >= 0) {
    replyPending = false;
    int st = radio.startTransmit((uint8_t*)&reply, sizeof(reply));
    if (st == RADIOLIB_ERR_NONE) { txBusy = true; txCount++; }
    else { Serial.printf("TX start failed (%d)\n", st); radio.startReceive(); }
  }

  if (ledOffAt && (long)(now - ledOffAt) >= 0) { digitalWrite(PIN_LED, LOW); ledOffAt = 0; }

  // статистика раз в 2 секунды
  if (now - statAt >= 2000) {
    statAt = now;
    if (rxOk || rxBad)
      Serial.printf("STAT ok %u  lost %u  bad %u  avg RSSI %ld dBm  replies %u\n",
                    (unsigned)rxOk, (unsigned)rxLost, (unsigned)rxBad, rxOk ? (long)(rssiSum / (int32_t)rxOk) : 0L, (unsigned)txCount);
  }
}
