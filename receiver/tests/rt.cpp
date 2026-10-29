#include "Arduino.h"
#include "Wire.h"
#include "Preferences.h"
#include <map>
#include <vector>
#include <deque>
#include <string>
#include <cmath>
// ---- время и ножки ----
static unsigned long T = 0;
void yield() {}
unsigned long millis() { return T; } unsigned long micros() { return T * 1000; }
int64_t esp_timer_get_time() { return (int64_t)T * 1000; }
static int pinLvl[64];
void pinMode(int, int) {}
int digitalRead(int p) { return pinLvl[p]; }
void digitalWrite(int p, int v) { pinLvl[p] = v; }
void gpio_set_level(int p, int v) { pinLvl[p] = v; }
static float busV = 12.0f; static bool relayStuck = false, relayNoClose = false;
static float fbV = 0;
uint32_t analogReadMilliVolts(int p) { float v = (p == 6) ? busV / 11.09f : fbV / 10.96f; return (uint32_t)(v * 1000.0f); }
void analogReadResolution(int) {}
void SerialC::begin(int) {} void SerialC::println(const char*) {} void SerialC::println() {} void SerialC::print(const char*) {}
int SerialC::printf(const char*, ...) { return 0; } SerialC::operator bool() { return false; } SerialC Serial;
float temperatureRead() { return 40; } uint32_t EspC::getFreeHeap() { return 200000; } static int restarts = 0; void EspC::restart() { restarts++; } EspC ESP; uint32_t esp_random() { return 1; }
// ---- I2C: PCA9685 и ADS1115 ----
static bool pcaPresent = true; static bool adsPres[2] = { true, true };
static float pcaDuty[16]; static int pcaWrites = 0; static int pcaPreReg = -1;
static float adsV[8];                          // напряжения входов (ADS 0x48 A0..A3, 0x49 A0..A3)
static std::vector<uint8_t> cur; static uint8_t curAddr = 0; static int adsSel[2] = { 0, 0 }; static int readIdx = 0; static uint8_t rdBuf[2];
void TwoWire::begin(int, int, long) {} void TwoWire::setClock(long) {} void TwoWire::setTimeOut(int) {}
static TwoWire* curWire = nullptr;
void TwoWire::beginTransmission(uint8_t a) { cur.clear(); curAddr = a; curWire = this; }
size_t TwoWire::write(uint8_t v) { cur.push_back(v); return 1; }
size_t TwoWire::write(const uint8_t* p, size_t n) { for (size_t i = 0; i < n; i++) cur.push_back(p[i]); return n; }
static float decDuty(const uint8_t* b) { // b[0..1]=ON, b[2..3]=OFF
  if (b[1] & 0x10) return 100.0f; if (b[3] & 0x10) return 0.0f; int off = b[2] | ((b[3] & 0x0F) << 8); return off / 40.96f; }
int TwoWire::endTransmission(bool) {
  if (this == &Wire) {
    if (curAddr != 0x40 || !pcaPresent) return 2;
    if (cur.size() >= 2) {
      uint8_t reg = cur[0];
      if (reg == 0xFE) pcaPreReg = cur[1];
      if (reg >= 0x06 && reg < 0x46 && cur.size() == 5) { pcaDuty[(reg - 0x06) / 4] = decDuty(&cur[1]); pcaWrites++; }
      if (reg == 0xFA && cur.size() == 5) { for (int i = 0; i < 16; i++) pcaDuty[i] = decDuty(&cur[1]); pcaWrites++; }
    }
    return 0;
  }
  int d = curAddr == 0x48 ? 0 : (curAddr == 0x49 ? 1 : -1);
  if (d < 0 || !adsPres[d]) return 2;
  if (cur.size() == 3 && cur[0] == 0x01) { uint16_t cfg = (cur[1] << 8) | cur[2]; adsSel[d] = ((cfg >> 12) & 7) - 4; }
  return 0;
}
int TwoWire::requestFrom(int a, int n) { int d = a == 0x48 ? 0 : 1; float v = adsV[d * 4 + adsSel[d]]; int16_t raw = (int16_t)lroundf(v / 0.000125f); rdBuf[0] = raw >> 8; rdBuf[1] = raw & 0xFF; readIdx = 0; return n; }
int TwoWire::read() { return rdBuf[readIdx++ & 1]; }
TwoWire Wire, Wire1;
#include "RadioLib.h"
#include "WiFi.h"
#include "HTTPClient.h"
#include "Update.h"
#include "WebServer.h"
#include "mbedtls/sha256.h"
#include "SPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
// фиктивные include-ы, нужные самому скетчу
#include "/home/claude/forester/receiver/receiver.ino"
