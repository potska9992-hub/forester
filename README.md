# FORESTER Remote

Беспроводной пульт для 8-секционного гидрораспределителя (BLB BC60/8 HE).

- Пульт: Heltec Wireless Stick Lite V3 (ESP32-S3 + SX1262), LoRa 868 МГц, OLED SH1106, MCP23017, 2x ADS1115.
- Приёмник: Heltec Wireless Stick Lite V3, PCA9685, драйверы BTS7960, ADS1115, реле питания драйверов.

Папки:

- `pult/` прошивка пульта (`pult.ino`, список библиотек `libs.txt`)
- `receiver/` прошивка приёмника (позже)
- `hwtest/` проверочная прошивка приёмника: проверка всех узлов и калибровка датчиков тока (ставится первой на каждое изделие, заливается с ПК, на GitHub не собирается; калибровка хранится в NVS и читается прошивкой приёмника; описание в `hwtest/README.md`)
- `docs/` концепт и протокол связи

## Сборка .bin

Actions -> Build firmware -> Run workflow -> выбрать pult или receiver (hwtest на GitHub не собирается, его заливают с ПК).

- Готовый файл: раздел Releases (`pult-latest`, `receiver-latest`): `pult_<версия>.bin` для обновления по Wi-Fi, `..._full.bin` для заливки по USB.
- Сводка по памяти: страница запуска.
- Полный лог: ветка `build-logs` (`pult_build.log`, `pult_summary.md`).

Версия ядра ESP32 и библиотек зафиксированы (core 3.3.11, плата `heltec_wireless_stick_lite_V3`).
