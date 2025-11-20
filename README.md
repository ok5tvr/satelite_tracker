# Multi-Sat Tracker – ESP32 + ST7789 + TFT_eSPI + SGP4

This project displays passes of multiple satellites (e.g. ISS, SO-50, FO-29, AO-91) over a given QTH.
It uses an ESP32, a 320×240 TFT (ST7789 with TFT_eSPI), NTP time, and an SGP4 propagator.

Main features

Satellite pass prediction for the next 24 hours (SGP4).

On-screen pass list (AOS/LOS time, max elevation, active pass highlighted).

Graphical “radar” view with satellite position and pass trail.

Automatic TLE download from CelesTrak and caching in SPIFFS (valid for 24 h).

Embedded web interface (ESP32 WebServer):

QTH configuration (lat, lon, alt, min elevation),

enable/disable individual satellites,

IP and SPIFFS usage info.

Configuration saved to /config.txt in SPIFFS.

Simple serial command pocitej to force recomputation of passes.

Hardware

ESP32 DevKit (WiFi, SPIFFS).

2.0–2.4" TFT display ST7789, 320×240, wired as configured in TFT_eSPI User_Setup.

Display backlight on TFT_BL pin (GPIO 4 in the code).

Optional: USB / 5 V power or external PSU.

Dependencies (Arduino)

ESP32 Arduino core

TFT_eSPI (enable SMOOTH_FONT, LOAD_GFXFF and set proper pins in User_Setup.h)

Sgp4 (Arduino SGP4 library)

WiFi, WiFiClientSecure, HTTPClient, WebServer

SPIFFS / FS, SPI

Quick start

Clone / copy the project into your Arduino / PlatformIO workspace.

Configure TFT_eSPI pins for your ST7789 in User_Setup.

Make sure SPIFFS contains the required smooth fonts (SansSerif-18, NotoSansBold-20, Orbitron-32).

Edit Wi-Fi credentials in the code:

const char* WIFI_SSID = "VAŠ_SSID";
const char* WIFI_PASS = "VAŠE_HESLO";


Build and upload to the ESP32.

On boot, the device connects to Wi-Fi, syncs NTP time and starts the webserver.

Web UI

Get the IP address from:

the bottom line of the TFT (IP:...), or

the serial monitor.

Open http://<IP_ADDRESS> in your browser:

set QTH (Lat, Lon, Alt, MinEl),

choose satellites (checkboxes),

save via “Ulozit a prepocitat” (Save & recalc).

# Multi‑Sat Tracker (ESP32 + ST7789)

Krátký popis
-------------
Jednoduchý tracker pro sledování satelitů na ESP32 s displejem ST7789, knihovnami TFT_eSPI a Sgp4. Zobrazuje polohu, průběh a další informace na TFT displeji.

Hlavní vlastnosti
-----------------
- Zobrazení dat o satelitech (TLE/Sgp4)
- Grafické rozhraní na ST7789 (320×240)
- Síťové funkce: Wi‑Fi, NTP
- Možnost aktualizace firmware přes PlatformIO / USB

Požadavky
---------
- ESP32 (board kompatibilní s PlatformIO)
- TFT ST7789 (nastaveno v User_Setup pro TFT_eSPI)
- PlatformIO (VS Code) nebo Arduino/PlatformIO CLI
- Python/utility pro případné konverze obrázků (volitelné)

Konfigurace
-----------
1. Otevřete `src/main.cpp`.
2. Upravte Wi‑Fi nastavení:
```cpp
const char* WIFI_SSID = "VAŠ_SSID";
const char* WIFI_PASS = "VAŠE_HESLO";