SAT Tracker – ESP32 Multi-Satellite Tracker (English)

1. Overview
-----------
This project is a multi-satellite tracker running on an ESP32 with a 320×240 ST7789 TFT (via TFT_eSPI) and an optional GPS module.

It uses the SGP4 model to track amateur radio satellites (e.g. ISS, SO-50, FO-29, AO-91), caches TLEs in SPIFFS, and shows:
- upcoming passes,
- current satellite position on a radar-style display,
- RX/TX frequencies including Doppler shift.

Configuration is done via a built-in web interface, including:
- QTH (lat/lon/alt, min elevation),
- timezone (POSIX TZ string),
- GPS options,
- Doppler,
- enabled satellites,
- Wi-Fi STA SSID/password.

If STA Wi-Fi is not available, the ESP32 starts its own AP (SAT_TRACKER / sat123456) for configuration.

2. Features
-----------
- Multi-satellite tracking (SGP4) for ISS, SO-50, FO-29, AO-91 (configurable).
- 24h pass prediction with minimum elevation filter.
- TLE handling:
  * Fetch TLEs from Celestrak (HTTPS).
  * Cache TLEs in SPIFFS (/tle_<ID>.txt) with max age 24 h.
  * Built-in fallback TLE for ISS.
  * Offline mode using cached TLEs only.
- Time sources:
  * NTP (if Wi-Fi STA is connected).
  * GPS (TinyGPSPlus), including GPS-only offline mode.
  * Local time via POSIX TZ string.
- Wi-Fi:
  * STA mode with configurable SSID/password (via web, stored in SPIFFS).
  * AP fallback: SAT_TRACKER / sat123456, IP 192.168.4.1.
- Web configuration:
  * QTH, min elevation.
  * Timezone (select).
  * Doppler on/off.
  * GPS enable / GPS-only / pins / baudrate.
  * Enabled satellites.
  * Wi-Fi STA SSID & password.
  * FS usage, IP, mode info.
- Display (ST7789 320×240, TFT_eSPI, smooth fonts):
  * LIST mode – when no active pass.
  * TRACKER mode – when at least one satellite is above min elevation.
  * Radar plot with N/E/S/W and ground track trail.
  * Satellite state (Az, El, distance, visual status, local time, QTH).
  * RX/TX line with Doppler.
  * Big local time in footer (LIST mode only).
- Storage with SPIFFS for config and TLE cache.

3. Hardware Requirements
------------------------
- ESP32 dev board.
- ST7789 320×240 TFT wired for TFT_eSPI (rotation = 1).
- TFT backlight on pin TFT_BL (GPIO 4 by default).
- Optional GPS module (NMEA, 9600 baud) on UART1:
  * RX pin default: GPIO 16.
  * TX pin default: GPIO 17.
- Power via USB or external 5 V (depending on board).

Fonts required in SPIFFS:
- SansSerif-18
- NotoSansBold-20
- Orbitron-32

4. /config.txt Format
---------------------
The configuration is stored in SPIFFS as /config.txt with the following lines:

1) QTH & GPS:
   lat lon alt minEl doppler gpsEnabled gpsOnly gpsRx gpsTx gpsBaud

2) Enabled satellites (space separated IDs):
   e.g. ISS SO50 FO29 AO91

3) Timezone (POSIX TZ string):
   e.g. CET-1CEST,M3.5.0/2,M10.5.0/3

4) Wi-Fi STA credentials:
   wifiSsid|wifiPass

If the file is missing, defaults (QTH, TZ, Wi-Fi) from the firmware are used.

5. TLE Cache
------------
Each satellite has a TLE cache file:

   /tle_<ID>.txt

Containing:
1) Download timestamp (UNIX time, UTC).
2) Satellite name.
3) TLE line 1.
4) TLE line 2.

If older than 24 hours, the firmware tries to refresh them (unless in GPS-only offline mode or AP config mode).

6. Web Interface – How To Use (EN)
----------------------------------
Access:

- STA mode:
  * ESP32 connects to configured Wi-Fi.
  * Open http://<device_IP>/ in your browser.

- AP mode:
  * SSID: SAT_TRACKER
  * Password: sat123456
  * IP: 192.168.4.1
  * Open http://192.168.4.1/ in your browser.

Sections:

1) QTH & Time
   - Latitude / Longitude / Altitude: your station coordinates.
   - Min. elev: minimum elevation in degrees.
   - WiFi SSID / WiFi password:
     * STA credentials; empty password field = keep existing password.
   - Timezone: POSIX TZ selection.
   - Doppler: enable or disable Doppler correction.

2) GPS
   - GPS enabled: use GPS for position and time.
   - GPS only (offline): no STA Wi-Fi/NTP, use GPS+cache only.
   - GPS RX/TX pins and baudrate.
   - GPS status: disabled / waiting for fix... / OK.

3) Satellites
   - Enable/disable individual satellites.
   - Shows their base RX/TX frequencies (MHz).

4) Info
   - Mode: AP or STA.
   - AP SSID/PASS.
   - Current IP address.
   - FS usage.
   - Timezone string.
   - Current Wi-Fi STA SSID.

Button:
- “Save & recalculate”:
  * Saves /config.txt.
  * Updates SGP4 site settings.
  * Recomputes passes if time is available.

7. Display Modes (EN)
---------------------
PASS LIST mode:
- Shows upcoming passes with date, time and max elevation.
- Active pass is highlighted in green with “>”.
- If no passes:
  * “Waiting for GPS...” if GPS enabled but no fix.
  * “No passes.” otherwise.
- Footer: big local time HH:MM:SS.

TRACKER mode:
- Active when a satellite pass is currently ongoing (between AOS and LOS).
- Radar view with N/E/S/W, track trail and current satellite dot.
- Info block:
  * Azimuth, Elevation, Distance, Visual status (BELOW/DAY/DIM/BRIGHT).
  * Local time (medium font).
  * QTH or “Waiting GPS...”.
- RX/TX line with Doppler-corrected frequencies.
- IP/FS footer at bottom.
- Big footer time is not drawn in this mode.

AP mode screen:
- If in AP mode and GPS is off:
  * Shows SSID/PASS, IP, and “Edit settings in browser”.

8. Typical First-Time Setup (EN)
--------------------------------
1) Flash firmware to the ESP32.
2) Open serial monitor at 115200 baud.
3) Let the device boot; if STA fails, it starts AP.
4) Connect to AP SAT_TRACKER if needed.
5) Open http://192.168.4.1/ in your browser.
6) Configure Wi-Fi SSID/PASS, QTH, timezone, satellites, GPS.
7) Save & recalculate.
8) On next boot, the tracker should connect via STA, sync time (NTP or GPS),
   update TLEs and start tracking with automatic mode switching.

