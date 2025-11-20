# Multi‑SAT Tracker (ESP32 + ST7789)

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
```
3. Upravit GPS/QTH parametry podle potřeby (soubory v `src/`).

Sestavení a nahrání (Windows PowerShell)
----------------------------------------
```powershell
cd "C:\Users\vlast\OneDrive\Documents\PlatformIO\Projects\satelite"
# Sestavit
pio run
# Nahrát do zařízení (připojeného přes USB)
pio run -t upload
```
Ve VS Code použijte PlatformIO panel -> Upload.

Poznámky k repozitáři
---------------------
- Přidejte soubory TLE nebo obrázky do repozitáře podle potřeby.
- V `.gitignore` jsou vyloučeny build složky (.pio, .vscode apod.).

Licence
-------
Uvést licenci projektu (např. MIT) nebo vlastní text.

Kontakt / Autor
---------------
OK5TVR
```// filepath: c:\Users\vlast\OneDrive\Documents\PlatformIO\Projects\satelite\README.md
# Multi‑SAT Tracker (ESP32 + ST7789)

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
```
3. Upravit GPS/QTH parametry podle potřeby (soubory v `src/`).
