// MULTI-SAT TRACKER – ESP32 + ST7789 (320x240) + TFT_eSPI + Smooth Fonts
// Features:
//  - multi-satellite tracking (SGP4)
//  - TLE cache in SPIFFS
//  - WiFi STA, fallback AP (SAT_TRACKER / sat123456)
//  - Web config (QTH, satellites, Doppler, GPS, TZ, GPS-only offline mode, WiFi SSID/PASS)
//  - GPS (TinyGPSPlus): QTH + time, offline mode
//  - Configurable timezone (POSIX TZ strings via <select>)
//  - RX/TX frequencies for satellites + Doppler shift
//  - Cached pass track on radar

#define SMOOTH_FONT
#define LOAD_GFXFF

#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <FS.h>
#include <SPIFFS.h>
#include <TFT_eSPI.h>
#include <time.h>
#include <sys/time.h>
#include <Sgp4.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>
#include "logo.h"

// ====================== WIFI ======================
// Default WiFi credentials (fallback if not set in config)
const char* WIFI_SSID = "Vxxxx";
const char* WIFI_PASS = "xxxx";

// Configurable WiFi values (loaded from config / web)
char g_wifiSsid[33] = "";   // max 32 chars + '\0'
char g_wifiPass[65] = "";   // max 64 chars + '\0'

// ====================== TIMEZONE DEFAULT ======================
const char* TZ_EU_PRAGUE = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ====================== QTH ======================
double g_qthLat = 49.7501;
double g_qthLon = 13.3800;
double g_qthAlt = 310.0;
float  g_minElDeg = 10.0f;

// ====================== DOPPLER ======================
bool   g_dopplerEnabled = false;

// ====================== TIME / TZ FLAGS ======================
bool   g_haveTime = false;       // Do we have valid time (NTP or GPS)?
char  g_tz[64] = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ====================== GPS ======================
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

bool     g_gpsEnabled   = false;
bool     g_gpsOnlyMode  = false;  // "GPS only offline" – do not start STA WiFi / NTP
int      g_gpsRxPin     = 16;     // default RX
int      g_gpsTxPin     = 17;     // default TX
uint32_t g_gpsBaud      = 9600;   // default baud

bool     g_gpsHasFix    = false;
bool     g_gpsTimeSet   = false;

// Flag to avoid repeated pass recomputation based on GPS
bool     g_passesInitByGps = false;

// ====================== TFT & BACKLIGHT ======================
TFT_eSPI tft = TFT_eSPI();  // pins in User_Setup

#define TFT_BL 4
#define DARKGREY 0x7BEF

// Background color for boot screen (#63A6C9 → BGR = 0x6539)
uint16_t boot_bg_color = tft.color565(201, 166, 99);
#define BOOT_BG boot_bg_color

const int RADAR_CX = 240;
const int RADAR_CY = 120;
const int RADAR_R  = 60;

String g_ipStr;
bool   g_isAPMode = false;

// ====================== WEB SERVER ======================
WebServer server(80);

// ====================== SATELLITES ======================
struct SatConfig {
  const char* id;
  const char* shortName;
  const char* defaultName;
  const char* tleUrl;

  char  name[32];
  char  l1[80];
  char  l2[80];

  float rxFreqMHz;   // 0 = not defined
  float txFreqMHz;

  bool  enabled;
};

SatConfig g_sats[] = {
  {
    "ISS",
    "ISS",
    "ISS (ZARYA)",
    "https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=tle",
    "", "", "",
    145.800f,   // RX example
    437.800f,   // TX example
    true
  },
  {
    "SO50",
    "SO50",
    "SO-50",
    "https://celestrak.org/NORAD/elements/gp.php?NAME=SO-50&FORMAT=tle",
    "", "", "",
    436.795f,
    145.850f,
    false
  },
  {
    "FO29",
    "FO29",
    "FO-29",
    "https://celestrak.org/NORAD/elements/gp.php?NAME=FO-29&FORMAT=tle",
    "", "", "",
    435.850f,
    145.900f,
    false
  },
  {
    "AO91",
    "AO91",
    "AO-91",
    "https://celestrak.org/NORAD/elements/gp.php?NAME=AO-91&FORMAT=tle",
    "", "", "",
    145.960f,
    435.250f,
    false
  }
};

const int SAT_COUNT = sizeof(g_sats) / sizeof(g_sats[0]);
Sgp4 g_sgp4[SAT_COUNT];

// ====================== PASS / TRAIL ======================
struct SatState {
  float az;
  float el;
  float distKm;
  int   vis;
};

struct PassInfo {
  time_t  aos;
  time_t  los;
  time_t  tMax;
  float   maxEl;
  float   aosAz;
  float   maxAz;
  float   losAz;
  uint8_t satIdx;
};

const int MAX_PASSES = 32;
PassInfo g_passes[MAX_PASSES];
int g_passCount = 0;

int  g_lastPassListMinute = -1;

struct TrailPoint {
  int x;
  int y;
  bool valid;
};

const int TRAIL_LEN = 120;
TrailPoint g_trail[TRAIL_LEN];
int g_trailCount   = 0;
int g_trailPassIdx = -1;

// For Doppler – remember previous ranges
float g_prevDistKm[SAT_COUNT];
bool  g_prevDistValid[SAT_COUNT];

// ====================== DISPLAY MODE ======================
enum DisplayMode {
  MODE_LIST,
  MODE_TRACKER
};

DisplayMode g_displayMode = MODE_LIST;

// ====================== CONFIG FILE ======================
const char* PATH_CONFIG = "/config.txt";

// ====================== FONT HELPERS ======================
void useFontSmall() {
  tft.unloadFont();
  tft.loadFont("SansSerif-18");
}
void useFontMedium() {
  tft.unloadFont();
  tft.loadFont("NotoSansBold-20");
}
void useFontLarge() {
  tft.unloadFont();
  tft.loadFont("Orbitron-32");
}

// ====================== FS ======================
void setupFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS ERROR");
  } else {
    Serial.println("SPIFFS OK");
  }
}

String getFSInfoString() {
  int usedKB  = SPIFFS.usedBytes()  / 1024;
  int totalKB = SPIFFS.totalBytes() / 1024;
  char buf[32];
  snprintf(buf, sizeof(buf), "%d/%d kB", usedKB, totalKB);
  return String(buf);
}

// ===== TLE CACHE IN FS =====
const time_t TLE_MAX_AGE = 24 * 3600;

String tlePathForSat(const SatConfig &sc) {
  String path = "/tle_";
  path += sc.id;
  path += ".txt";
  return path;
}

bool saveTleToFs(const SatConfig &sc, time_t nowUtc) {
  String path = tlePathForSat(sc);
  File f = SPIFFS.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("TLE save FAIL (%s)\n", path.c_str());
    return false;
  }

  f.printf("%ld\n", (long)nowUtc);
  f.println(sc.name);
  f.println(sc.l1);
  f.println(sc.l2);
  f.close();

  Serial.printf("TLE saved to %s\n", path.c_str());
  return true;
}

bool loadTleFromFs(SatConfig &sc, time_t nowUtc) {
  String path = tlePathForSat(sc);
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) {
    Serial.printf("TLE FS: file not found (%s)\n", path.c_str());
    return false;
  }

  String line;

  line = f.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    Serial.printf("TLE FS: invalid timestamp (%s)\n", path.c_str());
    f.close();
    return false;
  }

  time_t ts = (time_t)line.toInt();
  if (ts <= 0) {
    Serial.printf("TLE FS: bad timestamp (%s)\n", path.c_str());
    f.close();
    return false;
  }

  if (nowUtc < ts || (nowUtc - ts) > TLE_MAX_AGE) {
    Serial.printf("TLE FS: too old or future (%s) now=%ld ts=%ld\n",
                  path.c_str(), (long)nowUtc, (long)ts);
    f.close();
    return false;
  }

  String name = f.readStringUntil('\n');
  name.trim();

  String l1 = f.readStringUntil('\n');
  l1.trim();

  String l2 = f.readStringUntil('\n');
  l2.trim();

  f.close();

  if (l1.length() < 10 || l2.length() < 10) {
    Serial.printf("TLE FS: invalid content (%s)\n", path.c_str());
    return false;
  }

  name.toCharArray(sc.name, sizeof(sc.name));
  l1.toCharArray(sc.l1, sizeof(sc.l1));
  l2.toCharArray(sc.l2, sizeof(sc.l2));

  Serial.printf("TLE loaded from FS: %s (%s)\n", sc.id, sc.name);
  return true;
}

// ===== CONFIG LOAD / SAVE =====
// Format:
// line1: lat lon alt minEl doppler gpsEnabled gpsOnly gpsRx gpsTx gpsBaud
// line2: IDs of enabled satellites (space separated)
// line3: TZ string (POSIX)
// line4: wifiSsid|wifiPass
void loadConfig() {
  File f = SPIFFS.open(PATH_CONFIG, FILE_READ);
  if (!f) {
    Serial.println("Config not found, using defaults.");
    strncpy(g_tz, TZ_EU_PRAGUE, sizeof(g_tz));
    return;
  }

  Serial.println("Loading config...");

  String line1 = f.readStringUntil('\n');
  line1.trim();
  if (line1.length() > 0) {
    double lat, lon, alt;
    float minEl;
    int doppler = 0, gpsEn = 0, gpsOnly = 0;
    int rx = g_gpsRxPin, tx = g_gpsTxPin;
    long baud = g_gpsBaud;

    int n = sscanf(
      line1.c_str(),
      "%lf %lf %lf %f %d %d %d %d %d %ld",
      &lat, &lon, &alt, &minEl,
      &doppler, &gpsEn, &gpsOnly,
      &rx, &tx, &baud
    );

    if (n >= 3) {
      g_qthLat = lat;
      g_qthLon = lon;
      g_qthAlt = alt;
      if (n >= 4) g_minElDeg      = minEl;
      if (n >= 5) g_dopplerEnabled = (doppler != 0);
      if (n >= 6) g_gpsEnabled     = (gpsEn   != 0);
      if (n >= 7) g_gpsOnlyMode    = (gpsOnly != 0);
      if (n >= 9) {
        g_gpsRxPin = rx;
        g_gpsTxPin = tx;
      }
      if (n >= 10) {
        g_gpsBaud = (baud > 0) ? (uint32_t)baud : 9600;
      }

      Serial.printf(
        "QTH from config: %.4f %.4f %.1f  minEl=%.1f doppler=%d gpsEn=%d gpsOnly=%d RX=%d TX=%d baud=%lu\n",
        g_qthLat, g_qthLon, g_qthAlt, g_minElDeg,
        g_dopplerEnabled, g_gpsEnabled, g_gpsOnlyMode,
        g_gpsRxPin, g_gpsTxPin, (unsigned long)g_gpsBaud
      );
    }
  }

  String line2 = f.readStringUntil('\n');
  line2.trim();

  for (int i = 0; i < SAT_COUNT; i++) {
    g_sats[i].enabled = false;
  }

  int start = 0;
  while (start < (int)line2.length()) {
    int sp = line2.indexOf(' ', start);
    if (sp < 0) sp = line2.length();
    String token = line2.substring(start, sp);
    token.trim();
    if (token.length() > 0) {
      for (int i = 0; i < SAT_COUNT; i++) {
        if (token.equalsIgnoreCase(g_sats[i].id)) {
          g_sats[i].enabled = true;
          break;
        }
      }
    }
    start = sp + 1;
  }

  // third line: TZ
  String line3 = f.readStringUntil('\n');
  line3.trim();
  if (line3.length() > 0) {
    line3.toCharArray(g_tz, sizeof(g_tz));
  } else {
    strncpy(g_tz, TZ_EU_PRAGUE, sizeof(g_tz));
  }

  // fourth line: WiFi SSID|PASS (optional for older configs)
  String line4 = f.readStringUntil('\n');
  line4.trim();
  if (line4.length() > 0) {
    int sep = line4.indexOf('|');
    if (sep >= 0) {
      String ssid = line4.substring(0, sep);
      String pass = line4.substring(sep + 1);
      ssid.toCharArray(g_wifiSsid, sizeof(g_wifiSsid));
      pass.toCharArray(g_wifiPass, sizeof(g_wifiPass));
    } else {
      // If no '|', treat whole line as SSID, password empty
      line4.toCharArray(g_wifiSsid, sizeof(g_wifiSsid));
      g_wifiPass[0] = '\0';
    }
  }

  f.close();
  Serial.printf("Config loaded. TZ=%s\n", g_tz);
}

void saveConfig() {
  File f = SPIFFS.open(PATH_CONFIG, FILE_WRITE);
  if (!f) {
    Serial.println("Config save FAILED");
    return;
  }

  f.printf(
    "%.6f %.6f %.3f %.1f %d %d %d %d %d %lu\n",
    g_qthLat, g_qthLon, g_qthAlt, g_minElDeg,
    g_dopplerEnabled ? 1 : 0,
    g_gpsEnabled ? 1 : 0,
    g_gpsOnlyMode ? 1 : 0,
    g_gpsRxPin, g_gpsTxPin, (unsigned long)g_gpsBaud
  );

  for (int i = 0; i < SAT_COUNT; i++) {
    if (g_sats[i].enabled) {
      f.print(g_sats[i].id);
      f.print(" ");
    }
  }
  f.print("\n");

  // line 3: TZ
  f.println(g_tz);

  // line 4: WiFi SSID|PASS
  f.print(g_wifiSsid);
  f.print("|");
  f.println(g_wifiPass);

  f.close();
  Serial.println("Config saved.");
}

// ====================== WIFI / AP ======================
bool connectWiFiStation() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(g_wifiSsid, g_wifiPass);

  Serial.printf("WiFi STA... SSID='%s'\n", g_wifiSsid);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    g_ipStr = WiFi.localIP().toString();
    return true;
  } else {
    Serial.println(" FAIL");
    return false;
  }
}

void startApMode() {
  Serial.println("[WiFi] Starting AP mode...");

  // Disable STA, clean start
  WiFi.disconnect(true, true);
  delay(100);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  // Static IP for AP – classic 192.168.4.1
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.softAPConfig(local_ip, gateway, subnet)) {
    Serial.println("[WiFi] softAPConfig FAILED");
  }

  // channel 1, visible SSID, max 4 clients
  bool ok = WiFi.softAP("SAT_TRACKER", "sat123456", 1, 0, 4);
  if (!ok) {
    Serial.println("[WiFi] softAP FAILED!");
  } else {
    Serial.println("[WiFi] softAP OK");
  }

  IPAddress ip = WiFi.softAPIP();
  g_ipStr = ip.toString();
  g_isAPMode = true;

  Serial.printf("[WiFi] AP MODE: SSID=SAT_TRACKER PASS=sat123456 IP=%s\n",
                g_ipStr.c_str());
}

// ====================== NTP (UTC + TZ) ======================
void setupTimeNTP() {
  setenv("TZ", g_tz, 1);
  tzset();

  configTime(0, 0, "pool.ntp.org");

  Serial.print("NTP sync");
  time_t now = 0;
  struct tm localInfo;
  const uint32_t TIMEOUT_MS = 30000;
  uint32_t startMs = millis();

  while (true) {
    now = time(nullptr);
    bool okLocal = getLocalTime(&localInfo, 500);

    if (now > 1672531200 && okLocal) { // > 2023-01-01
      Serial.println(" OK");
      g_haveTime = true;
      break;
    }

    Serial.print(".");
    delay(500);

    if (millis() - startMs > TIMEOUT_MS) {
      Serial.println(" TIMEOUT, continuing with current time (may be inaccurate)");
      break;
    }
  }

  if (g_haveTime) {
    tm tu;
    gmtime_r(&now, &tu);
    Serial.printf("UTC   : %02d:%02d:%02d\n", tu.tm_hour, tu.tm_min, tu.tm_sec);
    Serial.printf("Local : %02d:%02d:%02d\n", localInfo.tm_hour, localInfo.tm_min, localInfo.tm_sec);
  }
}

// ====================== GPS TIME SET ======================
void setSystemTimeFromGps() {
  if (!gps.time.isValid() || !gps.date.isValid()) return;

  // Temporarily switch TZ to UTC so mktime gives UTC
  char oldTz[64];
  strncpy(oldTz, g_tz, sizeof(oldTz));
  setenv("TZ", "UTC0", 1);
  tzset();

  struct tm t {};
  t.tm_year = gps.date.year() - 1900;
  t.tm_mon  = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min  = gps.time.minute();
  t.tm_sec  = gps.time.second();

  time_t utc = mktime(&t);

  struct timeval tv;
  tv.tv_sec = utc;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);

  // Restore TZ
  setenv("TZ", oldTz, 1);
  tzset();

  g_haveTime = true;
  g_gpsTimeSet = true;

  Serial.printf("Time set from GPS: %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                gps.date.year(), gps.date.month(), gps.date.day(),
                gps.time.hour(), gps.time.minute(), gps.time.second());
}

// ====================== BOOT SCREEN ======================
void splashStatus(const char *msg) {
  const int statusY = 200;
  const int statusH = 40;

  tft.fillRect(0, statusY, 320, statusH, BOOT_BG);
  useFontMedium();
  tft.setTextColor(TFT_LIGHTGREY, BOOT_BG);
  tft.setCursor(10, statusY + 10);
  tft.print(msg);

  Serial.println(msg);
}

void showBootLogo() {
  tft.fillScreen(BOOT_BG);

  useFontSmall();
  tft.setTextColor(TFT_WHITE, BOOT_BG);
  tft.setCursor(10, 10);
  tft.print("SAT TRACKER");

  int x = (tft.width()  - LOGO_W) / 2;
  int y = (tft.height() - LOGO_H) / 2;

  tft.pushImage(x, y, LOGO_W, LOGO_H, logo_map);

  splashStatus("Starting tracker...");
}

// ====================== TLE ======================
bool downloadTleForSat(SatConfig &sc, time_t nowUtc) {
  if (!sc.tleUrl || !strlen(sc.tleUrl)) return false;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  if (!http.begin(client, sc.tleUrl)) {
    Serial.printf("HTTP begin fail: %s\n", sc.id);
    return false;
  }
  int code = http.GET();
  if (code != 200) {
    Serial.printf("HTTP %s code=%d\n", sc.id, code);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  int lineNo = 0;
  String name, l1, l2;
  int idx = 0;
  while (idx < (int)payload.length() && lineNo < 3) {
    int nl = payload.indexOf('\n', idx);
    if (nl < 0) nl = payload.length();
    String line = payload.substring(idx, nl);
    line.trim();
    if (line.length() > 0) {
      if (lineNo == 0)      name = line;
      else if (lineNo == 1) l1   = line;
      else if (lineNo == 2) l2   = line;
      lineNo++;
    }
    idx = nl + 1;
  }

  if (lineNo < 3) {
    Serial.printf("TLE format fail: %s\n", sc.id);
    return false;
  }

  name.toCharArray(sc.name, sizeof(sc.name));
  l1.toCharArray(sc.l1, sizeof(sc.l1));
  l2.toCharArray(sc.l2, sizeof(sc.l2));

  Serial.printf("TLE OK (download): %s\n", sc.name);

  saveTleToFs(sc, nowUtc);
  return true;
}

bool ensureTleForSat(SatConfig &sc, time_t nowUtc) {
  if (loadTleFromFs(sc, nowUtc)) {
    return true;
  }

  Serial.printf("TLE FS not usable, downloading: %s\n", sc.id);

  // In GPS-only OFFLINE mode, do not attempt internet
  if (!g_gpsOnlyMode && !g_isAPMode) {
    if (downloadTleForSat(sc, nowUtc)) {
      return true;
    }
  }

  Serial.printf("TLE ensure FAILED: %s\n", sc.id);
  return false;
}

void initSatConfigs() {
  for (int i = 0; i < SAT_COUNT; i++) {
    strncpy(g_sats[i].name, g_sats[i].defaultName, sizeof(g_sats[i].name));
    g_sats[i].name[sizeof(g_sats[i].name) - 1] = '\0';
    g_sats[i].l1[0] = '\0';
    g_sats[i].l2[0] = '\0';
  }

  // fallback ISS TLE
  strncpy(g_sats[0].name, "ISS (ZARYA)", sizeof(g_sats[0].name));
  strncpy(g_sats[0].l1,
          "1 25544U 98067A   25321.51385417  .00013833  00000-0  24663-3 0  9999",
          sizeof(g_sats[0].l1));
  strncpy(g_sats[0].l2,
          "2 25544  51.6416 307.6127 0004374 279.5544  80.5053 15.50090446 99999",
          sizeof(g_sats[0].l2));

  time_t nowUtc = time(nullptr);
  for (int i = 0; i < SAT_COUNT; i++) {
    if (!ensureTleForSat(g_sats[i], nowUtc)) {
      if (i == 0) {
        Serial.println("Using built-in fallback TLE for ISS.");
      } else {
        Serial.printf("No valid TLE for %s, satellite disabled.\n", g_sats[i].id);
        g_sats[i].l1[0] = '\0';
        g_sats[i].l2[0] = '\0';
        g_sats[i].enabled = false;
      }
    }
  }

  for (int i = 0; i < SAT_COUNT; i++) {
    g_sgp4[i].site(g_qthLat, g_qthLon, g_qthAlt);
    if (strlen(g_sats[i].l1) > 10 && strlen(g_sats[i].l2) > 10) {
      g_sgp4[i].init(g_sats[i].name, g_sats[i].l1, g_sats[i].l2);
    } else if (i == 0) {
      g_sgp4[0].init(g_sats[0].name, g_sats[0].l1, g_sats[0].l2);
    }
  }

  for (int i = 0; i < SAT_COUNT; i++) {
    g_prevDistKm[i] = 0.0f;
    g_prevDistValid[i] = false;
  }
}

void updateSatSites() {
  for (int i = 0; i < SAT_COUNT; i++) {
    g_sgp4[i].site(g_qthLat, g_qthLon, g_qthAlt);
  }
}

// ====================== SAT / PASSES ======================
SatState computeSatellite(int satIdx, time_t utcNow) {
  SatState s{};
  if (satIdx < 0 || satIdx >= SAT_COUNT) return s;

  g_sgp4[satIdx].findsat((unsigned long)utcNow);

  s.az     = (float)g_sgp4[satIdx].satAz;
  s.el     = (float)g_sgp4[satIdx].satEl;
  s.distKm = (float)g_sgp4[satIdx].satDist;
  s.vis    = (int)  g_sgp4[satIdx].satVis;
  return s;
}

void sortPassesByAos() {
  for (int i = 0; i < g_passCount - 1; i++) {
    for (int j = i + 1; j < g_passCount; j++) {
      if (g_passes[j].aos < g_passes[i].aos) {
        PassInfo tmp = g_passes[i];
        g_passes[i]  = g_passes[j];
        g_passes[j]  = tmp;
      }
    }
  }
}

void refinePassMax(PassInfo &p) {
  time_t center = p.tMax;
  if (center == 0) return;

  time_t tStart = center - 120;
  time_t tEnd   = center + 120;

  if (tStart < p.aos) tStart = p.aos;
  if (tEnd   > p.los) tEnd   = p.los;
  if (tEnd <= tStart) return;

  float bestEl = p.maxEl;
  time_t bestT = p.tMax;
  float bestAz = p.maxAz;

  for (time_t t = tStart; t <= tEnd; t += 1) {
    SatState s = computeSatellite(p.satIdx, t);
    if (s.el > bestEl) {
      bestEl = s.el;
      bestT  = t;
      bestAz = s.az;
    }
  }

  p.maxEl = bestEl;
  p.tMax  = bestT;
  p.maxAz = bestAz;
}

void predictPasses(time_t startUtc) {
  g_passCount = 0;
  const time_t endUtc = startUtc + 24 * 3600;
  const time_t step   = 10;

  if (g_minElDeg < 0.0f) g_minElDeg = 0.0f;
  if (g_minElDeg > 90.0f) g_minElDeg = 90.0f;

  Serial.printf("Predicting passes from UTC=%ld, minEl=%.1f deg\n",
                (long)startUtc, g_minElDeg);

  for (int si = 0; si < SAT_COUNT; si++) {
    if (!g_sats[si].enabled) continue;
    if (strlen(g_sats[si].l1) < 10 || strlen(g_sats[si].l2) < 10) continue;

    bool   above          = false;
    time_t aos            = 0;
    float  aosAz          = 0.0f;
    time_t lastAboveTime  = 0;
    float  lastAboveAz    = 0.0f;
    float  maxEl          = -90.0f;
    time_t tMax           = 0;
    float  maxAz          = 0.0f;

    for (time_t t = startUtc; t < endUtc; t += step) {
      SatState s = computeSatellite(si, t);

      if (!above && s.el > g_minElDeg) {
        above         = true;
        aos           = t;
        aosAz         = s.az;
        lastAboveTime = t;
        lastAboveAz   = s.az;
        maxEl         = s.el;
        tMax          = t;
        maxAz         = s.az;
      } else if (above && s.el > g_minElDeg) {
        lastAboveTime = t;
        lastAboveAz   = s.az;
        if (s.el > maxEl) {
          maxEl = s.el;
          tMax  = t;
          maxAz = s.az;
        }
      } else if (above && s.el <= g_minElDeg) {
        if (g_passCount < MAX_PASSES && lastAboveTime > aos) {
          g_passes[g_passCount].aos   = aos;
          g_passes[g_passCount].los   = lastAboveTime;
          g_passes[g_passCount].tMax  = tMax;
          g_passes[g_passCount].maxEl = maxEl;
          g_passes[g_passCount].aosAz = aosAz;
          g_passes[g_passCount].maxAz = maxAz;
          g_passes[g_passCount].losAz = lastAboveAz;
          g_passes[g_passCount].satIdx = si;
          g_passCount++;
        }
        above = false;
      }
    }

    if (above && g_passCount < MAX_PASSES && lastAboveTime > aos) {
      g_passes[g_passCount].aos   = aos;
      g_passes[g_passCount].los   = lastAboveTime;
      g_passes[g_passCount].tMax  = tMax;
      g_passes[g_passCount].maxEl = maxEl;
      g_passes[g_passCount].aosAz = aosAz;
      g_passes[g_passCount].maxAz = maxAz;
      g_passes[g_passCount].losAz = lastAboveAz;
      g_passes[g_passCount].satIdx = si;
      g_passCount++;
    }
  }

  sortPassesByAos();

  for (int i = 0; i < g_passCount; i++) {
    refinePassMax(g_passes[i]);
  }

  Serial.printf("Passes predicted: %d\n", g_passCount);
}

// ====================== SERIAL DUMP ======================
const char* MONTH_ABBR[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

void dumpPassesToSerial() {
  Serial.println();
  Serial.printf("=== PREDICTED PASSES (next 24h, local time, el > %.1f°) ===\r\n", g_minElDeg);
  Serial.printf("QTH: %.4f N, %.4f E, alt=%.1f m\r\n", g_qthLat, g_qthLon, g_qthAlt);

  if (g_passCount == 0) {
    Serial.println(F("No passes."));
    Serial.println(F("=== END ==="));
    return;
  }

  Serial.println(F("Date    Start      Az   Max        Az   El    End        Az"));
  Serial.println(F("--------------------------------------------------------------"));

  for (int i = 0; i < g_passCount; i++) {
    const PassInfo &p = g_passes[i];

    tm a, m, l;
    localtime_r(&p.aos, &a);
    localtime_r(&p.tMax, &m);
    localtime_r(&p.los, &l);

    const char* mon = MONTH_ABBR[a.tm_mon];

    Serial.printf(
      "%02d-%s  %02d:%02d:%02d  %3.0f%c  %02d:%02d:%02d  %3.0f%c %5.1f%c  %02d:%02d:%02d  %3.0f%c\r\n",
      a.tm_mday, mon,
      a.tm_hour, a.tm_min, a.tm_sec,
      p.aosAz,  0xB0,
      m.tm_hour, m.tm_min, m.tm_sec,
      p.maxAz,  0xB0, p.maxEl, 0xB0,
      l.tm_hour, l.tm_min, l.tm_sec,
      p.losAz,  0xB0
    );
  }

  Serial.println(F("=== END ==="));
}

// ====================== SERIAL CMD ======================
void processCommand(const String &cmd) {
  if (cmd.equalsIgnoreCase("pocitej") || cmd.equalsIgnoreCase("recalc")) {
    Serial.println(F("\n[CMD] recalc – recalculating passes for next 24h (from now) and dumping to serial..."));

    time_t nowUtc = time(nullptr);
    time_t startUtc = nowUtc;

    predictPasses(startUtc);
    dumpPassesToSerial();
  } else {
    Serial.print(F("[CMD] Unknown command: "));
    Serial.println(cmd);
  }
}

// ====================== TRAIL ======================
void clearTrail() {
  for (auto &p : g_trail) p.valid = false;
  g_trailCount   = 0;
  g_trailPassIdx = -1;
}

void computePassTrack(int passIdx) {
  clearTrail();
  if (passIdx < 0 || passIdx >= g_passCount) return;

  const PassInfo &p = g_passes[passIdx];
  int si = p.satIdx;

  time_t aos = p.aos;
  time_t los = p.los;
  time_t dur = los - aos;
  if (dur <= 0) return;

  double step = dur / (double)TRAIL_LEN;
  if (step < 5) step = 5;

  int n = 0;
  for (int i = 0; i < TRAIL_LEN; i++) {
    time_t t = aos + (time_t)(i * step);
    if (t > los) break;

    SatState s = computeSatellite(si, t);
    double r  = constrain(90 - s.el, 0, 90);
    double az = radians(s.az);
    double kr = (r / 90.0) * RADAR_R;

    int x = RADAR_CX + (int)(kr * sin(az));
    int y = RADAR_CY - (int)(kr * cos(az));

    g_trail[i] = {x, y, true};
    n++;
  }
  g_trailCount   = n;
  g_trailPassIdx = passIdx;
}

void drawTrail() {
  tft.setTextFont(1);
  for (int i = 1; i < g_trailCount; i++) {
    if (g_trail[i-1].valid && g_trail[i].valid) {
      tft.drawLine(
        g_trail[i-1].x, g_trail[i-1].y,
        g_trail[i].x,    g_trail[i].y,
        TFT_YELLOW
      );
    }
  }
}

// ====================== DISPLAY BASE ======================
void drawRadarBase() {
  tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R, DARKGREY);
  tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R * 2 / 3, DARKGREY);
  tft.drawCircle(RADAR_CX, RADAR_CY, RADAR_R * 1 / 3, DARKGREY);

  useFontMedium();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setCursor(RADAR_CX - 6, RADAR_CY - RADAR_R - 14);
  tft.print("N");

  tft.setCursor(RADAR_CX + RADAR_R + 3, RADAR_CY - 6);
  tft.print("E");

  tft.setCursor(RADAR_CX - 6, RADAR_CY + RADAR_R + 2);
  tft.print("S");

  tft.setCursor(RADAR_CX - RADAR_R - 12, RADAR_CY - 6);
  tft.print("W");
}

void drawIpFsFooter() {
  useFontMedium();
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);

  tft.fillRect(0, 218, 320, 20, TFT_BLACK);
  tft.setCursor(5, 228);
  tft.printf("IP:%s  FS:%s",
             g_ipStr.c_str(),
             getFSInfoString().c_str());
}

// RX/TX line (white, under radar, above IP line)
void drawRxTxLine(int satIdx, float dopplerFactorRx, float dopplerFactorTx) {
  const int y = 200;
  tft.fillRect(0, y, 320, 18, TFT_BLACK);

  useFontMedium();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, y+2);

  float rxMHz = g_sats[satIdx].rxFreqMHz;
  float txMHz = g_sats[satIdx].txFreqMHz;

  if (rxMHz <= 0 && txMHz <= 0) {
    tft.print("RX/TX: undefined");
    return;
  }

  tft.print("RX:");
  if (rxMHz > 0) {
    double rxHz   = rxMHz * 1e6;
    double rxHzD  = g_dopplerEnabled ? rxHz * dopplerFactorRx : rxHz;
    double rxOutM = rxHzD / 1e6;
    tft.printf(" %.6f MHz", rxOutM);
  } else {
    tft.print(" -");
  }

  tft.print("  TX:");
  if (txMHz > 0) {
    double txHz   = txMHz * 1e6;
    double txHzD  = g_dopplerEnabled ? txHz * dopplerFactorTx : txHz;
    double txOutM = txHzD / 1e6;
    tft.printf(" %.6f MHz", txOutM);
  } else {
    tft.print(" -");
  }
}

void drawStaticFrame() {
  tft.fillScreen(TFT_BLACK);

  useFontSmall();
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("SAT TRACKER");

  drawRadarBase();
  drawIpFsFooter();
}

// AP MODE INFO SCREEN, when AP + GPS off
void drawApModeInfo() {
  tft.fillScreen(TFT_BLACK);
  useFontMedium();
  tft.setTextColor(TFT_CYAN, TFT_BLACK);

  tft.setCursor(10, 20);
  tft.print("AP MODE");

  tft.setCursor(10, 50);
  tft.print("SSID: SAT_TRACKER");

  tft.setCursor(10, 70);
  tft.print("PASS: sat123456");

  tft.setCursor(10, 100);
  tft.print("IP: ");
  tft.print(g_ipStr);

  tft.setCursor(10, 130);
  tft.print("Edit settings in browser");
}

// ====================== DRAW – PASS LIST ======================
void drawPassList(time_t nowUtc) {
  tft.fillRect(0, 50, 320, 150, TFT_BLACK);

  useFontMedium();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 60);
  tft.print("PASSES:");

  int y = 90;
  int shown = 0;

  for (int i = 0; i < g_passCount && shown < 7; i++) {
    const PassInfo &p = g_passes[i];

    if (p.los <= nowUtc) continue;

    tm a, l;
    localtime_r(&p.aos, &a);
    localtime_r(&p.los, &l);

    int si = p.satIdx;
    const char* label = g_sats[si].shortName;

    bool active = (nowUtc >= p.aos && nowUtc <= p.los);

    if (active) {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
    } else {
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }

    tft.setCursor(10, y);
    useFontMedium();

    char prefix = active ? '>' : ' ';
    tft.printf("%c%d) %s ", prefix, shown + 1, label);
    tft.printf("%02d.%02d ", a.tm_mday, a.tm_mon + 1);
    tft.printf("%02d:%02d-", a.tm_hour, a.tm_min);
    tft.printf("%02d:%02d ", l.tm_hour, l.tm_min);
    tft.printf("%2.0f", p.maxEl);
    tft.print("°");

    y += 13;
    shown++;
  }

  if (shown == 0) {
    tft.setCursor(10, y);
    useFontMedium();
    if (g_gpsEnabled && !g_gpsHasFix) {
      tft.print("Waiting for GPS...");
    } else {
      tft.print("No passes.");
    }
  }
}

// ====================== DOPPLER HELPER ======================
float dopplerFactorFromRangeRate(float rangeRateKmS) {
  const float C_KM_S = 299792.458f;
  return 1.0f - (rangeRateKmS / C_KM_S);
}

// ====================== DRAW – SATELLITE STATE ======================
// Uses local time (tmLocal) and prints it in medium font in info block
void drawSatState(int satIdx, const SatState& s, const tm& tmLocal, float rangeRateKmS) {
  tft.fillRect(50, 60, 140, 120, TFT_BLACK);

  useFontMedium();
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.setCursor(50, 60);
  tft.printf("%3.0f", s.az);
  tft.print("°");

  tft.setCursor(50, 80);
  tft.printf("%3.0f", s.el);
  tft.print("°");

  useFontMedium();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setCursor(50, 100);
  tft.printf("%.0f km", s.distKm);

  tft.setCursor(50, 120);
  if (s.vis == -2)      tft.print("BELOW");
  else if (s.vis == -1) tft.print("DAY");
  else if (s.vis == 0)  tft.print("DIM");
  else                  tft.print("BRIGHT");

  // Local time – medium font
  tft.fillRect(50, 140, 140, 20, TFT_BLACK);
  tft.setCursor(50, 140);
  tft.printf("%02d:%02d:%02d", tmLocal.tm_hour, tmLocal.tm_min, tmLocal.tm_sec);

  tft.fillRect(50, 160, 180, 20, TFT_BLACK);
  tft.setCursor(50, 160);
  if (g_gpsEnabled && !g_gpsHasFix) {
    tft.print("Waiting GPS...");
  } else {
    tft.printf("%.3fN %.3fE", g_qthLat, g_qthLon);
  }

  tft.fillRect(10, 35, 200, 20, TFT_BLACK);
  tft.setCursor(10, 35);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.print(g_sats[satIdx].name);

  tft.fillCircle(RADAR_CX, RADAR_CY, RADAR_R - 2, TFT_BLACK);
  drawRadarBase();
  drawTrail();

  if (s.el > g_minElDeg) {
    double r  = constrain(90 - s.el, 0, 90);
    double az = radians(s.az);
    double kr = (r / 90.0) * RADAR_R;
    int x = RADAR_CX + (int)(kr * sin(az));
    int y = RADAR_CY - (int)(kr * cos(az));
    tft.fillCircle(x, y, 5, TFT_GREEN);
  }

  float dopplerFactor = g_dopplerEnabled ? dopplerFactorFromRangeRate(rangeRateKmS) : 1.0f;
  drawRxTxLine(satIdx, dopplerFactor, dopplerFactor);

  drawIpFsFooter();
}

// ====================== FOOTER ======================
void drawFooter(const tm& tmLocal) {
  tft.fillRect(0, 195, 215, 35, TFT_BLACK);

  useFontLarge();
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(5, 196);

  char buf[16];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
           tmLocal.tm_hour, tmLocal.tm_min, tmLocal.tm_sec);
  tft.print(buf);
}

// ====================== GPS UPDATE ======================
void updateGps() {
  if (!g_gpsEnabled) return;

  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    g_qthLat = gps.location.lat();
    g_qthLon = gps.location.lng();
    if (gps.altitude.isValid()) {
      g_qthAlt = gps.altitude.meters();
    }
    g_gpsHasFix = true;
    updateSatSites();
  }

  if (!g_gpsTimeSet && gps.date.isValid() && gps.time.isValid()) {
    setSystemTimeFromGps();
  }
}

// ====================== WEB UI ======================
String htmlEscape(const String &s) {
  String out;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

// Small helper for <option>
void addTzOption(String &html, const char* value, const char* label) {
  html += "<option value='";
  html += htmlEscape(value);
  html += "'";
  if (strcmp(g_tz, value) == 0) {
    html += " selected";
  }
  html += ">";
  html += htmlEscape(label);
  html += "</option>";
}

void handleRoot() {
  String html = F(
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<title>Sat Tracker</title>"
    "<style>"
    "body{font-family:sans-serif;background:#111;color:#eee;margin:20px;}"
    "h1{color:#0ff;}"
    "label{display:inline-block;width:110px;}"
    ".box{border:1px solid #444;padding:10px;margin-bottom:15px;border-radius:6px;background:#181818;}"
    "input[type=text]{width:140px;background:#222;border:1px solid #555;color:#eee;padding:2px 4px;}"
    ".satlist label{width:auto;margin-right:10px;}"
    "button{padding:6px 12px;border-radius:4px;border:1px solid #0aa;background:#033;color:#0ff;cursor:pointer;}"
    "button:hover{background:#055;}"
    "select{background:#222;border:1px solid #555;color:#eee;padding:2px 4px;}"
    "</style>"
    "</head><body>"
    "<h1>Sat Tracker</h1>"
  );

  html += F("<div class='box'><h2>QTH &amp; Time</h2><form method='POST' action='/config'>");

  html += F("<label>Latitude:</label><input type='text' name='lat' value='");
  html += String(g_qthLat, 6);
  html += F("'><br>");

  html += F("<label>Longitude:</label><input type='text' name='lon' value='");
  html += String(g_qthLon, 6);
  html += F("'><br>");

  html += F("<label>Altitude:</label><input type='text' name='alt' value='");
  html += String(g_qthAlt, 1);
  html += F("'> m<br>");

  html += F("<label>Min. elev:</label><input type='text' name='minel' value='");
  html += String(g_minElDeg, 1);
  html += F("'> &deg;<br>");

  // WiFi STA settings
  html += F("<label>WiFi SSID:</label><input type='text' name='wifi_ssid' value='");
  html += htmlEscape(String(g_wifiSsid));
  html += F("'><br>");

  // For safety, do not prefill password – empty = keep old
  html += F("<label>WiFi password:</label><input type='text' name='wifi_pass' value=''><small> (leave empty to keep)</small><br>");

  // Timezone select
  html += F("<label>Timezone:</label><select name='tz'>");
  addTzOption(html, "UTC0", "UTC");
  addTzOption(html, "CET-1CEST,M3.5.0/2,M10.5.0/3", "Europe/Prague");
  addTzOption(html, "EET-2EEST,M3.5.0/3,M10.5.0/4", "Eastern Europe");
  addTzOption(html, "EST5EDT,M3.2.0/2,M11.1.0/2", "US East");
  addTzOption(html, "PST8PDT,M3.2.0/2,M11.1.0/2", "US West");
  html += F("</select><br>");

  // Doppler
  html += F("<label>Doppler:</label><input type='checkbox' name='doppler'");
  if (g_dopplerEnabled) html += F(" checked");
  html += F("> apply shift<br>");

  // GPS section
  html += F("</div><div class='box'><h2>GPS</h2>");

  html += F("<label>GPS enabled:</label><input type='checkbox' name='gps_en'");
  if (g_gpsEnabled) html += F(" checked");
  html += F("><br>");

  html += F("<label>GPS only:</label><input type='checkbox' name='gps_only'");
  if (g_gpsOnlyMode) html += F(" checked");
  html += F("> offline mode<br>");

  html += F("<label>GPS RX pin:</label><input type='text' name='gps_rx' value='");
  html += String(g_gpsRxPin);
  html += F("'><br>");

  html += F("<label>GPS TX pin:</label><input type='text' name='gps_tx' value='");
  html += String(g_gpsTxPin);
  html += F("'><br>");

  html += F("<label>GPS baud:</label><input type='text' name='gps_baud' value='");
  html += String((unsigned long)g_gpsBaud);
  html += F("'><br>");

  // GPS status
  html += F("<p>GPS status: ");
  if (!g_gpsEnabled) {
    html += F("disabled");
  } else if (g_gpsEnabled && !g_gpsHasFix) {
    html += F("waiting for fix...");
  } else {
    html += F("OK");
  }
  html += F("</p>");

  // Satellites
  html += F("</div><div class='box'><h2>Satellites</h2><div class='satlist'>");

  for (int i = 0; i < SAT_COUNT; i++) {
    html += "<label><input type='checkbox' name='sat_";
    html += g_sats[i].id;
    html += "'";

    if (g_sats[i].enabled) html += " checked";
    html += "> ";
    html += htmlEscape(g_sats[i].shortName);
    html += " (";
    html += htmlEscape(g_sats[i].defaultName);
    html += ")";

    html += " RX:";
    if (g_sats[i].rxFreqMHz > 0) html += String(g_sats[i].rxFreqMHz, 3);
    else html += "-";
    html += "MHz TX:";
    if (g_sats[i].txFreqMHz > 0) html += String(g_sats[i].txFreqMHz, 3);
    else html += "-";
    html += "MHz";

    html += "</label><br>";
  }

  html += F("</div></div><button type='submit'>Save &amp; recalculate</button></form>");

  // Info
  html += F("<div class='box'><h2>Info</h2>");
  html += F("Mode: ");
  if (g_isAPMode) html += F("AP");
  else html += F("STA");
  html += F("<br>AP SSID: SAT_TRACKER, PASS: sat123456<br>");
  html += F("Current IP: ");
  html += htmlEscape(g_ipStr);
  html += F("<br>FS usage: ");
  html += getFSInfoString();
  html += F("<br>Timezone: ");
  html += htmlEscape(String(g_tz));
  html += F("<br>WiFi STA SSID: ");
  html += htmlEscape(String(g_wifiSsid));
  html += F("</div>");

  html += F("</body></html>");

  server.send(200, "text/html", html);
}

void handleConfig() {
  if (server.hasArg("lat"))   g_qthLat   = server.arg("lat").toFloat();
  if (server.hasArg("lon"))   g_qthLon   = server.arg("lon").toFloat();
  if (server.hasArg("alt"))   g_qthAlt   = server.arg("alt").toFloat();
  if (server.hasArg("minel")) g_minElDeg = server.arg("minel").toFloat();

  // WiFi settings from page
  if (server.hasArg("wifi_ssid")) {
    String s = server.arg("wifi_ssid");
    s.trim();
    if (s.length() > 0) {
      s.toCharArray(g_wifiSsid, sizeof(g_wifiSsid));
    }
  }

  // Password – if non-empty, change; empty = keep old
  if (server.hasArg("wifi_pass")) {
    String p = server.arg("wifi_pass");
    p.trim();
    if (p.length() > 0) {
      p.toCharArray(g_wifiPass, sizeof(g_wifiPass));
    }
  }

  if (server.hasArg("tz")) {
    String tz = server.arg("tz");
    tz.trim();
    if (tz.length() > 0) {
      tz.toCharArray(g_tz, sizeof(g_tz));
      setenv("TZ", g_tz, 1);
      tzset();
    }
  }

  g_dopplerEnabled = server.hasArg("doppler");
  g_gpsEnabled     = server.hasArg("gps_en");
  g_gpsOnlyMode    = server.hasArg("gps_only");

  if (server.hasArg("gps_rx")) g_gpsRxPin = server.arg("gps_rx").toInt();
  if (server.hasArg("gps_tx")) g_gpsTxPin = server.arg("gps_tx").toInt();
  if (server.hasArg("gps_baud")) {
    uint32_t b = server.arg("gps_baud").toInt();
    if (b > 0) g_gpsBaud = b;
  }

  if (g_minElDeg < 0.0f) g_minElDeg = 0.0f;
  if (g_minElDeg > 90.0f) g_minElDeg = 90.0f;

  for (int i = 0; i < SAT_COUNT; i++) {
    String argName = String("sat_") + g_sats[i].id;
    g_sats[i].enabled = server.hasArg(argName);
  }

  saveConfig();
  updateSatSites();

  time_t nowUtc = time(nullptr);
  time_t startUtc = nowUtc;
  if (g_haveTime) {
    predictPasses(startUtc);
  }

  g_lastPassListMinute = -1;
  clearTrail();
  g_passesInitByGps = false;

  drawStaticFrame();

  server.sendHeader("Location", "/");
  server.send(303);
}

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  delay(200);

  setupFS();

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);

  showBootLogo();

  splashStatus("Loading configuration...");
  loadConfig();

  // If WiFi not set in config, use default constants
  if (g_wifiSsid[0] == '\0') {
    strncpy(g_wifiSsid, WIFI_SSID, sizeof(g_wifiSsid));
    g_wifiSsid[sizeof(g_wifiSsid) - 1] = '\0';
  }
  if (g_wifiPass[0] == '\0') {
    strncpy(g_wifiPass, WIFI_PASS, sizeof(g_wifiPass));
    g_wifiPass[sizeof(g_wifiPass) - 1] = '\0';
  }

  // Set TZ immediately
  setenv("TZ", g_tz, 1);
  tzset();

  // GPS port, if enabled
  if (g_gpsEnabled) {
    SerialGPS.begin(g_gpsBaud, SERIAL_8N1, g_gpsRxPin, g_gpsTxPin);
    Serial.printf("GPS enabled: RX=%d TX=%d baud=%lu\n",
                  g_gpsRxPin, g_gpsTxPin, (unsigned long)g_gpsBaud);
  }

  bool wifiOk = false;

  if (g_gpsOnlyMode) {
    // GPS-only OFFLINE: do not start STA, go directly to AP for config
    splashStatus("Starting AP (GPS only)...");
    startApMode();
  } else {
    splashStatus("Connecting WiFi (STA)...");
    wifiOk = connectWiFiStation();
    if (!wifiOk) {
      splashStatus("STA failed, starting AP...");
      startApMode();
    }
  }

  if (wifiOk && !g_gpsOnlyMode) {
    splashStatus("Syncing time (NTP)...");
    setupTimeNTP();
  } else {
    g_haveTime = false; // Will possibly wait for GPS
  }

  splashStatus("Initializing TLE and satellites...");
  initSatConfigs();

  if (g_haveTime) {
    splashStatus("Computing passes...");
    time_t nowUtc   = time(nullptr);
    predictPasses(nowUtc);
  } else {
    splashStatus("Waiting for time (NTP/GPS)...");
  }

  splashStatus("Starting web server...");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_POST, handleConfig);
  server.begin();

  splashStatus("Done.");
  delay(800);

  // If AP + GPS off -> static AP info
  if (g_isAPMode && !g_gpsEnabled) {
    drawApModeInfo();
  } else {
    drawStaticFrame();
  }

  g_lastPassListMinute = -1;
  clearTrail();
  g_displayMode = MODE_LIST;
  g_passesInitByGps = false;

  Serial.println("HTTP server ready.");
}

// ====================== LOOP ======================
void loop() {
  server.handleClient();

  // GPS handling
  updateGps();

  // If we don't have time from NTP and GPS time just arrived -> recompute passes
  if (!g_haveTime && g_gpsEnabled && g_gpsTimeSet && !g_passesInitByGps) {
    time_t nowUtc = time(nullptr);
    predictPasses(nowUtc);
    g_passesInitByGps = true;
  }

  static String cmdBuf;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      cmdBuf.trim();
      if (cmdBuf.length() > 0) {
        processCommand(cmdBuf);
      }
      cmdBuf = "";
    } else {
      if (cmdBuf.length() < 64) {
        cmdBuf += c;
      }
    }
  }

  static unsigned long last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  // AP + GPS off: only static info screen
  if (g_isAPMode && !g_gpsEnabled) {
    // we might still update time in background if available, but leave display as is
    return;
  }

  time_t nowUtc = time(nullptr);

  tm tmLocal{};
  getLocalTime(&tmLocal);

  int active = -1;
  for (int i = 0; i < g_passCount; i++) {
    if (nowUtc >= g_passes[i].aos && nowUtc <= g_passes[i].los) {
      active = i;
      break;
    }
  }

  DisplayMode newMode = (active < 0) ? MODE_LIST : MODE_TRACKER;

  if (newMode != g_displayMode) {
    g_displayMode = newMode;
    drawStaticFrame();
    g_lastPassListMinute = -1;
    clearTrail();
  }

  if (g_displayMode == MODE_LIST) {
    if (g_lastPassListMinute != tmLocal.tm_min) {
      drawPassList(nowUtc);
      g_lastPassListMinute = tmLocal.tm_min;
    }
  } else {
    if (active >= 0) {
      if (g_trailPassIdx != active) {
        computePassTrack(active);
      }

      int si = g_passes[active].satIdx;
      SatState s = computeSatellite(si, nowUtc);

      float rangeRateKmS = 0.0f;
      if (g_prevDistValid[si]) {
        rangeRateKmS = s.distKm - g_prevDistKm[si]; // dt ~= 1s
      }
      g_prevDistKm[si]    = s.distKm;
      g_prevDistValid[si] = true;

      drawSatState(si, s, tmLocal, rangeRateKmS);
    }
  }

  // Big time at the bottom only in PASS LIST mode, not in TRACKER mode
  if (g_displayMode == MODE_LIST) {
    drawFooter(tmLocal);
  }
}
