// MULTI-SAT TRACKER – ESP32 + ST7789 (320x240) + TFT_eSPI + Smooth Fonts

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
#include "logo.h"

// ====================== WIFI ======================
const char* WIFI_SSID = "xxx";
const char* WIFI_PASS = "xxxgit";

// ====================== NTP & TIMEZONE ======================
const char* NTP_SERVER = "pool.ntp.org";
const char* TZ_EU_PRAGUE = "CET-1CEST,M3.5.0/2,M10.5.0/3";

// ====================== QTH ======================
double g_qthLat = 49.7501;
double g_qthLon = 13.3800;
double g_qthAlt = 310.0;
float g_minElDeg = 10.0;

// ====================== TFT & BACKLIGHT ======================
TFT_eSPI tft = TFT_eSPI();  // piny v User_Setup

#define TFT_BL 4
#define DARKGREY 0x7BEF

// Barva pozadí pro boot screen (#63A6C9 → BGR = 0x6539)
uint16_t boot_bg_color = tft.color565(201, 166, 99);
#define BOOT_BG boot_bg_color

const int RADAR_CX = 240;
const int RADAR_CY = 120;
const int RADAR_R  = 60;

String g_ipStr;

// ====================== WEB SERVER ======================
WebServer server(80);

// ====================== SATELITY ======================
struct SatConfig {
  const char* id;
  const char* shortName;
  const char* defaultName;
  const char* tleUrl;
  char name[32];
  char l1[80];
  char l2[80];
  bool enabled;
};

SatConfig g_sats[] = {
  {
    "ISS",
    "ISS",
    "ISS (ZARYA)",
    "https://celestrak.org/NORAD/elements/gp.php?CATNR=25544&FORMAT=tle",
    "", "", "", true
  },
  {
    "SO50",
    "SO50",
    "SO-50",
    "https://celestrak.org/NORAD/elements/gp.php?NAME=SO-50&FORMAT=tle",
    "", "", "", false
  },
  {
    "FO29",
    "FO29",
    "FO-29",
    "https://celestrak.org/NORAD/elements/gp.php?NAME=FO-29&FORMAT=tle",
    "", "", "", false
  },
  {
    "AO91",
    "AO91",
    "AO-91",
    "https://celestrak.org/NORAD/elements/gp.php?NAME=AO-91&FORMAT=tle",
    "", "", "", false
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

// ===== TLE CACHE VE FS =====

// Max stáří TLE v sekundách (24h)
const time_t TLE_MAX_AGE = 24 * 3600;

// Vygeneruje cestu k TLE souboru pro daný satelit
String tlePathForSat(const SatConfig &sc) {
  String path = "/tle_";
  path += sc.id;
  path += ".txt";
  return path;
}

// Uložení TLE do FS: 1. řádek = Unix timestamp, 2.=name, 3.=L1, 4.=L2
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

// Načtení TLE z FS, jen pokud není starší než 24h
bool loadTleFromFs(SatConfig &sc, time_t nowUtc) {
  String path = tlePathForSat(sc);
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) {
    Serial.printf("TLE FS: file not found (%s)\n", path.c_str());
    return false;
  }

  String line;

  // 1) timestamp
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

  // zkontroluj stáří, a i případný nesmysl (čas v budoucnosti)
  if (nowUtc < ts || (nowUtc - ts) > TLE_MAX_AGE) {
    Serial.printf("TLE FS: too old or future (%s) now=%ld ts=%ld\n",
                  path.c_str(), (long)nowUtc, (long)ts);
    f.close();
    return false;
  }

  // 2) name
  String name = f.readStringUntil('\n');
  name.trim();

  // 3) L1
  String l1 = f.readStringUntil('\n');
  l1.trim();

  // 4) L2
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
void loadConfig() {
  File f = SPIFFS.open(PATH_CONFIG, FILE_READ);
  if (!f) {
    Serial.println("Config not found, using defaults.");
    return;
  }

  Serial.println("Loading config...");

  String line1 = f.readStringUntil('\n');
  line1.trim();
  if (line1.length() > 0) {
    double lat, lon, alt;
    float minEl;
    int n = sscanf(line1.c_str(), "%lf %lf %lf %f", &lat, &lon, &alt, &minEl);
    if (n >= 3) {
      g_qthLat = lat;
      g_qthLon = lon;
      g_qthAlt = alt;
      if (n == 4) {
        g_minElDeg = minEl;
      }
      Serial.printf("QTH from config: %.4f %.4f %.1f  minEl=%.1f\n",
                    g_qthLat, g_qthLon, g_qthAlt, g_minElDeg);
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

  f.close();
  Serial.println("Config loaded.");
}

void saveConfig() {
  File f = SPIFFS.open(PATH_CONFIG, FILE_WRITE);
  if (!f) {
    Serial.println("Config save FAILED");
    return;
  }

  f.printf("%.6f %.6f %.3f %.1f\n", g_qthLat, g_qthLon, g_qthAlt, g_minElDeg);

  for (int i = 0; i < SAT_COUNT; i++) {
    if (g_sats[i].enabled) {
      f.print(g_sats[i].id);
      f.print(" ");
    }
  }
  f.print("\n");

  f.close();
  Serial.println("Config saved.");
}

// ====================== WIFI ======================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println(" OK");

  g_ipStr = WiFi.localIP().toString();
}

// ====================== NTP (UTC + TZ) ======================
void setupTime() {
  configTime(0, 0, NTP_SERVER);
  setenv("TZ", TZ_EU_PRAGUE, 1);
  tzset();

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
      break;
    }

    Serial.print(".");
    delay(500);

    if (millis() - startMs > TIMEOUT_MS) {
      Serial.println(" TIMEOUT, pokracuju s aktualnim casem (může být nepřesný)");
      break;
    }
  }

  tm tu;
  gmtime_r(&now, &tu);
  Serial.printf("UTC   : %02d:%02d:%02d\n", tu.tm_hour, tu.tm_min, tu.tm_sec);
  Serial.printf("Local : %02d:%02d:%02d\n", localInfo.tm_hour, localInfo.tm_min, localInfo.tm_sec);
}

// ====================== BMP HELPERS (splash logo) ======================



// ====================== BOOT SCREEN ======================

// Stavová hláška POD logem, světle šedá, useFontMedium()
void splashStatus(const char *msg) {
  // Logo je 159x145 uprostřed: horní okraj cca y=47, dolní cca y=192
  // Pod logem necháme pruh ~40 px výšky
  const int statusY = 200;
  const int statusH = 40;

  tft.fillRect(0, statusY, 320, statusH, BOOT_BG);
  useFontMedium();
  tft.setTextColor(TFT_LIGHTGREY, BOOT_BG);
  tft.setCursor(10, statusY + 10);
  tft.print(msg);

  Serial.println(msg);
}

// Zobrazení loga při startu
void showBootLogo() {
  tft.fillScreen(BOOT_BG);

  useFontSmall();
  tft.setTextColor(TFT_WHITE, BOOT_BG);
  tft.setCursor(10, 10);
  tft.print("SAT TRACKER");

  // Výpočet středu
  int x = (tft.width()  - LOGO_W) / 2;
  int y = (tft.height() - LOGO_H) / 2;

  // Vykreslení loga z paměti (RGB565)
  tft.pushImage(x, y, LOGO_W, LOGO_H, logo_map);

  splashStatus("Startuji tracker...");
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

  // uložit do FS s časovou známkou
  saveTleToFs(sc, nowUtc);

  return true;
}

// Zajistí platné TLE pro satelit:
// 1) pokus o načtení z FS (pokud <24h)
// 2) pokud selže, stáhne z internetu a uloží
bool ensureTleForSat(SatConfig &sc, time_t nowUtc) {
  // Nejdřív zkus FS cache
  if (loadTleFromFs(sc, nowUtc)) {
    return true;
  }

  Serial.printf("TLE FS not usable, downloading: %s\n", sc.id);

  // Pokud FS selže, stáhni z webu
  if (downloadTleForSat(sc, nowUtc)) {
    return true;
  }

  Serial.printf("TLE ensure FAILED: %s\n", sc.id);
  return false;
}

void initSatConfigs() {
  // základní reset jmen a TLE
  for (int i = 0; i < SAT_COUNT; i++) {
    strncpy(g_sats[i].name, g_sats[i].defaultName, sizeof(g_sats[i].name));
    g_sats[i].name[sizeof(g_sats[i].name) - 1] = '\0';
    g_sats[i].l1[0] = '\0';
    g_sats[i].l2[0] = '\0';
  }

  // fallback ISS TLE – použije se, když selže vše ostatní
  strncpy(g_sats[0].name, "ISS (ZARYA)", sizeof(g_sats[0].name));
  strncpy(g_sats[0].l1,
          "1 25544U 98067A   25321.51385417  .00013833  00000-0  24663-3 0  9999",
          sizeof(g_sats[0].l1));
  strncpy(g_sats[0].l2,
          "2 25544  51.6416 307.6127 0004374 279.5544  80.5053 15.50090446 99999",
          sizeof(g_sats[0].l2));

  time_t nowUtc = time(nullptr);

  // Pro každý satelit: nejdřív FS, pak download, jinak fallback (u ISS)
  for (int i = 0; i < SAT_COUNT; i++) {
    if (!ensureTleForSat(g_sats[i], nowUtc)) {
      // Pokud je to ISS a nepodařilo se nic, nech fallback
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

  // Inicializace SGP4
  for (int i = 0; i < SAT_COUNT; i++) {
    g_sgp4[i].site(g_qthLat, g_qthLon, g_qthAlt);
    if (strlen(g_sats[i].l1) > 10 && strlen(g_sats[i].l2) > 10) {
      g_sgp4[i].init(g_sats[i].name, g_sats[i].l1, g_sats[i].l2);
    } else if (i == 0) {
      // ISS fallback
      g_sgp4[0].init(g_sats[0].name, g_sats[0].l1, g_sats[0].l2);
    }
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
  if (cmd.equalsIgnoreCase("pocitej")) {
    Serial.println(F("\n[CMD] pocitej – recalculating passes for next 24h (from now) and dumping to serial..."));

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

void drawStaticFrame() {
  tft.fillScreen(TFT_BLACK);

  useFontSmall();
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("SAT TRACKER");

  drawRadarBase();
  drawIpFsFooter();
}

// ====================== DRAW – PASS LIST ======================
void drawPassList(time_t nowUtc) {
  tft.fillRect(0, 50, 320, 150, TFT_BLACK);

  useFontMedium();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 60);
  tft.print("PRULETY:");

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
    tft.print("Zadne prulety.");
  }
}

// ====================== DRAW – SATELLITE STATE ======================
void drawSatState(int satIdx, const SatState& s, const tm& tmUtc) {
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

  tft.fillRect(50, 140, 140, 20, TFT_BLACK);
  tft.setCursor(50, 140);
  tft.printf("%02d:%02d:%02dZ", tmUtc.tm_hour, tmUtc.tm_min, tmUtc.tm_sec);

  tft.fillRect(50, 160, 180, 20, TFT_BLACK);
  tft.setCursor(50, 160);
  tft.printf("%.3fN %.3fE", g_qthLat, g_qthLon);

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

void handleRoot() {
  String html = F(
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<title>Sat Tracker</title>"
    "<style>"
    "body{font-family:sans-serif;background:#111;color:#eee;margin:20px;}"
    "h1{color:#0ff;}"
    "label{display:inline-block;width:80px;}"
    ".box{border:1px solid #444;padding:10px;margin-bottom:15px;border-radius:6px;}"
    "input[type=text]{width:100px;}"
    ".satlist label{width:auto;margin-right:10px;}"
    "button{padding:6px 12px;border-radius:4px;border:1px solid #0aa;background:#033;color:#0ff;}"
    "</style>"
    "</head><body>"
    "<h1>Sat Tracker</h1>"
  );

  html += F("<div class='box'><h2>QTH</h2><form method='POST' action='/config'>");

  html += F("<label>Lat:</label><input type='text' name='lat' value='");
  html += String(g_qthLat, 4);
  html += F("'><br>");

  html += F("<label>Lon:</label><input type='text' name='lon' value='");
  html += String(g_qthLon, 4);
  html += F("'><br>");

  html += F("<label>Alt:</label><input type='text' name='alt' value='");
  html += String(g_qthAlt, 1);
  html += F("'> m<br>");

  html += F("<label>MinEl:</label><input type='text' name='minel' value='");
  html += String(g_minElDeg, 1);
  html += F("'> &deg;<br>");

  html += F("</div><div class='box'><h2>Satelity</h2><div class='satlist'>");

  for (int i = 0; i < SAT_COUNT; i++) {
    html += "<label><input type='checkbox' name='sat_";
    html += g_sats[i].id;
    html += "'";
    if (g_sats[i].enabled) html += " checked";
    html += "> ";
    html += htmlEscape(g_sats[i].shortName);
    html += " (";
    html += htmlEscape(g_sats[i].defaultName);
    html += ")</label><br>";
  }

  html += F("</div></div><button type='submit'>Ulozit a prepocitat</button></form>");

  html += F("<div class='box'><h2>Info</h2>");
  html += F("Aktualni IP: ");
  html += htmlEscape(g_ipStr);
  html += F("<br>FS: ");
  html += getFSInfoString();
  html += F("</div>");

  html += F("</body></html>");

  server.send(200, "text/html", html);
}

void handleConfig() {
  if (server.hasArg("lat"))   g_qthLat   = server.arg("lat").toFloat();
  if (server.hasArg("lon"))   g_qthLon   = server.arg("lon").toFloat();
  if (server.hasArg("alt"))   g_qthAlt   = server.arg("alt").toFloat();
  if (server.hasArg("minel")) g_minElDeg = server.arg("minel").toFloat();

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
  predictPasses(startUtc);

  g_lastPassListMinute = -1;
  clearTrail();

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

  splashStatus("Nacitam konfiguraci...");
  loadConfig();

  splashStatus("Pripojuji WiFi...");
  connectWiFi();

  splashStatus("Synchronizuji cas (NTP)...");
  setupTime();

  splashStatus("Inicializuji TLE a satelity...");
  initSatConfigs();

  splashStatus("Pocitam prelety...");
  time_t nowUtc   = time(nullptr);
  time_t startUtc = nowUtc;
  predictPasses(startUtc);

  splashStatus("Startuji web server...");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_POST, handleConfig);
  server.begin();

  splashStatus("Hotovo.");
  delay(800);

  drawStaticFrame();
  g_lastPassListMinute = -1;
  clearTrail();
  g_displayMode = MODE_LIST;

  Serial.println("HTTP server ready.");
}

// ====================== LOOP ======================
void loop() {
  server.handleClient();

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

  time_t nowUtc = time(nullptr);

  tm tmUtc;
  gmtime_r(&nowUtc, &tmUtc);

  tm tmLocal;
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
      drawSatState(si, s, tmUtc);
    }
  }

  drawFooter(tmLocal);
}
