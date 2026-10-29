// =====================================================================
//  การพัฒนาระบบเก็บขยะลอยนํ้าเคลื่อนที่ด้วยพลังงานแสงอาทิตย์  -  ESP32 (core 3.x)   ปั๊ม 4 ตัว + เซนเซอร์ JSN-SR04T 4 ตัว
//
//  การใช้งาน
//   1) ไฟ: ESP32 ใช้ 5V จาก buck (ปรับให้ได้ 5.0V ใส่ตัวเก็บประจุ 470uF ใกล้ขา VIN)
//          ปั๊ม 12V ผ่านบอร์ด MOSFET  ต่อ GND ร่วมกัน  สาย PWM แต่ละเส้นดึงลง GND ด้วย 10k
//   2) อัปโหลด: Board = ESP32 Dev Module  / Serial Monitor 115200
//   3) ดูสถานะ: ต่อ WiFi "ESP32-ST" รหัส 1234567890 แล้วเปิด port: http://192.168.4.1
//      (ข้อความธรรมดา รีเฟรชเองทุก 1 วินาที  ESP-32U)
//
//  การทำงาน (วนไม่สิ้นสุด ค่าเริ่มต้น รอบละ ~66 วินาที)
//   ดูดน้ำ      ปั๊มทั้ง 4 พร้อมกัน เปิด 10 วิ / พัก 5 วิ x 2 รอบ (ครั้งท้ายค่อยๆ เบา)
//   เคลื่อนที่   วน 8 ทิศ ทิศละ 4 วิ  ใช้เฉพาะปั๊มที่ต้องใช้ เต็ม 100%
//   หยุด / นิ่ง  ปั๊มดับ ~4 วิ แล้วกลับไปดูดน้ำ
//   ใกล้กำแพง (ตอนเคลื่อนที่/หยุด/นิ่ง): ปั๊มที่ดันไปทางกำแพงผ่อนแรง ปั๊มมุมที่ใกล้ดันหนี
//
//   มุม 3 บนซ้าย | มุม 2 บนขวา | มุม 4 ล่างซ้าย | มุม 1 ล่างขวา   (มุม n = เซนเซอร์ n + ปั๊ม n)
//   ปั๊มที่ใช้แต่ละทิศ: ขึ้น 1+4  ขวา 3+4  ลง 2+3  ซ้าย 1+2  เฉียง = ตัวเดียว
// =====================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <stdarg.h>
#include "esp_system.h"

// ============================== ตั้งค่า ==============================
// ช่วงดูดน้ำ (ปั๊มทั้ง 4 พร้อมกัน)
constexpr float PULSE_ON_S    = 10;    // เปิดกี่วินาที
constexpr float PULSE_OFF_S   = 5;     // พักกี่วินาที
constexpr int   PULSE_CYCLES  = 2;     // กี่รอบ
constexpr float PULSE_FADE_S  = 5;     // รอบสุดท้ายค่อยๆ เบากี่วินาที (0 = ไม่เบา)
constexpr int   ON_PCT        = 100;   // แรงตอนเปิด (%)
constexpr int   MIN_PCT       = 35;    // แรงต่ำสุดที่ปั๊มของคุณยังหมุน (ปรับตามปั๊มจริง)

// เคลื่อนที่
constexpr int   ROUTE_MODE    = 0;     // 0 = วน 8 ทิศ (เหมาะโชว์)   1 = เดินตรง เลี้ยวเมื่อเจอกำแพง
constexpr unsigned long LEG_SEC = 4;   // โหมด 0: วินาทีต่อทิศ
constexpr unsigned long MOVE_SEC = 30; // โหมด 1: วินาทีต่อรอบ
constexpr int   MOVE_PCT      = 100;   // แรงตอนเคลื่อนที่ (%)
constexpr unsigned long SETTLE_SEC = 3;// นิ่งหลังเคลื่อนที่ (วินาที)

// กำแพง
constexpr int   FAR_CM        = 100;   // ใกล้กว่านี้เริ่มหนี (อ่างเล็กใช้ 60)
constexpr int   NEAR_CM       = 40;    // ใกล้กว่านี้หนีเต็มที่
constexpr bool  PULSE_IGNORE_WALL = true; // true = ช่วงดูดน้ำไม่สนกำแพง (ทำงานครบ 4 ตัวเสมอ)

// บอร์ด
constexpr int   CPU_MHZ       = 160;   // 80 / 160 / 240
constexpr int   TOTAL_MAX_PCT = 300;   // เพดานรวมกำลังปั๊มตอนใกล้กำแพง (400 = ไม่จำกัด)
constexpr bool  USE_WATCHDOG  = true;  // โปรแกรมค้าง = บอร์ดรีเซ็ตเอง

// WiFi
constexpr bool  USE_WEB       = true;  // true = เปิด WiFi, false = ปิด

// แบต (ถ้าต่อวงจรวัดแรงดัน 47k + 10k เข้าขา 34)
constexpr bool  USE_BATT_MONITOR = false;
constexpr int   BATT_PIN      = 34;
constexpr float DIV_RATIO     = 5.7f;
constexpr float LOW_V = 11.6f, RESUME_V = 12.4f;

// ขา (index 0..3 = มุม 1..4)
const int TRIG[4] = {13, 27, 25, 32};
const int ECHO[4] = {14, 26, 33, 35};
const int PUMP[4] = {16, 17, 18, 19};

constexpr int   TOUR_LEGS       = 8;
constexpr float TOUR_START_DEG  = 90;
constexpr float TOUR_STEP_DEG   = -45;
constexpr float BOUNCE_TURN_DEG = 45;
constexpr float MOVE_STRENGTH   = 0.8f;
constexpr float ACTIVE_MIN      = 0.2f;
constexpr float PULSE_STAGGER_S = 0.05f;
constexpr float PULSE_UP = 400, PULSE_DOWN = 1000;
constexpr float UP_RATE = 400, DOWN_RATE = 1000, UP_AVOID = 600, DOWN_SAFE = 1000;
constexpr float EASE_DOWN_RATE  = 80;
constexpr unsigned long STAGGER_MS = 100;
constexpr unsigned long STOP_MAX_MS = 5000;
constexpr int   GAP_MS          = 60;
constexpr int   MAX_VALID_CM    = 450;
constexpr int   BLIND_EDGE_CM   = 35;
constexpr unsigned long BLIND_MAX_MS = 2500;
constexpr unsigned long LOW_HOLD_MS = 10000, RESUME_HOLD_MS = 30000;
constexpr int   STATUS_EVERY_S  = 1;

// WiFi:
const char*     AP_SSID         = "ESP32-ST";
const char*     AP_PASS         = "1234567890";
constexpr int   WIFI_CHANNEL    = 6;
constexpr int   WEB_MAX_CLIENTS = 2;
constexpr wifi_power_t WIFI_TX  = WIFI_POWER_13dBm;

static_assert(PULSE_CYCLES >= 1 && PULSE_CYCLES <= 50, "PULSE_CYCLES must be 1..50");
static_assert(PULSE_ON_S > 0 && PULSE_OFF_S >= 0 && PULSE_FADE_S >= 0, "pulse times must be positive");
static_assert(ON_PCT >= 0 && ON_PCT <= 100 && MOVE_PCT >= 0 && MOVE_PCT <= 100, "ON_PCT / MOVE_PCT must be 0..100");
static_assert(MIN_PCT >= 0 && MIN_PCT <= 100, "MIN_PCT must be 0..100");
static_assert(ROUTE_MODE == 0 || ROUTE_MODE == 1, "ROUTE_MODE must be 0 or 1");
static_assert(LEG_SEC >= 1 && MOVE_SEC >= 1 && TOUR_LEGS >= 1, "LEG_SEC, MOVE_SEC, TOUR_LEGS must be at least 1");
static_assert(FAR_CM > NEAR_CM && NEAR_CM >= 10, "need FAR_CM > NEAR_CM >= 10");
static_assert(CPU_MHZ == 80 || CPU_MHZ == 160 || CPU_MHZ == 240, "CPU_MHZ must be 80, 160 or 240 (WiFi needs 80 or more)");
static_assert(TOTAL_MAX_PCT >= 50, "TOTAL_MAX_PCT must be at least 50");
static_assert(WIFI_CHANNEL >= 1 && WIFI_CHANNEL <= 13, "WIFI_CHANNEL must be 1..13");
static_assert(WEB_MAX_CLIENTS >= 1 && WEB_MAX_CLIENTS <= 4, "WEB_MAX_CLIENTS must be 1..4");

constexpr float PULSE_FADE_EFF = (PULSE_FADE_S < PULSE_ON_S) ? PULSE_FADE_S : PULSE_ON_S;
constexpr unsigned long IDLE_MS = (unsigned long)(PULSE_CYCLES * (PULSE_ON_S + PULSE_OFF_S) * 1000.0f);

const float CORNER_DEG[4] = {-45, 45, 135, 225};

enum State { HOLD, WINDDOWN, SETTLE, MOVE, SAFE };
State st = HOLD;
unsigned long tState = 0, lastApply = 0, nextStart = 0;
float cur[4] = {0, 0, 0, 0};
float tx[4], ty[4];
float battV = 0;
float headingDeg = TOUR_START_DEG;
int   bounceSign = 1;
int   lastLeg = -1;

int  tgt[4] = {0, 0, 0, 0};
bool pulseMode = false;
bool avoidNow = false;
const char* roleG[4] = {"หยุด", "หยุด", "หยุด", "หยุด"};

int   rawCm[4]  = {-1, -1, -1, -1};
int   distG[4]  = {999, 999, 999, 999};
float wG[4]     = {0, 0, 0, 0};

int  buf[4][3] = {{999,999,999},{999,999,999},{999,999,999},{999,999,999}};
int  bi[4] = {0, 0, 0, 0};
int  closeStreak[4] = {0, 0, 0, 0};
bool failing[4] = {false, false, false, false};
unsigned long failStart[4] = {0, 0, 0, 0};

bool avoidLatch = false, clearing = false;
unsigned long clearSince = 0;
bool lowFlag = false, okFlag = false;
unsigned long lowSince = 0, okSince = 0;

WebServer server(80);

const char* stName(State s) {
  switch (s) {
    case HOLD: return "HOLD"; case WINDDOWN: return "STOP";
    case SETTLE: return "SETTLE"; case MOVE: return "MOVE"; default: return "SAFE";
  }
}

const char* DIR_EN[8] = {"RIGHT", "UP-RIGHT", "UP", "UP-LEFT", "LEFT", "DOWN-LEFT", "DOWN", "DOWN-RIGHT"};
const char* DIR_TH[8] = {"ขวา", "ขวาบน", "ขึ้น", "ซ้ายบน", "ซ้าย", "ซ้ายล่าง", "ลง", "ขวาล่าง"};
const char* CORNER_TH[4] = {"ขวาล่าง", "ขวาบน", "ซ้ายบน", "ซ้ายล่าง"};

int dirIdx(float h) { int i = (int)roundf(h / 45.0f); return ((i % 8) + 8) % 8; }

float wrapDeg(float a) {
  while (a > 180) a -= 360;
  while (a <= -180) a += 360;
  return a;
}

unsigned long since(unsigned long t0, unsigned long now) {
  unsigned long d = now - t0;
  return (d > 0x7FFFFFFFUL) ? 0 : d;
}

unsigned long moveMs() {
  return (ROUTE_MODE == 0) ? (unsigned long)TOUR_LEGS * LEG_SEC * 1000UL : MOVE_SEC * 1000UL;
}

static void put(char* b, size_t cap, size_t& n, const char* fmt, ...) {
  if (n + 1 >= cap) return;
  va_list ap;
  va_start(ap, fmt);
  int k = vsnprintf(b + n, cap - n, fmt, ap);
  va_end(ap);
  if (k <= 0) return;
  if ((size_t)k < cap - n) { n += (size_t)k; return; }
  n = cap - 1;
  while (n > 0 && (((unsigned char)b[n - 1]) & 0xC0) == 0x80) n--;
  if (n > 0 && (((unsigned char)b[n - 1]) & 0xC0) == 0xC0) n--;
  b[n] = 0;
  n = cap;
}

static void sensorText(int k, char* out, size_t cap) {
  if (distG[k] >= 900) { snprintf(out, cap, "ไกล / ไม่เจอ"); return; }
  const char* tag = (wG[k] >= 0.8f) ? "อันตราย" : (wG[k] >= 0.1f ? "ใกล้" : "ปลอดภัย");
  if (rawCm[k] > 0) snprintf(out, cap, "%d cm  %s", distG[k], tag);
  else              snprintf(out, cap, "ชิดมาก (โซนบอด) %s", tag);
}

void handleStatus() {
  unsigned long now = millis();
  unsigned long el = since(tState, now);
  char b[2000]; size_t n = 0; const size_t cap = sizeof(b);
  unsigned long up = now / 1000UL;
  put(b, cap, n, "ESP32 (core 3.x) - สถานะสด (หน้านี้รีเฟรชเองทุก 1 วินาที)\nเปิดมา %lu:%02lu:%02lu\n\n",
      up / 3600UL, (up / 60UL) % 60UL, up % 60UL);

  switch (st) {
    case HOLD: {
      float period = PULSE_ON_S + PULSE_OFF_S;
      float t = el / 1000.0f;
      int idx = (int)(t / period);
      if (idx < PULSE_CYCLES) {
        float cyc = t - idx * period;
        if (cyc < PULSE_ON_S) {
          bool fade = (PULSE_FADE_EFF > 0 && idx == PULSE_CYCLES - 1 && cyc > PULSE_ON_S - PULSE_FADE_EFF);
          put(b, cap, n, "สถานะ: ดูดน้ำ พัลส์ %d/%d - ปั๊มทำงาน เหลือ %d วิ%s\n", idx + 1, PULSE_CYCLES,
              (int)ceilf(PULSE_ON_S - cyc), fade ? " (กำลังค่อยๆ เบาลง)" : "");
        } else {
          put(b, cap, n, "สถานะ: พัก พัลส์ %d/%d - ปั๊มหยุด น้ำไหลเข้าถัง เหลือ %d วิ\n", idx + 1, PULSE_CYCLES,
              (int)ceilf(period - cyc));
        }
      } else put(b, cap, n, "สถานะ: จบพัลส์\n");
      break;
    }
    case MOVE: {
      unsigned long tot = moveMs();
      unsigned long rem = (tot > el) ? (tot - el + 999UL) / 1000UL : 0;
      if (ROUTE_MODE == 0) {
        int leg = (int)(el / (LEG_SEC * 1000UL)) + 1;
        if (leg > TOUR_LEGS) leg = TOUR_LEGS;
        put(b, cap, n, "สถานะ: เคลื่อนที่ ทิศ%s (%d/%d) เหลือ %lu วิ\n", DIR_TH[dirIdx(headingDeg)], leg, TOUR_LEGS, rem);
      } else {
        put(b, cap, n, "สถานะ: เคลื่อนที่ (เดินตรง) ทิศ %d องศา เหลือ %lu วิ\n", (int)headingDeg, rem);
      }
      break;
    }
    case WINDDOWN: put(b, cap, n, "สถานะ: หยุดปั๊ม\n"); break;
    case SETTLE: {
      unsigned long tot = SETTLE_SEC * 1000UL;
      put(b, cap, n, "สถานะ: นิ่ง เหลือ %lu วิ\n", (tot > el) ? (tot - el + 999UL) / 1000UL : 0UL);
      break;
    }
    default: put(b, cap, n, "สถานะ: หยุดเพื่อป้องกันแบต (แรงดันต่ำ)\n"); break;
  }

  int running = 0;
  for (int k = 0; k < 4; k++) if (cur[k] >= 1.0f) running++;
  put(b, cap, n, "ปั๊มที่หมุนตอนนี้: %d จาก 4 ตัว%s\n", running,
      running == 4 ? " (ครบทุกตัว)" : (running == 0 ? " (หยุดหมด)" : ""));
  put(b, cap, n, "หนีกำแพง: %s\n\n", avoidNow ? "ใช่" : "ไม่");

  for (int k = 0; k < 4; k++) {
    char s[128];
    sensorText(k, s, sizeof(s));
    put(b, cap, n, "มุม %d %s\n  เซนเซอร์: %s\n  ปั๊ม: %d%% %s\n", k + 1, CORNER_TH[k], s, (int)(cur[k] + 0.5f), roleG[k]);
  }
  if (USE_BATT_MONITOR) put(b, cap, n, "\nแบต: %.1f V\n", (double)battV);
  put(b, cap, n, "\n(ปั๊ม 100%% = เต็มแรง, 0%% = หยุด)\n");

  server.sendHeader("Refresh", "1");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain; charset=utf-8", b);
}

void webTask(void*) {
  for (;;) {
    server.handleClient();
    delay(5);
  }
}

void startWeb() {
  WiFi.mode(WIFI_AP);
  const char* pass = (strlen(AP_PASS) >= 8) ? AP_PASS : nullptr;
  if (!WiFi.softAP(AP_SSID, pass, WIFI_CHANNEL, 0, WEB_MAX_CLIENTS)) { Serial.println("WiFi AP failed"); return; }
  WiFi.setTxPower(WIFI_TX);
  server.on("/", handleStatus);
  server.onNotFound(handleStatus);
  server.begin();
  xTaskCreatePinnedToCore(webTask, "web", 8192, NULL, 1, NULL, 0);
  Serial.print("Status page: connect to WiFi "); Serial.print(AP_SSID);
  if (pass) { Serial.print(" (password "); Serial.print(AP_PASS); Serial.print(")"); }
  Serial.print(", channel "); Serial.print(WIFI_CHANNEL);
  Serial.println(", then open http://192.168.4.1");
}

void pumpSet(int i, int pct) {
  pct = constrain(pct, 0, 100);
  ledcWrite(PUMP[i], map(pct, 0, 100, 0, 255));
}

float pulseGoal(float t) {
  if (t < 0) return 0;
  const float period = PULSE_ON_S + PULSE_OFF_S;
  int idx = (int)(t / period);
  if (idx >= PULSE_CYCLES) return 0;
  float cyc = t - idx * period;
  if (cyc >= PULSE_ON_S) return 0;
  if (PULSE_FADE_EFF > 0 && idx == PULSE_CYCLES - 1 && cyc > PULSE_ON_S - PULSE_FADE_EFF) {
    float g = ON_PCT * (PULSE_ON_S - cyc) / PULSE_FADE_EFF;
    return (g < MIN_PCT) ? 0 : g;
  }
  return ON_PCT;
}

void applyPumps() {
  unsigned long now = millis();
  float dt = (now - lastApply) / 1000.0f;
  lastApply = now;
  if (dt > 0.2f) dt = 0.2f;

  float goal[4];
  float up, down;
  if (pulseMode) {
    float t = since(tState, now) / 1000.0f;
    for (int k = 0; k < 4; k++) goal[k] = pulseGoal(t - k * PULSE_STAGGER_S);
    up = PULSE_UP; down = PULSE_DOWN;
  } else {
    for (int k = 0; k < 4; k++) goal[k] = tgt[k];
    up = avoidNow ? UP_AVOID : UP_RATE;
    down = (st == SAFE) ? DOWN_SAFE : (avoidNow ? EASE_DOWN_RATE : DOWN_RATE);
  }

  bool stagger = !pulseMode && !avoidNow;
  for (int k = 0; k < 4; k++) {
    float g = goal[k];
    if (stagger && cur[k] < 0.5f && g > 0) {
      if ((int32_t)(now - nextStart) < 0) g = 0;
      else nextStart = now + STAGGER_MS;
    }
    if (g > cur[k]) cur[k] = min(cur[k] + up * dt, g);
    else            cur[k] = max(cur[k] - down * dt, g);
    pumpSet(k, (int)(cur[k] + 0.5f));
  }
}

void waitMs(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    applyPumps();
    delay(10);
  }
}

int readCm(int i) {
  digitalWrite(TRIG[i], LOW);  delayMicroseconds(4);
  digitalWrite(TRIG[i], HIGH); delayMicroseconds(12);
  digitalWrite(TRIG[i], LOW);
  unsigned long us = pulseIn(ECHO[i], HIGH, 25000);
  if (us == 0) return -1;
  int d = (int)(us * 0.0343f / 2.0f);
  if (d <= 0 || d > MAX_VALID_CM) return -1;
  return d;
}

int median3(int a, int b, int c) {
  int t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}

int filteredCm(int i) {
  int d = readCm(i);
  rawCm[i] = d;
  unsigned long now = millis();
  int v;
  if (d > 0) {
    failing[i] = false;
    if (d <= BLIND_EDGE_CM) { if (closeStreak[i] < 100) closeStreak[i]++; } else closeStreak[i] = 0;
    v = d;
  } else {
    if (!failing[i]) { failing[i] = true; failStart[i] = now; }

    bool blind = (closeStreak[i] >= 2) && (now - failStart[i] < BLIND_MAX_MS);
    if (blind) v = NEAR_CM;
    else { v = 999; closeStreak[i] = 0; }
  }
  buf[i][bi[i]] = v;
  bi[i] = (bi[i] + 1) % 3;
  return median3(buf[i][0], buf[i][1], buf[i][2]);
}

float prox(int d) {
  if (d >= FAR_CM) return 0;
  if (d <= NEAR_CM) return 1;
  return (float)(FAR_CM - d) / (FAR_CM - NEAR_CM);
}

float readBatt() {
  long mv = 0;
  for (int k = 0; k < 8; k++) mv += analogReadMilliVolts(BATT_PIN);
  return (mv / 8.0f) / 1000.0f * DIV_RATIO;
}

void selfTest() {
  Serial.println("sensor check (pumps off):");
  for (int i = 0; i < 4; i++) {
    int ok = 0, last = -1;
    for (int n = 0; n < 5; n++) {
      int d = readCm(i);
      if (d > 0) { ok++; last = d; }
      delay(GAP_MS);
    }
    Serial.print("  corner "); Serial.print(i + 1); Serial.print(": ");
    if (ok == 0) Serial.println("no echo (open space, too close, or wiring?)");
    else {
      Serial.print(last); Serial.println(" cm");
      if (last < FAR_CM) {
        Serial.println("    WARNING: closer than FAR_CM, so it counts as a WALL.");
        Serial.println("    Move the rig away from walls / people, aim the sensor clear of the frame, or lower FAR_CM.");
      }
    }
  }
}

void setState(State s) {
  st = s;
  tState = millis();
  Serial.print(">> "); Serial.println(stName(s));
}

void setup() {
  for (int i = 0; i < 4; i++) { pinMode(PUMP[i], OUTPUT); digitalWrite(PUMP[i], LOW); }
  setCpuFrequencyMhz(CPU_MHZ);
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(TRIG[i], OUTPUT);
    digitalWrite(TRIG[i], LOW);
    pinMode(ECHO[i], INPUT);
    ledcAttach(PUMP[i], 1000, 8);
    pumpSet(i, 0);
    tx[i] = cos((CORNER_DEG[i] + 180) * DEG_TO_RAD);
    ty[i] = sin((CORNER_DEG[i] + 180) * DEG_TO_RAD);
  }
  delay(500);

  esp_reset_reason_t rr = esp_reset_reason();
  Serial.print("reset reason: "); Serial.println((int)rr);
  if (rr == ESP_RST_BROWNOUT) Serial.println("!! BROWNOUT: supply dipped -> check buck / add capacitors");

  Serial.printf("settings: HOLD %d x (%.0fs ON + %.0fs OFF) | %s | walls far=%d near=%d cm | CPU %d MHz\n",
                PULSE_CYCLES, (double)PULSE_ON_S, (double)PULSE_OFF_S,
                ROUTE_MODE == 0 ? "MOVE tour" : "MOVE bounce", FAR_CM, NEAR_CM, CPU_MHZ);

  selfTest();

  Serial.println(USE_WEB ? "WiFi: ON" : "WiFi: OFF");
  if (USE_WEB) startWeb();

  lastApply = millis();
  nextStart = lastApply;
  headingDeg = TOUR_START_DEG;
  setState(HOLD);
  pulseMode = true;

  if (USE_WATCHDOG) {
    enableLoopWDT();
    Serial.println("watchdog: ON");
  }
}

void loop() {

  float w[4]; int dist[4]; float maxW = 0;
  for (int i = 0; i < 4; i++) {
    dist[i] = filteredCm(i);
    w[i] = prox(dist[i]);
    distG[i] = dist[i];
    wG[i] = w[i];
    if (w[i] > maxW) maxW = w[i];
    waitMs(GAP_MS);
  }
  unsigned long now = millis();

  if (maxW > 0.08f) { avoidLatch = true; clearing = false; }
  else if (avoidLatch) {
    if (!clearing) { clearing = true; clearSince = now; }
    else if (now - clearSince > 1000) { avoidLatch = false; clearing = false; }
  }
  bool avoid = avoidLatch;

  if (USE_BATT_MONITOR) {
    battV = readBatt();
    if (battV > 5.0f) {
      if (st != SAFE) {
        if (battV < LOW_V) {
          if (!lowFlag) { lowFlag = true; lowSince = now; }
          else if (now - lowSince > LOW_HOLD_MS) { setState(SAFE); lowFlag = false; okFlag = false; }
        } else lowFlag = false;
      } else {
        if (battV > RESUME_V) {
          if (!okFlag) { okFlag = true; okSince = now; }
          else if (now - okSince > RESUME_HOLD_MS) { setState(HOLD); okFlag = false; }
        } else okFlag = false;
      }
    }
  }

  unsigned long el = since(tState, now);
  bool allZero = (cur[0] < 0.5f && cur[1] < 0.5f && cur[2] < 0.5f && cur[3] < 0.5f);
  switch (st) {
    case HOLD:     if (el > IDLE_MS) { lastLeg = -1; setState(MOVE); } break;
    case MOVE:     if (el > moveMs()) setState(WINDDOWN); break;
    case WINDDOWN: if (allZero || el > STOP_MAX_MS) setState(SETTLE); break;
    case SETTLE:   if (el > SETTLE_SEC * 1000UL) setState(HOLD); break;
    case SAFE:     break;
  }

  if (st == MOVE && ROUTE_MODE == 0) {
    int leg = (int)(since(tState, now) / (LEG_SEC * 1000UL));
    if (leg >= TOUR_LEGS) leg = TOUR_LEGS - 1;
    headingDeg = wrapDeg(TOUR_START_DEG + leg * TOUR_STEP_DEG);
    if (leg != lastLeg) {
      lastLeg = leg;
      Serial.print(">> leg "); Serial.print(leg + 1); Serial.print("/"); Serial.print(TOUR_LEGS);
      Serial.print("  "); Serial.println(DIR_EN[dirIdx(headingDeg)]);
    }
  }

  static bool bounced = false;
  static unsigned long avoidSince = 0;
  if (!avoid) { bounced = false; avoidSince = 0; }
  else if (avoidSince == 0) avoidSince = now;

  if (ROUTE_MODE == 1 && st == MOVE && avoid && !bounced && (now - avoidSince) > 800) {
    float rx = 0, ry = 0;
    for (int k = 0; k < 4; k++) { rx += w[k] * tx[k]; ry += w[k] * ty[k]; }
    if (fabsf(rx) + fabsf(ry) > 0.05f) {
      float away = atan2f(ry, rx) * 57.29578f;
      bounceSign = -bounceSign;
      headingDeg = wrapDeg(away + bounceSign * BOUNCE_TURN_DEG);
      bounced = true;
      Serial.print(">> bounce, new heading "); Serial.println((int)headingDeg);
    }
  }

  for (int k = 0; k < 4; k++) { tgt[k] = 0; roleG[k] = "หยุด"; }
  float mx = 0, my = 0;

  bool avoidEff = avoid && !(PULSE_IGNORE_WALL && st == HOLD);
  pulseMode = (st == HOLD && !avoidEff);

  bool useVector = false;
  if (st == SAFE) {

  } else if (st == MOVE) {
    mx = cos(headingDeg * DEG_TO_RAD) * MOVE_STRENGTH;
    my = sin(headingDeg * DEG_TO_RAD) * MOVE_STRENGTH;
    useVector = true;
  } else if (avoidEff) {
    useVector = true;
  }

  if (useVector) {

    for (int k = 0; k < 4; k++) {
      float wanted = mx * tx[k] + my * ty[k];
      float head   = (wanted >= ACTIVE_MIN) ? (float)MOVE_PCT : 0.0f;
      float toward = w[(k + 2) % 4];
      float moveP  = head * (1.0f - toward);
      float awayP  = 100.0f * w[k];
      float t = max(moveP, awayP);
      tgt[k] = (t < 8.0f) ? 0 : (int)(t + 0.5f);
      if (tgt[k] > 0) {
        if (awayP > moveP)                       roleG[k] = "ดันหนีกำแพง";
        else if (head > 0 && toward > 0.05f)     roleG[k] = "ผ่อนแรง (กำแพงฝั่งตรงข้ามใกล้)";
        else                                     roleG[k] = "เคลื่อนที่";
      }
    }
  }

  int sum = tgt[0] + tgt[1] + tgt[2] + tgt[3];
  if (sum > TOTAL_MAX_PCT) {
    for (int k = 0; k < 4; k++)
      if (tgt[k] > 0) tgt[k] = max(MIN_PCT, tgt[k] * TOTAL_MAX_PCT / sum);
  }

  avoidNow = avoidEff;
  applyPumps();

  for (int k = 0; k < 4; k++) {
    if (pulseMode)                         roleG[k] = (cur[k] >= 1.0f) ? "ดูดน้ำ" : "พัก";
    else if (tgt[k] == 0 && cur[k] >= 1.0f) roleG[k] = "กำลังหยุด";
  }

  static unsigned long lastPrint = 0;
  if (STATUS_EVERY_S > 0 && now - lastPrint > (unsigned long)STATUS_EVERY_S * 1000UL) {
    lastPrint = now;
    Serial.print(stName(st)); Serial.print(pulseMode ? "(pulse)" : "");
    if (st == MOVE) { Serial.print(" "); Serial.print(DIR_EN[dirIdx(headingDeg)]); }
    Serial.print(" t="); Serial.print(since(tState, now) / 1000); Serial.print("s ");
    if (avoidEff) Serial.print("AVOID ");
    for (int k = 0; k < 4; k++) {
      Serial.print("C"); Serial.print(k + 1); Serial.print(":");
      Serial.print(dist[k]); Serial.print("cm/"); Serial.print((int)(cur[k] + 0.5f)); Serial.print("% ");
    }
    if (USE_BATT_MONITOR) { Serial.print(" V="); Serial.print(battV, 1); }
    Serial.println();
  }
}