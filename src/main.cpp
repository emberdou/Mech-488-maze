#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

// ─── Pin Definitions ──────────────────────────────────────────────────────────
#define PIN_JOY_X     36   // KY-023 VRx — real hardware (unused in sim)
#define PIN_JOY_Y     35   // KY-023 VRy — real hardware (unused in sim)
#define PIN_JOY_SW    14   // KY-023 button — real hardware (unused in sim)
// Simulation-only directional buttons (remove when using real joystick)
#define PIN_DIR_UP    12
#define PIN_DIR_DOWN  13
#define PIN_DIR_LEFT  19
#define PIN_DIR_RIGHT 23
#define PIN_VIBRO     26   // coin motor via 2N2222
#define PIN_SERVO     27   // SG90 compass
#define PIN_NEOPIXEL  25   // WS2812B data
#define PIN_BUZZER    33   // passive buzzer
#define PIN_BTN_EASY  32
#define PIN_BTN_MED    4
#define PIN_BTN_HARD  18

// ─── PWM Channels (ESP32 ledc) ────────────────────────────────────────────────
#define LEDC_VIBRO_CH   0
#define LEDC_BUZZER_CH  1
#define LEDC_VIBRO_FREQ 5000
#define LEDC_RES        8    // 8-bit resolution → values 0-255

// ─── NeoPixel ─────────────────────────────────────────────────────────────────
#define NEO_COUNT 8

// ─── OLED ─────────────────────────────────────────────────────────────────────
#define SCREEN_W  128
#define SCREEN_H   64
#define OLED_ADDR 0x3C
#define RADAR_RADIUS 80.0f   // world-units visible around player on radar

// ─── Game Constants ───────────────────────────────────────────────────────────
#define WORLD_SIZE    500
#define START_X       60.0f
#define START_Y       250.0f
#define PLAYER_RADIUS 8.0f
#define PLAYER_SPEED  2.5f
#define JOY_CENTER    512    // 1023/2 — potentiometer center in Wokwi (swap to 2048 for real KY-023)
#define JOY_DEADZONE  50     // scaled for 10-bit pot range (swap to 200 for real KY-023)

// ─────────────────────────────────────────────────────────────────────────────
// Data types
// ─────────────────────────────────────────────────────────────────────────────

struct Wall {
    float x1, y1, x2, y2;
};

struct Checkpoint {
    float x, y, radius;
};

enum Difficulty { EASY = 0, MEDIUM = 1, HARD = 2 };

// ─────────────────────────────────────────────────────────────────────────────
// EASY maze — 4 rooms (BL/TL/TR/BR), 3 checkpoints
//
// Layout (y increases downward):
//   TL | TR        y = 10–160
//   ---+---  wall at y=160–170, gaps at x=70–170 (BL→TL) and x=340–440 (TR→BR)
//   BL | BR        y = 170–490
//        wall at x=245–255, gap at y=40–130 (TL→TR)
//
// Path: BL(start) → TL(CP1) → TR(CP2) → BR(CP3/win)
// ─────────────────────────────────────────────────────────────────────────────
const Wall easyWalls[] = {
    // Outer boundary
    {  0,   0, 500,  10},
    {  0, 490, 500, 500},
    {  0,   0,  10, 500},
    {490,   0, 500, 500},
    // Horizontal divider y=160–170, gaps at x=70–170 (BL→TL) and x=340–440 (TR→BR)
    { 10, 160,  70, 170},
    {170, 160, 340, 170},
    {440, 160, 490, 170},
    // Vertical divider x=245–255, gap at y=40–130 (TL→TR)
    {245,  10, 255,  40},
    {245, 130, 255, 160},
    {245, 170, 255, 490},
    // BL (x=10–245, y=170–490) — start area
    { 70, 300, 180, 310},   // U top
    { 70, 310,  80, 430},   // U left leg
    {160, 310, 170, 430},   // U right leg
    {140, 200, 240, 210},   // upper shelf
    {230, 210, 240, 300},   // shelf right leg
    // TL (x=10–245, y=10–160)
    { 10,  95, 180, 105},   // horizontal bar
    {170, 105, 180, 155},   // right leg
    { 10, 130,  60, 140},   // lower left stub
    // TR (x=255–490, y=10–160)
    {280,  70, 430,  80},   // horizontal bar
    {280,  80, 290, 150},   // left leg
    {390,  10, 400,  70},   // right pillar
    // BR (x=255–490, y=170–490)
    {280, 290, 390, 300},   // upper horizontal
    {370, 300, 380, 440},   // right leg
    {255, 400, 360, 410},   // lower shelf
};
const int easyWallCount = sizeof(easyWalls) / sizeof(Wall);

const Checkpoint easyCheckpoints[] = {
    {120,  45, 22},   // TL — upper clear area above obstacle bar
    {420, 125, 22},   // TR — lower right clear area
    {420, 380, 22},   // BR — right side clear of obstacles
};
const int easyCheckpointCount = 3;

// ─────────────────────────────────────────────────────────────────────────────
// MEDIUM maze — 6 rooms (3×2 grid), 5 checkpoints
//
//   A1 | A2 | A3      y = 10–160
//   ---+----+---  wall at y=160–170
//   B1 | B2 | B3      y = 170–490   (B1 = start)
//        walls at x=165–175 and x=330–340
//
// Path: B1 → A1(CP1) → A2(CP2) → B2(CP3) → B3(CP4) → A3(CP5/win)
// ─────────────────────────────────────────────────────────────────────────────
const Wall medWalls[] = {
    // Outer boundary
    {  0,   0, 500,  10},
    {  0, 490, 500, 500},
    {  0,   0,  10, 500},
    {490,   0, 500, 500},
    // Horizontal divider y=160–170, gaps at x=40–120, x=200–280, x=370–450
    { 10, 160,  40, 170},
    {120, 160, 200, 170},
    {280, 160, 370, 170},
    {450, 160, 490, 170},
    // Vertical divider Col1/Col2 at x=165–175, gap at y=50–130
    {165,  10, 175,  50},
    {165, 130, 175, 160},
    {165, 170, 175, 490},
    // Vertical divider Col2/Col3 at x=330–340, gap at y=280–380
    {330,  10, 340, 160},
    {330, 170, 340, 280},
    {330, 380, 340, 490},
    // Obstacles inside B1 (x=10–165, y=170–490)
    { 40, 290, 130, 300},
    {110, 300, 120, 430},
    // Obstacles inside A1 (x=10–165, y=10–160)
    { 40,  70, 130,  80},
    { 40,  80,  50, 140},
    // Obstacles inside A2 (x=175–330, y=10–160)
    {200,  60, 280,  70},
    {270,  70, 280, 140},
    // Obstacles inside B2 (x=175–330, y=170–490)
    {200, 260, 290, 270},
    {200, 370, 290, 380},
    {280, 270, 290, 370},
    // Obstacles inside B3 (x=340–490, y=170–490)
    {370, 260, 450, 270},
    {370, 370, 450, 380},
    {440, 270, 450, 370},
    // Obstacles inside A3 (x=340–490, y=10–160)
    {370,  60, 450,  70},
    {440,  70, 450, 140},
};
const int medWallCount = sizeof(medWalls) / sizeof(Wall);

const Checkpoint medCheckpoints[] = {
    { 90, 100, 18},   // A1
    {250,  90, 18},   // A2
    {250, 320, 18},   // B2
    {410, 320, 18},   // B3
    {410,  90, 18},   // A3 — win
};
const int medCheckpointCount = 5;

// ─────────────────────────────────────────────────────────────────────────────
// HARD maze — 8 rooms (4×2 grid), 7 checkpoints
//
//   A1 | A2 | A3 | A4      y = 10–160
//   ---+----+----+---  wall at y=160–170
//   B1 | B2 | B3 | B4      y = 170–490   (B1 = start)
//        walls at x=125–135, x=255–265, x=385–395
//
// Path: B1 → A1(CP1) → A2(CP2) → B2(CP3) → B3(CP4) → A3(CP5) → A4(CP6) → B4(CP7/win)
// ─────────────────────────────────────────────────────────────────────────────
const Wall hardWalls[] = {
    // Outer boundary
    {  0,   0, 500,  10},
    {  0, 490, 500, 500},
    {  0,   0,  10, 500},
    {490,   0, 500, 500},
    // Horizontal divider y=160–170, gaps at x=30–100, x=155–225, x=280–350, x=415–480
    { 10, 160,  30, 170},
    {100, 160, 155, 170},
    {225, 160, 280, 170},
    {350, 160, 415, 170},
    {480, 160, 490, 170},
    // Vertical Col1/Col2 x=125–135, gap at y=45–115
    {125,  10, 135,  45},
    {125, 115, 135, 160},
    {125, 170, 135, 490},
    // Vertical Col2/Col3 x=255–265, gap at y=285–380
    {255,  10, 265, 160},
    {255, 170, 265, 285},
    {255, 380, 265, 490},
    // Vertical Col3/Col4 x=385–395, gap at y=45–115
    {385,  10, 395,  45},
    {385, 115, 395, 160},
    {385, 170, 395, 490},
    // Obstacles inside B1 (x=10–125, y=170–490)
    { 30, 270, 100, 280},
    { 80, 280,  90, 410},
    { 30, 390,  80, 400},
    // Obstacles inside A1 (x=10–125, y=10–160)
    { 30,  60, 100,  70},
    { 80,  70,  90, 140},
    // Obstacles inside A2 (x=135–255, y=10–160)
    {155,  55, 225,  65},
    {215,  65, 225, 135},
    // Obstacles inside B2 (x=135–255, y=170–490)
    {155, 260, 235, 270},
    {155, 370, 235, 380},
    {225, 270, 235, 370},
    // Obstacles inside B3 (x=265–385, y=170–490)
    {285, 260, 365, 270},
    {285, 370, 365, 380},
    {355, 270, 365, 370},
    // Obstacles inside A3 (x=265–385, y=10–160)
    {285,  55, 355,  65},
    {345,  65, 355, 135},
    // Obstacles inside A4 (x=395–490, y=10–160)
    {415,  55, 475,  65},
    {465,  65, 475, 135},
    // Obstacles inside B4 (x=395–490, y=170–490)
    {415, 260, 475, 270},
    {415, 370, 475, 380},
    {465, 270, 475, 370},
};
const int hardWallCount = sizeof(hardWalls) / sizeof(Wall);

const Checkpoint hardCheckpoints[] = {
    { 65,  90, 15},   // A1
    {190,  90, 15},   // A2
    {190, 320, 15},   // B2
    {320, 320, 15},   // B3
    {320,  90, 15},   // A3
    {440,  90, 15},   // A4
    {440, 320, 15},   // B4 — win
};
const int hardCheckpointCount = 7;

// ─────────────────────────────────────────────────────────────────────────────
// Game state
// ─────────────────────────────────────────────────────────────────────────────
struct GameState {
    float px, py;            // current player position
    float cpx, cpy;          // last checkpoint (respawn point)
    int   checkpointIndex;   // index of next checkpoint to reach
    Difficulty difficulty;
    bool  gameWon;
};

GameState g;

// Active maze pointers (set by startGame)
const Wall*       walls;
int               wallCount;
const Checkpoint* checkpoints;
int               checkpointCount;

// ─── Peripheral objects ───────────────────────────────────────────────────────
Adafruit_SSD1306  oled(SCREEN_W, SCREEN_H, &Wire, -1);
Adafruit_NeoPixel strip(NEO_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Servo             compassServo;

// ─────────────────────────────────────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────────────────────────────────────
void initHardware();
void selectDifficulty();
void startGame();
void updateJoystick();
bool checkCollisions();
void updateCompass();
void updateVibration();
void updateOLED();
void checkCheckpoint();
void onCollision();
void onCheckpointReached();
void onWin();
void buzzerTone(int freq, int durationMs);
void buzzerOff();
void setVibro(uint8_t intensity);
void showCheckpointProgress();
void rainbowWin();
float pointToSegDist(float px, float py, float x1, float y1, float x2, float y2);
float minWallDist(float px, float py);

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    initHardware();
    selectDifficulty();
    startGame();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    if (g.gameWon) {
        rainbowWin();
        if (digitalRead(PIN_BTN_EASY) == LOW ||
            digitalRead(PIN_BTN_MED)  == LOW ||
            digitalRead(PIN_BTN_HARD) == LOW) {
            selectDifficulty();
            startGame();
        }
        delay(100);
        return;
    }

    updateJoystick();

    if (checkCollisions()) {
        onCollision();
        return;
    }

    checkCheckpoint();
    updateCompass();
    updateVibration();
    updateOLED();

    delay(20);  // ~50 Hz
}

// ─────────────────────────────────────────────────────────────────────────────
// initHardware — configure pins, ledc channels, and peripheral drivers
// ─────────────────────────────────────────────────────────────────────────────
void initHardware() {
    pinMode(PIN_JOY_SW,   INPUT_PULLUP);
    pinMode(PIN_DIR_UP,    INPUT_PULLUP);
    pinMode(PIN_DIR_DOWN,  INPUT_PULLUP);
    pinMode(PIN_DIR_LEFT,  INPUT_PULLUP);
    pinMode(PIN_DIR_RIGHT, INPUT_PULLUP);
    pinMode(PIN_BTN_EASY, INPUT_PULLUP);
    pinMode(PIN_BTN_MED,  INPUT_PULLUP);
    pinMode(PIN_BTN_HARD, INPUT_PULLUP);

    // Servo must attach before ledcSetup so ESP32Servo claims its LEDC channel first
    compassServo.attach(PIN_SERVO);
    compassServo.write(90);

    ledcSetup(LEDC_VIBRO_CH,  LEDC_VIBRO_FREQ, LEDC_RES);
    ledcAttachPin(PIN_VIBRO,  LEDC_VIBRO_CH);
    ledcWrite(LEDC_VIBRO_CH,  0);

    // Buzzer channel frequency is reconfigured per tone; initial setup only
    ledcSetup(LEDC_BUZZER_CH, 1000, LEDC_RES);
    ledcAttachPin(PIN_BUZZER, LEDC_BUZZER_CH);
    ledcWrite(LEDC_BUZZER_CH, 0);

    Wire.begin(21, 22);
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 init failed");
        for (;;);
    }
    oled.clearDisplay();
    oled.display();

    strip.begin();
    strip.setBrightness(80);
    strip.show();
}

// ─────────────────────────────────────────────────────────────────────────────
// selectDifficulty — block until one of the three buttons is pressed
// ─────────────────────────────────────────────────────────────────────────────
void selectDifficulty() {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(8, 4);  oled.println("INVISIBLE MAZE");
    oled.setCursor(8, 20); oled.println("Pick difficulty:");
    oled.setCursor(8, 36); oled.println("[E] Easy");
    oled.setCursor(8, 46); oled.println("[M] Medium  [H] Hard");
    oled.display();

    while (true) {
        if (digitalRead(PIN_BTN_EASY) == LOW) { g.difficulty = EASY;   break; }
        if (digitalRead(PIN_BTN_MED)  == LOW) { g.difficulty = MEDIUM; break; }
        if (digitalRead(PIN_BTN_HARD) == LOW) { g.difficulty = HARD;   break; }
        delay(50);
    }
    delay(200);  // debounce
}

// ─────────────────────────────────────────────────────────────────────────────
// startGame — point maze pointers at chosen difficulty, reset player state
// ─────────────────────────────────────────────────────────────────────────────
void startGame() {
    switch (g.difficulty) {
        case EASY:
            walls = easyWalls;             wallCount = easyWallCount;
            checkpoints = easyCheckpoints; checkpointCount = easyCheckpointCount;
            break;
        case MEDIUM:
            walls = medWalls;              wallCount = medWallCount;
            checkpoints = medCheckpoints;  checkpointCount = medCheckpointCount;
            break;
        case HARD:
            walls = hardWalls;             wallCount = hardWallCount;
            checkpoints = hardCheckpoints; checkpointCount = hardCheckpointCount;
            break;
    }

    g.px = START_X;  g.py = START_Y;
    g.cpx = START_X; g.cpy = START_Y;
    g.checkpointIndex = 0;
    g.gameWon = false;

    showCheckpointProgress();

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(16, 28);
    oled.println("GO! Find the path.");
    oled.display();
    delay(1500);
}

// ─────────────────────────────────────────────────────────────────────────────
// updateJoystick — simulation version using 4 directional buttons.
// To switch to real KY-023 joystick, replace this function body with:
//   int dx = analogRead(PIN_JOY_X) - JOY_CENTER;
//   int dy = analogRead(PIN_JOY_Y) - JOY_CENTER;
//   if (abs(dx) < JOY_DEADZONE) dx = 0;
//   if (abs(dy) < JOY_DEADZONE) dy = 0;
//   g.px = constrain(g.px + (dx / 2048.0f) * PLAYER_SPEED, PLAYER_RADIUS, WORLD_SIZE - PLAYER_RADIUS);
//   g.py = constrain(g.py - (dy / 2048.0f) * PLAYER_SPEED, PLAYER_RADIUS, WORLD_SIZE - PLAYER_RADIUS);
// ─────────────────────────────────────────────────────────────────────────────
void updateJoystick() {
    if (digitalRead(PIN_DIR_UP)    == LOW) g.py -= PLAYER_SPEED;
    if (digitalRead(PIN_DIR_DOWN)  == LOW) g.py += PLAYER_SPEED;
    if (digitalRead(PIN_DIR_LEFT)  == LOW) g.px -= PLAYER_SPEED;
    if (digitalRead(PIN_DIR_RIGHT) == LOW) g.px += PLAYER_SPEED;
    g.px = constrain(g.px, PLAYER_RADIUS, WORLD_SIZE - PLAYER_RADIUS);
    g.py = constrain(g.py, PLAYER_RADIUS, WORLD_SIZE - PLAYER_RADIUS);
}

// ─────────────────────────────────────────────────────────────────────────────
// pointToSegDist — shortest distance from point to a line segment
// ─────────────────────────────────────────────────────────────────────────────
float pointToSegDist(float px, float py, float x1, float y1, float x2, float y2) {
    float dx = x2 - x1, dy = y2 - y1;
    float lenSq = dx * dx + dy * dy;
    if (lenSq == 0.0f) return hypot(px - x1, py - y1);
    float t = constrain(((px - x1) * dx + (py - y1) * dy) / lenSq, 0.0f, 1.0f);
    return hypot(px - (x1 + t * dx), py - (y1 + t * dy));
}

// ─────────────────────────────────────────────────────────────────────────────
// checkCollisions — test player circle against all four edges of every wall rect
// ─────────────────────────────────────────────────────────────────────────────
bool checkCollisions() {
    for (int i = 0; i < wallCount; i++) {
        float x1 = walls[i].x1, y1 = walls[i].y1;
        float x2 = walls[i].x2, y2 = walls[i].y2;

        // Inside the rectangle itself
        if (g.px >= x1 && g.px <= x2 && g.py >= y1 && g.py <= y2) return true;

        if (pointToSegDist(g.px, g.py, x1, y1, x2, y1) < PLAYER_RADIUS) return true;
        if (pointToSegDist(g.px, g.py, x2, y1, x2, y2) < PLAYER_RADIUS) return true;
        if (pointToSegDist(g.px, g.py, x2, y2, x1, y2) < PLAYER_RADIUS) return true;
        if (pointToSegDist(g.px, g.py, x1, y2, x1, y1) < PLAYER_RADIUS) return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// minWallDist — minimum distance from player to any wall edge (for vibration)
// ─────────────────────────────────────────────────────────────────────────────
float minWallDist(float px, float py) {
    float minD = 1e9f;
    for (int i = 0; i < wallCount; i++) {
        float x1 = walls[i].x1, y1 = walls[i].y1;
        float x2 = walls[i].x2, y2 = walls[i].y2;
        float d = min({
            pointToSegDist(px, py, x1, y1, x2, y1),
            pointToSegDist(px, py, x2, y1, x2, y2),
            pointToSegDist(px, py, x2, y2, x1, y2),
            pointToSegDist(px, py, x1, y2, x1, y1)
        });
        if (d < minD) minD = d;
    }
    return minD;
}

// ─────────────────────────────────────────────────────────────────────────────
// updateCompass — point servo toward the next checkpoint using atan2
// ─────────────────────────────────────────────────────────────────────────────
void updateCompass() {
    if (g.checkpointIndex >= checkpointCount) return;

    float tx = checkpoints[g.checkpointIndex].x;
    float ty = checkpoints[g.checkpointIndex].y;

    float angleDeg = degrees(atan2(ty - g.py, tx - g.px));  // -180..180
    int servoAngle = constrain((int)((angleDeg + 180.0f) * 0.5f), 0, 180);
    compassServo.write(servoAngle);
}

// ─────────────────────────────────────────────────────────────────────────────
// updateVibration — PWM intensity ramps up quadratically as walls get closer
// ───────────────────────────────────────────────────────────────────∏──────────
void updateVibration() {
    const float maxDist = 50.0f;
    const float minDist = PLAYER_RADIUS + 2.0f;

    float dist = minWallDist(g.px, g.py);
    if (dist > maxDist) { setVibro(0); return; }

    float t = 1.0f - constrain((dist - minDist) / (maxDist - minDist), 0.0f, 1.0f);
    setVibro((uint8_t)(t * t * 200));  // quadratic, cap at 200/255
}

// ─────────────────────────────────────────────────────────────────────────────
// updateOLED — draw walls and next checkpoint within RADAR_RADIUS of player
// ─────────────────────────────────────────────────────────────────────────────
void updateOLED() {
    oled.clearDisplay();

    const float cx    = SCREEN_W / 2.0f;
    const float cy    = SCREEN_H / 2.0f;
    const float scale = cx / RADAR_RADIUS;  // world units → pixels

    for (int i = 0; i < wallCount; i++) {
        float wx1 = walls[i].x1, wy1 = walls[i].y1;
        float wx2 = walls[i].x2, wy2 = walls[i].y2;

        // Quick cull: skip walls whose nearest point exceeds radar radius
        float nx = constrain(g.px, wx1, wx2);
        float ny = constrain(g.py, wy1, wy2);
        if (hypot(g.px - nx, g.py - ny) > RADAR_RADIUS) continue;

        int sx1 = (int)(cx + (wx1 - g.px) * scale);
        int sy1 = (int)(cy + (wy1 - g.py) * scale);
        int sx2 = (int)(cx + (wx2 - g.px) * scale);
        int sy2 = (int)(cy + (wy2 - g.py) * scale);
        oled.fillRect(sx1, sy1, max(sx2 - sx1, 1), max(sy2 - sy1, 1), SSD1306_WHITE);
    }

    // Player dot at display center
    oled.fillCircle((int)cx, (int)cy, 2, SSD1306_WHITE);

    // Next checkpoint circle if within radar range
    if (g.checkpointIndex < checkpointCount) {
        float tx = checkpoints[g.checkpointIndex].x;
        float ty = checkpoints[g.checkpointIndex].y;
        if (hypot(tx - g.px, ty - g.py) <= RADAR_RADIUS) {
            int scx = (int)(cx + (tx - g.px) * scale);
            int scy = (int)(cy + (ty - g.py) * scale);
            oled.drawCircle(scx, scy, 3, SSD1306_WHITE);
        }
    }

    oled.display();
}

// ─────────────────────────────────────────────────────────────────────────────
// checkCheckpoint — detect if player reached the next checkpoint
// ─────────────────────────────────────────────────────────────────────────────
void checkCheckpoint() {
    if (g.checkpointIndex >= checkpointCount) return;
    const Checkpoint& cp = checkpoints[g.checkpointIndex];
    if (hypot(g.px - cp.x, g.py - cp.y) < cp.radius) {
        onCheckpointReached();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// onCollision — reset to last checkpoint, full vibration burst + low tone
// ─────────────────────────────────────────────────────────────────────────────
void onCollision() {
    g.px = g.cpx;
    g.py = g.cpy;

    setVibro(255);
    buzzerTone(200, 300);  // 200 Hz, 300 ms (blocks)
    setVibro(0);
}

// ─────────────────────────────────────────────────────────────────────────────
// onCheckpointReached — save new spawn point, advance index, play ascending tones
// ─────────────────────────────────────────────────────────────────────────────
void onCheckpointReached() {
    g.cpx = checkpoints[g.checkpointIndex].x;
    g.cpy = checkpoints[g.checkpointIndex].y;
    g.checkpointIndex++;

    showCheckpointProgress();

    buzzerTone(880,  150);
    buzzerTone(1100, 150);
    buzzerOff();

    if (g.checkpointIndex >= checkpointCount) onWin();
}

// ─────────────────────────────────────────────────────────────────────────────
// onWin — display message, play fanfare, enter rainbow loop
// ─────────────────────────────────────────────────────────────────────────────
void onWin() {
    g.gameWon = true;

    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(20, 22);
    oled.println("YOU WIN!");
    oled.display();

    const int fanfare[] = {523, 659, 784, 1047};
    for (int i = 0; i < 4; i++) {
        buzzerTone(fanfare[i], 200);
    }
    buzzerOff();
}

// ─────────────────────────────────────────────────────────────────────────────
// buzzerTone — reconfigure ledc channel frequency and play for durationMs
// ─────────────────────────────────────────────────────────────────────────────
void buzzerTone(int freq, int durationMs) {
    ledcSetup(LEDC_BUZZER_CH, freq, LEDC_RES);
    ledcAttachPin(PIN_BUZZER, LEDC_BUZZER_CH);
    ledcWrite(LEDC_BUZZER_CH, 128);  // 50% duty
    if (durationMs > 0) {
        delay(durationMs);
        buzzerOff();
    }
}

void buzzerOff() {
    ledcWrite(LEDC_BUZZER_CH, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// setVibro — set vibration motor PWM intensity 0–255
// ─────────────────────────────────────────────────────────────────────────────
void setVibro(uint8_t intensity) {
    ledcWrite(LEDC_VIBRO_CH, intensity);
}

// ─────────────────────────────────────────────────────────────────────────────
// showCheckpointProgress — green for reached, dim white for pending
// ─────────────────────────────────────────────────────────────────────────────
void showCheckpointProgress() {
    strip.clear();
    for (int i = 0; i < NEO_COUNT; i++) {
        if (i < g.checkpointIndex)
            strip.setPixelColor(i, strip.Color(0, 200, 0));    // reached — green
        else if (i < checkpointCount)
            strip.setPixelColor(i, strip.Color(200, 0, 0));    // pending — red
    }
    strip.show();
}

// ─────────────────────────────────────────────────────────────────────────────
// rainbowWin — cycle hue across all LEDs; called every loop() tick after win
// ─────────────────────────────────────────────────────────────────────────────
void rainbowWin() {
    static uint16_t hue = 0;
    for (int i = 0; i < NEO_COUNT; i++) {
        strip.setPixelColor(i, strip.ColorHSV(hue + (uint16_t)(i * 65536UL / NEO_COUNT)));
    }
    strip.show();
    hue += 512;
}
