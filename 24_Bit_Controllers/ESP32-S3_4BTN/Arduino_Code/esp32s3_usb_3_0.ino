#include <Joystick_ESP32S2.h>
#include <SPI.h>
#include <ADS1220_WE.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// ─── Pin Definitions ────────────────────────────────────────────────────────
#define ADS1220_CS_PIN    10
#define ADS1220_DRDY_PIN   9
#define SPI_SCK           13
#define SPI_MISO          12
#define SPI_MOSI          11
#define OLED_SDA           4
#define OLED_SCL           5
#define BUTTON_PREV        6
#define BUTTON_NEXT_PAGE   7
#define BUTTON_CALIBRATE   2
#define BUTTON_DEFAULT     3

// ─── Display ────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_ADDRESS  0x3C
#define OLED_REFRESH_MS 500   // 2 Hz display update

// ─── Calibration Timing ─────────────────────────────────────────────────────
#define CALIBRATION_TIME  4000  // ms to capture min or max
#define DONE_TIME         1000  // ms to show "DONE" message

// ─── Input / Filter ─────────────────────────────────────────────────────────
#define DEBOUNCE_TIME      50   // ms
#define DEADBAND_COUNTS  3355   // ~2 mV at 5V Vref, 24-bit (2/5000 * 2^23)
// EMA alpha stored as integer percentage (5–100, step 5).
// α=5 → very smooth/slow, α=100 → no filtering (pass-through).
#define EMA_ALPHA_STEP     5    // increment per button press
#define EMA_ALPHA_MAX    100
#define EMA_ALPHA_DEFAULT 10    // good starting point (~light smoothing)

// ─── Joystick Output ────────────────────────────────────────────────────────
#define JOYSTICK_SEND_INTERVAL_MS  4   // 250 Hz USB report rate

// ─── EEPROM ─────────────────────────────────────────────────────────────────
#define EEPROM_SIZE             64
#define EEPROM_MAGIC            0x12345678
#define EEPROM_MAGIC_ADDR        0
#define EEPROM_MIN_CLUTCH_ADDR   4
#define EEPROM_MAX_CLUTCH_ADDR   8
#define EEPROM_MIN_BRAKE_ADDR   12
#define EEPROM_MAX_BRAKE_ADDR   16
#define EEPROM_MIN_GAS_ADDR     20
#define EEPROM_MAX_GAS_ADDR     24
#define EEPROM_MIN_HB_ADDR      28
#define EEPROM_MAX_HB_ADDR      32
#define EEPROM_ALPHA_CLUTCH_ADDR   36
#define EEPROM_ALPHA_BRAKE_ADDR    40
#define EEPROM_ALPHA_GAS_ADDR      44
#define EEPROM_ALPHA_HB_ADDR       48

// ─── Brake Pressure Sensor (fixed to sensor voltage, NOT pedal calibration) ─
// Sensor: 0.5 V = 0 PSI, 4.5 V = 100 PSI, Vref = 5.0 V, 24-bit ADC
// Count = (V / Vref) * 2^23
#define BRAKE_SENSOR_ZERO_PSI_COUNTS  839681   // 0.5 V
#define BRAKE_SENSOR_MAX_PSI_COUNTS   7951127  // 4.5 V
#define BRAKE_SENSOR_MAX_PSI          100.0f

// ────────────────────────────────────────────────────────────────────────────

Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
                   0, 0, true, true, true, true, false, false, false, false);
ADS1220_WE ads(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─── Mutex for shared state between cores ───────────────────────────────────
SemaphoreHandle_t dataMutex;

// ─── Calibration State Machine ──────────────────────────────────────────────
enum CalibState { IDLE, MIN_CALIB, MAX_CALIB, MIN_DONE, MAX_DONE };

// Guarded by dataMutex
CalibState calibState   = IDLE;
int        selectedAxis = 0;   // 0=C, 1=B, 2=G, 3=H
int        calibAxis    = -1;
int        currentScreen = 0;  // 0=Filter, 1=Min/Max

unsigned long calibStartTime = 0;
int32_t       currentMin = 0, currentMax = 0;

// ─── Calibration Values (guarded by dataMutex) ──────────────────────────────
int32_t minClutch = 0, maxClutch = 8388607;
int32_t minBrake  = 0, maxBrake  = 8388607;
int32_t minGas    = 0, maxGas    = 8388607;
int32_t minHB     = 0, maxHB     = 8388607;

// EMA alpha per axis as integer percentage (5–100, step 5).
// Stored under dataMutex.
uint8_t alphaClutch = EMA_ALPHA_DEFAULT, alphaBrake = EMA_ALPHA_DEFAULT;
uint8_t alphaGas    = EMA_ALPHA_DEFAULT, alphaHB    = EMA_ALPHA_DEFAULT;

// ─── ADC Values (guarded by dataMutex) ──────────────────────────────────────
int32_t valueClutch = 0, valueBrake = 0, valueGas = 0, valueHB = 0;

// ─── DRDY Interrupt ─────────────────────────────────────────────────────────
volatile bool dataReady = false;
void IRAM_ATTR drdyISR() { dataReady = true; }

// ────────────────────────────────────────────────────────────────────────────
// Helpers
// ────────────────────────────────────────────────────────────────────────────

long customMap(int32_t x, int32_t in_min, int32_t in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min; // Guard against divide-by-zero
  int64_t result = (int64_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min) return out_min;
  if (result > out_max) return out_max;
  return (long)result;
}

// PSI is derived from raw ADC counts tied to sensor voltage, not pedal calibration.
// Changing min/max calibration will NOT affect PSI readings.
float calculatePSIBrake(int32_t raw) {
  float psi = (float)(raw - BRAKE_SENSOR_ZERO_PSI_COUNTS)
              * BRAKE_SENSOR_MAX_PSI
              / (BRAKE_SENSOR_MAX_PSI_COUNTS - BRAKE_SENSOR_ZERO_PSI_COUNTS);
  return constrain(psi, 0.0f, BRAKE_SENSOR_MAX_PSI);
}

// Human-readable label for EMA alpha percentage.
const char* alphaLabel(uint8_t alpha) {
  if (alpha <= 20) return "Smooth";
  if (alpha <= 60) return "Mid";
  return "Fast";
}


// ────────────────────────────────────────────────────────────────────────────

void saveCalibration() {
  EEPROM.put(EEPROM_MIN_CLUTCH_ADDR,     minClutch);
  EEPROM.put(EEPROM_MAX_CLUTCH_ADDR,     maxClutch);
  EEPROM.put(EEPROM_MIN_BRAKE_ADDR,      minBrake);
  EEPROM.put(EEPROM_MAX_BRAKE_ADDR,      maxBrake);
  EEPROM.put(EEPROM_MIN_GAS_ADDR,        minGas);
  EEPROM.put(EEPROM_MAX_GAS_ADDR,        maxGas);
  EEPROM.put(EEPROM_MIN_HB_ADDR,         minHB);
  EEPROM.put(EEPROM_MAX_HB_ADDR,         maxHB);
  EEPROM.put(EEPROM_ALPHA_CLUTCH_ADDR,   alphaClutch);
  EEPROM.put(EEPROM_ALPHA_BRAKE_ADDR,    alphaBrake);
  EEPROM.put(EEPROM_ALPHA_GAS_ADDR,      alphaGas);
  EEPROM.put(EEPROM_ALPHA_HB_ADDR,       alphaHB);
  EEPROM.commit();
}

void loadCalibration() {
  uint32_t magic;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC) {
    saveCalibration();
    EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
    EEPROM.commit();
  } else {
    EEPROM.get(EEPROM_MIN_CLUTCH_ADDR,     minClutch);
    EEPROM.get(EEPROM_MAX_CLUTCH_ADDR,     maxClutch);
    EEPROM.get(EEPROM_MIN_BRAKE_ADDR,      minBrake);
    EEPROM.get(EEPROM_MAX_BRAKE_ADDR,      maxBrake);
    EEPROM.get(EEPROM_MIN_GAS_ADDR,        minGas);
    EEPROM.get(EEPROM_MAX_GAS_ADDR,        maxGas);
    EEPROM.get(EEPROM_MIN_HB_ADDR,         minHB);
    EEPROM.get(EEPROM_MAX_HB_ADDR,         maxHB);
    EEPROM.get(EEPROM_ALPHA_CLUTCH_ADDR,   alphaClutch);
    EEPROM.get(EEPROM_ALPHA_BRAKE_ADDR,    alphaBrake);
    EEPROM.get(EEPROM_ALPHA_GAS_ADDR,      alphaGas);
    EEPROM.get(EEPROM_ALPHA_HB_ADDR,       alphaHB);
    // Clamp to valid step-aligned range
    alphaClutch = constrain((alphaClutch / EMA_ALPHA_STEP) * EMA_ALPHA_STEP, EMA_ALPHA_STEP, EMA_ALPHA_MAX);
    alphaBrake  = constrain((alphaBrake  / EMA_ALPHA_STEP) * EMA_ALPHA_STEP, EMA_ALPHA_STEP, EMA_ALPHA_MAX);
    alphaGas    = constrain((alphaGas    / EMA_ALPHA_STEP) * EMA_ALPHA_STEP, EMA_ALPHA_STEP, EMA_ALPHA_MAX);
    alphaHB     = constrain((alphaHB     / EMA_ALPHA_STEP) * EMA_ALPHA_STEP, EMA_ALPHA_STEP, EMA_ALPHA_MAX);
  }
}

void resetDefaultCalibration(int axis) {
  if      (axis == 0) { minClutch = 0; maxClutch = 8388607; alphaClutch = EMA_ALPHA_DEFAULT; }
  else if (axis == 1) { minBrake  = 0; maxBrake  = 8388607; alphaBrake  = EMA_ALPHA_DEFAULT; }
  else if (axis == 2) { minGas    = 0; maxGas    = 8388607; alphaGas    = EMA_ALPHA_DEFAULT; }
  else                { minHB     = 0; maxHB     = 8388607; alphaHB     = EMA_ALPHA_DEFAULT; }
  saveCalibration();
}

// ────────────────────────────────────────────────────────────────────────────
// Core 0 — Display & Control Task
// ────────────────────────────────────────────────────────────────────────────

// Returns true once on a falling edge; updates lastState each call.
bool buttonPressed(int pin, bool &lastState) {
  bool state = digitalRead(pin);
  bool pressed = (state == LOW && lastState == HIGH);
  lastState = state;
  return pressed;
}

void displayAndControlTask(void *pvParameters) {
  bool lastPrev      = HIGH;
  bool lastNextPage  = HIGH;
  bool lastCalibrate = HIGH;
  bool lastDefault   = HIGH;

  unsigned long lastButtonCheck   = 0;
  unsigned long lastDisplayUpdate = 0;
  unsigned long pendingSaveAt     = 0; // Deferred EEPROM save timestamp

  while (1) {
    unsigned long now = millis();

    // ── Button polling (debounced) ─────────────────────────────────────────
    if (now - lastButtonCheck >= DEBOUNCE_TIME) {
      lastButtonCheck = now;

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      CalibState cs = calibState;
      xSemaphoreGive(dataMutex);

      if (cs == IDLE) {
        if (buttonPressed(BUTTON_PREV, lastPrev)) {
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          selectedAxis = (selectedAxis - 1 + 4) % 4;
          xSemaphoreGive(dataMutex);
        }

        if (buttonPressed(BUTTON_NEXT_PAGE, lastNextPage)) {
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          currentScreen = (currentScreen + 1) % 2;
          xSemaphoreGive(dataMutex);
        }

        if (buttonPressed(BUTTON_CALIBRATE, lastCalibrate)) {
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          if (currentScreen == 0) {
            // Increment alpha by one step, wrap 100 → 5
            switch (selectedAxis) {
              case 0: alphaClutch = (alphaClutch >= EMA_ALPHA_MAX) ? EMA_ALPHA_STEP : alphaClutch + EMA_ALPHA_STEP; break;
              case 1: alphaBrake  = (alphaBrake  >= EMA_ALPHA_MAX) ? EMA_ALPHA_STEP : alphaBrake  + EMA_ALPHA_STEP; break;
              case 2: alphaGas    = (alphaGas    >= EMA_ALPHA_MAX) ? EMA_ALPHA_STEP : alphaGas    + EMA_ALPHA_STEP; break;
              default: alphaHB   = (alphaHB     >= EMA_ALPHA_MAX) ? EMA_ALPHA_STEP : alphaHB     + EMA_ALPHA_STEP; break;
            }
            pendingSaveAt = now + 2000;
          } else {
            calibState     = MIN_CALIB;
            calibAxis      = selectedAxis;
            calibStartTime = now;
            currentMin     =  8388608;
            currentMax     = -8388608;
          }
          xSemaphoreGive(dataMutex);
        }

        if (buttonPressed(BUTTON_DEFAULT, lastDefault)) {
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          resetDefaultCalibration(selectedAxis);
          xSemaphoreGive(dataMutex);
        }
      }
    }

    // ── Deferred EEPROM save ───────────────────────────────────────────────
    if (pendingSaveAt != 0 && now >= pendingSaveAt) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      saveCalibration();
      xSemaphoreGive(dataMutex);
      pendingSaveAt = 0;
    }

    // ── Calibration state machine ──────────────────────────────────────────
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    if (calibState != IDLE && calibAxis >= 0 && calibAxis < 4) {
      int32_t v = (calibAxis == 0) ? valueClutch
                : (calibAxis == 1) ? valueBrake
                : (calibAxis == 2) ? valueGas
                :                    valueHB;

      switch (calibState) {
        case MIN_CALIB:
          if (v < currentMin) currentMin = v;
          if (now - calibStartTime >= CALIBRATION_TIME) {
            calibState = MIN_DONE; calibStartTime = now;
          }
          break;

        case MIN_DONE:
          if (now - calibStartTime >= DONE_TIME) {
            switch (calibAxis) {
              case 0: minClutch = currentMin; break;
              case 1: minBrake  = currentMin; break;
              case 2: minGas    = currentMin; break;
              default: minHB    = currentMin; break;
            }
            calibState = MAX_CALIB; calibStartTime = now; currentMax = -8388608;
          }
          break;

        case MAX_CALIB:
          if (v > currentMax) currentMax = v;
          if (now - calibStartTime >= CALIBRATION_TIME) {
            calibState = MAX_DONE; calibStartTime = now;
          }
          break;

        case MAX_DONE:
          if (now - calibStartTime >= DONE_TIME) {
            switch (calibAxis) {
              case 0: maxClutch = currentMax; break;
              case 1: maxBrake  = currentMax; break;
              case 2: maxGas    = currentMax; break;
              default: maxHB    = currentMax; break;
            }
            saveCalibration();
            calibState = IDLE; calibAxis = -1;
          }
          break;

        default: break;
      }
    }
    xSemaphoreGive(dataMutex);

    // ── OLED update ───────────────────────────────────────────────────────
    if (now - lastDisplayUpdate >= OLED_REFRESH_MS) {
      lastDisplayUpdate = now;

      // Snapshot all display data under mutex before rendering
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      CalibState cs  = calibState;
      int  ca        = calibAxis;
      int  screen    = currentScreen;
      int  axis      = selectedAxis;
      int32_t cMin   = currentMin,  cMax = currentMax;
      int32_t vC     = valueClutch, vB   = valueBrake;
      int32_t vG     = valueGas,    vH   = valueHB;
      int32_t mnC    = minClutch,   mxC  = maxClutch;
      int32_t mnB    = minBrake,    mxB  = maxBrake;
      int32_t mnG    = minGas,      mxG  = maxGas;
      int32_t mnH    = minHB,       mxH  = maxHB;
      uint8_t aC     = alphaClutch, aB = alphaBrake;
      uint8_t aG     = alphaGas,    aH = alphaHB;
      unsigned long cst = calibStartTime;
      xSemaphoreGive(dataMutex);

      const char* axLabel = (ca == 0) ? "C" : (ca == 1) ? "B" : (ca == 2) ? "G" : "H";
      int32_t axVal = (ca == 0) ? vC : (ca == 1) ? vB : (ca == 2) ? vG : vH;

      display.clearDisplay();
      display.setCursor(0, 0);

      if (cs == MIN_CALIB) {
        float rem = (CALIBRATION_TIME - (float)(now - cst)) / 1000.0f;
        display.print(axLabel); display.print(" MIN: "); display.println(axVal);
        display.print("Time: "); display.print(rem, 1); display.println("s");
      } else if (cs == MIN_DONE) {
        display.print(axLabel); display.print(" MIN: "); display.print(cMin); display.println(" DONE");
      } else if (cs == MAX_CALIB) {
        float rem = (CALIBRATION_TIME - (float)(now - cst)) / 1000.0f;
        display.print(axLabel); display.print(" MAX: "); display.println(axVal);
        display.print("Time: "); display.print(rem, 1); display.println("s");
      } else if (cs == MAX_DONE) {
        display.print(axLabel); display.print(" MAX: "); display.print(cMax); display.println(" DONE");
      } else if (screen == 0) {
        display.println("EMA Filter (%)");
        display.print(axis == 0 ? ">C: " : " C: "); display.print(aC); display.print("  "); display.println(alphaLabel(aC));
        display.print(axis == 1 ? ">B: " : " B: "); display.print(aB); display.print("  "); display.println(alphaLabel(aB));
        display.print(axis == 2 ? ">G: " : " G: "); display.print(aG); display.print("  "); display.println(alphaLabel(aG));
        display.print(axis == 3 ? ">H: " : " H: "); display.print(aH); display.print("  "); display.println(alphaLabel(aH));
      } else {
        display.println("Pedals Min/Max");
        display.print(axis == 0 ? ">C: " : " C: "); display.print(mnC); display.print(" "); display.println(mxC);
        display.print(axis == 1 ? ">B: " : " B: "); display.print(mnB); display.print(" "); display.println(mxB);
        display.print(axis == 2 ? ">G: " : " G: "); display.print(mnG); display.print(" "); display.println(mxG);
        display.print(axis == 3 ? ">H: " : " H: "); display.print(mnH); display.print(" "); display.println(mxH);
        display.print("PSI B: "); display.println(calculatePSIBrake(vB), 1);
      }
      display.display();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ────────────────────────────────────────────────────────────────────────────
// Setup
// ────────────────────────────────────────────────────────────────────────────

void setup() {
  joystick.begin();
  joystick.setXAxisRange(0, 65535);
  joystick.setYAxisRange(0, 65535);
  joystick.setZAxisRange(0, 65535);
  joystick.setRxAxisRange(0, 65535);

  EEPROM.begin(EEPROM_SIZE);
  loadCalibration();

  pinMode(BUTTON_PREV,      INPUT_PULLUP);
  pinMode(BUTTON_NEXT_PAGE, INPUT_PULLUP);
  pinMode(BUTTON_CALIBRATE, INPUT_PULLUP);
  pinMode(BUTTON_DEFAULT,   INPUT_PULLUP);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, ADS1220_CS_PIN);
  if (!ads.init()) while (1);
  ads.setSPIClockSpeed(10000000);
  ads.setVRefSource(ADS1220_VREF_REFP0_REFN0);
  ads.setVRefValue_V(5.0);
  ads.setGain(ADS1220_GAIN_1);
  ads.bypassPGA(true);
  ads.setDataRate(ADS1220_DR_LVL_6);
  ads.setOperatingMode(ADS1220_TURBO_MODE);
  ads.setConversionMode(ADS1220_CONTINUOUS);
  ads.setFIRFilter(ADS1220_50HZ_60HZ);

  pinMode(ADS1220_DRDY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ADS1220_DRDY_PIN), drdyISR, FALLING);

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) while (1);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Pedals Calibration");
  display.display();

  dataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    displayAndControlTask, "DisplayControlTask", 4096, NULL, 1, NULL, 0
  );
}

// ────────────────────────────────────────────────────────────────────────────
// Core 1 — ADC Read Loop
// ────────────────────────────────────────────────────────────────────────────

void loop() {
  // EMA accumulators — persist across iterations (static)
  static int32_t emaClutch = 0, emaBrake = 0, emaGas = 0, emaHB = 0;
  static bool    emaInit   = false;
  static int32_t lastValueClutch = 0, lastValueBrake = 0;
  static int32_t lastValueGas    = 0, lastValueHB    = 0;
  static unsigned long lastJoySend = 0;

  // Snapshot alpha values under mutex
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  uint8_t aC = alphaClutch, aB = alphaBrake;
  uint8_t aG = alphaGas,    aH = alphaHB;
  xSemaphoreGive(dataMutex);

  // ── Single read per channel ───────────────────────────────────────────────
  ads.setCompareChannels(ADS1220_MUX_0_AVSS);
  while (!dataReady); dataReady = false;
  int32_t rawC = ads.getRawData();

  ads.setCompareChannels(ADS1220_MUX_1_AVSS);
  while (!dataReady); dataReady = false;
  int32_t rawB = ads.getRawData();

  ads.setCompareChannels(ADS1220_MUX_2_AVSS);
  while (!dataReady); dataReady = false;
  int32_t rawG = ads.getRawData();

  ads.setCompareChannels(ADS1220_MUX_3_AVSS);
  while (!dataReady); dataReady = false;
  int32_t rawH = ads.getRawData();

  // ── Seed EMA on first run to avoid startup transient ─────────────────────
  if (!emaInit) {
    emaClutch = rawC; emaBrake = rawB; emaGas = rawG; emaHB = rawH;
    lastValueClutch = rawC; lastValueBrake = rawB;
    lastValueGas    = rawG; lastValueHB    = rawH;
    emaInit = true;
  }

  // ── EMA: output = prev + alpha% * (raw - prev) ───────────────────────────
  // Integer arithmetic: multiply first, divide last to preserve precision.
  emaClutch += (int32_t)(rawC - emaClutch) * aC / 100;
  emaBrake  += (int32_t)(rawB - emaBrake)  * aB / 100;
  emaGas    += (int32_t)(rawG - emaGas)    * aG / 100;
  emaHB     += (int32_t)(rawH - emaHB)     * aH / 100;

  // ── Deadband on top of EMA — suppresses jitter on held pedals ─────────────
  if (abs(emaClutch - lastValueClutch) >= DEADBAND_COUNTS) lastValueClutch = emaClutch;
  if (abs(emaBrake  - lastValueBrake)  >= DEADBAND_COUNTS) lastValueBrake  = emaBrake;
  if (abs(emaGas    - lastValueGas)    >= DEADBAND_COUNTS) lastValueGas    = emaGas;
  if (abs(emaHB     - lastValueHB)     >= DEADBAND_COUNTS) lastValueHB     = emaHB;

  // ── Publish filtered values and read calibration ranges under mutex ───────
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  valueClutch = lastValueClutch;
  valueBrake  = lastValueBrake;
  valueGas    = lastValueGas;
  valueHB     = lastValueHB;
  int32_t mnC = minClutch, mxC = maxClutch;
  int32_t mnB = minBrake,  mxB = maxBrake;
  int32_t mnG = minGas,    mxG = maxGas;
  int32_t mnH = minHB,     mxH = maxHB;
  xSemaphoreGive(dataMutex);

  // ── Map to 0–65535 ────────────────────────────────────────────────────────
  int mappedC = customMap(lastValueClutch, mnC, mxC, 0, 65535);
  int mappedB = customMap(lastValueBrake,  mnB, mxB, 0, 65535);
  int mappedG = customMap(lastValueGas,    mnG, mxG, 0, 65535);
  int mappedH = customMap(lastValueHB,     mnH, mxH, 0, 65535);

  joystick.setXAxis(mappedC);
  joystick.setYAxis(mappedB);
  joystick.setZAxis(mappedG);
  joystick.setRxAxis(mappedH);

  // ── Send at explicit 250 Hz rate ──────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastJoySend >= JOYSTICK_SEND_INTERVAL_MS) {
    joystick.sendState();
    lastJoySend = now;
  }
}
