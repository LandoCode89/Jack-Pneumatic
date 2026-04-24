#include <Joystick_ESP32S2.h>
#include <SPI.h>
#include <ADS1220_WE.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define ADS1220_CS_PIN 10      // ADS1220 Chip Select
#define ADS1220_DRDY_PIN 9     // ADS1220 Data Ready
#define SPI_SCK 13             // SPI SCLK
#define SPI_MISO 12            // SPI MISO
#define SPI_MOSI 11            // SPI MOSI
#define OLED_SDA 4             // OLED I2C SDA
#define OLED_SCL 5             // OLED I2C SCL
#define BUTTON_PREV 6          // Previous axis button
#define BUTTON_NEXT_PAGE 7     // Next page button
#define BUTTON_CALIBRATE 2     // Calibrate/filter button
#define BUTTON_DEFAULT 3       // Restore default button

#define SCREEN_WIDTH 128       // OLED width
#define SCREEN_HEIGHT 64       // OLED height
#define OLED_ADDRESS 0x3C      // OLED I2C address
#define CALIBRATION_TIME 4000  // 4 seconds per min/max
#define DONE_TIME 1000         // 1 second for "DONE"
#define DEBOUNCE_TIME 50       // 50 ms debounce
#define OLED_REFRESH_MS 500    // OLED update every 500 ms (2 Hz)
#define DEADBAND_COUNTS 3355   // 2 mV equivalent (2/5000 * 2^23)
#define MAX_SAMPLES 20         // Max averaging samples

#define EEPROM_SIZE 64         // Allocate 64 bytes for EEPROM
#define EEPROM_MAGIC 0x12345678 // Magic number for EEPROM validation
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MIN_CLUTCH_ADDR 4
#define EEPROM_MAX_CLUTCH_ADDR 8
#define EEPROM_MIN_BRAKE_ADDR 12
#define EEPROM_MAX_BRAKE_ADDR 16
#define EEPROM_MIN_GAS_ADDR 20
#define EEPROM_MAX_GAS_ADDR 24
#define EEPROM_MIN_HB_ADDR 28
#define EEPROM_MAX_HB_ADDR 32
#define EEPROM_SAMPLES_CLUTCH_ADDR 36
#define EEPROM_SAMPLES_BRAKE_ADDR 40
#define EEPROM_SAMPLES_GAS_ADDR 44
#define EEPROM_SAMPLES_HB_ADDR 48

Joystick_ joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_JOYSTICK,
                  0, 0, true, true, true, true, false, false, false, false);
ADS1220_WE ads(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Calibration state
enum CalibState { IDLE, MIN_CALIB, MAX_CALIB, MIN_DONE, MAX_DONE };
CalibState calibState = IDLE;
int selectedAxis = 0; // 0=C, 1=B, 2=G, 3=H
int calibAxis = -1;   // Axis being calibrated
unsigned long calibStartTime = 0;
int currentScreen = 0; // 0=Filter Adjust, 1=Min/Max Calibration

// Button states for debouncing
bool lastPrevState = HIGH;
bool lastNextPageState = HIGH;
bool lastCalibrateState = HIGH;
bool lastDefaultState = HIGH;
unsigned long lastButtonCheck = 0;

// Min/max values (defaults: 0 to 2^23-1)
int32_t minClutch = 0, maxClutch = 8388607;
int32_t minBrake = 0, maxBrake = 8388607;
int32_t minGas = 0, maxGas = 8388607;
int32_t minHB = 0, maxHB = 8388607;
int32_t currentMin, currentMax;

// Averaging samples (default: 2)
uint8_t samplesClutch = 2, samplesBrake = 2, samplesGas = 2, samplesHB = 2;

// Thread-safe ADC values
volatile int32_t valueClutch = 0, valueBrake = 0, valueGas = 0, valueHB = 0;
volatile int32_t lastValueClutch = 0, lastValueBrake = 0, lastValueGas = 0, lastValueHB = 0;

// Circular buffer for ADC samples
int32_t sampleBuffer[4][MAX_SAMPLES]; // Buffer for C, B, G, H
uint8_t sampleIndex[4] = {0, 0, 0, 0}; // Current write index per channel
uint8_t sampleCount[4] = {0, 0, 0, 0}; // Valid sample count per channel
uint8_t currentChannel = 0; // Current ADC channel (0=C, 1=B, 2=G, 3=H)

// ISR flags - ONLY set flags in ISR!
volatile bool newDataAvailable = false;
volatile bool channelSwitchNeeded = false;

// Critical section mutex
portMUX_TYPE adcMutex = portMUX_INITIALIZER_UNLOCKED;

// FIXED: ISR now only sets flags - no complex operations!
void IRAM_ATTR drdyISR() {
  newDataAvailable = true;
}

// Thread-safe value getters
int32_t getValueClutch() {
  portENTER_CRITICAL(&adcMutex);
  int32_t val = valueClutch;
  portEXIT_CRITICAL(&adcMutex);
  return val;
}

int32_t getValueBrake() {
  portENTER_CRITICAL(&adcMutex);
  int32_t val = valueBrake;
  portEXIT_CRITICAL(&adcMutex);
  return val;
}

int32_t getValueGas() {
  portENTER_CRITICAL(&adcMutex);
  int32_t val = valueGas;
  portEXIT_CRITICAL(&adcMutex);
  return val;
}

int32_t getValueHB() {
  portENTER_CRITICAL(&adcMutex);
  int32_t val = valueHB;
  portEXIT_CRITICAL(&adcMutex);
  return val;
}

// Integer-based mapping with overflow protection
long customMap(int32_t x, int32_t in_min, int32_t in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min; // Prevent division by zero
  int64_t result = (int64_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (result < out_min) return out_min;
  if (result > out_max) return out_max;
  return (long)result;
}

// PSI calculation for BRAKE (0.5V=0 PSI, 4.5V=100 PSI)
float calculatePSIBrake(int32_t valueBrake) {
  float psi = (float)(valueBrake - 839681) * 100.0 / (7951127 - 839681);
  return constrain(psi, 0.0, 100.0);
}

void saveCalibration() {
  EEPROM.put(EEPROM_MIN_CLUTCH_ADDR, minClutch);
  EEPROM.put(EEPROM_MAX_CLUTCH_ADDR, maxClutch);
  EEPROM.put(EEPROM_MIN_BRAKE_ADDR, minBrake);
  EEPROM.put(EEPROM_MAX_BRAKE_ADDR, maxBrake);
  EEPROM.put(EEPROM_MIN_GAS_ADDR, minGas);
  EEPROM.put(EEPROM_MAX_GAS_ADDR, maxGas);
  EEPROM.put(EEPROM_MIN_HB_ADDR, minHB);
  EEPROM.put(EEPROM_MAX_HB_ADDR, maxHB);
  EEPROM.put(EEPROM_SAMPLES_CLUTCH_ADDR, samplesClutch);
  EEPROM.put(EEPROM_SAMPLES_BRAKE_ADDR, samplesBrake);
  EEPROM.put(EEPROM_SAMPLES_GAS_ADDR, samplesGas);
  EEPROM.put(EEPROM_SAMPLES_HB_ADDR, samplesHB);
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
    EEPROM.get(EEPROM_MIN_CLUTCH_ADDR, minClutch);
    EEPROM.get(EEPROM_MAX_CLUTCH_ADDR, maxClutch);
    EEPROM.get(EEPROM_MIN_BRAKE_ADDR, minBrake);
    EEPROM.get(EEPROM_MAX_BRAKE_ADDR, maxBrake);
    EEPROM.get(EEPROM_MIN_GAS_ADDR, minGas);
    EEPROM.get(EEPROM_MAX_GAS_ADDR, maxGas);
    EEPROM.get(EEPROM_MIN_HB_ADDR, minHB);
    EEPROM.get(EEPROM_MAX_HB_ADDR, maxHB);
    EEPROM.get(EEPROM_SAMPLES_CLUTCH_ADDR, samplesClutch);
    EEPROM.get(EEPROM_SAMPLES_BRAKE_ADDR, samplesBrake);
    EEPROM.get(EEPROM_SAMPLES_GAS_ADDR, samplesGas);
    EEPROM.get(EEPROM_SAMPLES_HB_ADDR, samplesHB);
    samplesClutch = constrain(samplesClutch, 1, MAX_SAMPLES);
    samplesBrake = constrain(samplesBrake, 1, MAX_SAMPLES);
    samplesGas = constrain(samplesGas, 1, MAX_SAMPLES);
    samplesHB = constrain(samplesHB, 1, MAX_SAMPLES);
  }
}

void resetDefaultCalibration(int axis) {
  if (axis == 0) { minClutch = 0; maxClutch = 8388607; samplesClutch = 2; }
  else if (axis == 1) { minBrake = 0; maxBrake = 8388607; samplesBrake = 2; }
  else if (axis == 2) { minGas = 0; maxGas = 8388607; samplesGas = 2; }
  else if (axis == 3) { minHB = 0; maxHB = 8388607; samplesHB = 2; }
  saveCalibration();
}

// FIXED: Proper circular buffer handling
void addSample(uint8_t channel, int32_t value) {
  if (channel >= 4) return; // Bounds check
  
  sampleBuffer[channel][sampleIndex[channel]] = value;
  sampleIndex[channel] = (sampleIndex[channel] + 1) % MAX_SAMPLES;
  
  if (sampleCount[channel] < MAX_SAMPLES) {
    sampleCount[channel]++;
  }
}

// FIXED: Safe averaging with proper bounds checking
int32_t getAverageValue(uint8_t channel) {
  if (channel >= 4 || sampleCount[channel] == 0) return 0;
  
  uint8_t samplesToUse;
  switch (channel) {
    case 0: samplesToUse = min(samplesClutch, sampleCount[channel]); break;
    case 1: samplesToUse = min(samplesBrake, sampleCount[channel]); break;
    case 2: samplesToUse = min(samplesGas, sampleCount[channel]); break;
    case 3: samplesToUse = min(samplesHB, sampleCount[channel]); break;
    default: return 0;
  }
  
  int64_t sum = 0; // Use 64-bit to prevent overflow
  uint8_t startIndex = (sampleIndex[channel] - samplesToUse + MAX_SAMPLES) % MAX_SAMPLES;
  
  for (uint8_t i = 0; i < samplesToUse; i++) {
    sum += sampleBuffer[channel][(startIndex + i) % MAX_SAMPLES];
  }
  
  return (int32_t)(sum / samplesToUse);
}

// Core 0 task for buttons, calibration, and OLED
void displayAndControlTask(void *pvParameters) {
  unsigned long lastDisplayUpdate = 0;
  
  while (1) {
    unsigned long currentMillis = millis();

    // FIXED: Proper button handling with better debouncing
    if (currentMillis - lastButtonCheck >= DEBOUNCE_TIME && calibState == IDLE) {
      bool prevState = digitalRead(BUTTON_PREV);
      bool nextPageState = digitalRead(BUTTON_NEXT_PAGE);
      bool calibrateState = digitalRead(BUTTON_CALIBRATE);
      bool defaultState = digitalRead(BUTTON_DEFAULT);

      if (prevState == LOW && lastPrevState == HIGH) {
        selectedAxis = (selectedAxis + 3) % 4; // Fixed: proper previous calculation
        delay(200); // Simple debounce delay
      }
      lastPrevState = prevState;

      if (nextPageState == LOW && lastNextPageState == HIGH) {
        currentScreen = (currentScreen + 1) % 2;
        delay(200);
      }
      lastNextPageState = nextPageState;

      if (calibrateState == LOW && lastCalibrateState == HIGH) {
        if (currentScreen == 0) {
          // Filter adjustment
          if (selectedAxis == 0) samplesClutch = (samplesClutch % MAX_SAMPLES) + 1;
          else if (selectedAxis == 1) samplesBrake = (samplesBrake % MAX_SAMPLES) + 1;
          else if (selectedAxis == 2) samplesGas = (samplesGas % MAX_SAMPLES) + 1;
          else samplesHB = (samplesHB % MAX_SAMPLES) + 1;
          saveCalibration();
        } else {
          // Start calibration
          calibState = MIN_CALIB;
          calibAxis = selectedAxis;
          calibStartTime = currentMillis;
          currentMin = 8388608;
          currentMax = -8388608;
        }
        delay(200);
      }
      lastCalibrateState = calibrateState;

      if (defaultState == LOW && lastDefaultState == HIGH) {
        resetDefaultCalibration(selectedAxis);
        delay(200);
      }
      lastDefaultState = defaultState;

      lastButtonCheck = currentMillis;
    }

    // FIXED: Calibration using thread-safe getters
    if (calibState != IDLE && calibAxis >= 0 && calibAxis < 4) {
      int32_t value;
      switch (calibAxis) {
        case 0: value = getValueClutch(); break;
        case 1: value = getValueBrake(); break;
        case 2: value = getValueGas(); break;
        case 3: value = getValueHB(); break;
        default: value = 0;
      }
      
      if (calibState == MIN_CALIB) {
        if (value < currentMin) currentMin = value;
        if (currentMillis - calibStartTime >= CALIBRATION_TIME) {
          calibState = MIN_DONE;
          calibStartTime = currentMillis;
        }
      } else if (calibState == MIN_DONE && currentMillis - calibStartTime >= DONE_TIME) {
        if (calibAxis == 0) minClutch = currentMin;
        else if (calibAxis == 1) minBrake = currentMin;
        else if (calibAxis == 2) minGas = currentMin;
        else minHB = currentMin;
        calibState = MAX_CALIB;
        calibStartTime = currentMillis;
        currentMax = -8388608; // Reset for max calibration
      } else if (calibState == MAX_CALIB) {
        if (value > currentMax) currentMax = value;
        if (currentMillis - calibStartTime >= CALIBRATION_TIME) {
          calibState = MAX_DONE;
          calibStartTime = currentMillis;
        }
      } else if (calibState == MAX_DONE && currentMillis - calibStartTime >= DONE_TIME) {
        if (calibAxis == 0) maxClutch = currentMax;
        else if (calibAxis == 1) maxBrake = currentMax;
        else if (calibAxis == 2) maxGas = currentMax;
        else maxHB = currentMax;
        saveCalibration();
        calibState = IDLE;
        calibAxis = -1;
      }
    }

    // FIXED: Display update using thread-safe getters
    if (currentMillis - lastDisplayUpdate >= OLED_REFRESH_MS) {
      display.clearDisplay();
      display.setCursor(0, 0);
      
      if (calibState == MIN_CALIB) {
        display.print(calibAxis == 0 ? "C" : calibAxis == 1 ? "B" : calibAxis == 2 ? "G" : "H");
        display.print(" MIN: ");
        switch (calibAxis) {
          case 0: display.println(getValueClutch()); break;
          case 1: display.println(getValueBrake()); break;
          case 2: display.println(getValueGas()); break;
          case 3: display.println(getValueHB()); break;
        }
        display.print("Time: ");
        float remaining = (CALIBRATION_TIME - (currentMillis - calibStartTime)) / 1000.0;
        display.print(remaining, 1);
        display.println("s");
      } else if (calibState == MIN_DONE) {
        display.print(calibAxis == 0 ? "C" : calibAxis == 1 ? "B" : calibAxis == 2 ? "G" : "H");
        display.print(" MIN: ");
        display.print(currentMin);
        display.println(" DONE");
      } else if (calibState == MAX_CALIB) {
        display.print(calibAxis == 0 ? "C" : calibAxis == 1 ? "B" : calibAxis == 2 ? "G" : "H");
        display.print(" MAX: ");
        switch (calibAxis) {
          case 0: display.println(getValueClutch()); break;
          case 1: display.println(getValueBrake()); break;
          case 2: display.println(getValueGas()); break;
          case 3: display.println(getValueHB()); break;
        }
        display.print("Time: ");
        float remaining = (CALIBRATION_TIME - (currentMillis - calibStartTime)) / 1000.0;
        display.print(remaining, 1);
        display.println("s");
      } else if (calibState == MAX_DONE) {
        display.print(calibAxis == 0 ? "C" : calibAxis == 1 ? "B" : calibAxis == 2 ? "G" : "H");
        display.print(" MAX: ");
        display.print(currentMax);
        display.println(" DONE");
      } else if (currentScreen == 0) {
        display.println("Filter Settings");
        display.print(selectedAxis == 0 ? ">C: " : " C: "); display.println(samplesClutch);
        display.print(selectedAxis == 1 ? ">B: " : " B: "); display.println(samplesBrake);
        display.print(selectedAxis == 2 ? ">G: " : " G: "); display.println(samplesGas);
        display.print(selectedAxis == 3 ? ">H: " : " H: "); display.println(samplesHB);
      } else {
        display.println("Pedals Min/Max");
        display.print(selectedAxis == 0 ? ">C: " : " C: "); display.print(minClutch); display.print(" "); display.println(maxClutch);
        display.print(selectedAxis == 1 ? ">B: " : " B: "); display.print(minBrake); display.print(" "); display.println(maxBrake);
        display.print(selectedAxis == 2 ? ">G: " : " G: "); display.print(minGas); display.print(" "); display.println(maxGas);
        display.print(selectedAxis == 3 ? ">H: " : " H: "); display.print(minHB); display.print(" "); display.println(maxHB);
        display.print("PSI B: "); display.println(calculatePSIBrake(getValueBrake()), 1);
      }
      display.display();
      lastDisplayUpdate = currentMillis;
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200); // Added for debugging
  
  joystick.begin();
  joystick.setXAxisRange(0, 65535);
  joystick.setYAxisRange(0, 65535);
  joystick.setZAxisRange(0, 65535);
  joystick.setRxAxisRange(0, 65535);

  EEPROM.begin(EEPROM_SIZE);
  loadCalibration();

  pinMode(BUTTON_PREV, INPUT_PULLUP);
  pinMode(BUTTON_NEXT_PAGE, INPUT_PULLUP);
  pinMode(BUTTON_CALIBRATE, INPUT_PULLUP);
  pinMode(BUTTON_DEFAULT, INPUT_PULLUP);

  // FIXED: Initialize SPI properly
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, ADS1220_CS_PIN);
  
  if (!ads.init()) {
    Serial.println("ADS1220 init failed!");
    while (1) {
      delay(1000);
    }
  }
  
  ads.setSPIClockSpeed(10000000);
  ads.setVRefSource(ADS1220_VREF_REFP0_REFN0);
  ads.setVRefValue_V(5.0);
  ads.setGain(ADS1220_GAIN_1);
  ads.bypassPGA(true);
  ads.setDataRate(ADS1220_DR_LVL_6); // 1200 SPS
  ads.setOperatingMode(ADS1220_TURBO_MODE);
  ads.setConversionMode(ADS1220_CONTINUOUS);
  ads.setFIRFilter(ADS1220_50HZ_60HZ);

  // Initialize first channel
  ads.setCompareChannels(ADS1220_MUX_0_AVSS); // Start with CLUTCH
  pinMode(ADS1220_DRDY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ADS1220_DRDY_PIN), drdyISR, FALLING);

  // FIXED: Initialize I2C properly
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED init failed!");
    while (1) {
      delay(1000);
    }
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Pedals Calibration");
  display.display();

  xTaskCreatePinnedToCore(
    displayAndControlTask, "DisplayControlTask", 4096, NULL, 1, NULL, 0
  );
  
  Serial.println("Setup complete");
}

void loop() {
  // FIXED: Proper ADC handling outside ISR
  if (newDataAvailable) {
    newDataAvailable = false;
    
    // Read ADC data (moved from ISR)
    int32_t data = ads.getRawData();
    
    // Add to circular buffer
    addSample(currentChannel, data);
    
    // Switch to next channel
    currentChannel = (currentChannel + 1) % 4;
    
    // Set next channel for ADC
    switch (currentChannel) {
      case 0: ads.setCompareChannels(ADS1220_MUX_0_AVSS); break;
      case 1: ads.setCompareChannels(ADS1220_MUX_1_AVSS); break;
      case 2: ads.setCompareChannels(ADS1220_MUX_2_AVSS); break;
      case 3: ads.setCompareChannels(ADS1220_MUX_3_AVSS); break;
    }
  }
  
  // FIXED: Compute averages safely
  int32_t newValueClutch = getAverageValue(0);
  int32_t newValueBrake = getAverageValue(1);
  int32_t newValueGas = getAverageValue(2);
  int32_t newValueHB = getAverageValue(3);

  // FIXED: Apply deadband filter with thread safety
  portENTER_CRITICAL(&adcMutex);
  
  if (abs(newValueClutch - lastValueClutch) >= DEADBAND_COUNTS) {
    valueClutch = newValueClutch;
    lastValueClutch = newValueClutch;
  }
  if (abs(newValueBrake - lastValueBrake) >= DEADBAND_COUNTS) {
    valueBrake = newValueBrake;
    lastValueBrake = newValueBrake;
  }
  if (abs(newValueGas - lastValueGas) >= DEADBAND_COUNTS) {
    valueGas = newValueGas;
    lastValueGas = newValueGas;
  }
  if (abs(newValueHB - lastValueHB) >= DEADBAND_COUNTS) {
    valueHB = newValueHB;
    lastValueHB = newValueHB;
  }
  
  // Get current values for mapping
  int32_t currentClutch = valueClutch;
  int32_t currentBrake = valueBrake;
  int32_t currentGas = valueGas;
  int32_t currentHB = valueHB;
  
  portEXIT_CRITICAL(&adcMutex);

  // Map to 0–65535
  int mappedClutch = customMap(currentClutch, minClutch, maxClutch, 0, 65535);
  int mappedBrake = customMap(currentBrake, minBrake, maxBrake, 0, 65535);
  int mappedGas = customMap(currentGas, minGas, maxGas, 0, 65535);
  int mappedHB = customMap(currentHB, minHB, maxHB, 0, 65535);

  // Send joystick data
  joystick.setXAxis(mappedClutch);
  joystick.setYAxis(mappedBrake);
  joystick.setZAxis(mappedGas);
  joystick.setRxAxis(mappedHB);
  joystick.sendState();
  
  // Small delay to prevent overwhelming the system
  delay(1);
}