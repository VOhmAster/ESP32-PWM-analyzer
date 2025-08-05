// -------------------------
// 1. Libraries
// -------------------------
#include <SPI.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

// -------------------------
// 2. Wi-Fi settings
// -------------------------
const char* ssid = "Németh";
const char* password = "HalaszAniko.1975";

// -------------------------
// 3. GPIO definitions
// -------------------------

// RGB LED pins
#define RED_PIN    37
#define GREEN_PIN  36
#define BLUE_PIN   38

// Encoder pins
#define ENC_A   17
#define ENC_B   18
#define ENC_SW  9

// Menu buttons
#define BTN_MENU1  20
#define BTN_MENU2  21
#define BTN_MENU3  47

// PWM input pins
#define PWM1_PIN 45
#define PWM2_PIN 46

// SPI display pin mapping
#define SPI_MOSI_PIN 11
#define SPI_SCK_PIN  12
#define SPI_CS_PIN   10
#define SPI_RES_PIN  13
#define SPI_DC_PIN   14

// -------------------------
// 4. Display initialization
// -------------------------
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(
  U8G2_R2,
  /*CS=*/SPI_CS_PIN,
  /*DC=*/SPI_DC_PIN,
  /*RESET=*/SPI_RES_PIN
);

// -------------------------
// 5. Variables for display rendering
// -------------------------
int dutyX = 220, dutyY = 7, barWidth = 8, maxHeight = 15, offset = 20;

const int waveformStartX = 40;
const int waveformEndX = 200;
const int waveformWidth = waveformEndX - waveformStartX;

uint8_t waveform1[waveformWidth] = {0};
uint8_t waveform2[waveformWidth] = {0};
#define WAVEFORM_EMPTY 255

int yTop1 = 10, yBottom1 = 20;
int yTop2 = 35, yBottom2 = 45;

int frequDisplayUnit = 0;  // 0: Hz, 1: kHz, 2: MHz

// -------------------------
// 6. Zoom and view state
// -------------------------
int freqZoomLevel = 0;         // 0: 300-999Hz, 1: 1k-499kHz, 2: 500k-999kHz, 3: 1M-2MHz
const int maxZoomLevel = 3;
bool freezeMode = false;

// -------------------------
// 7. Menu system variables
// -------------------------
byte selectedMenu = 0;
static byte lastSelectedMenu = 255;
bool prevBtn1 = HIGH, prevBtn2 = HIGH, prevBtn3 = HIGH;

// -------------------------
// 8. Encoder state handling
// -------------------------
int lastEncoderValue = 0;
volatile int encoderValue = 0;
int lastA = 0;
int lastB = 0;
int lastButtonState = HIGH;

// -------------------------
// 9. PWM signal processing – timing and detection
// -------------------------
unsigned long lastISRTime1 = 0;
unsigned long lastISRTime2 = 0;

volatile unsigned long riseTime1 = 0, pwmPeriod1 = 0, highTime1 = 0, lowTime1 = 0;
volatile unsigned long riseTime2 = 0, pwmPeriod2 = 0, highTime2 = 0, lowTime2 = 0;

volatile unsigned long dutyCycle1 = 0;
volatile unsigned long dutyCycle2 = 0;

// -------------------------
// 10. Averaged frequencies and strings for display
// -------------------------
float avgFreq1 = 0, avgFreq2 = 0;
String freq1Str = "", unit1 = "";
String freq2Str = "", unit2 = "";

// -------------------------
// 11. Sampling buffers
// -------------------------
#define NUM_SAMPLES 8

// CH1 - First channel
volatile unsigned long periods1[NUM_SAMPLES] = {0};
volatile unsigned long freqPeriods1[NUM_SAMPLES] = {0};
volatile unsigned long dutyCycles1[NUM_SAMPLES] = {0};
volatile byte sampleIndex1 = 0;

// CH2 - Second channel
volatile unsigned long periods2[NUM_SAMPLES] = {0};
volatile unsigned long freqPeriods2[NUM_SAMPLES] = {0};
volatile unsigned long dutyCycles2[NUM_SAMPLES] = {0};
volatile byte sampleIndex2 = 0;

// -------------------------
// 12. PWM state and counters (for waveform rendering, etc.)
// -------------------------
int state1 = 0, counter1 = 0;
int state2 = 0, counter2 = 0;

int highLen1 = 3, lowLen1 = 3;
int highLen2 = 3, lowLen2 = 3;

// -------------------------
// 13. Other settings
// -------------------------
#define SIGNAL_TIMEOUT 200        // Timeout detection in milliseconds
unsigned int updateInterval = 200;  // Data refresh interval

bool hasSignal1() {
  return (millis() - (lastISRTime1 / 1000)) < SIGNAL_TIMEOUT;
}

bool hasSignal2() {
  return (millis() - (lastISRTime2 / 1000)) < SIGNAL_TIMEOUT;
}

void setup() {
  // Serial for debugging if needed
  // Serial.begin(115200);
  delay(1000);

  // --- OLED display initialization ---
  u8g2.setBusClock(2000000);
  u8g2.begin();
  u8g2.setPowerSave(0);

  // Set RGB LED pins as outputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Configure PWM channels using LEDC API
  ledcAttach(RED_PIN, 5000, 8);    // Frequency: 5kHz, resolution: 8-bit
  ledcAttach(GREEN_PIN, 5000, 8);
  ledcAttach(BLUE_PIN, 5000, 8);

  u8g2.setFont(u8g2_font_ncenB08_tr);

  // Initialize waveform buffers
  for (int i = 0; i < waveformWidth; i++) {
    waveform1[i] = WAVEFORM_EMPTY;
    waveform2[i] = WAVEFORM_EMPTY;
  }

  // --- Wi-Fi connection ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long wifiStart = millis();
  bool wifiOk = false;

  while (millis() - wifiStart < 5000) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiOk = true;
      break;
    }
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "Connecting to WiFi...");
    u8g2.sendBuffer();
    delay(500);
  }

  if (wifiOk) {
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "WiFi OK");
    String ipText = "IP: " + WiFi.localIP().toString();
    u8g2.drawStr(0, 28, ipText.c_str());
    u8g2.sendBuffer();
    delay(2000);

    // --- OTA configuration (only if WiFi is available) ---
    ArduinoOTA.setHostname("esp32-pwm-analyzer");

    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) type = "Sketch";
      else type = "Filesystem";
      String msg = "Starting: " + type;

      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, "OTA update starting");
      u8g2.drawStr(0, 28, msg.c_str());
      u8g2.sendBuffer();
    });

    ArduinoOTA.onEnd([]() {
      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, "Update complete!");
      u8g2.sendBuffer();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      uint8_t percent = progress / (total / 100);
      String msg = "Updating: " + String(percent) + "%";

      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, msg.c_str());
      u8g2.sendBuffer();
    });

    ArduinoOTA.onError([](ota_error_t error) {
      u8g2.clearBuffer();
      u8g2.drawStr(0, 12, "OTA Error!");

      switch (error) {
        case OTA_AUTH_ERROR:
          u8g2.drawStr(0, 28, "Auth failed");
          break;
        case OTA_BEGIN_ERROR:
          u8g2.drawStr(0, 28, "Begin failed");
          break;
        case OTA_CONNECT_ERROR:
          u8g2.drawStr(0, 28, "Connect failed");
          break;
        case OTA_RECEIVE_ERROR:
          u8g2.drawStr(0, 28, "Receive failed");
          break;
        case OTA_END_ERROR:
          u8g2.drawStr(0, 28, "End failed");
          break;
      }

      u8g2.sendBuffer();
    });

    ArduinoOTA.begin();
  } else {
    u8g2.clearBuffer();
    u8g2.drawStr(0, 12, "WiFi not available");
    u8g2.drawStr(0, 28, "Starting offline");
    u8g2.sendBuffer();
    delay(2000);
  }

  // Input pin setup
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);

  pinMode(BTN_MENU1, INPUT_PULLUP);
  pinMode(BTN_MENU2, INPUT_PULLUP);
  pinMode(BTN_MENU3, INPUT_PULLUP);

  pinMode(PWM1_PIN, INPUT);
  pinMode(PWM2_PIN, INPUT);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM1_PIN), pwmISR_CH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM2_PIN), pwmISR_CH2, CHANGE);

  // Initialize encoder state
  lastA = digitalRead(ENC_A);
  lastB = digitalRead(ENC_B);
  delay(1000);
}

void loop() {
  ArduinoOTA.handle();  
  
  ReadEncoderButton();

  handleStatusLED(hasSignal1(), hasSignal2());

  int btn1 = digitalRead(BTN_MENU1);
  int btn2 = digitalRead(BTN_MENU2);
  int btn3 = digitalRead(BTN_MENU3);

  if (btn1 == LOW && prevBtn1 == HIGH) selectedMenu = 0;  // Main screen
  if (btn2 == LOW && prevBtn2 == HIGH) selectedMenu = 1;  // Detailed screen
  if (btn3 == LOW && prevBtn3 == HIGH) selectedMenu = 2;  // Graphical screen

  prevBtn1 = btn1;
  prevBtn2 = btn2;
  prevBtn3 = btn3;

  drawMenu();
}

void drawMenu() {
  u8g2.clearBuffer();

  // Underline the active menu item
  switch (selectedMenu) {
    case 0: u8g2.drawHLine(5, 63, 25); break;
    case 1: u8g2.drawHLine(65, 63, 25); break;
    case 2: u8g2.drawHLine(115, 63, 30); break;
  }

  // Render content based on selected menu
  switch (selectedMenu) {
    case 0: MainDisplay(); break;
    case 1: drawDetailScreen(); break;
    case 2: drawGraphScreen(); break;
  }

  u8g2.sendBuffer();
} 

void drawDetailScreen() {
  // -----------------------------
  // 1. Static variables (retain values between calls)
  // -----------------------------
  static unsigned long lastUpdate = 0;

  // Values for display that do not change every loop
  static float dutyPct1 = 0, dutyPct2 = 0;
  static unsigned long riseToRiseCH1 = 0, riseToRiseCH2 = 0;
  static uint8_t glitchCountCH1 = 0, glitchCountCH2 = 0;
  static unsigned long savedPwmPeriod1 = 0, savedPwmPeriod2 = 0;

  // Previous rising edge timestamps for both channels
  static unsigned long prevRiseTime1 = 0;
  static unsigned long prevRiseTime2 = 0;

  // -----------------------------
  // 2. Read encoder (for update interval adjustment)
  // -----------------------------
  handleEncoderUpdateInterval();

  // -----------------------------
  // 3. Timing: update only when due and not in freeze mode
  // -----------------------------
  unsigned long currentMillis = millis();
  if (!freezeMode && currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis;

    // Timeout handling: reset data if ISR hasn't triggered recently
    if ((micros() - lastISRTime1) > 100000) {
      highTime1 = 0;
      pwmPeriod1 = 0;
      lowTime1 = 0;
      dutyPct1 = 0;
      savedPwmPeriod1 = 0;
      riseToRiseCH1 = 0;
      glitchCountCH1 = 0;
    }

    if ((micros() - lastISRTime2) > 100000) {
      highTime2 = 0;
      pwmPeriod2 = 0;
      lowTime2 = 0;
      dutyPct2 = 0;
      savedPwmPeriod2 = 0;
      riseToRiseCH2 = 0;
      glitchCountCH2 = 0;
    }

    // Save values for rendering
    savedPwmPeriod1 = pwmPeriod1;
    savedPwmPeriod2 = pwmPeriod2;

    // -----------------------------
    // 4. CH1 – calculations
    // -----------------------------
    dutyPct1 = (pwmPeriod1 > 0) ? (float)highTime1 / pwmPeriod1 * 100.0 : 0;
    riseToRiseCH1 = (riseTime1 > prevRiseTime1) ? (riseTime1 - prevRiseTime1) : 0;
    prevRiseTime1 = riseTime1;

    glitchCountCH1 = 0;
    if (highTime1 < 10) glitchCountCH1++;
    if (lowTime1 < 10) glitchCountCH1++;

    // -----------------------------
    // 5. CH2 – calculations
    // -----------------------------
    dutyPct2 = (pwmPeriod2 > 0) ? (float)highTime2 / pwmPeriod2 * 100.0 : 0;
    riseToRiseCH2 = (riseTime2 > prevRiseTime2) ? (riseTime2 - prevRiseTime2) : 0;
    prevRiseTime2 = riseTime2;

    glitchCountCH2 = 0;
    if (highTime2 < 10) glitchCountCH2++;
    if (lowTime2 < 10) glitchCountCH2++;
  }

  // -----------------------------
  // 6. Display update (runs every loop)
  // -----------------------------
  u8g2.setFont(u8g2_font_6x12_t_symbols);
  char buf[32];

  // Show freeze status
  if (freezeMode) {
    sprintf(buf, "Freeze");
  } else {
    sprintf(buf, "%u ms", updateInterval);
  }
  u8g2.drawStr(213, 60, buf);

  // -----------------------------
  // 7. CH1 – display values
  // -----------------------------
  u8g2.drawStr(5, 10, "CH1");
  sprintf(buf, "Pulse:  %.1f %%", dutyPct1);                u8g2.drawStr(30, 10, buf);
  sprintf(buf, "Period: %lu \xB5s", savedPwmPeriod1);       u8g2.drawStr(30, 22, buf);
  sprintf(buf, "R->R:  %.1f ms", riseToRiseCH1 / 1000.0);   u8g2.drawStr(30, 34, buf);
  if (glitchCountCH1 == 0) strcpy(buf, "Glitch: none");
  else sprintf(buf, "Glitch: %u short", glitchCountCH1);
  u8g2.drawStr(30, 46, buf);

  // -----------------------------
  // 8. CH2 – display values
  // -----------------------------
  u8g2.drawStr(135, 10, "CH2");
  sprintf(buf, "Pulse:  %.1f %%", dutyPct2);                u8g2.drawStr(160, 10, buf);
  sprintf(buf, "Period: %lu \xB5s", savedPwmPeriod2);       u8g2.drawStr(160, 22, buf);
  sprintf(buf, "R->R:  %.1f ms", riseToRiseCH2 / 1000.0);   u8g2.drawStr(160, 34, buf);
  if (glitchCountCH2 == 0) strcpy(buf, "Glitch: none");
  else sprintf(buf, "Glitch: %u short", glitchCountCH2);
  u8g2.drawStr(160, 46, buf);

  // -----------------------------
  // 9. Bottom menu rendering
  // -----------------------------
  drawBottomMenu();
}


void drawGraphScreen() {
  // -----------------------------
  // 1. Static variables to retain values between iterations
  // -----------------------------
  static unsigned long lastUpdate = 0;
  static float avgDutyCycle1 = 0;
  static float avgDutyCycle2 = 0;

  // -----------------------------
  // 2. Handle encoder (for adjusting update interval)
  // -----------------------------
  handleEncoderUpdateInterval();

  // -----------------------------
  // 3. Timed update (only if not frozen)
  // -----------------------------
  unsigned long currentMillis = millis();
  if (!freezeMode && currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis;

    // -----------------------------
    // 4. Check PWM validity on both channels
    // (If no data, set estimated value)
    // -----------------------------
    bool pwm1_active = (micros() - lastISRTime1) < 100000;
    bool valid_pwm1 = pwm1_active && avgDutyCycle1 > 0.0 && avgDutyCycle1 < 100.0;
    if (!valid_pwm1 && digitalRead(PWM1_PIN) == HIGH) avgDutyCycle1 = 100.0;
    else if (!valid_pwm1) avgDutyCycle1 = 0.0;

    bool pwm2_active = (micros() - lastISRTime2) < 100000;
    bool valid_pwm2 = pwm2_active && avgDutyCycle2 > 0.0 && avgDutyCycle2 < 100.0;
    if (!valid_pwm2 && digitalRead(PWM2_PIN) == HIGH) avgDutyCycle2 = 100.0;
    else if (!valid_pwm2) avgDutyCycle2 = 0.0;

    // -----------------------------
    // 5. Get averaged values (from external functions)
    // -----------------------------
    avgDutyCycle1 = getAverageDuty1(); 
    avgDutyCycle2 = getAverageDuty2(); 
    float freq1 = getAverageFreq1();  
    float freq2 = getAverageFreq2();  

    // -----------------------------
    // 6. Calculate waveform height based on frequency
    // -----------------------------
    int height1 = mapFrequencyToHeight(freq1);
    int height2 = mapFrequencyToHeight(freq2);

    yTop1 = yBottom1 - height1;
    yTop2 = yBottom2 - height2;

    // -----------------------------
    // 7. Calculate waveform segments based on duty cycle
    // -----------------------------
    int fullCycleLen = 20;
    highLen1 = max(1, (int)(fullCycleLen * avgDutyCycle1 / 100.0));
    lowLen1  = fullCycleLen - highLen1;
    highLen2 = max(1, (int)(fullCycleLen * avgDutyCycle2 / 100.0));
    lowLen2  = fullCycleLen - highLen2;

    // -----------------------------
    // 8. Scroll waveform left by 1 pixel
    // -----------------------------
    memmove(&waveform1[0], &waveform1[1], waveformWidth - 1);
    memmove(&waveform2[0], &waveform2[1], waveformWidth - 1);

    // -----------------------------
    // 9. CH1 – state machine adds new pixel
    // -----------------------------
    if ((int)avgDutyCycle1 == 100) {
      waveform1[waveformWidth - 1] = 1;
    } else if ((int)avgDutyCycle1 == 0) {
      waveform1[waveformWidth - 1] = 0;
    } else {
      switch (state1) {
        case 0: waveform1[waveformWidth - 1] = 0; if (++counter1 >= lowLen1) { counter1 = 0; state1 = 1; } break;
        case 1: waveform1[waveformWidth - 1] = 2; state1 = 2; break;
        case 2: waveform1[waveformWidth - 1] = 1; if (++counter1 >= highLen1 - 1) { counter1 = 0; state1 = 3; } break;
        case 3: waveform1[waveformWidth - 1] = 3; state1 = 0; break;
      }
    }

    // -----------------------------
    // 10. CH2 – state machine adds new pixel
    // -----------------------------
    if ((int)avgDutyCycle2 == 100) {
      waveform2[waveformWidth - 1] = 1;
    } else if ((int)avgDutyCycle2 == 0) {
      waveform2[waveformWidth - 1] = 0;
    } else {
      switch (state2) {
        case 0: waveform2[waveformWidth - 1] = 0; if (++counter2 >= lowLen2) { counter2 = 0; state2 = 1; } break;
        case 1: waveform2[waveformWidth - 1] = 2; state2 = 2; break;
        case 2: waveform2[waveformWidth - 1] = 1; if (++counter2 >= highLen2 - 1) { counter2 = 0; state2 = 3; } break;
        case 3: waveform2[waveformWidth - 1] = 3; state2 = 0; break;
      }
    }
  }

  // -----------------------------
  // 11. UI elements on the display
  // -----------------------------
  char buf[10];
  if (freezeMode) {
    sprintf(buf, "Freeze");
  } else {
    sprintf(buf, "%u ms", updateInterval);
  }
  u8g2.drawStr(213, 60, buf);   

  u8g2.drawStr(5, 12, "CH1 ");
  u8g2.drawStr(5, 34, "CH2 ");

  // -----------------------------
  // 12. Draw CH1 duty cycle bar
  // -----------------------------
  int dutyHeight = (int)(maxHeight * (avgDutyCycle1 / 100.0));
  u8g2.drawFrame(dutyX - 2, dutyY - 2, barWidth + 4, maxHeight + 4);
  u8g2.drawBox(dutyX, dutyY + (maxHeight - dutyHeight), barWidth, dutyHeight);
  char dutyText[10];
  snprintf(dutyText, sizeof(dutyText), "%d%%", (int)avgDutyCycle1);
  u8g2.drawStr(dutyX + barWidth + 4, dutyY + maxHeight / 2, dutyText);

  // -----------------------------
  // 13. Draw CH2 duty cycle bar
  // -----------------------------
  int dutyX2 = 220;
  int dutyY2 = dutyY + maxHeight + 6;
  int dutyHeight2 = (int)(maxHeight * (avgDutyCycle2 / 100.0));
  u8g2.drawFrame(dutyX2 - 2, dutyY2 - 2, barWidth + 4, maxHeight + 4);
  u8g2.drawBox(dutyX2, dutyY2 + (maxHeight - dutyHeight2), barWidth, dutyHeight2);
  char dutyText2[10];
  snprintf(dutyText2, sizeof(dutyText2), "%d%%", (int)avgDutyCycle2);
  u8g2.drawStr(dutyX2 + barWidth + 4, dutyY2 + maxHeight / 2, dutyText2);

  // -----------------------------
  // 14. Draw waveform for CH1
  // -----------------------------
  for (int x = 0; x < waveformWidth; x++) {
    int screenX = x + waveformStartX;
    switch (waveform1[x]) {
      case 0: u8g2.drawPixel(screenX, yBottom1); break;
      case 1: u8g2.drawPixel(screenX, yTop1); break;
      case 2: u8g2.drawLine(screenX, yBottom1, screenX, yTop1); break;
      case 3: u8g2.drawLine(screenX, yTop1, screenX, yBottom1); break;
      default: break;
    }
  }

  // -----------------------------
  // 15. Draw waveform for CH2
  // -----------------------------
  for (int x = 0; x < waveformWidth; x++) {
    int screenX = x + waveformStartX;
    switch (waveform2[x]) {
      case 0: u8g2.drawPixel(screenX, yBottom2); break;
      case 1: u8g2.drawPixel(screenX, yTop2); break;
      case 2: u8g2.drawLine(screenX, yBottom2, screenX, yTop2); break;
      case 3: u8g2.drawLine(screenX, yTop2, screenX, yBottom2); break;
      case WAVEFORM_EMPTY: default: break;
    }
  }

  // -----------------------------
  // 16. Draw bottom menu
  // -----------------------------
  drawBottomMenu();
}

void pushWaveform(uint8_t* waveform, float duty, int width) {
  int pulseWidth = (int)(width * (duty / 100.0));
  pulseWidth = constrain(pulseWidth, 1, width - 2);  // Do not allow 0 or too large
  int lowWidth = width - pulseWidth;

  // Shift left
  for (int i = 0; i < waveformWidth - width; i++) {
    waveform[i] = waveform[i + width];
  }

  // Write new waveform
  int idx = waveformWidth - width;
  waveform[idx++] = 2; // Rising edge
  for (int i = 1; i < pulseWidth - 1; i++) waveform[idx++] = 1; // HIGH
  waveform[idx++] = 3; // Falling edge
  for (int i = 0; i < lowWidth - 1; i++) waveform[idx++] = 0; // LOW
}

void drawWaveform(uint8_t* waveform, int yTop, int yBottom) {
  for (int x = 0; x < waveformWidth; x++) {
    switch (waveform[x]) {
      case 0: u8g2.drawPixel(x, yBottom); break;        // LOW
      case 1: u8g2.drawPixel(x, yTop); break;           // HIGH
      case 2: u8g2.drawLine(x, yBottom, x, yTop); break; // Rising edge
      case 3: u8g2.drawLine(x, yTop, yBottom, x); break; // Falling edge
    }
  }
}

void MainDisplay() {
  static unsigned long lastUpdate = 0;

  // Adjust updateInterval value based on encoder (rotation speeds up/slows down refresh rate)
  handleEncoderUpdateInterval();

  // Static variables so they only update when the interval has passed
  static float avgDutyCycle1 = 0;
  static int highT1 = 0, lowT1 = 0;
  static float avgDutyCycle2 = 0;
  static int highT2 = 0, lowT2 = 0;

  unsigned long currentMillis = millis();

  // Update data only if not in "Freeze" mode and updateInterval has elapsed
  if (!freezeMode && millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    // Refresh average frequencies
    avgFreq1 = getAverageFreq1();
    avgFreq2 = getAverageFreq2();

    // If no interrupt for a long time, reset values
    if ((micros() - lastISRTime1) > 100000) {
      highTime1 = 0;
      pwmPeriod1 = 0;
    }

    if ((micros() - lastISRTime2) > 100000) {
      highTime2 = 0;
      pwmPeriod2 = 0;
    }

    // Update display strings (e.g., frequency text)
    updateFrequencyDisplayStrings();

    // Update duty and timing values
    avgDutyCycle1 = getAverageDuty1();
    highT1 = highTime1;
    lowT1 = pwmPeriod1 > highTime1 ? pwmPeriod1 - highTime1 : 0;

    avgDutyCycle2 = getAverageDuty2();
    highT2 = highTime2;
    lowT2 = pwmPeriod2 > highTime2 ? pwmPeriod2 - highTime2 : 0;
  }

  // ----------------- DISPLAY -----------------

  u8g2.setFont(u8g2_font_6x12_t_symbols);  // For Unicode characters

  // Display refresh state (Freeze or update interval in ms)
  char buf[10];
  if (freezeMode) {
    sprintf(buf, "Freeze");
  } else {
    sprintf(buf, "%u ms", updateInterval);
  }
  u8g2.drawStr(213, 60, buf);

  // --- CH1 ---
  u8g2.drawStr(5, 10, "CH1 Freq:");

  if (avgFreq1 > 9999) {
    // Over the limit, show warning
    u8g2.drawStr(70, 10, "Limit!");
  } else {
    u8g2.drawStr(70, 10, (freq1Str + " " + unit1).c_str());
  }

  u8g2.drawStr(5, 22, "    Duty: ");

  // Check PWM activity
  bool pwm1_active = (micros() - lastISRTime1) < 100000;
  bool valid_pwm = pwm1_active && avgDutyCycle1 > 0.0 && avgDutyCycle1 < 100.0;

  if (!valid_pwm) {
    u8g2.drawStr(70, 22, "No PWM");
  } else {
    u8g2.drawStr(70, 22, (String(avgDutyCycle1, 1) + " %").c_str());
  }

  u8g2.drawStr(5, 34, "    HIGH: ");
  u8g2.drawStr(70, 34, (String(highT1) + " \xB5s").c_str());

  u8g2.drawStr(5, 46, "    LOW: ");
  u8g2.drawStr(70, 46, (String(lowT1) + " \xB5s").c_str());

  // --- CH2 ---
  u8g2.drawStr(135, 10, "CH2 Freq:");

  if (avgFreq2 > 9999) {
    u8g2.drawStr(200, 10, "Limit!");
  } else {
    u8g2.drawStr(200, 10, (freq2Str + " " + unit2).c_str());
  }

  u8g2.drawStr(135, 22, "    Duty: ");

  bool pwm2_active = (micros() - lastISRTime2) < 100000;
  bool valid_pwm2 = pwm2_active && avgDutyCycle2 > 0.0 && avgDutyCycle2 < 100.0;

  if (!valid_pwm2) {
    u8g2.drawStr(200, 22, "No PWM");
  } else {
    u8g2.drawStr(200, 22, (String(avgDutyCycle2, 1) + " %").c_str());
  }

  u8g2.drawStr(135, 34, "    HIGH: ");
  u8g2.drawStr(200, 34, (String(highT2) + " \xB5s").c_str());

  u8g2.drawStr(135, 46, "    LOW: ");
  u8g2.drawStr(200, 46, (String(lowT2) + " \xB5s").c_str());

  // --- Call bottom menu ---
  drawBottomMenu();
}

void updateFrequencyDisplayStrings() {
  float displayFreq1 = avgFreq1;
  unit1 = "Hz";
  if (frequDisplayUnit == 1) {
    displayFreq1 /= 1000;
    unit1 = "kHz";
  } else if (frequDisplayUnit == 2) {
    displayFreq1 /= 1000000;
    unit1 = "MHz";
  } else {
    displayFreq1 = round(displayFreq1);
  }
  freq1Str = (frequDisplayUnit == 0) ? String((int)displayFreq1)
                                     : String(round(displayFreq1 * 100.0) / 100.0);

  float displayFreq2 = avgFreq2;
  unit2 = "Hz";
  if (frequDisplayUnit == 1) {
    displayFreq2 /= 1000;
    unit2 = "kHz";
  } else if (frequDisplayUnit == 2) {
    displayFreq2 /= 1000000;
    unit2 = "MHz";
  } else {
    displayFreq2 = round(displayFreq2);
  }
  freq2Str = (frequDisplayUnit == 0) ? String((int)displayFreq2)
                                     : String(round(displayFreq2 * 100.0) / 100.0);
}

float getAverageFreq1() {
  if ((micros() - lastISRTime1) > 100000) {  // No activity > 100 ms
    return 0.0;
  }
  unsigned long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += freqPeriods1[i];
  if (sum == 0) return 0;
  return 1000000.0 / (sum / (float)NUM_SAMPLES);
}

float getAverageDuty1() {
  if ((micros() - lastISRTime1) > 100000) {
    // Signal disappeared – check stuck state
    if (digitalRead(PWM1_PIN) == HIGH) {
      return 100.0;  // fixed HIGH
    } else {
      return 0.0;    // fixed LOW
    }
  }

  unsigned long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += dutyCycles1[i];
  return sum / (float)NUM_SAMPLES;
}

float getAverageFreq2() {
  if ((micros() - lastISRTime2) > 100000) {  // No activity > 100 ms
    return 0.0;
  }
  unsigned long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += freqPeriods2[i];
  if (sum == 0) return 0;
  return 1000000.0 / (sum / (float)NUM_SAMPLES);
}

float getAverageDuty2() {
  if ((micros() - lastISRTime2) > 100000) {
    if (digitalRead(PWM2_PIN) == HIGH) {
      return 100.0;
    } else {
      return 0.0;
    }
  }

  unsigned long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += dutyCycles2[i];
  return sum / (float)NUM_SAMPLES;
}

void pwmISR_CH1() {
  unsigned long now = micros();
  bool pinState = digitalRead(PWM1_PIN);

  if (pinState == HIGH) {
    if (riseTime1 > 0) pwmPeriod1 = now - riseTime1;
    riseTime1 = now;
  } else {
    highTime1 = now - riseTime1;
    if (pwmPeriod1 > 0) {
      unsigned long duty = (highTime1 * 100) / pwmPeriod1;
      freqPeriods1[sampleIndex1] = pwmPeriod1;
      dutyCycles1[sampleIndex1] = duty;
      sampleIndex1 = (sampleIndex1 + 1) % NUM_SAMPLES;
      lastISRTime1 = now;
    }
  }
}

void pwmISR_CH2() {
  unsigned long now = micros();
  bool pinState = digitalRead(PWM2_PIN);

  if (pinState == HIGH) {
    if (riseTime2 > 0) pwmPeriod2 = now - riseTime2;
    riseTime2 = now;
  } else {
    highTime2 = now - riseTime2;
    if (pwmPeriod2 > 0) {
      unsigned long duty = (highTime2 * 100) / pwmPeriod2;
      freqPeriods2[sampleIndex2] = pwmPeriod2;
      dutyCycles2[sampleIndex2] = duty;
      sampleIndex2 = (sampleIndex2 + 1) % NUM_SAMPLES;
      lastISRTime2 = now;
    }
  }
}

void ReadEncoderButton() {
  int currentButtonState = digitalRead(ENC_SW);

  if (currentButtonState == LOW && lastButtonState == HIGH) {
    if (selectedMenu == 2) {
      // If the graph menu is active, switch zoom level
      freqZoomLevel = (freqZoomLevel + 1) % (maxZoomLevel + 1);
    } else if (selectedMenu == 0 || selectedMenu == 1) {
      // If main screen or detail screen is active, switch frequency unit
      frequDisplayUnit = (frequDisplayUnit + 1) % 3;  // Cycle: Hz -> kHz -> MHz -> Hz ...
    }
    updateFrequencyDisplayStrings();  // Refresh displayed text
  }

  lastButtonState = currentButtonState;
}

void handleEncoderUpdateInterval() {
  static int lastEncoderValue = 0;
  int delta = encoderValue - lastEncoderValue;

  if (delta != 0) {
    lastEncoderValue = encoderValue;

    if (!freezeMode) {
      int newInterval = updateInterval + delta * 50;

      if (newInterval < 50 && delta < 0) {
        freezeMode = true;
        updateInterval = 0;
      } else if (newInterval > 2000 && delta > 0) {
        freezeMode = true;
        updateInterval = 2000;
      } else {
        updateInterval = constrain(newInterval, 50, 2000);
      }

    } else {
      // If freeze is active and you scroll up from bottom → exit freeze, start from 50 ms
      if (updateInterval == 0 && delta > 0) {
        freezeMode = false;
        updateInterval = 50;
      }
      // If freeze is active and you scroll down from top → exit freeze, start from 2000 ms
      else if (updateInterval == 2000 && delta < 0) {
        freezeMode = false;
        updateInterval = 2000;
      }
    }
  }
}

void encoderISR() {
  int currentA = digitalRead(ENC_A);
  int currentB = digitalRead(ENC_B);

  if (currentA != lastA) {
    if (currentB != currentA) {
      encoderValue++;
    } else {
      encoderValue--;
    }
  }

  lastA = currentA;
  lastB = currentB;
}

void drawBottomMenu() {
  u8g2.drawLine(0, 49, 256, 49); // horizontal line

  if (selectedMenu != 2) {
    u8g2.drawLine(128, 0, 128, 48); // only if not graph screen
  }

  u8g2.drawStr(5, 60, "Main");    // Main Display
  u8g2.drawStr(65, 60, "Data");   // Detailed Data
  u8g2.drawStr(115, 60, "Graph"); // Graph View
  u8g2.drawStr(168, 60, "Unit/SR");
}

int mapFrequencyToHeight(float freqHz) {
  const int maxHeight = 15;
  const int minHeight = 2;
  
  // Dynamically set min/max frequency based on zoom level
  float minFreq, maxFreq;

  switch (freqZoomLevel) {
    case 0: // 300Hz - 999Hz
      minFreq = 300.0;
      maxFreq = 999.0;
      break;
    case 1: // 1kHz - 499kHz
      minFreq = 1000.0;
      maxFreq = 499000.0;
      break;
    case 2: // 500kHz - 999kHz
      minFreq = 500000.0;
      maxFreq = 999000.0;
      break;
    case 3: // 1MHz - 2MHz
      minFreq = 1000000.0;
      maxFreq = 2000000.0;
      break;
    default:
      minFreq = 300.0;
      maxFreq = 999.0;
      break;
  }

  // Constrain frequency to the selected range
  freqHz = constrain(freqHz, minFreq, maxFreq);

  // Calculate ratio between min and max frequency
  float ratio = (freqHz - minFreq) / (maxFreq - minFreq);
  
  // Convert ratio to height
  return maxHeight - (int)(ratio * (maxHeight - minHeight));
}

void handleStatusLED(bool signal1, bool signal2) {
  static unsigned long lastMillis = 0;
  static int ledPhase = 0;
  unsigned long currentMillis = millis();

  // Get duty cycle values
  float duty1 = getAverageDuty1();
  float duty2 = getAverageDuty2();

  // If both channels are 100%, LED stays blue
  if (duty1 == 100.0 && duty2 == 100.0) {
    ledcWrite(RED_PIN, 0);
    ledcWrite(GREEN_PIN, 0);
    ledcWrite(BLUE_PIN, 255);
    return;
  }

  // If one channel is 100% and the other 0%, LED blinks blue
  if ((duty1 == 100.0 && duty2 == 0.0) || (duty1 == 0.0 && duty2 == 100.0)) {
    unsigned long phaseDurations[] = {50, 50, 50, 50, 50, 50, 50, 1000};  // Blink pattern
    if (currentMillis - lastMillis >= phaseDurations[ledPhase]) {
      lastMillis = currentMillis;
      ledPhase = (ledPhase + 1) % 8;  // 4 flashes, then pause

      if (ledPhase == 0 || ledPhase == 2 || ledPhase == 4 || ledPhase == 6) {
        ledcWrite(RED_PIN, 0);
        ledcWrite(GREEN_PIN, 0);
        ledcWrite(BLUE_PIN, 255);  // Blue flash
      } else {
        ledcWrite(RED_PIN, 0);
        ledcWrite(GREEN_PIN, 0);
        ledcWrite(BLUE_PIN, 0);    // Off
      }
    }
    return;
  }

  // If no signal on both channels, blink red
  if (!signal1 && !signal2) {
    unsigned long phaseDurations[] = {50, 50, 50, 1000}; // Blink pattern

    if (currentMillis - lastMillis >= phaseDurations[ledPhase]) {
      lastMillis = currentMillis;
      ledPhase = (ledPhase + 1) % 4;

      if (ledPhase == 0 || ledPhase == 2) {
        ledcWrite(RED_PIN, 255);
        ledcWrite(GREEN_PIN, 0);
        ledcWrite(BLUE_PIN, 0);
      } else {
        ledcWrite(RED_PIN, 0);
        ledcWrite(GREEN_PIN, 0);
        ledcWrite(BLUE_PIN, 0);
      }
    }
  } else {
    // Signal present – green pulsing
    int brightness = (sin(millis() / 2500.0 * 2 * PI) + 1.0) * 30;
    ledcWrite(RED_PIN, 0);
    ledcWrite(GREEN_PIN, brightness);
    ledcWrite(BLUE_PIN, 0);
  }
}

