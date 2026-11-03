
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <arduinoFFT.h>
#include <math.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// ---------- Wi-Fi / ThingSpeak ----------
// Enter your NEW credentials locally.
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_NEW_WIFI_PASSWORD";

unsigned long CHANNEL_ID = 3531547;
const char* WRITE_API_KEY = "YOUR_NEW_THINGSPEAK_WRITE_KEY";

WiFiClient tsClient;
unsigned long lastUpload = 0;
unsigned long lastWiFiAttempt = 0;

const unsigned long UPLOAD_INTERVAL_MS = 20000;
const unsigned long WIFI_RETRY_MS = 15000;

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET
);

// ---------- MPU6050 ----------
Adafruit_MPU6050 mpu;

// Separate I2C bus for MPU6050
TwoWire I2C_MPU = TwoWire(1);

const int MPU_SDA = 19;
const int MPU_SCL = 18;
const uint8_t MPU_ADDRESS = 0x68;

const uint16_t MPU_SAMPLES = 128;
const unsigned long MPU_SAMPLE_PERIOD_US = 10000;

double mpuX[MPU_SAMPLES];
double mpuY[MPU_SAMPLES];
double mpuZ[MPU_SAMPLES];

double mpuRms = 0;

// ---------- Pins ----------
const int SENSOR_PIN = 34;
const int GREEN_LED  = 27;
const int RED_LED    = 14;
const int BUZZER_PIN = 26;
const int BUTTON_PIN = 13;

// ---------- FFT ----------
const uint16_t SAMPLES = 128;
const double SAMPLING_FREQUENCY = 1000.0;
const unsigned long SAMPLE_PERIOD_US = 1000;

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT(
  vReal, vImag, SAMPLES, SAMPLING_FREQUENCY
);

// ---------- Provisional raw ADC RMS thresholds ----------
const double WARNING_RMS = 6.0;
const double ALERT_RMS = 10.0;

// Hysteresis: exit thresholds are lower than entry thresholds.
const double WARNING_ENTER = 6.0;
const double WARNING_EXIT_RMS = 5.0;
const double ALERT_ENTER = 10.0;
const double ALERT_EXIT_RMS = 8.5;

// Average the last five RMS measurements.
const int RMS_HISTORY_SIZE = 5;
double rmsHistory[RMS_HISTORY_SIZE] = {0};

int rmsHistoryIndex = 0;
int rmsHistoryCount = 0;
double filteredRms = 0;

// ---------- Status ----------
enum MachineStatus {
  LOW_SIGNAL = 0,
  WARNING = 1,
  ALERT = 2
};

MachineStatus machineStatus = LOW_SIGNAL;

// ---------- Button ----------
bool buzzerMuted = false;
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 40;

void checkButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if (millis() - lastDebounceTime > DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        buzzerMuted = !buzzerMuted;

        Serial.print("Buzzer mute: ");
        Serial.println(buzzerMuted ? "ON" : "OFF");
      }
    }
  }

  lastButtonReading = reading;
}

double smoothRms(double newRms) {
  rmsHistory[rmsHistoryIndex] = newRms;

  rmsHistoryIndex =
      (rmsHistoryIndex + 1) % RMS_HISTORY_SIZE;

  if (rmsHistoryCount < RMS_HISTORY_SIZE) {
    rmsHistoryCount++;
  }

  double total = 0;

  for (int i = 0; i < rmsHistoryCount; i++) {
    total += rmsHistory[i];
  }

  return total / rmsHistoryCount;
}

void updateStatus(double rms) {
  switch (machineStatus) {
    case LOW_SIGNAL:
      if (rms >= ALERT_RMS) {
        machineStatus = ALERT;
      } else if (rms >= WARNING_RMS) {
        machineStatus = WARNING;
      }
      break;

    case WARNING:
      if (rms >= ALERT_RMS) {
        machineStatus = ALERT;
      } else if (rms < WARNING_EXIT_RMS) {
        machineStatus = LOW_SIGNAL;
      }
      break;

    case ALERT:
      if (rms < ALERT_EXIT_RMS) {
        machineStatus =
            (rms >= WARNING_EXIT_RMS)
            ? WARNING : LOW_SIGNAL;
      }
      break;
  }
}

const char* statusText() {
  switch (machineStatus) {
    case ALERT: return "ALERT";
    case WARNING: return "WARNING";
    default: return "LOW SIGNAL";
  }
}

void updateOutputs() {
  digitalWrite(
    GREEN_LED,
    machineStatus == LOW_SIGNAL ? HIGH : LOW
  );

  digitalWrite(
    RED_LED,
    machineStatus == LOW_SIGNAL ? LOW : HIGH
  );

  digitalWrite(
    BUZZER_PIN,
    (machineStatus == ALERT && !buzzerMuted)
        ? HIGH : LOW
  );
}

void connectWiFiIfNeeded() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  if (millis() - lastWiFiAttempt >= WIFI_RETRY_MS) {
    lastWiFiAttempt = millis();

    Serial.println("Attempting Wi-Fi connection...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// ---------- ThingSpeak ----------
void uploadToThingSpeak(
    double rms,
    double peakMagnitude,
    double dominantFrequency,
    double accelerometerRms) {

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (ThingSpeak.setField(1, (float)rms) == false) return;
  if (ThingSpeak.setField(2, (float)peakMagnitude) == false) return;
  if (ThingSpeak.setField(3, (int)machineStatus) == false) return;
  if (ThingSpeak.setField(4, (float)dominantFrequency) == false) return;
  if (ThingSpeak.setField(5, (float)accelerometerRms) == false) return;

  int response = ThingSpeak.writeFields(
      CHANNEL_ID, WRITE_API_KEY
  );

  Serial.print("ThingSpeak HTTP response: ");
  Serial.println(response);
}

// ---------- MPU6050 sampling ----------
void readMPU6050() {

  sensors_event_t a, g, temp;

  unsigned long nextSample = micros();

  double sumX = 0;
  double sumY = 0;
  double sumZ = 0;

  for (uint16_t i = 0; i < MPU_SAMPLES; i++) {

    while ((int32_t)(micros() - nextSample) < 0) {
      // Wait for sampling interval.
    }

    mpu.getEvent(&a, &g, &temp);

    mpuX[i] = a.acceleration.x;
    mpuY[i] = a.acceleration.y;
    mpuZ[i] = a.acceleration.z;

    sumX += mpuX[i];
    sumY += mpuY[i];
    sumZ += mpuZ[i];

    nextSample += MPU_SAMPLE_PERIOD_US;
  }

  // Remove DC offset and gravity component.
  double meanX = sumX / MPU_SAMPLES;
  double meanY = sumY / MPU_SAMPLES;
  double meanZ = sumZ / MPU_SAMPLES;

  double sumSquares = 0;

  for (uint16_t i = 0; i < MPU_SAMPLES; i++) {

    double x = mpuX[i] - meanX;
    double y = mpuY[i] - meanY;
    double z = mpuZ[i] - meanZ;

    sumSquares += x * x + y * y + z * z;
  }

  // Combined three-axis acceleration RMS.
  mpuRms = sqrt(sumSquares / MPU_SAMPLES);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  updateOutputs();

  // OLED uses the default I2C bus.
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED initialization failed!");
    while (true) delay(100);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Vibration Monitor");
  display.println("Starting...");
  display.display();

  // Initialize MPU6050 on separate I2C bus.
  I2C_MPU.begin(MPU_SDA, MPU_SCL, 100000);

  if (!mpu.begin(MPU_ADDRESS, &I2C_MPU)) {

    Serial.println("MPU6050 initialization failed!");

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MPU6050 ERROR");
    display.println("Check connections");
    display.display();

    while (true) delay(100);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  Serial.println("MPU6050 initialized successfully!");

  WiFi.mode(WIFI_STA);
  ThingSpeak.begin(tsClient);

  lastWiFiAttempt = millis();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Smoothed RMS + FFT + MPU6050 monitor ready");
}

// ---------- Main loop ----------
void loop() {
  checkButton();
  connectWiFiIfNeeded();

  // ---------- Original piezo sampling ----------
  unsigned long nextSample = micros();
  double sum = 0;

  for (uint16_t i = 0; i < SAMPLES; i++) {

    while ((long)(micros() - nextSample) < 0) {
      // Wait for sample timing.
    }

    vReal[i] = analogRead(SENSOR_PIN);
    vImag[i] = 0;
    sum += vReal[i];

    nextSample += SAMPLE_PERIOD_US;
  }

  // ---------- Original piezo RMS ----------
  double mean = sum / SAMPLES;
  double sumSquares = 0;

  for (uint16_t i = 0; i < SAMPLES; i++) {
    double centered = vReal[i] - mean;

    sumSquares += centered * centered;
    vReal[i] = centered;
  }

  double rawRms = sqrt(sumSquares / SAMPLES);
  filteredRms = smoothRms(rawRms);

  // ---------- Original piezo FFT ----------
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  uint16_t peakBin = 1;
  double peakMagnitude = vReal[1];

  for (uint16_t i = 2; i < (SAMPLES / 2) - 1; i++) {
    if (vReal[i] > peakMagnitude) {
      peakMagnitude = vReal[i];
      peakBin = i;
    }
  }

  double dominantFrequency =
      peakBin * SAMPLING_FREQUENCY / SAMPLES;

  // ---------- Additional MPU6050 measurement ----------
  readMPU6050();

  // ---------- Original status logic ----------
  updateStatus(filteredRms);
  updateOutputs();

  // ---------- OLED ----------
  display.clearDisplay();
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("Vibration Monitor");

  display.setCursor(0, 10);
  display.print("Piezo RMS: ");
  display.println(filteredRms, 2);

  display.setCursor(0, 20);
  display.print("MPU RMS: ");
  display.print(mpuRms, 3);
  display.println(" m/s2");

  display.setCursor(0, 30);
  display.print("Freq: ");
  display.print(dominantFrequency, 1);
  display.println(" Hz");

  display.setCursor(0, 40);
  display.print("Magnitude: ");
  display.println(peakMagnitude, 1);

  display.setCursor(0, 48);
  display.print("Status: ");
  display.println(statusText());

  display.setCursor(0, 56);
  display.print("Mute:");
  display.print(buzzerMuted ? "ON" : "OFF");
  display.print(" WiFi:");
  display.print(WiFi.status() == WL_CONNECTED ? "OK" : "--");

  display.display();

  // ---------- Serial ----------
  Serial.print("Raw RMS: ");
  Serial.print(rawRms, 2);

  Serial.print(" | Filtered RMS: ");
  Serial.print(filteredRms, 2);

  Serial.print(" | Dominant frequency: ");
  Serial.print(dominantFrequency, 1);

  Serial.print(" Hz | Peak magnitude: ");
  Serial.print(peakMagnitude, 1);

  Serial.print(" | Status: ");
  Serial.print(statusText());

  Serial.print(" | MPU6050 RMS: ");
  Serial.print(mpuRms, 4);
  Serial.println(" m/s2");

  // ---------- ThingSpeak upload ----------
  if (millis() - lastUpload >= UPLOAD_INTERVAL_MS) {
    lastUpload = millis();

    uploadToThingSpeak(
      filteredRms,
      peakMagnitude,
      dominantFrequency,
      mpuRms
    );
  }
}
