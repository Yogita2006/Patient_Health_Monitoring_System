/* ESP32 multi-sensor sketch with Blynk + I2C LCD
   Sensors:
   - ECG (analog)        -> ECG_PIN (good for plotting on Blynk)
   - ADXL335 (X/Y/Z)     -> ADXL_X_PIN, ADXL_Y_PIN, ADXL_Z_PIN
   - LM35 temp (analog)  -> LM35_PIN
   - DHT11 (digital)     -> DHT_PIN
   - Pulse sensor        -> PULSE_PIN (simple peak-detect)
   - I2C LCD (0x27)      -> SDA 21, SCL 22 (default)
   - Blynk virtual pins used in comments

   Replace BLYNK_AUTH, WIFI_SSID, WIFI_PASS with your values.
*/

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ---------- USER CONFIG ----------
char auth[] = "83LcWS-qBgymmkCUpZhdWOIoadOwh5kZ";
char ssid[] = "Airtel_Khan's family";
char pass[] = "Jishan#786kh";


// I2C LCD address (common 0x27 or 0x3F)
#define LCD_I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
// ---------- PINS ----------
const int ECG_PIN   = 34; // analog input (ECG)
const int ADXL_X_PIN= 35; // analog input
const int ADXL_Y_PIN= 32;
const int ADXL_Z_PIN= 33;
const int LM35_PIN  = 36; // analog input
const int DHT_PIN   = 15; // digital
const int DHT_TYPE  = DHT11;
const int PULSE_PIN = 39; // analog input for pulse sensor

// ---------- BLYNK VIRTUAL PINS ----------
const int VECG = V1;     // ECG stream
const int VACCX = V2;    // ADXL X
const int VACCY = V3;    // ADXL Y
const int VACCC = V4;    // ADXL Z
const int VLM35 = V5;    // LM35 temp
const int VDHTT = V6;    // DHT temp
const int VDHTh = V7;    // DHT humidity
const int VPULSE = V8;   // BPM value

// ---------- OBJECTS ----------
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
DHT dht(DHT_PIN, DHT_TYPE);

// ---------- TIMING ----------
#include <BlynkSimpleEsp32.h>
BlynkTimer timer;
unsigned long lastPulseSampleMillis = 0;
const unsigned long PULSE_SAMPLE_INTERVAL = 50; // ms
unsigned long lastBeatMillis = 0;
bool pulseDetected = false;
int BPM = 0;

// Pulse detection parameters (tweak threshold per your sensor)
int pulseThreshold = 550; // ADC value threshold (ESP32 ADC range ~0-4095)
int lastSignal = 0;
int signalPeak = 0;
int signalTrough = 4095;

// ECG streaming downsample control
unsigned long lastECGMillis = 0;
const unsigned long ECG_SEND_INTERVAL = 40; // ms -> ~25 samples/sec

// Utility: read analog with 12-bit ADC range on ESP32 (0-4095)
int readAnalog(int pin) {
  // ADC attenuation and calibration could be set in setup if desired
  return analogRead(pin);
}

// ---------- SENSOR READ/REPORT FUNCTIONS ----------
void sendECG() {
  int ecgRaw = readAnalog(ECG_PIN);
  // Send ECG raw value to Blynk (for graph) and Serial
  Blynk.virtualWrite(VECG, ecgRaw);
  Serial.print("ECG: "); Serial.println(ecgRaw);
}

void sendADXL() {
  int ax = readAnalog(ADXL_X_PIN);
  int ay = readAnalog(ADXL_Y_PIN);
  int az = readAnalog(ADXL_Z_PIN);
  Blynk.virtualWrite(VACCX, ax);
  Blynk.virtualWrite(VACCY, ay);
  Blynk.virtualWrite(VACCC, az);
  Serial.print("ADXL X: "); Serial.print(ax);
  Serial.print(" Y: "); Serial.print(ay);
  Serial.print(" Z: "); Serial.println(az);
}

void sendLM35() {
  int raw = readAnalog(LM35_PIN); // 0..4095
  // Convert ADC reading to voltage: V = raw/4095 * Vref (approx 3.3V)
  float voltage = (raw / 4095.0) * 3.3;
  // LM35 gives 10 mV per °C => tempC = voltage (V) * 100
  float tempC = voltage * 100.0;
  Blynk.virtualWrite(VLM35, tempC);
  Serial.print("LM35 Raw: "); Serial.print(raw);
  Serial.print(" TempC: "); Serial.println(tempC);
}

void sendDHT() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    Serial.println("DHT read failed");
    return;
  }
  Blynk.virtualWrite(VDHTT, t);
  Blynk.virtualWrite(VDHTh, h);
  Serial.print("DHT Temp: "); Serial.print(t);
  Serial.print(" Hum: "); Serial.println(h);
}

void updateLCD(float lm35T, float dhtT, float dhtH, int bpm) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("L:");
  lcd.print(lm35T,1);
  lcd.print((char)223); lcd.print("C"); // degree symbol
  lcd.print(" D:");
  lcd.print(dhtT,1);
  lcd.setCursor(0,1);
  lcd.print("H:");
  lcd.print(dhtH,0);
  lcd.print("% BPM:");
  lcd.print(bpm);
}

// ---------- SIMPLE PULSE/PEAK DETECTION ----------
void samplePulseSensor() {
  int signal = readAnalog(PULSE_PIN);
  unsigned long now = millis();

  // Dynamic peak/trough search (very simple)
  if (signal > signalPeak) signalPeak = signal;
  if (signal < signalTrough) signalTrough = signal;

  // Peak detection: rising edge above threshold -> beat
  if (signal > pulseThreshold && !pulseDetected) {
    // Detected a beat
    unsigned long beatTime = now;
    if (lastBeatMillis != 0) {
      unsigned long interval = beatTime - lastBeatMillis;
      if (interval > 250 && interval < 2000) { // ignore too short/long
        BPM = (int)(60000.0 / interval);
      }
    }
    lastBeatMillis = beatTime;
    pulseDetected = true;
    Serial.print("Beat! signal=");
    Serial.print(signal);
    Serial.print(" BPM=");
    Serial.println(BPM);
    Blynk.virtualWrite(VPULSE, BPM);
  }

  // Reset detection when the signal falls below a safety threshold
  if (signal < (pulseThreshold - 150)) {
    pulseDetected = false;
    // adapt threshold slowly (optional)
    // pulseThreshold = (pulseThreshold*9 + (signal + 200))/10;
  }

  lastSignal = signal;

  // Periodically adjust threshold to the signal amplitude if needed
  static unsigned long lastAdjust = 0;
  if (millis() - lastAdjust > 5000) {
    int amplitude = signalPeak - signalTrough;
    // set threshold to trough + 55% of amplitude (tweakable)
    if (amplitude > 100) {
      pulseThreshold = signalTrough + (amplitude * 55 / 100);
    }
    // decay peaks/troughs slowly
    signalPeak = signalPeak * 0.8 + signal * 0.2;
    signalTrough = signalTrough * 0.9 + signal * 0.1;
    lastAdjust = millis();
    Serial.print("Adjust thr to ");
    Serial.println(pulseThreshold);
  }
}

// ---------- TIMED TASKS ----------
void taskSensors() {
  // ECG streaming
  unsigned long now = millis();
  if (now - lastECGMillis >= ECG_SEND_INTERVAL) {
    sendECG();
    lastECGMillis = now;
  }

  // ADXL every 500ms
  static unsigned long lastADXL = 0;
  if (now - lastADXL >= 500) {
    sendADXL();
    lastADXL = now;
  }

  // LM35 every 2000ms
  static unsigned long lastLM35 = 0;
  static float lastLM35Temp = 0;
  if (now - lastLM35 >= 2000) {
    int raw = readAnalog(LM35_PIN);
    float voltage = (raw / 4095.0) * 3.3;
    float lm35Temp = voltage * 100.0;
    Blynk.virtualWrite(VLM35, lm35Temp);
    lastLM35Temp = lm35Temp;
    lastLM35 = now;
  }

  // DHT every 3000ms
  static unsigned long lastDHT = 0;
  static float lastDHTT = 0, lastDHTH = 0;
  if (now - lastDHT >= 3000) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      lastDHTT = t; lastDHTH = h;
      Blynk.virtualWrite(VDHTT, t);
      Blynk.virtualWrite(VDHTh, h);
    }
    lastDHT = now;
  }

  // Update LCD with the most recent values every 2s
  static unsigned long lastLCD = 0;
  if (now - lastLCD >= 2000) {
    // read values again for display
    int rawLM = readAnalog(LM35_PIN);
    float lm35T = (rawLM / 4095.0) * 3.3 * 100.0;
    float dhtT = dht.readTemperature();
    float dhtH = dht.readHumidity();
    updateLCD(lm35T, isnan(dhtT) ? 0.0 : dhtT, isnan(dhtH) ? 0.0 : dhtH, BPM);
    lastLCD = now;
  }
}

// Pulse sampling via timer
void taskPulse() {
  samplePulseSensor();
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 multi-sensor starting...");

  // ADC setup (optional): set attenuation for pins to read full range
  // For better readings you can call analogSetPinAttenuation(pin, ADC_11db) etc,
  // but analogRead should work for many sensors as-is.
  analogReadResolution(12); // 0-4095

  // init LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Starting...");

  // init DHT
  dht.begin();

  // Connect to Blynk
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);

  // Setup timers
  timer.setInterval(50L, taskPulse);   // pulse sample every 50ms
  timer.setInterval(200L, taskSensors); // sensors handler every 200ms

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Blynk connected");
  delay(800);
}

// ---------- LOOP ----------
void loop() {
  Blynk.run();
  timer.run();
}