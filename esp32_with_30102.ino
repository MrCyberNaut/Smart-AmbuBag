#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <DHT.h>
#include "MAX30105.h"     // Changed from MAX30100
#include "heartRate.h"    // Added for heart rate calculation
#include "spo2_algorithm.h"  // Added for SpO2 calculation

// Define I2C pins for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// Other Pin Definitions
#define SERVO_PIN 13
#define DHT_PIN 14
#define BUZZER_PIN 12
#define MODE_BUTTON_PIN 27
#define DHT_TYPE DHT11

// Constants for MAX30102
const uint8_t RATE_SIZE = 4; // Increase this for more averaging
const byte FINGER_ON = 50000; // If red signal is lower than this, assume finger is not on sensor
uint32_t irBuffer[100];  // IR LED sensor data
uint32_t redBuffer[100]; // Red LED sensor data
int32_t bufferLength;    // Data length
int32_t spo2;           // SPO2 value
int8_t validSPO2;       // Indicator to show if the SPO2 calculation is valid
int32_t heartRate;      // Heart rate value
int8_t validHeartRate;  // Indicator to show if the heart rate calculation is valid
byte pulseLED = 2;      // Must be on PWM pin
byte readLED = 19;      // Blinks with each data read

// Constants for different age groups
enum VentMode {
  INFANT,
  TEEN,
  ADULT
};

struct AgeGroup {
  const char* name;
  int minBPM;
  int maxBPM;
  int defaultBPM;
  int minHR;
};

const AgeGroup AGE_GROUPS[] = {
  {"INFANT", 20, 40, 30, 100},
  {"TEEN  ", 12, 20, 16, 75},
  {"ADULT ", 10, 12, 11, 60}
};

// Initialize objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo ambuServo;
DHT dht(DHT_PIN, DHT_TYPE);
MAX30105 particleSensor;  // Changed from PulseOximeter to MAX30105

// Global variables
VentMode currentMode = ADULT;
unsigned long lastUpdate = 0;
unsigned long lastButtonCheck = 0;
unsigned long lastServoMove = 0;
unsigned long lastSerialOutput = 0;
unsigned long lastBeatCheck = 0;
bool buttonPressed = false;
int currentAngle = 0;
float currentTemp = 0;
float currentHumidity = 0;
const int SERVO_MIN = 0;
const int SERVO_MAX = 90;
const unsigned long DEBOUNCE_DELAY = 200;
const unsigned long REPORTING_PERIOD_MS = 1000;
const unsigned long SERIAL_UPDATE_MS = 500;

void printSerialHeader() {
  Serial.println("\n=================================================");
  Serial.println("Smart Ventilator System - Status Report");
  Serial.println("=================================================");
}

void printSerialStatus() {
  if (millis() - lastSerialOutput < SERIAL_UPDATE_MS) return;
  
  printSerialHeader();
  
  // Print current mode and settings
  Serial.println("\nCurrent Settings:");
  Serial.printf("Mode: %s\n", AGE_GROUPS[currentMode].name);
  Serial.printf("Target BPM Range: %d-%d\n", AGE_GROUPS[currentMode].minBPM, AGE_GROUPS[currentMode].maxBPM);
  Serial.printf("Current BPM Setting: %d\n", AGE_GROUPS[currentMode].defaultBPM);
  Serial.printf("HR Alert Threshold: %d\n", AGE_GROUPS[currentMode].minHR);
  
  // Print vital signs
  Serial.println("\nVital Signs:");
  if (validHeartRate && validSPO2) {
    Serial.printf("Heart Rate: %d BPM\n", heartRate);
    Serial.printf("SpO2: %d%%\n", spo2);
  } else {
    Serial.println("Finger not detected on sensor");
  }
  
  // Print environmental data
  currentTemp = dht.readTemperature();
  currentHumidity = dht.readHumidity();
  Serial.println("\nEnvironmental Conditions:");
  if (!isnan(currentTemp) && !isnan(currentHumidity)) {
    Serial.printf("Temperature: %.1f°C\n", currentTemp);
    Serial.printf("Humidity: %.1f%%\n", currentHumidity);
  } else {
    Serial.println("DHT Sensor reading error!");
  }
  
  // Print ventilator status
  Serial.println("\nVentilator Status:");
  Serial.printf("Servo Position: %d degrees\n", currentAngle);
  Serial.printf("Cycles/min: %d\n", AGE_GROUPS[currentMode].defaultBPM);
  
  // Print system uptime
  Serial.println("\nSystem Info:");
  Serial.printf("Uptime: %lu seconds\n", millis() / 1000);
  
  Serial.println("=================================================\n");
  
  lastSerialOutput = millis();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  Serial.println("\nInitializing Smart Ventilator System...");
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C initialized");
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Starting up...");
  Serial.println("LCD initialized");
  
  // Initialize servo
  ESP32PWM::allocateTimer(0);
  ambuServo.setPeriodHertz(50);
  ambuServo.attach(SERVO_PIN, 500, 2400);
  ambuServo.write(SERVO_MIN);
  Serial.println("Servo initialized");
  
  // Initialize other components
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);
  dht.begin();
  Serial.println("Pins and DHT sensor initialized");
  
  // Initialize MAX30102
  Serial.print("Initializing MAX30102...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("FAILED");
    lcd.clear();
    lcd.print("Sensor Failed!");
    while(1) {
      Serial.println("MAX30102 initialization failed!");
      delay(1000);
    }
  }
  Serial.println("SUCCESS");
  
  // Configure MAX30102 sensor
  particleSensor.setup(60, 4, 2, 100, 411, 4096); // Configure sensor with typical settings
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  
  updateDisplay();
  Serial.println("System initialization complete!\n");
  
  // Initialize the sensor buffer with initial samples
  bufferLength = 100;
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }
}

void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Mode:");
  lcd.print(AGE_GROUPS[currentMode].name);
  lcd.setCursor(0, 1);
  lcd.print("HR:");
}

void checkModeButton() {
  if (millis() - lastButtonCheck < DEBOUNCE_DELAY) return;
  
  if (digitalRead(MODE_BUTTON_PIN) == LOW && !buttonPressed) {
    buttonPressed = true;
    lastButtonCheck = millis();
    currentMode = static_cast<VentMode>((currentMode + 1) % 3);
    Serial.printf("\nMode changed to: %s\n", AGE_GROUPS[currentMode].name);
    updateDisplay();
  } else if (digitalRead(MODE_BUTTON_PIN) == HIGH) {
    buttonPressed = false;
  }
}

void operateVentilator() {
  unsigned long currentTime = millis();
  int currentBPM = AGE_GROUPS[currentMode].defaultBPM;
  unsigned long cycleTime = (60000 / currentBPM) / 2;
  
  if (currentTime - lastServoMove > cycleTime) {
    if (currentAngle == SERVO_MIN) {
      ambuServo.write(SERVO_MAX);
      currentAngle = SERVO_MAX;
      Serial.println("Ventilator: Compression");
    } else {
      ambuServo.write(SERVO_MIN);
      currentAngle = SERVO_MIN;
      Serial.println("Ventilator: Release");
    }
    lastServoMove = currentTime;
  }
}

void checkVitals() {
  // Check if finger is on sensor
  if (particleSensor.getRed() < FINGER_ON) {
    Serial.println("No finger detected");
    return;
  }
  
  // Get new samples
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false)
      particleSensor.check();
      
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }
  
  // Calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  if (validHeartRate && validSPO2) {
    lcd.setCursor(3, 1);
    lcd.print("    ");
    lcd.setCursor(3, 1);
    lcd.print(heartRate);
    
    if (heartRate < AGE_GROUPS[currentMode].minHR) {
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.printf("!!! ALERT: Low Heart Rate: %d (Threshold: %d) !!!\n", 
                   heartRate, AGE_GROUPS[currentMode].minHR);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void loop() {
  checkModeButton();
  operateVentilator();
  
  if (millis() - lastUpdate > REPORTING_PERIOD_MS) {
    checkVitals();
    lastUpdate = millis();
  }
  
  // Print detailed status to serial monitor
  printSerialStatus();
}
