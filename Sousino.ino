// DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Sparkfun SerLCD
#include <SoftwareSerial.h>
#include <serLCD.h>

// PID Library
#include <PID_v1.h>

// EEPROM
#include <EEPROMEx.h>

// Pins
const int LCD_PIN = 4;
const int BUTTON_LEFT_PIN = 6;
const int BUTTON_RIGHT_PIN = 7;
const int RELAY_PIN = 9;
const int SENSOR_PIN = 10;

// PID
const unsigned int SAMPLE_TIME = 1000;
const unsigned int WINDOW_SIZE = 10000;

double input;
double target;
double output;

// Values tuned for 1.5L 500W rice cooker (http://www.clasohlson.com/no/Riskoker/34-9719)
double kP = 875;
double kI = 0.5;
double kD = 0.1;

volatile long onTime = 0; // How long to keep the relay on
unsigned long windowStartTime;
unsigned long now;

PID pid(&input, &output, &target, kP, kI, kD, DIRECT);

// EEPROM
const unsigned int EEPROM_TARGET = 0;

// Sensor
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress temperatureSensor;

double temperature;

// LCD
serLCD lcd(LCD_PIN);

// Buttons
boolean buttonLeft = 0;
boolean buttonRight = 0;

void setup() {
  Serial.begin(9600);

  loadConfiguration();
  
  showSplashScreen(2000);
  showCookingScreen();

  setupTemperatureSensor();
  setupPID();
  setupRelay();
  setupTimer();
  setupButtons();
}

void loop() {
  readButtons();
  readTemperature();

  updatePID();
  updateCookingScreen();

  delay(100);
}

void showSplashScreen(int duration) {
  lcd.clear();
  lcd.print(F("    Sousino!    "));
  lcd.print(F("      v1.0      "));

  delay(duration);
}

void showCookingScreen() {
  lcd.clear();
  lcd.print(F("Current:"));
  lcd.selectLine(2);
  lcd.print(F(" Target:"));
}

void setupTemperatureSensor() {
  sensors.begin();
  sensors.getAddress(temperatureSensor, 0);
  sensors.setResolution(temperatureSensor, 12);
  sensors.setWaitForConversion(false);
}

void setupPID() {
  pid.SetTunings(kP, kI, kD);
  pid.SetSampleTime(SAMPLE_TIME);
  pid.SetOutputLimits(0, WINDOW_SIZE);
  pid.SetMode(AUTOMATIC);
}

void setupTimer() {
  TCCR2A = 0;
  TCCR2B = 1 << CS22 | 1 << CS21 | 1 << CS20;

  // Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1 << TOIE2;
}

SIGNAL(TIMER2_OVF_vect) {
  updateRelay();
}

void setupRelay() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Make sure relay is off
}

void setupButtons() {
  pinMode(BUTTON_LEFT_PIN, INPUT);
  pinMode(BUTTON_RIGHT_PIN, INPUT);
}

void readButtons() {
  buttonLeft = digitalRead(BUTTON_LEFT_PIN);
  buttonRight = digitalRead(BUTTON_RIGHT_PIN);

  if (buttonLeft) {
    target -= 0.5;
  } else if (buttonRight) {
    target += 0.5;
  }  
}

void readTemperature() {
  if (sensors.isConversionAvailable(0)) {
    temperature = sensors.getTempC(temperatureSensor);
    
    // Correct for noise
    if (temperature > 0) {
      input = temperature;
    }
    
    sensors.requestTemperatures();
    
    // MegunoLink Lite (http://www.megunolink.com/megunolink-lite) 

    // Target
    Serial.print(F("{Target,T,"));
    Serial.print(target, 2);
    Serial.println(F("}"));    
    
    // Input
    Serial.print(F("{Input,T,"));
    Serial.print(input, 2);
    Serial.println(F("}"));    
    
    // Target
    Serial.print(F("{Output,T,"));
    Serial.print(output / 100, 2);
    Serial.println(F("}"));
  }
}

void updateCookingScreen() {
  // Current
  lcd.setCursor(1, 10);
  lcd.print(input, 1);

  // Target
  lcd.setCursor(2, 10);
  lcd.print(target, 1);
}

void updatePID() {
  pid.Compute();

  onTime = output;
}

void updateRelay() {
  now = millis();

  if (now - windowStartTime > WINDOW_SIZE) {
    windowStartTime += WINDOW_SIZE;
  }

  digitalWrite(RELAY_PIN, onTime > 100 && onTime > (now - windowStartTime));
}

void saveConfiguration() {
  EEPROM.updateDouble(EEPROM_TARGET, target);
}

void loadConfiguration() {
  target = EEPROM.readDouble(EEPROM_TARGET);
  
  // Default values
  if (isnan(target)) target = 55;
}
