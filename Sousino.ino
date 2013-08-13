// DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Sparkfun SerLCD
#include <SoftwareSerial.h>
#include <serLCD.h>

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// EEPROM
#include <EEPROMEx.h>

// Pins
const int LCD_PIN = 4;
const int BUTTON_LEFT_PIN = 6;
const int BUTTON_RIGHT_PIN = 7;
const int RELAY_PIN = 9;
const int SENSOR_PIN = 10;

// PID
const unsigned int PID_SAMPLE_TIME = 1000;
const unsigned int WINDOW_SIZE = 10000;

double input;
double target = 60;
double output = 50;

double pidP = 850;
double pidI = 0.25;
double pidD = 0.1;

volatile long onTime = 0; // How long to keep the relay on
unsigned long windowStartTime;

PID pid(&input, &output, &target, pidP, pidI, pidD, DIRECT);

// PID Tuner
const double TUNER_OUTPUT_STEP = 500;
const double TUNER_NOISE_BAND = 1;
const unsigned int TUNER_LOOKBACK_DURATION = 20;

boolean tuning = false;

PID_ATune pidTuner(&input, &output);

// EEPROM
const unsigned int EEPROM_TARGET = 0;
const unsigned int EEPROM_P = 8;
const unsigned int EEPROM_I = 16;
const unsigned int EEPROM_D = 24;

// Sensor
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress temperatureSensor;

// LCD
serLCD lcd(LCD_PIN);

// Buttons
boolean buttonLeft = 0;
boolean buttonRight = 0;

void setup() {
  Serial.begin(9600);

  showSplashScreen(2000);

  // loadConfiguration();
  showCookingScreen();

  setupTemperatureSensor();
  setupPID();
  setupRelay();
  setupTimer();
  setupButtons();
}

void loop() {
  while (true) {
    readButtons();
    
    if (buttonLeft && buttonRight) {
      if (tuning) {
        stopTuning();
      } else {
        startTuning();
      }
    } else if (buttonLeft || buttonRight) {
      if (buttonLeft) {
        target -= 0.5;
      } else {
        target += 0.5;
      }

      delay(200);
    }

    readTemperature();

    if (tuning) {
      updatePIDTuner();
    } else {
      updatePID();
    }

    onTime = output;

    updateCookingScreen();

    delay(100);
  }
}

void showSplashScreen(int duration) {
  lcd.clear();
  lcd.print(F("    Sousino!    "));
  lcd.print(F("      v0.1      "));

  delay(duration);
}

void showCookingScreen() {
  lcd.clear();
  lcd.print(F("Temp:"));
  lcd.selectLine(2);
  lcd.print(F("Target:"));
}

void setupTemperatureSensor() {
  sensors.begin();
  sensors.getAddress(temperatureSensor, 0);
  sensors.setResolution(temperatureSensor, 10);
  sensors.setWaitForConversion(false);
}

void setupPID() {
  pid.SetSampleTime(PID_SAMPLE_TIME);
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
}

void readTemperature() {
  if (sensors.isConversionAvailable(0)) {
    input = sensors.getTempC(temperatureSensor);
    sensors.requestTemperatures();
    
    Serial.print(1);
    Serial.print(F(","));
    Serial.print(target);
    Serial.print(F(","));
    Serial.print(input);
    Serial.print(F(","));
    Serial.println(tuning ? 1 : 0);
  }
}

void updateCookingScreen() {
  // Current
  lcd.setCursor(1, 7);
  lcd.print(input, 1);

  // Target
  lcd.setCursor(2, 9);
  lcd.print(target, 1);

  // PID output
  lcd.setCursor(1, 12);
  lcd.print("     ");
  lcd.setCursor(1, 12);
  lcd.print(output, 0);

  // Tuning
  lcd.setCursor(2, 16);
  lcd.print(tuning ? "T" : " ");
}

void updatePID() {
  pid.Compute();
}

void updateRelay() {
  long now = millis();

  if (now - windowStartTime > WINDOW_SIZE) {
    windowStartTime += WINDOW_SIZE;
  }
  
  digitalWrite(RELAY_PIN, onTime > 100 && onTime > (now - windowStartTime));
}

void startTuning() {
  //Serial.println(F("Starting tuner"));

  tuning = true;

  pid.SetMode(MANUAL);

  pidTuner.SetNoiseBand(TUNER_NOISE_BAND);
  pidTuner.SetOutputStep(TUNER_OUTPUT_STEP);
  pidTuner.SetLookbackSec((int)TUNER_LOOKBACK_DURATION);
}

void stopTuning() {
  //Serial.println(F("Done tuning"));

  tuning = false;

  pid.SetMode(AUTOMATIC);

  pidTuner.Cancel();
}

void updatePIDTuner() {
  if (pidTuner.Runtime()) {
    Serial.print(2);
    Serial.print(F(", "));
    Serial.print(pidTuner.GetKp());
    Serial.print(F(", "));
    Serial.print(pidTuner.GetKi());
    Serial.print(F(", "));
    Serial.println(pidTuner.GetKd());

    tuning = false;

    pidP = pidTuner.GetKp();
    pidI = pidTuner.GetKi();
    pidD = pidTuner.GetKd();

    pid.SetTunings(pidP, pidI, pidD);
    pid.SetMode(AUTOMATIC);
  }
}

void saveConfiguration() {
  EEPROM.updateDouble(EEPROM_TARGET, target);
  EEPROM.updateDouble(EEPROM_P, pidP);
  EEPROM.updateDouble(EEPROM_I, pidI);
  EEPROM.updateDouble(EEPROM_D, pidD);
}

void loadConfiguration() {
  target = EEPROM.readDouble(EEPROM_TARGET);
  pidP = EEPROM.readDouble(EEPROM_P);
  pidI = EEPROM.readDouble(EEPROM_I);
  pidD = EEPROM.readDouble(EEPROM_D);
}
