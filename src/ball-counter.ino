#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L0X.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
VL53L0X sensor;
int switchPin = 2;

int numBalls = 0;
bool ballInView = false; // Tracks if a ball is currently in front of the sensor

void setup() {
  Wire.begin();
  Serial.begin(9600);

  pinMode(2, INPUT);

  // 1. Start LCD first
  lcd.init();
  lcd.backlight();
  lcd.print("LCD OK...");
  delay(500); // Wait for power to stabilize after backlight turns on

  // 2. Initialize Sensor
  sensor.setTimeout(500);
  if (!sensor.init()) {
    // If it fails here, the LCD and Sensor are likely fighting for the bus
    lcd.setCursor(0,1);
    lcd.print("Sensor Fail!");
    while (1); 
  }
  
  sensor.startContinuous();
  lcd.clear();
}

void loop() {
  int distance = sensor.readRangeContinuousMillimeters();

  if(digitalRead(switchPin) == HIGH) {
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("All Systems Go");
    lcd.setCursor(0, 1);
    lcd.print("Balls: " + String(numBalls));
    checkIfBallDetected(distance);
  } else {
    lcd.noBacklight();
    numBalls = 0;
    ballInView = false;
    lcd.clear();
  }

  // Check for sensor timeout/error
  if (sensor.timeoutOccurred()) { 
    Serial.println("TIMEOUT"); 
  }

  delay(50); // Faster polling to catch quick-moving balls
}

void checkIfBallDetected(int aDistance) {
  // --- Detection Logic ---
  // If object is closer than 75mm and we haven't counted it yet
  if (aDistance > 0 && aDistance < 150 && !ballInView) {
    ballInView = true;
    numBalls++;
    
    // Update LCD
    lcd.setCursor(7, 1);
    lcd.print("   "); // Clear old digits
    lcd.setCursor(7, 1);
    lcd.print(numBalls);
    
    Serial.print("Ball Detected! Total: ");
    Serial.println(numBalls);
  } 
  // Reset the trigger once the ball clears the sensor (distance > 85mm)
  else if (aDistance > 150 && ballInView) {
    ballInView = false;
  }
}