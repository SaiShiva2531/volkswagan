#include <Arduino.h>

// --- RPM Measurement Variables ---
volatile int pulseCount = 0; // Counts the number of pulses
unsigned long prevTime = 0; // Stores the previous time for interval calculation
int pulsesPerRevolution = 20; // Number of pulses per motor revolution
float rpm = 0.0; // Calculated RPM

// --- Motor Control Pins ---
int motorPin1 = 6; // Motor connected to pin 6
int motorPin2 = 7; // Motor connected to pin 7
int motorSpeed = 255; // Speed of the motor (0-255 for PWM)

// --- Temperature Sensor Pins ---
int tempPin1 = A1; // Coolant temperature sensor connected to pin A1
int tempPin2 = A2; // Lubricant oil temperature sensor connected to pin A2

// --- Pressure Sensor Pins ---
int pressurePin1 = A0; // Fuel pressure sensor connected to pin A0
int pressurePin2 = A3; // Lubricant oil pressure sensor connected to pin A3

#define kpa2atm 0.00986923267
float pkPaFuel, pkPaLubOil; // Pressure in kPa for fuel and lubricant oil
float pAtmFuel, pAtmLubOil; // Pressure in Atm for fuel and lubricant oil

void setup() {
  // --- Motor Control Setup ---
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // --- Encoder Setup ---
  pinMode(2, INPUT); // Encoder pulse signal connected to pin 2
  attachInterrupt(digitalPinToInterrupt(2), countPulse, RISING); // Interrupt on rising edge for pin 2

  // --- Serial Communication Setup ---
  Serial.begin(9600);

  // --- Start Motor ---
  analogWrite(motorPin1, motorSpeed); // Provide PWM signal to run the motor
  digitalWrite(motorPin2, LOW); // Set the second motor pin to LOW
}

void loop() {
  // --- RPM Calculation ---
  unsigned long currentTime = millis(); // Get the current time
  unsigned long timeInterval = currentTime - prevTime; // Calculate time interval

  if (timeInterval >= 1000) { // Calculate RPM every 1 second
    noInterrupts(); // Temporarily disable interrupts
    rpm = (pulseCount * 60.0) / pulsesPerRevolution; // Calculate RPM
    pulseCount = 0; // Reset pulse count
    prevTime = currentTime; // Update previous time
    interrupts(); // Re-enable interrupts

    // Print RPM to Serial Monitor
    Serial.print("Engine RPM: ");
    Serial.println(rpm);
  }

  // --- Temperature Measurement ---
  // Coolant Temperature
  int tempVal1 = analogRead(tempPin1);
  float voltage1 = (tempVal1 * 5.0) / 1024.0; // Convert to voltage
  float cel1 = voltage1 * 100.0; // Convert voltage to temperature for LM35 (10mV per °C)
  float farh1 = (cel1 * 9) / 5 + 32; // Convert to Fahrenheit

  // Lubricant Oil Temperature
  int tempVal2 = analogRead(tempPin2);
  float voltage2 = (tempVal2 * 5.0) / 1024.0; // Convert to voltage
  float cel2 = voltage2 * 100.0; // Convert voltage to temperature for LM35 (10mV per °C)
  float farh2 = (cel2 * 9) / 5 + 32; // Convert to Fahrenheit

  // --- Pressure Measurement ---
  // Fuel Pressure
  int pressureVal1 = analogRead(pressurePin1);
  pkPaFuel = ((float)pressureVal1 / 1023) * 100; // Assuming 0-5V corresponds to 0-100kPa
  pAtmFuel = pkPaFuel * kpa2atm; // Convert to Atm

  // Lubricant Oil Pressure
  int pressureVal2 = analogRead(pressurePin2);
  pkPaLubOil = ((float)pressureVal2 / 1023) * 100; // Assuming 0-5V corresponds to 0-100kPa
  pAtmLubOil = pkPaLubOil * kpa2atm; // Convert to Atm

  // Print Sensor Data
  Serial.print("Fuel Pressure = ");
  Serial.print(pkPaFuel);
  Serial.print(" kPa ");
  Serial.print(pAtmFuel);
  Serial.println(" Atm");

  Serial.print("Lubricant Oil Pressure = ");
  Serial.print(pkPaLubOil);
  Serial.print(" kPa ");
  Serial.print(pAtmLubOil);
  Serial.println(" Atm");

  Serial.print("Coolant Temperature = ");
  Serial.print(cel1);
  Serial.print(" *C ");
  Serial.print(farh1);
  Serial.println(" F");

  Serial.print("Lubricant Oil Temperature = ");
  Serial.print(cel2);
  Serial.print(" *C ");
  Serial.print(farh2);
  Serial.println(" F");

  delay(3000); // Delay to update all readings every 3 seconds
}

// --- Interrupt Service Routine (ISR) for RPM ---
void countPulse() {
  pulseCount++; // Increment pulse count on each pulse
}
