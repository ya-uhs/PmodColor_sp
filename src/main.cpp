#include <Arduino.h>
#include <Wire.h>
#include "PmodColor.h"

int LED_EN = 4;
uint16_t clearData = 0;
uint16_t redData = 0;
uint16_t greenData = 0;
uint16_t blueData = 0;

PmodColor colorSensor;

void setup() {
  Serial.begin(115200);

  pinMode(LED_EN, OUTPUT);
  digitalWrite(LED_EN, LOW);  //no need to accidentally blind people

  if(!colorSensor.begin()){
    Serial.println("Color sensor not found, check your connections");
    while(1); //the sketch stops here
  }
  colorSensor.setIntegrationCycles(7);  //7 was choose for the delay calculations

}//END of setup

void loop() {
  //turn on light
  //wait at least 2x of the integration time + 2.4 ms of setup time
  //  to ensure latest data set in the register had proper lighting.
  //Get data from the part -- the TCS3472 hardware setup prevents
  //  the reading of mismatched data

  //digitalWrite(LED_EN, HIGH); //turn on the embedded LED, it is bright so be careful
  int waitTime = (2.4 + 2.4*2*7);
  delay(waitTime);
  colorSensor.getData(clearData, redData, greenData, blueData);
  //digitalWrite(LED_EN, LOW); //turn off the embedded LED, uncomment if you are turning on the LED

  
  Serial.print("Clear data: ");Serial.println(clearData);
  Serial.print("Red data: ");Serial.println(redData);
  Serial.print("Green data: ");Serial.println(greenData);
  Serial.print("Blue data: ");Serial.println(blueData);
  Serial.println('\n');

  delay(500); //small delay to better view the measured data.
  
}
