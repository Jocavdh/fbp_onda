/***********************************************************
  ONDA - Prototype code - V11 - Joca van der Horst
  
  Code to read values from Sharp GP2 and MQ135. The values are sorted into categories.
  Each category defines a fading speed for a connected LED Strip.
  After the sensor value of a TCRT5000 proximity sensor is below a threshold, the LED strip fades for a limited time.
  After the sensor value of a TCRT5000 proximity sensor is below a second threshold, the LED strip stays on for a limited time.

  Circuit schematic is located in the same folder as this sketch and called circuit.png

  ---------

  Code uses (modified) parts of the following code:
  Jgillick - LEDFADER library - https://github.com/jgillick/arduino-LEDFader
  GeorgK - MQ135 library - https://github.com/GeorgK/MQ135
  Adeept.com - Sharp GP2 code - http://www.adeept.com/index.php?con=index&act=blogdetails&id=122
  Loe Feijs and TU/e (2013) - LightTimeLDR (example of Finite State Machine on Arduino) - http://www.idemployee.id.tue.nl/l.m.g.feijs/DBB120/ & http://www.idemployee.id.tue.nl/l.m.g.feijs/DBB120/LightTimeLDR.zip

  
***********************************************************/



/***********************************************************
  Libraries
***********************************************************/

#include <LEDFader.h> // Simplifies LED fading
#include <MQ135.h>    // Converts voltage of the sensor to pollution in ppm
#include <Arduino.h>


/***********************************************************
  DEFINITIONS & CONSTANTS
***********************************************************/

// SHARP GP2 - Optical dust sensor
#define PIN_DATA_OUT A0 //Connect the IO port of the GP2 sensor analog A0 output
#define PIN_LED_VCC 4 //The pin in the GP2 sensor that supplies power to the internal Led

const int DELAY_BEFORE_SAMPLING = 280; //Waiting time before sampling
const int DELAY_AFTER_SAMPLING = 40; //Wait time after sampling
const int DELAY_LED_OFF = 9680; //Intervals

unsigned long previousSerialMillis; // millis() timing Variable, just for fading
int SerialInterval = 500; // How fast to increment?

float outputV; // global variable to influence fading speed



// MQ-135 - Gas sensor
#define ANALOGPIN 2 //Connect the IO port of the MQ135 sensor analog A2 output



float airthreshold[3] = {400, 400, 800}; // Required by the MQ135 library
MQ135 gasSensor = MQ135(ANALOGPIN);  // Initialize the gas Sensor
float ppm = gasSensor.getPPM();



// TCRT5000 - Proximity sensor
const int touchPin = A3;  // the number of the touch pin
int threshold = 450; // Sensor value that indicates a light press
int thresholdHard = 75; // Sensor value that indicates a hard press

int touchState = 1024; // Initial touch state set to 1024 (no touch)
int HardpressState = 0; // To debounce after a light press is detected



// LED - Pin connected to the gate of a mosfet, that controls a LED strip

#define LED_PIN 9    // Gate of mosfet connected to pin 9

float FADE_TIME;     // Registering a variable for the fadetime

LEDFader led; // Required by LedFADER library
int direction;
#define DIR_UP 1    // State for fading up
#define DIR_DOWN -1 // State for fading down



// TIMER - Used for certain state changes
int interactionTime = 10000; // time to fade after touch in ms
#define DELAY 30000 //milliseconds

static unsigned long time;
static unsigned int state;


/***********************************************************
  SETUP & LOOP
***********************************************************/


void setup() {

  Serial.begin(9600); // Turn the Serial Protocol ON
  Serial.print("Onda is switched on");

  pinMode(PIN_DATA_OUT, INPUT); //Dust sensor Defined as input (ADC reads analog)
  pinMode(PIN_LED_VCC, OUTPUT); //Dust sensor LED Defined as output

  led = LEDFader(LED_PIN); // Define LED pin
  digitalWrite(LED_PIN, LOW); // Switch LED off at the start

  led.update();
  if (led.is_fading() == false) {

    direction = DIR_DOWN;
    led.fade(0, 100);
  }

  time = 0; // Set timer to 0
  state = 1; // Define start state of finite state machine
}

void loop() {
  unsigned long currentMillis = millis(); // Start timer of Arduino
  sensorValues(currentMillis);            // Send timer value to sensor function to define when to send data over serial

  led.update();

  touchState = analogRead(touchPin); // Get sensor values of proximity sensor to define the touchState


  // Finite state machine
  // 1 Start: LED off, no touch. Switch to 2 after user touches textile
  // 2 LED's start fading. If a hard press is detected, go to 3. If the timer is done, go back to 1.
  // 3 LED's stay on for limited time. Touching the fabric makes the state go back to 2

  switch (state)
  {
    case 1:
      if (touchState < threshold) { // enter next state and save current time
        state = 2;
        time = millis();
      }
      break;
    case 2:


      if (touchState < thresholdHard && millis() > time + 1000) {
        state = 3;
        HardpressState = 1;
      }

      else if (millis() > time + interactionTime ) { //check if time has passed
        led.fade(0, 500);
        state = 1;
      }

      ledFade();

      break;
    case 3:
      ledOn();

      if (touchState > threshold) { // enter next state and save current time
        HardpressState = 0;
      }

      if (touchState < threshold && HardpressState == 0) { // enter next state and save current time
        led.fade(0, 500);
        state = 1;
      }

      else if (millis() > time + 30000 ) { //check if time has passed
        led.fade(0, 500);
        state = 2
                ;
      }

      break;
  }


}


/***********************************************************
  SHARP GP2 & MQ-135 FUNCTIONS
***********************************************************/

/* Read the GP2 sensor output voltage */
double getOutputV() {
  digitalWrite(PIN_LED_VCC, LOW);
  delayMicroseconds(DELAY_BEFORE_SAMPLING);
  double analogOutput = analogRead(PIN_DATA_OUT);
  delayMicroseconds(DELAY_AFTER_SAMPLING);
  digitalWrite(PIN_LED_VCC, HIGH);
  delayMicroseconds(DELAY_LED_OFF);
  //Arduino analog read value range is 0 ~ 1023, the following conversion is 0 ~ 5v
  double outputV = analogOutput / 1024 * 5;
  return outputV;
}

/* Calculate the dust density based on the output voltage */
double getDustDensity(double outputV) {
  //Output voltage and dust density conversion formula: ug / m3 = (V - 0.9) / 5 * 1000
  double ugm3 = (outputV - 0.9) / 5 * 1000;
  //Remove the undetected range
  if (ugm3 < 0) {
    ugm3 = 0;
  }
  return ugm3;
}

/* Calculate AQI based on dust density  Environmental air quality index (AQI) technical requirements (Trial) */
double getAQI(double ugm3) {
  double aqiL = 0;
  double aqiH = 0;
  double bpL = 0;
  double bpH = 0;
  double aqi = 0;
  //According to the correspondence between pm and aqi were calculated aqi
  if (ugm3 >= 0 && ugm3 <= 35) {
    aqiL = 0;
    aqiH = 50;
    bpL = 0;
    bpH = 35;
  } else if (ugm3 > 35 && ugm3 <= 75) {
    aqiL = 50;
    aqiH = 100;
    bpL = 35;
    bpH = 75;
  } else if (ugm3 > 75 && ugm3 <= 115) {
    aqiL = 100;
    aqiH = 150;
    bpL = 75;
    bpH = 115;
  } else if (ugm3 > 115 && ugm3 <= 150) {
    aqiL = 150;
    aqiH = 200;
    bpL = 115;
    bpH = 150;
  } else if (ugm3 > 150 && ugm3 <= 250) {
    aqiL = 200;
    aqiH = 300;
    bpL = 150;
    bpH = 250;
  } else if (ugm3 > 250 && ugm3 <= 350) {
    aqiL = 300;
    aqiH = 400;
    bpL = 250;
    bpH = 350;
  } else if (ugm3 > 350) {
    aqiL = 400;
    aqiH = 500;
    bpL = 350;
    bpH = 500;
  }
  //formula: aqi = (aqiH - aqiL) / (bpH - bpL) * (desity - bpL) + aqiL;
  aqi = (aqiH - aqiL) / (bpH - bpL) * (ugm3 - bpL) + aqiL;
  return aqi;
}

/**
   According to aqi access level description
*/
String getGradeInfo(double aqi) {
  float ppm = gasSensor.getPPM();

  String gradeInfo;
  if ((aqi >= 0 && aqi <= 100) || (ppm <= 550)) {
    gradeInfo = String("Good");
    FADE_TIME = int(5000);
  }

  if ((aqi > 100 && aqi <= 200) || (550 < ppm && ppm <= 2000) ) {
    gradeInfo = String("Medium polluted");
    FADE_TIME = int(2500);
  }

  if ((aqi > 200 && aqi <= 300) || (2000 < ppm && ppm <= 5000 )) {
    gradeInfo = String("Heavily polluted");
    FADE_TIME = int(1250);
  }

  if (aqi > 300 || ppm > 5000) {
    gradeInfo = String("Broken roof!!!");
    FADE_TIME = int(500);
  }
  return gradeInfo;
}

void sensorValues(unsigned long thisMillis) {

  double outputV = getOutputV(); //Sampling Get the output voltage
  double ugm3 = getDustDensity(outputV); //Calculate the dust concentration
  double aqi = getAQI(ugm3); //Calculate aqi
  String gradeInfo = getGradeInfo(aqi); //Calculate the level
  float ppm = gasSensor.getPPM();





  // is it time to update yet?
  // if not, nothing happens
  if (thisMillis - previousSerialMillis >= SerialInterval) {
    // yup, it's time!
    //Print to the serial port
    Serial.println(String("outputV=") + outputV + "\tug/m3=" + ugm3 + "\tAQI=" + aqi + "\tCO2=" + ppm  + "\tgradeInfo=" + gradeInfo + "\tdist=" + 
                  );
    // reset millis for the next iteration (fade timer only)
    previousSerialMillis = thisMillis;
  }
}


/***********************************************************
  LED FADING
***********************************************************/

void ledOn() {
  led.update();

  if (led.is_fading() == true || led.is_fading() == false) {

    led.fade(255, 500);

  }
}

/*  Fades a single LED up and down using LED Fader. */
void ledFade() {
  led.update();

  if (led.is_fading() == false) {

    // Fade from 255 - 0
    if (led.get_value() == 255) {
      direction = DIR_DOWN;
      led.fade(0, (FADE_TIME * 0.5));
    }
    // Fade from 0 - 255
    else {
      direction = DIR_UP;
      led.fade(255, (FADE_TIME * 0.5));
    }
  }

}
