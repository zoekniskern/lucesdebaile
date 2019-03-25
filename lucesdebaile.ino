//LIBRARIES

//Accelerometer
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>
Adafruit_LSM303 lsm;

//PRESSURE SENSOR
int fsrAnalogPin = 0; // FSR is connected to analog 0
// connect Red LED to pin 11 (PWM pin)
int fsrReading;      // the analog reading from the FSR resistor divider
int LEDbrightness;

//LEDS
#include <Adafruit_NeoPixel.h>
#define PIN 5
#define NUM_LEDS 23
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

int r = 50;
int g = 0;
int b = 0;

int rDir = 1;
int gDir = 1;
int bDir = 1;

lsm303AccelData oldMag;
lsm303AccelData newMag;

void setup() {
  //Accelerometer
  #ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
  #endif
  Serial.begin(9600);

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }

  //PRESSURE SENSOR
  Serial.begin(9600);   // We'll send debugging information via the Serial monitor

  //LEDS
  strip.begin();
  strip.show();
  
  setAll(r,g,b);
}

void loop() {
  //Check Accelerometer
  checkAccel();
  //Check Pressure Sensor
  checkPress();
  //Change LEDs accordingly
  colorCycle();
  //Loop frequency
  delay(100);
}

//Accelerometer function
//changes LED variable for color
void checkAccel(){
  lsm.read();
  Serial.print("AX: "); Serial.print((int)lsm.accelData.x); Serial.println(" ");
  //Serial.print("AY: "); Serial.println((int)lsm.accelData.y); Serial.print(" ");
  //Serial.print("AZ: "); Serial.println((int)lsm.accelData.z); Serial.print(" ");
  //Serial.print("MX: "); Serial.print((int)lsm.magData.x);   Serial.print(" ");
  //Serial.print("MY: "); Serial.print((int)lsm.magData.y);   Serial.print(" ");
  //Serial.print("MZ: "); Serial.println((int)lsm.magData.z); Serial.print(" ");

  if(lsm.accelData.x < 0){
    rDir = rDir * -1;
  } else{
    rDir = 1;
  }
  if(lsm.accelData.y < 0){
    gDir = gDir * -1;
  } else{
    gDir = 1;
  }
  if(lsm.accelData.z < 0){
    bDir = bDir * -1;
  } else{
    bDir = 1;
  }
}

//Accelerometer function
//triggers LED animation
void checkPress() {
  fsrReading = analogRead(fsrAnalogPin);
  if(fsrReading > 100){
    colorWipe(strip.Color(255, 0, 0), 20); // Red
    colorWipe(strip.Color(r, g, b), 50);
  } else{
    setAll(r,g,b);
  }
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);
}

////LED CODE

//Show Strip
void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

//Set Pixel
void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

//set all
void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  Serial.println("Color whipe");
  for(uint16_t i=23; i>0; i--) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void colorCycle(){
  //Serial.println(rDir);
  if(r>50 && r<255){
    r += rDir;
  } else{
    rDir = rDir * -1;
  }

  if(g>0 && g<150){
    g += gDir;
  } else{
    gDir = gDir * -1;
  }

  if(b>0 && b<100){
    b += bDir;
  } else{
    bDir = bDir * -1;
  }
  setAll(r,g,b);
}
