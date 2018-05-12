#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

//Setup for Neopixel
#define PIN 7
#define LED_COUNT 1
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_COUNT, PIN, NEO_RGB + NEO_KHZ400);

//Setup for I2C ADXL345
#define DEVICE (0x53)      //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)
byte buff[TO_READ] ;        //6 bytes buffer for saving data read from the device
char str[512];              //string buffer to transform data before sending it to the serial port
int regAddress = 0x32;      //first axis-acceleration-data register on the ADXL345
int x, y, z;                //three axis acceleration data
double roll = 0.00, pitch = 0.00;   //Roll & Pitch are the angles which rotate by the axis X and y

//Setup for Soil moisture
int sensor_pin = A0;
int output_value;
char final_output[40] = "";




void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.print("Welcome to IoT NCR Hardware 101!");

  //Initializing NeoPixel
  leds.begin();
  leds.show();
  colorWipe(leds.Color(0, 0, 0), 40);
  
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
}

void loop()
{
  float soilVal = analogSensor();
  controlLED(soilVal);
  delay(1000);
}

void controlLED(float soilVal){
if(soilVal < 500){
    lightLED();
  } else{
    turnLEDOff();
  }  
  
}


//Analog Sensor Code
float analogSensor(){
  output_value = analogRead(sensor_pin);
  Serial.print("Moisture : ");
  Serial.println(output_value);
  return output_value;
}



//Digital NeopPixel Code
void lightLED(){
  Serial.println("Turing on the Neopixel!!");
  colorWipe(leds.Color(255, 0, 0), 40);
}

void turnLEDOff(){
  Serial.println("Turing off the Neopixel!!");
  colorWipe(leds.Color(0, 0, 0), 40);
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<leds.numPixels(); i++) {
    leds.setPixelColor(i, c);
    leds.show();
    delay(wait);
  }
}




//I2C Sensor codes
void calculateAcc(){
  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
                                              //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
                                              //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];   
  y = (((int)buff[3])<< 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];

  //we send the x y z values as a string to the serial port
  Serial.print("The acceleration info of x, y, z are:");
  sprintf(str, "%d %d %d", x, y, z);  
  Serial.print(str);
  Serial.write(10);
  //Roll & Pitch calculate
  RP_calculate();
  Serial.print("Roll:"); Serial.println( roll ); 
  Serial.print("Pitch:"); Serial.println( pitch );
  Serial.println("");
}


//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

    Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

//calculate the Roll&Pitch
void RP_calculate(){
  double x_Buff = float(x);
  double y_Buff = float(y);
  double z_Buff = float(z);
  roll = atan2(y_Buff , z_Buff) * 57.3;
  pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
}
