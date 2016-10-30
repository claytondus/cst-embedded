/*ECE 455/555 Weather Balloon Design Project Microcontroller Program
 *Micro: ATSAMD21 Board: Adafruit Feather M0 Datalogger
 *
 *Program By: Sunay Bhat, Clayton Davis, Thomas Turner 
 */

#include <Arduino.h>
#include "wiring_private.h"
#include "Adafruit_FONA.h"
#include <Adafruit_VC0706.h>
//#include <math.h> 
#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Battery.h> 


#define FONA_RX 0
#define FONA_TX 1
#define FONA_RST A4
#define FONA_ENA A3
#define GAS_PWR 5
#define CAMERA_PWR 6
#define CAMERA_TX 10
#define CAMERA_RX 12
#define JETTISON 11

Uart Serial2 (&sercom1, CAMERA_RX, CAMERA_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}
Adafruit_VC0706 cam = Adafruit_VC0706(&Serial2);

SFE_BMP180 pressure;

const int chipSelect = 4;
const int methane = A1;
const int carbMon = A2;


boolean firstSD = true;
double baseline,T,P,a;          
String errorData = "";     
String dataString, sendStr; 

void setup() {
  Serial.begin(115200);
  Serial2.begin(38400);

  pinPeripheral(CAMERA_RX, PIO_SERCOM);
  pinPeripheral(CAMERA_TX, PIO_SERCOM);
  
  while(!Serial);
  Serial.println("Hello");

  analogReadResolution(12);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("SD Card fail");
    errorData += "SD Card fail";
  }
  else Serial.println("SD Success!");

  if (pressure.begin()) Serial.println("BMP180 Success!");
  else {
    Serial.println("BMP180 fail");
    errorData += "BMP180 fail";
  }
  baseline = getPress();
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb"); 

  /*if (cam.begin()) Serial.println("Camera Found:");
  else {
    Serial.println("Camera Fail");
    errorData += "Camera fail"
  }
  cam.setImageSize(VC0706_160x120); 
  uint8_t imgsize = cam.getImageSize()
  if (imgsize == VC0706_160x120){
    Serial.print("Image size: ");
    Serial.println("160x120");
  }*/

  pinMode(methane, INPUT);
  pinMode(carbMon, INPUT);
  pinMode(JETTISON, OUTPUT);
}

void loop() {
  float sumP = 0;
  float sumA = 0;
  float sumT = 0;
  float sumM = 0;
  float sumC = 0;

  dataString = ""; 
  sendStr = "";

  for(int i = 0; i < 11; i ++){
    //dataString += "Pressure: ";
    getPress();
    sumP += P;
    dataString += String(P);
    dataString += ", ";
    //dataString += "Altitude: ";
    sumA += a;
    dataString += String(a);
    dataString += ", ";
    //dataString += "Temp: ";
    sumT += T;
    dataString += String(T);;
    dataString += ", ";
    float mthV = getMeth();
    sumM += mthV;
    //dataString += "MethaneV: ";
    dataString += String(mthV);
    dataString += ", ";
    float coV = getCarbon();
    sumC += coV;
    //dataString += "COV: ";
    dataString += String(coV);
    dataString += "\n";
    delay(5500);
  }

  //sendStr += "Pressure: ";
  sendStr += String(sumP / 10);
  sendStr += ", ";
  //sendStr += "Altitude: ";
  sendStr += String(sumA / 10);
  sendStr += ", ";
  //sendStr += "Temp: ";
  sendStr += String(sumT / 10);
  sendStr += ", ";
  //sendStr += "MethaneV: ";
  sendStr += String(sumM / 10);
  sendStr += ", ";
  //sendStr += "COV: ";
  sendStr += String(sumC / 10);
  sendStr += "\n";

  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    if (firstSD == true){  
      dataFile.print(errorData);
      firstSD = false;
    }
    dataFile.print(dataString);
    dataFile.close();    
  }
  
}

float getMeth(){
  int raw = analogRead(methane);
  float volt = raw * (3.3/ 4095.0);
  return volt;
}

float getCarbon(){
  int raw = analogRead(carbMon);
  float volt = raw * (3.3/ 4095.0);
  return volt;
}

double getPress(){
  char status;
  status = pressure.startTemperature();
  if (status != 0){
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0){
      status = pressure.startPressure(3);
      if (status != 0){
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0){
          a = pressure.altitude(P,baseline);
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
