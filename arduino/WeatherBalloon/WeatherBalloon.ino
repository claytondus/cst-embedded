/*ECE 455/555 Weather Balloon Design Project Microcontroller Program
 *Micro: ATSAMD21 Board: Adafruit Feather M0 Datalogger
 *
 *Program By: Sunay Bhat, Clayton Davis, Thomas Turner 
 */

#include <Arduino.h>
#include "wiring_private.h"
#include <Base64.h>
#include "Adafruit_FONA.h"
#include <Adafruit_VC0706.h>
//#include <math.h> 
#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Battery.h> 


#define FONA_RX 12
#define FONA_TX 10
#define FONA_RST A4
#define GAS_PWR 5
#define CAMERA_PWR 6
#define CAMERA_TX 0
#define CAMERA_RX 1
#define JETTISON 12

Uart Serial2 (&sercom1, CAMERA_RX, CAMERA_TX, SERCOM_RX_PAD_3, UART_TX_PAD_2);
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

Adafruit_VC0706 cam = Adafruit_VC0706(&Serial1);
char img_filename[27];

SFE_BMP180 pressure;

const int chipSelect = 4;
const int methane = A1;
const int carbMon = A2;


boolean firstSD = true;
double baseline,T,P,a;          
String errorData = "";     
String dataString, sendStr; 

//FONA
uint8_t fona_type;
// this is a large buffer for replies
char replybuffer[255];
char gpsData[120];
HardwareSerial *fonaSerial = &Serial2;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
char sendto[21] = "8652402612";
char smsBuffer[140], fileBuffer[1020], emailBuffer[1360];
char email_address[13] = "cd@ae4ax.net";
char fonaTime[23];
uint16_t vbat, vpct;

void setup() {

  pinPeripheral(FONA_RX, PIO_SERCOM);
  pinPeripheral(FONA_TX, PIO_SERCOM);
  Serial.begin(115200);
  Serial1.begin(38400);  //Camera
  
  //while(!Serial);
  Serial.println("Hello");

  //Start FONA
  fona_type = FONA808_V2;
  fona.setGPRSNetworkSettings(F("wholesale"), F(""), F(""));
  fona.setEmailSettings(F("mailtrap.io"), F("2525"), F("f010e174516ec4"), F("c37b35c7867b88"));
  fonaSerial->begin(38400);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

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

  if (cam.begin()) {
    Serial.println("Camera Found:");
  }
  else {
    Serial.println("Camera Fail");
    errorData += "Camera fail";
  }
  cam.setImageSize(VC0706_640x480); 

  //Set RTC from GPRS signal
  Serial.println("Waiting to start GPRS");
  delay(10000);
  enableGPRS();
  delay(5000);
  enableNTPSync();
  disableGPRS();

  //Start GPS to get a fix, hopefully it will have one
  //before the first SMS is sent!
  enableGPS();

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

  Serial.println("Starting measurement");
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


  fona.getBattVoltage(&vbat);
  fona.getBattPercent(&vpct);
  fona.getTime(fonaTime, 23);

  sendStr += String(fonaTime);
  sendStr += ",";
  //sendStr += "Pressure: ";
  sendStr += String(sumP / 10);
  sendStr += ",";
  //sendStr += "Altitude: ";
  sendStr += String(sumA / 10);
  sendStr += ",";
  //sendStr += "Temp: ";
  sendStr += String(sumT / 10);
  sendStr += ",";
  //sendStr += "MethaneV: ";
  sendStr += String(sumM / 10);
  sendStr += ",";
  //sendStr += "COV: ";
  sendStr += String(sumC / 10);
  sendStr += ",";
  sendStr += String(vbat);
  sendStr += ",";
  sendStr += String(vpct);
  sendStr += "\n";

  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    if (firstSD == true){  
      dataFile.print(errorData);
      firstSD = false;
    }
    Serial.println("Writing data");
    dataFile.print(dataString);
    dataFile.close();    
  } else {
    Serial.println("Could not open data file");
  }

  Serial.println(sendStr);

  sendSMS(sendStr);
  getGPSData(gpsData);
  sendSMS(gpsData);
  fona.getTime(fonaTime, 23);

  takeAndSaveImage();
  enableGPRS();
  sendEmail(String(img_filename), img_filename, String(img_filename));
  disableGPRS();
  
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

//FONA functions
boolean enableGPRS() {
  if (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to turn on GPRS"));
    return false;       
  } else {
    Serial.println(F("GPRS is on"));
    return true;
  }
}

boolean disableGPRS() {
  if (!fona.enableGPRS(false)) {
    Serial.println(F("Failed to turn off GPRS"));
    return false;       
  } else {
    Serial.println(F("GPRS is off"));
    return true;
  }
}

boolean enableGPS() {
  if (!fona.enableGPS(true)) {
    Serial.println(F("Failed to turn on GPS"));
    return false;       
  } else {
    Serial.println(F("GPS is on"));
    return true;
  }
}

boolean disableGPS() {
  if (!fona.enableGPS(false)) {
    Serial.println(F("Failed to turn off GPS"));
    return false;       
  } else {
    Serial.println(F("GPS is off"));
    return true;
  }
}

int8_t checkGPSFix() {
    int8_t stat;
  // check GPS fix
  stat = fona.GPSstatus();
  if (stat < 0)
    Serial.println(F("Failed to query"));
  if (stat == 0) Serial.println(F("GPS off"));
  if (stat == 1) Serial.println(F("No fix"));
  if (stat == 2) Serial.println(F("2D fix"));
  if (stat == 3) Serial.println(F("3D fix"));
  return stat;
}

boolean getGPSData(char *gpsdata) {
  if (!fona.getGPS(0, gpsdata, 120)) {
    Serial.println(F("Failed to get GPS data"));
    return false;       
  } else {
    Serial.println(gpsdata);
    return true;
  }
}

boolean enableNTPSync() {
    //Must enable GPRS first
    if (!fona.enableNTPTimeSync(true, F("pool.ntp.org"))) {
      Serial.println(F("Failed to enable NTP"));
    } else {
      Serial.println(F("Enabled NTP"));
    }
}

boolean sendSMS(String msg) { 
  msg.toCharArray(smsBuffer, 140);
  if (!fona.sendSMS(sendto, smsBuffer)) {
    Serial.println(F("Failed to send SMS"));
    return false;
  } else {
    Serial.println(F("Sent SMS!"));
    return true;
  }
}

boolean sendEmail(String subject, char *attachment, String message) {
  char subjectBuf[140], messageBuf[140];
  subject.toCharArray(subjectBuf,140);
  message.toCharArray(messageBuf,140);
  unsigned long mmsFileSize = 0;
  unsigned int bytesToRead = 0;
  int encodedLen = 0;
  
  File myImage = SD.open(attachment, FILE_READ);
  if (myImage) {
    mmsFileSize = myImage.size();
    Serial.print(F("Attachment is "));
    Serial.print(mmsFileSize);
    Serial.println(F(" bytes"));
  } else {
    Serial.println(F("Failed to open image"));
    return false;
  }
  
  if (!fona.sendEmailWithAttachment(messageBuf, attachment, email_address, email_address, subjectBuf)) {
    Serial.println(F("Failed to start email"));
    return false;
  } else {
    Serial.println(F("Started email"));
  }

  while (mmsFileSize > 0) {
    if (mmsFileSize < 1020) {
      bytesToRead = mmsFileSize;
    } else {
      bytesToRead = 1020;
    }
    myImage.read(fileBuffer, bytesToRead);
    encodedLen = base64_enc_len(bytesToRead);
    base64_encode(emailBuffer, fileBuffer, bytesToRead);
    fona.sendAttachment(emailBuffer, encodedLen);
    mmsFileSize -= bytesToRead;
  }
  fona.sendAttachment(emailBuffer, 0);

  if (!fona.checkEmailResult()) {
    Serial.println(F("Failed to send email"));
    return false;
  } else {
    Serial.println(F("Sent email!"));
  }
  return true;
}

boolean takeAndSaveImage() {

  cam.resumeVideo();
  delay(2000);
  if (! cam.takePicture()) {
    Serial.println("Failed to snap!");
    return false;
  } else {
    Serial.println("Picture taken!");
  }
  
  // Create an image with the name <time>.JPG
  
  strncpy(img_filename, fonaTime+10, 2);
  strncpy(img_filename+2, fonaTime+13, 2);
  strncpy(img_filename+4, fonaTime+16, 2);
  strcpy(img_filename+6, ".JPG");
  Serial.println(img_filename);
  
  // Open the file for writing
  File imgFile = SD.open(img_filename, FILE_WRITE);

  // Get the size of the image (frame) taken  
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(64, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");
  
  return true;
}

