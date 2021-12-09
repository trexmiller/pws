/**********************************************************************************************************************
 * This is code to read the Miller Personal Weather Station. This script uses basic Wifi101 example code to connect to 
 * a home Wifi network and send data to web hosting services for personal weather stations. The script reads a Davis
 * mechanical anemometer and a Bosch BME680 breakout board from Adafruit using an Adafruit Feather M0 containing the 
 * WINC1500 Wifi module. This is a work in progress!!
 */
/**********************************************************************************************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <SPI.h>
#include <WiFi101.h>
#include <avr/dtostrf.h>
#include <RTCZero.h>
/****************************************setup wifi connection*********************************************************/
char ssid[] = "Sawadii_2.4GEXT";        //network SSID (name)
char pass[] = "SongkhlaSidRex";    // network password 
int status = WL_IDLE_STATUS;     // the WiFi radio's status
WiFiClient client;
long  rssi;

//This is a function to initially setup wifi conection. 
void connectWIFI(){
   //Configure pins for Adafruit ATWINC1500 Feather
  Serial.println("Starting connection to WiFi");
  Serial.println(" ");
  
  WiFi.setPins(8,7,4,2);
  
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }
  
 //attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    
    // wait 10 seconds for connection:
    delay(10000);
    // you're connected now, so print out the data:
  Serial.print("Connected to the network");
  printCurrentNet();
  printWiFiData(); 
  }
}

/**********************************************Timer****************************************************/
long previousMillis = 0;
long interval = 60*1000*1;

/***********************************************Wind***************************************************/
#define WindSensorPin (11)
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine
float WindSpeedMPH; // speed miles per hour
float WindMax; //Maximum wind speed from four 3 second measurements
float Gust; //WindMax if it is greater than WindSpeedMPH = 5
int VaneValue;// raw analog value from wind vane
int WindDir;// translated 0 - 360 direction
int LastValue;

void readWindDir() {
  VaneValue = analogRead(A3);
  WindDir = map(VaneValue, 0, 4096, 0, 360);
  
}

void isr_rotation () {
  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}

void readWindSpeed() {
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);
  Rotations = 0; // Set Rotations count to 0 ready for calculations
  delay(3000); // Wait 3 seconds to average
  WindSpeedMPH = Rotations * 0.75;
}


//Gust is measured here as the maximum of four wind speed readings over 12 seconds if that maximum is at least 5 mph greater than the average wind speed.
void readGust(){
  int WindRep[4];
  for (int i = 0; i < 4; i++){
    attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);
    Rotations = 0; // Set Rotations count to 0 ready for calculations
    delay(3000); // Wait 3 seconds to average
    WindRep[i] = Rotations * 0.75;
  }

  int WindMax = WindRep[0];
  for (int i = 0; i < 4; i++){
    if (WindRep[i] > WindMax){
      WindMax = WindRep[i];
    }
  }
  
  if (WindMax > WindSpeedMPH + 5){
    Gust =  WindMax;
  } else {
    Gust =  WindSpeedMPH;
  }
}

/***********************************************BME680***************************************************/
#define SEALEVELPRESSURE_HPA (1013.25)
float AirTC;
float AirTF;
float Pressure;
float PressureIn;
float Humidity;
float Gas;
float Alt;
float WC;
float dewptC;
float dewptF;

Adafruit_BME680 bme; //initialize the BME

//Setup the BME
void setupBME(){
  bme.begin();
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void readBME(){
  bme.performReading();
  AirTC = bme.temperature;
  AirTF = (AirTC*1.8)+32;
  Pressure = bme.pressure/100.0;
  Humidity = bme.humidity;
  Gas = bme.gas_resistance/1000.0;  
  Alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  PressureIn = Pressure * 0.029529983071445;
}

void windChill(){
  if ((AirTF <80.0) && (WindSpeedMPH > 3.0))
  {
  WC=35.74+0.6215*AirTF-
      35.75*pow(WindSpeedMPH,0.16)+0.4275*AirTF*pow(WindSpeedMPH,0.16);
  }
    else
  {
    WC=AirTF;
  }
}

void dewPoint(){
  //constants
  float b = 18.678;
  float c = 257.14;

  float TRH = (log(Humidity/100))+ (b*AirTC/(c+AirTC));
  dewptC = (c*TRH)/(b-TRH);
  dewptF = dewptC*1.8 + 32; 
}

/***********************************************Timestamp************************************************/
RTCZero rtc;
int Day; 
int Month; 
int Year;
int Hour;
int Minute; 
int Seconds;

//Set the time with wifi modem that gets it from NTP servers 
void SetTime(){
  rtc.setEpoch(WiFi.getTime()-6*60*60);
  Day = rtc.getDay(); 
  Month = rtc.getMonth(); 
  Year = rtc.getYear(); 
  Hour = rtc.getHours(); 
  Minute = rtc.getMinutes(); 
  Seconds = rtc.getSeconds(); 
}

/***********************************************Battery**************************************************/
float Batv;
float mvolts;

void readBat() {
  Batv = readVolts(A7);
}

float readVolts(int pin) {
  analogReadResolution(12);
  mvolts = analogRead(pin);
  mvolts *= 2;
  mvolts *= (3.3 / 4096);
  mvolts *= 1000;
  return mvolts;
}

//Send data functions to various servers below
/***********************************************ThingSpeak***********************************************/
char server1[] = "api.thingspeak.com";
char ThingString[500];
String ThingAPIKey = "CVWRRWJ8WRCSDQIX";

void ThingSpeak() {
  sprintf(ThingString, "field1=%s&field2=%s&field3=%s&field4=%s&field5=%s&field6=%s&field7=%s&field8=%s"
          ,f2s(AirTF, 2),
          f2s(Pressure, 2),
          f2s(Humidity, 2),
          f2s(WindSpeedMPH, 2),
          f2s(WindDir, 2),
          f2s(WC, 2),
          f2s(Alt, 2),
          f2s(Gas, 2)
         );
  Serial.print("Connecting to ");
  Serial.println(server1);
  if (client.connect(server1, 80)) {
    Serial.print("connected to ");
    Serial.println(client.remoteIP());
    Serial.println("");
    delay(100);
  } else {
    Serial.println("connection failed");
  }

    client.println("POST /update HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("Connection: close");
    client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("X-THINGSPEAKAPIKEY: "+ThingAPIKey);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    String data = ThingString; 
    client.print(data.length());
    client.print("\n\n");
    client.print(ThingString);
    client.flush(); //this seems to be necessary to prevent buffer issue (https://github.com/arduino-libraries/WiFi101/issues/118)
    client.stop();
}

/***********************************************ThingSpeak2***********************************************/
String ThingAPIKey2 = "SX05ZT3QN55ZTV00";
char ThingString2[500];

void ThingSpeak2() {
  sprintf(ThingString2, "field1=%s&field2=%s&field3=%s&field4=%02d&field5=%02d&field6=%02d&field7=%02d&field8=%02d"
          ,f2s(Batv, 2),
          f2s(Gust, 2),
          f2s(dewptF, 2),
          Day,
          Hour,
          Minute,
          Seconds,
          rssi
         );
  Serial.print("Connecting to ");
  Serial.println(server1);
  if (client.connect(server1, 80)) {
    Serial.print("connected to ");
    Serial.println(client.remoteIP());
    Serial.println("");
    delay(100);
  } else {
    Serial.println("connection failed");
  }
  
    client.println("POST /update HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("Connection: close");
    client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("X-THINGSPEAKAPIKEY: "+ThingAPIKey2);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.print("Content-Length: ");
    String data = ThingString2; 
    client.print(data.length());
    client.print("\n\n");
    client.print(ThingString2);
    client.flush();
    client.stop();
    
}

/***********************************************PWS***********************************************/
char server2[] = "pwsupdate.pwsweather.com";
char PWSKey [] = "93335aa830ccbe779d1a3f3713fec60a";
char PWSID [] = "STW001";

void PWS(void){
  Serial.print("Connecting to ");
  Serial.println(server2);
 if (client.connect(server2, 80)) {
    Serial.print("connected to ");
    Serial.println(client.remoteIP());
    Serial.println("");
    delay(100);
  } else {
    Serial.println("connection failed");
  }
  
  client.print("GET /api/v1/submitwx?");
  client.print("ID=");
  client.print(PWSID);
  client.print("&PASSWORD=");
  client.print(PWSKey);
  client.print("&dateutc=now&winddir=");
  client.print(WindDir);
  client.print("&tempf=");
  client.print(AirTF);
  client.print("&windspeedmph=");
  client.print(WindSpeedMPH);    
  client.print("&windgustmph=");
  client.print(Gust);
  client.print("&dewptf=");
  client.print(dewptF);
  client.print("&humidity=");
  client.print(Humidity);
  client.print("&baromin=");
  client.print(PressureIn);
  client.print("&softwaretype=Arduino-SAMD21&action=updateraw");
  client.print("/ HTTP/1.1\r\nHost: pwsupdate.pwsweather.com:80\r\nConnection: close\r\n\r\n");
  Serial.println(" ");
  client.flush();
  client.stop();
  delay(1000); 
}

/***********************************************Wunderground***********************************************/
char server3 [] = "rtupdate.wunderground.com";
char WunderID [] = "KWICEDAR67";
char WunderKey [] = "b4i0M6BW";  

void wunderground(void)
{
  Serial.print("Connecting to ");
  Serial.println(server3);
 if (client.connect(server3, 80)) {
    Serial.print("connected to ");
    Serial.println(client.remoteIP());
    Serial.println("");
    delay(100);
  } else {
    Serial.println("connection failed");
  }
  client.print("GET /weatherstation/updateweatherstation.php?");
  client.print("ID=");
  client.print(WunderID);
  client.print("&PASSWORD=");
  client.print(WunderKey);
  client.print("&dateutc=now&winddir=");
  client.print(WindDir);
  client.print("&tempf=");
  client.print(AirTF);
  client.print("&windspeedmph=");
  client.print(WindSpeedMPH);    
  client.print("&windgustmph=");
  client.print(Gust);
  client.print("&dewptf=");
  client.print(dewptF);
  client.print("&humidity=");
  client.print(Humidity);
  client.print("&baromin=");
  client.print(PressureIn);
  client.print("&softwaretype=Arduino-SAMD21&action=updateraw&realtime=1&rtfreq=30");
  client.print("/ HTTP/1.1\r\nHost: rtupdate.wunderground.com:80\r\nConnection: close\r\n\r\n");
  Serial.println(" ");
  client.flush();
  delay(1000);
}

void setup(void) {
  Serial.begin(115200); 
  delay(2000);   
  Serial.println("Setting things up");
  delay(2000);                                                                                                                               
  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);
  interrupts();
  rtc.begin();
  setupBME();
  connectWIFI();
  SetTime();
  previousMillis = interval;
}

void loop(void) {
  unsigned long currentMillis = millis();
    if (currentMillis - previousMillis>interval) {  
      Serial.println(" ");  
      Serial.println("Starting a round of measurements...."); 
      Serial.println(" "); 
      
      Serial.println("Reading battery voltage...."); 
      readBat();
      Serial.print("Battery Voltage = "); Serial.print(Batv); Serial.println(" Volts");
      Serial.println(" "); 
      
      Serial.println("Reading wind...."); 
      readWindDir();
      readWindSpeed();
      readGust();
      Serial.print("Wind Speed = "); Serial.print(WindSpeedMPH); Serial.println(" mph");
      Serial.print("Wind Direction = "); Serial.print(WindDir); Serial.println(" degrees");
      Serial.print("Wind Gust = "); Serial.print(Gust); Serial.println(" mph");
      Serial.println(" "); 
      
      Serial.println("Reading air temp, humidity, pressure etc");
      readBME();
      windChill(); //Read after wind and BME
      dewPoint(); //Read after BME

      Serial.print("Air Temperature (F) = "); Serial.print(AirTF); Serial.println(" *F");
      Serial.print("Air Temperature (C) = "); Serial.print(AirTC); Serial.println(" *C");
      Serial.print("Barometric Pressure (hPa) = "); Serial.print(Pressure); Serial.println(" hPa");
      Serial.print("Barometric Pressure (in Hg) = "); Serial.print(PressureIn); Serial.println(" in Hg");
      Serial.print("Relative Humidity = "); Serial.print(Humidity); Serial.println(" %");
      Serial.print("Air Quality Index = "); Serial.print(Gas); Serial.println(" KOhms");
      Serial.print("Approx. Altitude = "); Serial.print(Alt); Serial.println(" m");
      Serial.print("Wind Chill = "); Serial.print(WC); Serial.println(" *F");
      Serial.print("Dew Point (F) = "); Serial.print(dewptF); Serial.println(" *F");
      Serial.print("Dew Point (C) = "); Serial.print(dewptC); Serial.println(" *C");
      Serial.println(" "); 
      Serial.println();

      //Repeat connection to Wifi to try and improve connectivity
      Serial.print("Finished reading sensors. Reconnecting to WPA SSID: ");
      Serial.println(ssid);
      status = WiFi.begin(ssid, pass);
      // wait 10 seconds for connection:
      delay(10000);
      
      Serial.print("Connected to the network");
      printCurrentNet();
      printWiFiData(); 
      Serial.print("RSSI = "); Serial.print(rssi); Serial.println(" dBm");

      Serial.println("Setting the time...");
      SetTime();
      Serial.print("Year = "); Serial.println(Year); 
      Serial.print("Month = "); Serial.println(Month); 
      Serial.print("Day = "); Serial.println(Day); 
      Serial.print("Hour = "); Serial.println(Hour); 
      Serial.print("Minute = "); Serial.println(Minute); 
      Serial.print("Seconds = "); Serial.println(Seconds); 
            
      //Now send the data
      Serial.println("Sending the data..."); 
      ThingSpeak();
      ThingSpeak2();
      PWS();
      wunderground();
      
      //Turn off modem
      WiFi.end();
      
      Serial.println("Finished. Modem is off");
  
      Serial.println("");
      Serial.println("***********************************************************************************************************************");
      Serial.println("********************************************Waiting for next round*****************************************************");
      Serial.println("***********************************************************************************************************************");
      Serial.println("");

      previousMillis = currentMillis;
  }
}

void printWiFiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

char *f2s(float f, int p) {
  char * pBuff;                         // use to remember which part of the buffer to use for dtostrf
  const int iSize = 12;                 // number of bufffers, one for each float before wrapping around
  static char sBuff[iSize][20];         // space for 20 characters including NULL terminator for each float
  static int iCount = 0;                // keep a tab of next place in sBuff to use
  pBuff = sBuff[iCount];                // use this buffer
  if (iCount >= iSize - 1) {            // check for wrap
    iCount = 0;                         // if wrapping start again and reset
  }
  else {
    iCount++;                           // advance the counter
  }
  return dtostrf(f, 0, p, pBuff);       // call the library function
}
