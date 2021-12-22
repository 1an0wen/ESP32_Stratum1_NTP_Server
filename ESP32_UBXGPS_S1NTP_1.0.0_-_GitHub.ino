//setup comms
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include<stdio.h>
#include<time.h>

//setup GPS UBX Protocol
#include <UBX_Parser.h>
#include <HardwareSerial.h>

//--- DEVICE VARIABLES ---
//config wifi include your ssid and password
const char* ssid = "**********";
const char* password = "**********";

//define software and version
const char* appName = "GPS Stratum-1 NTP Server";
const char softVer[] = "1.0.0";

//setup global variables
int i;
byte mac[6];
int time_ticker;
int time_delay = 100;
unsigned char gpsFix;
double latitude;
double longitude;
double alt;
uint32_t gpsSats;
String gpsFT;
uint16_t gpsYear;
uint8_t gpsMonth;
uint8_t gpsDay;
uint8_t gpsHour;
uint8_t gpsMin;
uint8_t gpsSec;
long gpsNano;
String gpsDate;
String deviceID;
uint32_t tempval;

const unsigned long seventyYears = 2208988800UL; // 1970 - 1900 in seconds (Unix to Epoch)

//Declare GPS u-blox UBX Configuration strings
//Enabled UBX protocols for POSLLH, POSECEF, SOL and TIMEUTC (force configure on boot in case GPS config corrupt or GPS on-board battery for ROM)
byte POSLLH_[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x02,0x01,0x0E,0x47};
byte SOL_[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x06,0x01,0x12,0x4F};
byte TIMEUTC_[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x21,0x01,0x2D,0x85};
byte SAVE_CONFIG_[] = {0xB5,0x62,0x06,0x09,0x0D,0x00,0x58,0x2D,0xC7,0x06,0x03,0x00,0x00,0x00,0x68,0x2D,0xC7,0x06,0x17,0xEA,0x65};

//define UBX_Parser data structure
class UBXParser : public UBX_Parser {
    void handle_NAV_POSLLH(unsigned long iTOW, 
        long lon, 
        long lat, 
        long height, 
        long hMSL, 
        unsigned long hAcc, 
        unsigned long vAcc) {
          latitude = lat/1e7;
          longitude = lon/1e7;
          alt = hMSL/1000;
        }  
      void handle_NAV_TIMEUTC(unsigned long iTOW,
        unsigned long tAcc,
        long nano,
        unsigned short year,
        unsigned char month,
        unsigned char day,
        unsigned char hour,
        unsigned char min,
        unsigned char sec){
          gpsYear = year;
          gpsMonth = month;
          gpsDay = day;
          gpsHour = hour;
          gpsMin = min;
          gpsSec = sec;
          gpsNano = nano;
        }
      void handle_NAV_SOL(unsigned long iTOW, 
        long fTOW, 
        short week, 
        unsigned char gpsFix,
        char flags,
        long ecefX,
        long ecefY,
        long ecefZ,
        unsigned long pAcc,
        long ecefVX,
        long ecefVY,
        long ecefVZ,
        unsigned long sAcc,
        unsigned short pDOP,
        unsigned char numSV) {
         switch (gpsFix) {
            case 0x00:
              gpsFT = "No Fix";
            break;
            case 0x01:
              gpsFT = "Dead Reckoning";
            break;
            case 0x02:
              gpsFT = "2D-fix";
            break;
            case 0x03:
              gpsFT = "3D-fix";
            break;
            case 0x04:
              gpsFT = "GPS & DR";
            break;
            case 0x05:
              gpsFT = "Time only";
            break;      
            default:
              gpsFT = "Unknown"; 
         }
         gpsSats = numSV;
      }
};

//--- SETUP ESP-32 DEVICE ---
//setup GPS hardware serial port
HardwareSerial SerialGPS(2);

//setup UBX_Parser
UBXParser parser;

//setup WiFiUDP config
WiFiUDP udp;
static const int NTP_PACKET_SIZE = 48;
static const int NTP_PORT = 123;
// buffers for receiving and sending data
byte packetBuffer[NTP_PACKET_SIZE];

//setup wifi client config
WiFiClient client;
int WiFiTimeout = 1000; //client response timeout in millis

void setup() {
  //serial start
  Serial.begin(115200);
  
  //wifi start
  WiFi.begin(ssid,password);
  udp.begin(NTP_PORT);
  
  WiFi.setHostname("S1_NTP");
  IPAddress port(80);
  
  //identify deviceID (mac address) for the device
  WiFi.macAddress(mac);
  for(i=0;i<5;i++){
    deviceID += mac[i];
  }
  deviceID += mac[5];
  i = 0;
  Serial.print("deviceID: ");
  Serial.println(deviceID);
  Serial.println();
  Serial.println("-------------------------------------");
  Serial.print("| ");
  Serial.print(appName);
  Serial.print(" | v");
  Serial.print(softVer);
  Serial.println(" |");
  Serial.println("-------------------------------------");
  
  // setting up ublox GPS M8N to switch to from NMEA to UBX and run on 115200baud 
  Serial.print("Setting up GPS for UBX...");
  SerialGPS.begin(9600); //GPIO16 = RX and GPIO17 = TX
  SerialGPS.write(POSLLH_,HEX);
  delay(200);
  SerialGPS.write(SOL_,HEX);
  delay(200);
  SerialGPS.write(TIMEUTC_,HEX);
  delay(200);
  SerialGPS.write(SAVE_CONFIG_,HEX);
  Serial.println(" OK.");

  //connecting to WiFi
  Serial.print("Connecting to WiFi.");
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  } 
  if (WiFi.status() == WL_CONNECTED){
    Serial.print(" OK.");
  }
  Serial.println();
  Serial.print("Connected to: ");
  Serial.print(ssid);
  Serial.print(" | IP Address: ");
  Serial.print(WiFi.localIP());

  //configure blue led
  pinMode(2,OUTPUT);
  
  //Configuring GPS
  smartDelay(1000); // wait 1000ms to gather the GPS string data
  Serial.print(" | Configure GPS >> Satellites: ");
  Serial.print(gpsSats);
  Serial.print(" | Fix Type: ");
  Serial.print(gpsFT);
  Serial.print(" | UTC Time: ");
  
  // format date
  if (gpsDay < 10) Serial.print("0");
  Serial.print(gpsDay);
  Serial.print("/");
  if (gpsMonth < 10) Serial.print("0");
  Serial.print(gpsMonth);
  Serial.print("/");
  Serial.print(gpsYear);
  Serial.print(" ");
  if (gpsHour < 10) Serial.print("0");
  Serial.print(gpsHour);
  Serial.print(":");
  if (gpsMin < 10) Serial.print("0");
  Serial.print(gpsMin);
  Serial.print(":");
  if (gpsSec < 10) Serial.print("0");
  Serial.println(gpsSec);
  Serial.println("-------------------------------------");
}

void loop() {

  //get gps data
  smartDelay(1000);

  Serial.print("GPS Status: ");
  Serial.print(gpsFT);
  Serial.print(" | Uptime(sec): ");
  Serial.println(String(millis()/1000));

  if(gpsSats >= 1){
    if(gpsYear < 2000){
      Serial.println("Invalid date.");
      digitalWrite(2,HIGH);
      delay(500);
      digitalWrite(2,LOW);
    }

    //convert utc to unix 
    struct tm t;
    time_t t_of_day;
    t.tm_year = gpsYear - 1900;  
    t.tm_mon = gpsMonth - 1;           
    t.tm_mday = gpsDay;          
    t.tm_hour = gpsHour;
    t.tm_min = gpsMin;
    t.tm_sec = gpsSec;
    t.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    t_of_day = mktime(&t);

    // if there's data available, read a packet
    int packetSize = udp.parsePacket();
    if(packetSize)
    {
      //convert unix to epoch
      int timestamp = t_of_day + seventyYears;
      
      udp.read(packetBuffer,NTP_PACKET_SIZE);
      IPAddress Remote = udp.remoteIP();
      int PortNum = udp.remotePort();
      
      Serial.print("NTP request recieved from ");
      Serial.print(Remote);
      Serial.print(":");
      Serial.print(PortNum);
      Serial.print(" | UTC: ");
      if (gpsDay < 10) Serial.print("0");
      Serial.print(gpsDay);
      Serial.print("/");
      if (gpsMonth < 10) Serial.print("0");
      Serial.print(gpsMonth);
      Serial.print("/");
      Serial.print(gpsYear);
      Serial.print(" ");
      if (gpsHour < 10) Serial.print("0");
      Serial.print(gpsHour);
      Serial.print(":");
      if (gpsMin < 10) Serial.print("0");
      Serial.print(gpsMin);
      Serial.print(":");
      if (gpsSec < 10) Serial.print("0");
      Serial.print(gpsSec);
      Serial.print(" | Unix: ");
      Serial.print(t_of_day);
      Serial.print(" | Satellites: ");
      Serial.println(gpsSats);
        
      packetBuffer[0] = 0b00100100;   // LI = 0 , Version = 4, Mode = 4
      packetBuffer[1] = 0x01 ;   // stratum (GPS)
      packetBuffer[2] = 0x06 ;   // polling minimum (64 seconds - default)
      packetBuffer[3] = 0xF7; // precision (~2 milliseconds)
  
      packetBuffer[7] = 0x00;    // root delay
      packetBuffer[8] = 0x00;
      packetBuffer[9] = 0x08;
      packetBuffer[10] = 0x00;
  
      packetBuffer[11] = 0x00;   // root dispersion
      packetBuffer[12] = 0x00;
      packetBuffer[13] = 0xC0;
      packetBuffer[14] = 0x00;
      
      // Reference identifier (for Stratum 1 type)
      packetBuffer[12] = 0x47; //"G";
      packetBuffer[13] = 0x50; //"P";
      packetBuffer[14] = 0x53; //"S";
      packetBuffer[15] = 0x00;  //"0";
  
      // Reference timestamp
      tempval = timestamp;
      packetBuffer[16] = (tempval >> 24) & 0XFF;
      tempval = timestamp;
      packetBuffer[17] = (tempval >> 16) & 0xFF;
      tempval = timestamp;
      packetBuffer[18] = (tempval >> 8) & 0xFF;
      tempval = timestamp;
      packetBuffer[19] = (tempval) & 0xFF;
      packetBuffer[20] = 0x00;
      packetBuffer[21] = 0x00;
      packetBuffer[22] = 0x00;
      packetBuffer[23] = 0x00;

      // Originate timestamp from incoming UDP transmit timestamp
      packetBuffer[24] = packetBuffer[40];
      packetBuffer[25] = packetBuffer[41];
      packetBuffer[26] = packetBuffer[42];
      packetBuffer[27] = packetBuffer[43];
      packetBuffer[28] = packetBuffer[44];
      packetBuffer[29] = packetBuffer[45];
      packetBuffer[30] = packetBuffer[46];
      packetBuffer[31] = packetBuffer[47];
  
      // Receive timestamp
      tempval = timestamp;    // Same as reference timestamp
      packetBuffer[32] = (tempval >> 24) & 0XFF;
      tempval = timestamp;
      packetBuffer[33] = (tempval >> 16) & 0xFF;
      tempval = timestamp;
      packetBuffer[34] = (tempval >> 8) & 0xFF;
      tempval = timestamp;
      packetBuffer[35] = (tempval) & 0xFF;
      packetBuffer[36] = 0x00;
      packetBuffer[37] = 0x00;
      packetBuffer[38] = 0x00;
      packetBuffer[39] = 0x00;
  
      // Transmit timestamp
      packetBuffer[40] = (tempval >> 24) & 0XFF;
      tempval = timestamp;
      packetBuffer[41] = (tempval >> 16) & 0xFF;
      tempval = timestamp;
      packetBuffer[42] = (tempval >> 8) & 0xFF;
      tempval = timestamp;
      packetBuffer[43] = (tempval) & 0xFF;
      packetBuffer[44] = 0x00;
      packetBuffer[45] = 0x00;
      packetBuffer[46] = 0x00;
      packetBuffer[47] = 0x00;

     // Reply to the IP address and port that sent the NTP request
      udp.beginPacket(Remote, PortNum);
      udp.write(packetBuffer,NTP_PACKET_SIZE);
      udp.endPacket();

      digitalWrite(2,HIGH);
      delay(400);
      digitalWrite(2,LOW);
      delay(200);
      digitalWrite(2,HIGH);
      delay(400);
      digitalWrite(2,LOW);
    }
  
  } else {
      Serial.print("Uptime: ");
      Serial.print(String(millis()));
      Serial.println("ms. No satellites visible to aquire accurate time.");
  }
}

static void smartDelay(unsigned long ms){ 
  unsigned long start = millis();
  do{
    while (SerialGPS.available())
      parser.parse(SerialGPS.read());
  } while (millis() - start < ms);
}
