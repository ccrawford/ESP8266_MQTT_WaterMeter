// ESP8266 Water Meter monitor. Dec 2021. Target is a D1 Mini Pro.

// Tricky part here is setting the threasholds for the trigger. I'm sure this varies for every meter, and even varies
// with how tightly you clamp the sensor to the meter. Ideally, I'd figure this out with some adaptive intelligence, but 
// for now depends on int16_t neutral=9000, peakHigh=9040; 
// Telnet (23) to the device and watch the reads come across the wire in a sliding display.

#include "Secrets.h"  // Local login info
#include <ESP8266WiFi.h>
#include <PubSubClient.h>  

#include <ESPDateTime.h> //https://github.com/mcxiaoke/ESPDateTime 

#include <WiFiUdp.h>     
#include <ArduinoOTA.h>  //Use the ESP8266 one.
#include <TelnetStream.h>  // Used for OTA debugging.

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads; //High res i2c a2d converter.
                      // Connected to a 296-DRV5056A1ELPGMQ1CT-ND https://www.digikey.com/en/products/detail/texas-instruments/DRV5056A1ELPGMQ1/10434634

const char* ssid = SECRET_SSID;
const char* password =  SECRET_PASSWORD;

const char* mqttServer = SECRET_MQTT_SERVER;
const int mqttPort = SECRET_MQTT_PORT;
const char* mqttUser = SECRET_MQTT_USER;
const char* mqttPassword = SECRET_MQTT_PASSWORD;

unsigned int hourlyTotal = 0;
unsigned int dailyTotal = 0;
unsigned int yesterdayTotal = 0;
unsigned int pulsesToday = 0;

static char realtime_flow_id[] = "water_meter_flow";
static char today_total_id[] = "water_meter_total";
static char last_hour_total_id[] = "water_meter_last_hour";
static char yesterday_total_id[] = "water_meter_yesterday";

#define MS_IN_HOUR 3600000 
// #define MS_IN_HOUR 60000  //Minute for testing...

#define MS_IN_DAY 86400000
// #define MS_IN_DAY 60*2*1000 // Two minutes for testing

#define RA_COUNT 1  // Number of reads to average

int16_t neutral=9000, peakHigh=9040; // TODO set these in software.

#define SCALE_FACTOR 0


double avgTot;
int16_t rc=0;
int16_t ra[RA_COUNT];


#define PULSE_PER_GALLON 14

WiFiClient espClient;
PubSubClient mqttClient(espClient);


void setupTime()
{
  DateTime.setServer("time.google.com");
  DateTime.setTimeZone("CST6CDT,M3.2.0,M11.1.0");
  DateTime.begin();
  if (!DateTime.isTimeValid()) {
    Serial.println("Failed to get time from server.");
  }
  else {
    Serial.printf("Date Now is %s\n", DateTime.toISOString().c_str());
    DateTimeParts p = DateTime.getParts();  
    Serial.printf("%02d\n",p.getMonthDay());
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
  Serial.println();
  Serial.println("-----------------------");
 
}

void setupOtaUpdate()
{
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("WaterMQTTEsp");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() { Serial.println("\nEnd OTA");});

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void reconnectMqtt()
{

  if(WiFi.status() != WL_CONNECTED)
  {
    WiFi.begin(ssid, password);
    Serial.println("Connecting to da WiFi..");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(". ");
    }
    
    Serial.println("Reconnected to WiFi ");
  }

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
 
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect("ESP8266WaterClient", mqttUser, mqttPassword )) {
      Serial.println("Mqtt connected"); 
    } 
    else {
      Serial.print("failed with state ");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
}

bool registerSensor()
{        
    char path[255];
    char message[1024];
    unsigned int msgLen;
    
//    WiFi.macAddress(mac);
//    sprintf(uniqId, "%02x%02x%02x%02x%02x%02x%02d",mac[5],mac[4],mac[3],mac[2],mac[1],mac[0],0);       
    
    sprintf(path, "homeassistant/sensor/%s/config", realtime_flow_id);    
    msgLen = sprintf(message, "{\"name\": \"506 Water Current Flow\", \"uniq_id\":\"%s\", \"state_topic\":\"homeassistant/sensor/%s\", \"unit_of_meas\":\"gpm\"}",realtime_flow_id,realtime_flow_id);
    mqttClient.publish(path, (const unsigned char *)message, msgLen, true);
    
    sprintf(path, "homeassistant/sensor/%s/config", today_total_id);    
    msgLen = sprintf(message, "{\"name\": \"506 Water Total Today\", \"uniq_id\":\"%s\", \"state_topic\":\"homeassistant/sensor/%s\", \"unit_of_meas\":\"Gal\"}",today_total_id, today_total_id);
    mqttClient.publish(path, (const unsigned char *)message, msgLen, true);

    sprintf(path, "homeassistant/sensor/%s/config", last_hour_total_id);    
    msgLen = sprintf(message, "{\"name\": \"506 Water Last Hour\", \"uniq_id\":\"%s\", \"state_topic\":\"homeassistant/sensor/%s\"}",last_hour_total_id,last_hour_total_id);
    mqttClient.publish(path, (const unsigned char *)message, msgLen, true);

    sprintf(path, "homeassistant/sensor/%s/config", yesterday_total_id);    
    msgLen = sprintf(message, "{\"name\": \"506 Water Total Yesterday\", \"uniq_id\":\"%s\", \"state_topic\":\"homeassistant/sensor/%s\"}",yesterday_total_id, yesterday_total_id);
    mqttClient.publish(path, (const unsigned char *)message, msgLen, true);
    
    Serial.println("Sensor Registered.");
    return true;
}


bool sendInstant()
{
  char path[255];
  char msg[16];

  //Send current flow rate in GPM
  sprintf(path, "homeassistant/sensor/%s", realtime_flow_id);
  mqttClient.publish(path, (const unsigned char*)msg, strlen(msg), true);

  //Send current daily total gallons
  sprintf(path, "homeassistant/sensor/%s", today_total_id);
  sprintf(msg, "%d", dailyTotal);
  return mqttClient.publish(path, (const unsigned char*)msg, strlen(msg), true);

}

bool sendHour()
{
  char path[255], msg[16];
  
  sprintf(path, "homeassistant/sensor/%s", last_hour_total_id);
  sprintf(msg, "%d", hourlyTotal);
  return mqttClient.publish(path, (const unsigned char*)msg, strlen(msg), true);
}

bool sendYesterday()
{
  // When the clock ticks over at midnight, send the current daily running total as "yesterday"
  
  char path[255];
  char msg[16];
  
  // Send today and day prior as yesterday. Total reset in other function.
  sprintf(path, "homeassistant/sensor/%s", yesterday_total_id);
  sprintf(msg, "%d", dailyTotal);
  return mqttClient.publish(path, (const unsigned char*)msg, strlen(msg), true);
}



void setup() {

  Serial.begin(115200);
  Serial.println("EspMQTTHomeAssistantWM.ino Dec 2021. CAC");
  
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(ssid, password);
 
 
  Serial.println("Connecting to da WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(". ");
  }
  Serial.println("Connected to WiFi ");

  setupOtaUpdate();

  TelnetStream.begin();


  // Risky code avoidance...give the thing a chance to check for updates if you broke it elsewhere. 10 seconds.
  for(int i=0; i< 100; i++) 
  {
    ArduinoOTA.handle();
    yield();
    delay(100);
  }

  setupTime();
  reconnectMqtt();
  registerSensor();


  Serial.println("Setting up ADC");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
   ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  
  // ads.setMode(0);
  ads.setDataRate(RATE_ADS1115_16SPS);  // Slow data rate will average.
  if(!ads.begin())
  {
    Serial.println("Could not setup ADC. Check wiring.");
  }
  
  for(int i=0;i<RA_COUNT;i++) { //Prime the average...
    ra[i]=ads.readADC_SingleEnded(0)>>SCALE_FACTOR;
    delay(10);
  }

  Serial.println("Done with Setup");

}

unsigned long lastPulse = millis();  // Record the last pulse so we know if water is running.
unsigned long lastDailyUpdate = millis();
unsigned long lastHourlyUpdate = millis();
unsigned long lastGallon = 0; // Record the last gallon to get a current GPH
unsigned long lastTimeSync = millis(); // The clock drifts over time. Update it from NTP every once in a while.
                                       // BUT DON'T do it on day cross-over to prevent re-running day cross-over.

void loop() {

// Read the sensors and count the pulses.
// Send the current water flow (gallon per minute?)
// and the total for the last hour every hour on the hour.

  DateTimeParts dateParts = DateTime.getParts();
  static int curDayOfMonth = dateParts.getMonthDay();
  static bool cycleSet = false;
  static double cycleCount = 0;

  int32_t adc0, runningAverage, avgTot; 
  static int32_t raLow=32768, raHigh=0;

  adc0 = ads.readADC_SingleEnded(0)>>SCALE_FACTOR;
  // adc0 = ads.readADC_Differential_0_1();
  unsigned long thisMs = millis();

  if (RA_COUNT == 1) runningAverage = adc0;
  else {
    ra[rc++]=adc0;
    if (rc>=RA_COUNT) rc=0;
    avgTot=0;
    for(int i=0;i<RA_COUNT;i++) avgTot+=ra[i];
    runningAverage=avgTot/RA_COUNT;
  }
  raHigh = max(raHigh, runningAverage);
  raLow = min(raLow, runningAverage);
  
  if(runningAverage>peakHigh && !cycleSet) {
    pulsesToday++;
    cycleCount++;
    cycleSet = true;
//    TelnetStream.println("increment...");
//    pulsePerMinute = (60 * 1000)/ (thisMs - lastPulse);
    lastPulse = thisMs;
    
  }
  if(runningAverage<=neutral) {
    cycleSet = false;
  }

  if(cycleCount >= PULSE_PER_GALLON) {
    cycleCount = 0;
    hourlyTotal++;
    dailyTotal++;
    sendInstant();
  }

  if(thisMs > lastHourlyUpdate + MS_IN_HOUR)
  {
    sendHour();
    hourlyTotal = 0;
    lastHourlyUpdate = thisMs;
  }

  if(thisMs > lastTimeSync + MS_IN_DAY)
  {
    // Update the ntp Clock.
    // Maybe use datetime's force update instead?
    DateTime.forceUpdate();
    // setupTime();
    lastTimeSync = thisMs;
  }

  if(curDayOfMonth != dateParts.getMonthDay())
  {
    sendYesterday(); 
    dailyTotal = 0;
    pulsesToday = 0;
    lastDailyUpdate = thisMs;
    curDayOfMonth = dateParts.getMonthDay();
  }

// Display
  int32_t ra_range, l_pad;
  ra_range = raHigh - raLow;
  ra_range = max(1, ra_range); // make sure the range is at least 1.
  
  l_pad = (float)(runningAverage - raLow) * (float)(40 / (float)ra_range);
  l_pad = max(1,l_pad);

  char buf[200];
  
  char paddingL[40];
  char paddingR[40];

  memset(paddingL, 0, 40);
  memset(paddingR, 0, 40);

  memset(paddingL, ' ', l_pad);
  memset(paddingR, ' ', 40 - l_pad);
  
  // Use ANSI escape sequences to make the moving graph in the telnet terminal. https://en.wikipedia.org/wiki/ANSI_escape_code 
  // sprintf(buf, "\033[2J%d%s%s%d\033[0m%s%d   \033[1;34m%d %d %d\033[0m",raLow,paddingL,(cycleSet)?"\033[1;32m":"\033[1;31m",runningAverage,paddingR, raHigh, dailyTotal, curDayOfMonth, pulsesToday);
          //    esc, set position, show low, pad spaces, set color, show avg, pad spaces, show remaining stats, reset color and attributes.
  TelnetStream.printf("\033[2JTime: %s\r\n",dateParts.format("%x %X").c_str());
  sprintf(buf, "%d%s%s%d\033[0m%s%d   \033[1;34m%d %d %d\033[0m",raLow,paddingL,(cycleSet)?"\033[1;32m":"\033[1;31m",runningAverage,paddingR, raHigh, dailyTotal, curDayOfMonth, pulsesToday);
  TelnetStream.println(buf);
  // TelnetStream.printf("Next Time sync in: %ld",MS_IN_DAY - (thisMs - lastTimeSync));

  if (! mqttClient.connected()) { 
    //Serial.println("Not connected...");
    reconnectMqtt();
  }

  // If you telnet to the device, you can watch the real-time readings from the sensor
  // And you can reset stats or reboot the device.

  if(TelnetStream.available()) {
    switch (TelnetStream.read()) {
      case 'R':
      TelnetStream.stop();
      delay(100);
      ESP.reset();
        break;
      case 'Z':
        raHigh = 0;
        raLow = 32768;
        break;
    }
  }
 
  mqttClient.loop();

  ArduinoOTA.handle();
}
