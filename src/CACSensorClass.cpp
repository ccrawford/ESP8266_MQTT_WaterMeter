#include "CACSensorClass.h"
#include "Arduino.h"
#include "ESP8266WiFi.h"


Sensor::Sensor(int pin, const char *sensorId, const char *sensorName, PubSubClient *client)
{
  this->_pinNumber = pin;
  strcpy(this->_name, sensorName);
  strcpy(this->_id, sensorId);
  _lastValue = false;
  pinMode(_pinNumber, INPUT_PULLUP);
} 

bool Sensor::updateSensor(bool force)
{
  this->_curValue = digitalRead(_pinNumber);
  if (this->_curValue != this->_lastValue || force)
  {
    bool retval = false;
    char path[255];
    char msg[4];
    
    sprintf(path, "homeassistant/binary_sensor/%s/state", this->_id);
    strcpy(msg, this->_curValue ? "ON" : "OFF");
    
    retval = client.publish(path, (const unsigned char*)msg, strlen(msg), true);
    if (! retval) {
      setup();
      return false;
    }
    else {
      this->_lastValue = this->_curValue;
      return true;
    }
  }
  return false;
}

bool Sensor::registerSensor()
{        
    char path[255];
    char message[512];
    unsigned int msgLen;
    byte mac[6];
    char uniqId[15];

    WiFi.macAddress(mac);
    sprintf(uniqId, "%02x%02x%02x%02x%02x%02x%02d",mac[5],mac[4],mac[3],mac[2],mac[1],mac[0],this->_pinNumber);       
    sprintf(path, "homeassistant/binary_sensor/%s/config", this->_id);
    
    msgLen = sprintf(message, "{\"name\": \"%s\", \"uniq_id\":\"%s\", \"dev_cla\": \"door\", \"stat_t\":\"homeassistant/binary_sensor/%s/state\"}",this->_name,uniqId,this->_id);
    // Serial.println(message);
    client.publish(path, (const unsigned char *)message, msgLen, true);
    return true;
}
