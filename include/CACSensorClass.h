#ifndef CACSensorClass_h
#define CACSensorClass_h

#include "Arduino.h"
#include "PubSubClient.h"

class Sensor
{
  private: 
    int _pinNumber;
    bool _lastValue;
    bool _curValue;
    char _id[20];
    char _name[50];
    PubSubClient client;
  public:
    Sensor(int pin, const char *sensorId, const char *sensorName, PubSubClient *client);
    bool updateSensor(bool force);
    bool registerSensor();
};

#endif
