/*
  Created by `Volkov E.A., December,5, 2019
  Released into the public domain.
*/
#ifndef Dronekit_h
#define Dronekit_h
 
#include "Arduino.h"

class Dronekit
{
  public:
    Drinekit();
    uint8_t connect(int port);
    void close();
	int _port;
  private:
	uint8_t _packetBuffer[_buf_len] = {0};
	int _buf_len = 1000;
};
 
#endif
