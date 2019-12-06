/*
  Created by `Volkov E.A., December,5, 2019
  Released into the public domain.
*/
#ifndef Dronekit_h
#define Dronekit_h
 
#include "Arduino.h"
#include <WiFiUdp.h>
#include "../src/mavlink/common/mavlink.h"

class Dronekit
{
  public:
    Dronekit();
	WiFiUDP udp;
	
    uint8_t connect(int);
    void close();
	void request_data();
	int send_data( uint8_t* buf , uint16_t len);
	mavlink_message_t receive_data();
	int _port;
	
  private:
	uint8_t _buf_len = 1000;
	
	
};
 
#endif
