/*
  Created by `Volkov E.A., December,5, 2019
  Released into the public domain.
*/
#ifndef Dronekit_h
#define Dronekit_h
 
#include "Arduino.h"
#include <ESP8266WiFi.h>
#include "ESPAsyncUDP.h"
#include "../src/mavlink/common/mavlink.h"

class Dronekit
{
  public:
    Dronekit();
	//WiFiUDP udp;
	AsyncUDP udp;

    	uint8_t connect(int,int);
    	void close();
	void request_data();
	void update();
	void arm();
	void disarm();
	bool setMode();
	bool takeoff();
	bool land();
	bool setVelocity();
	
	
	size_t send_data( uint8_t* buf , uint16_t len);
	mavlink_message_t receive_data(AsyncUDPPacket* packet);
	int _hport;
	int _cport;

	bool armed;
	int mode;
	
	uint16_t bat_volt;
	int16_t bat_amp;
	
	float roll;
	float pitch;
	float yaw;
	
	
	int max_stream = 1;
	int stream_rate[];
	int stream_msg[];
	
	
	
  private:
	uint8_t _buf_len = 1000;
	
	
};
 
 
 class Attitude
 {
	public:
		Attitude();
		uint8_t roll;
		uint8_t pitch;
		uint8_t yaw;
		
 };
 
 class Altitude
 {
	public:
		uint8_t baroAlt;
		uint8_t rangeAlt;
 } ;
#endif
