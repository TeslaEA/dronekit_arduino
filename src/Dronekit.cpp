/*
  Created by `Volkov E.A., December,5, 2019
  Released into the public domain.
*/
#include "Dronekit.h"
#include <WiFiUdp.h>

Dronekit::Dronekit()
{
	WiFiUDP Udp;
}

uint8_t Dronekit::connect(int port)
{
	_port = port;
	Udp.begin(_port);
}

void Dronekit::connect()
{
	Udp.close();
}