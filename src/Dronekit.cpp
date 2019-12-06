/*
  Created by `Volkov E.A., December,5, 2019
  Released into the public domain.
*/
#include "Dronekit.h"


Dronekit::Dronekit()
{
	
}

uint8_t Dronekit::connect(int port)
{
	_buf_len = 500;
	_port = port;
	
	uint8_t c = this->udp.begin(_port);
	return c;
}

void Dronekit::close()
{
	Dronekit::udp.stop();
}

int Dronekit::send_data( uint8_t* buf, uint16_t len)
{
	int result = this->udp.beginPacket(this->udp.remoteIP(), this->udp.remotePort());
	this->udp.write(buf, len);
	this->udp.endPacket();
	return result; 
}

void Dronekit::request_data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STREAMS that can be requested
  /*
     Definitions are in common.h: enum MAV_DATA_STREAM

     MAV_DATA_STREAM_ALL=0, // Enable all data streams
     MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
     MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
     MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
     MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
     MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
     MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
     MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
     MAV_DATA_STREAM_ENUM_END=13,

     Data in PixHawk available in:
      - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
      - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
  */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int  maxStreams = 1;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_ALL};
  const uint16_t MAVRates[maxStreams] = {0x02};

  for (int i = 0; i < maxStreams; i++) {
    /*
       mavlink_msg_request_data_stream_pack(system_id, component_id,
          &msg,
          target_system, target_component,
          MAV_DATA_STREAM_POSITION, 10000000, 1);

       mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id,
          mavlink_message_t* msg,
          uint8_t target_system, uint8_t target_component, uint8_t req_stream_id,
          uint16_t req_message_rate, uint8_t start_stop)

    */

    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	this->send_data(buf, len);
	
  }
}


mavlink_message_t Dronekit::receive_data()
{
	
	uint8_t packetBuffer[_buf_len] ;
	memset(packetBuffer, 0, _buf_len);
	int packetSize = this->udp.parsePacket();
	int p = this->udp.read(packetBuffer, _buf_len);
	if (packetSize) 
	{
		mavlink_message_t msg;
		mavlink_status_t stat;
		for (int i = 0; i < packetSize; ++i)
		{
			if (mavlink_parse_char(MAVLINK_COMM_0, packetBuffer[i], &msg, &stat)) 
			{
				return msg;
			}
		}
    }
}

