/*
  Created by `Volkov E.A., December,5, 2019
  Released into the public domain.
*/
#include "Dronekit.h"


Dronekit::Dronekit()
{
	
}

uint8_t Dronekit::connect(int h_port,int c_port)
{
	_buf_len = 500;
	//_port = port;
	
	//uint8_t c = this->udp.begin(_port);

	if(udp.connect(WiFi.gatewayIP(), h_port)) {
        	Serial.println("UDP connected");
        	Serial.println(udp.listen(WiFi.gatewayIP(), c_port));
        	udp.onPacket([](AsyncUDPPacket packet) {
            		Serial.print("UDP Packet Type: ");
            		Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            		Serial.print(", From: ");
            		Serial.print(packet.remoteIP());
            		Serial.print(":");
            		Serial.print(packet.remotePort());
            		Serial.print(", To: ");
            		Serial.print(packet.localIP());
            		Serial.print(":");
            		Serial.print(packet.localPort());
            		Serial.print(", Length: ");
            		Serial.print(packet.length());
            		Serial.print(", Data: ");
            		Serial.write(packet.data(), packet.length());
            		Serial.println();
            //reply to the client
            //packet.printf("Got %u bytes of data", packet.length());
        });
	}
	return 1;
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
  this->stream_msg[this->max_stream] = {MAV_DATA_STREAM_ALL};
  this->stream_rate[this->max_stream] = {0x02};

  for (int i = 0; i < this->max_stream; i++) {
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

    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, this->stream_msg[i], this->stream_rate[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	
	this->send_data(buf, len);
	
  }
}


/*mavlink_message_t Dronekit::receive_data()
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
}*/

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

void Dronekit::update()
{
	this->request_data();
	mavlink_message_t recv_m;
	recv_m = this->receive_data();
	switch (recv_m.msgid)
	{
		case MAVLINK_MSG_ID_HEARTBEAT:
		
			mavlink_heartbeat_t hb;
			mavlink_msg_heartbeat_decode(&recv_m, &hb);
			if (hb.base_mode == 209) {this->armed = true;} else {this->armed = false;}
			this->mode = hb.custom_mode;
		
		break;
		case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
        
			mavlink_sys_status_t sys_status;
			mavlink_msg_sys_status_decode(&recv_m, &sys_status);
			this->bat_volt = sys_status.voltage_battery;
			this->bat_amp = sys_status.current_battery;
        
        break;
		case MAVLINK_MSG_ID_ATTITUDE:  // #30
            
              /* Message decoding: PRIMITIVE
                    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
              */
              mavlink_attitude_t attitude;
              mavlink_msg_attitude_decode(&recv_m, &attitude);
              this->roll = attitude.roll;
			  this->pitch = attitude.pitch;
			  this->yaw = attitude.yaw;
            
            break;
	}
	
}
