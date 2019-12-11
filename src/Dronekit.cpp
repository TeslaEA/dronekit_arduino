/*
  Created by `Volkov E.A., December,5, 2019
  Released into the public domain.
*/
#include "Dronekit.h"


Dronekit::Dronekit()
{
	
}

uint8_t Dronekit::connect(int h_port, int c_port)
{
	_buf_len = 500;
	_hport = h_port;
	_cport = c_port;
	
	if(udp.listen( _hport)) {
        	Serial.println("UDP connected");
        	udp.onPacket([this](AsyncUDPPacket packet) 
			{
			this->receive_data(&packet);	
			//Serial.print("recv:");
			//Serial.write(packet.data(), packet.length());
            //Serial.println();
			});
	}
	return 1;
}

void Dronekit::close()
{
	Dronekit::udp.close();
}

size_t Dronekit::send_data( uint8_t* buf, uint16_t len)
{	
	size_t result = this->udp.writeTo(buf,len,WiFi.gatewayIP(),_cport);
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
  this->stream_rate[this->max_stream] = {0x05};

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

    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 1, this->stream_msg[i], this->stream_rate[i], 1);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

	this->send_data(buf, len);
	
  }
}


mavlink_message_t Dronekit::receive_data(AsyncUDPPacket *packet)
{
	//packet.data(), 
	//uint8_t packetBuffer[_buf_len] ;
	//memset(packetBuffer, 0, _buf_len);
	if (packet->length()) 
	{
		mavlink_message_t msg;
		mavlink_status_t stat;
		for (int i = 0; i < packet->length(); ++i)
		{
			if (mavlink_parse_char(MAVLINK_COMM_0, packet->data()[i], &msg, &stat)) 
			{
				switch (msg.msgid)
				{
				case MAVLINK_MSG_ID_HEARTBEAT:
		
					mavlink_heartbeat_t hb;
					mavlink_msg_heartbeat_decode(&msg, &hb);
					if (hb.base_mode == 209) {this->armed = true;} else {this->armed = false;}
					this->mode = hb.custom_mode;
		
					break;
				case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
        
					mavlink_sys_status_t sys_status;
					mavlink_msg_sys_status_decode(&msg, &sys_status);
					this->bat_volt = sys_status.voltage_battery;
					this->bat_amp = sys_status.current_battery;
        
        				break;
				case MAVLINK_MSG_ID_ATTITUDE:  // #30
            
              				mavlink_attitude_t attitude;
              				mavlink_msg_attitude_decode(&msg, &attitude);
              				this->roll = attitude.roll;
			  		this->pitch = attitude.pitch;
			  		this->yaw = attitude.yaw;
            
            				break;
				}
			}
		}
    	}
}

