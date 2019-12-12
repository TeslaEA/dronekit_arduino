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
    Dronekit(); //конструктор
	AsyncUDP udp; //класс асинхронного udp соединения

    uint8_t connect(uint16_t,uint16_t); //функция подключения к коптеру
    void close(); //функция отключения от коптера
	void request_data(); //запрос данных от коптера

	void arm(); //запуск моторов
	void disarm(); //остановка моторов
	bool setMode(); //установить режим
	bool takeoff(); //автоматический взлет
	bool land(); //автоматическая посадка
	bool setVelocity(); //установить текущую скорость
	
	
	size_t send_data( uint8_t* buf , uint16_t len); //отправка данных по udp
	mavlink_message_t receive_data(AsyncUDPPacket* packet); //парсинг данных
	void heartbeat();
	
	bool armed; //моторы запущены?
	int mode; //текущий режим?
	
	uint16_t bat_volt; //текущее напряжение акб?
	int16_t bat_amp; //текущее ток акб?
	
	float roll;  //текущий угол тангажа
	float pitch; //текущий угол крена
	float yaw; //текущий курс
	
	uint16_t ack_com;
	uint16_t ack_com_result;
	
	int max_stream = 1; //максимальное количество запрашиваемых потоков
	int stream_rate[]; //скорость обновления запрашиваемых потоков
	int stream_msg[]; //запрашиваемые потоки
	
  private:
	uint16_t _hport; //хост порт
	uint16_t _cport; //клиент порт
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
