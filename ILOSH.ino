/*
	Version: 1.00
	Purpose: This sample use Linkit ONE, GPS, BME280 and Gemtek LoRa module, 
             to build wearable sensors for Institute of Labor, Occupational Safety And Health (ILOSH)
             project - 
             Heatwave exposure sensing and pre-alarm system for outdoor high temperature environment workers
             Using sensors:
             	-- GPS: 		for position
             	-- BME280: 		for temperature, humidity and ambient pressure
             	-- LoRa module: GIoT GL6509 (Gemtek LoRa)
             First, make sure all the sensors and lora module are working properly 
             in this project - BME280, GIoT LoRa module and GPS.
             Due to the bandwidth limitation when using Gemtek module, in Taipei, Taiwan,
             we need some bitwise operation, shifting everything into 11 bytes, the function "LoRaBitMap"
             below will help you in doing this. 
             For more information about, please refer: http://www.slideshare.net/HuChengLee/taipei-iot-lora-workshop
    
    
	History:
	1.00  by Hu-Cheng Lee (Jack, jack77121@gmail.com)  03/06/2016 (DD/MM/YYYY) 
*/

#include "Adafruit_SHT31.h"
#include <BME280_MOD-1022.h>
#include <Wire.h>
#include <LGPS.h>
#include <LBattery.h>
#define BATTERY_FULL 100
#define BATTERY_HALF 66
#define BATTERY_LOW 33

short fix_num = 15;			// 15 for fake GPS, who don't have GPS module

byte app_id = 1;
byte lora_trans[11];

float temperature = 0;
float humidity = 0;
float embient_pressure = 0;

gpsSentenceInfoStruct info;
char buff[256];
unsigned long currentTime = 0;
unsigned long pendingTime = 0;
unsigned long lastDataTrans = 0;
/*  
*	GPS example for IIS NRL, Academia Sinica in Taipei, Taiwan.
* 	Using DMS format: 
*	D-25 M-02 S-28 for latitude
*	D-121 M-36 S-52 for latitude
* 	Replace this position to your location whatever you are :)
*/


char gps_lat[20]; // device's gps latitude,  
char gps_lon[20]; // device's gps longitude
double latitude;
double longitude;
short tmp, hour, minute, second, num;



// copy from cmaglie on:
//https://github.com/arduino/Arduino/blob/a2e7413d229812ff123cb8864747558b270498f1/hardware/arduino/sam/cores/arduino/avr/dtostrf.c
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
	char fmt[20];
	sprintf(fmt, "%%%d.%df", width, prec);
	sprintf(sout, fmt, val);
	return sout;
}


static unsigned char getComma(unsigned char num,const char *str)
{
	unsigned char i,j = 0;
	int len=strlen(str);
	for(i = 0;i < len;i ++){
		if(str[i] == ',')
			j++;
		if(j == num)
			return i + 1; 
	}
	return 0; 
}

static double getDoubleNumber(const char *s)
{
	char buf[10];
	unsigned char i;
	double rev;
	
	i=getComma(1, s);
	i = i - 1;
	strncpy(buf, s, i);
	buf[i] = 0;
	rev=atof(buf);
	return rev; 
}

static double getIntNumber(const char *s)
{
	char buf[10];
	unsigned char i;
	double rev;
	
	i=getComma(1, s);
	i = i - 1;
	strncpy(buf, s, i);
	buf[i] = 0;
	rev=atoi(buf);
	return rev; 
}

void parseGPGGA(const char* GPGGAstr)
{
	/* Refer to http://www.gpsinformation.org/dale/nmea.htm#GGA
	* Sample data: $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
	* Where:
	*  GGA          Global Positioning System Fix Data
	*  123519       Fix taken at 12:35:19 UTC
	*  4807.038,N   Latitude 48 deg 07.038' N
	*  01131.000,E  Longitude 11 deg 31.000' E
	*  1            Fix quality: 0 = invalid
	*                            1 = GPS fix (SPS)
	*                            2 = DGPS fix
	*                            3 = PPS fix
	*                            4 = Real Time Kinematic
	*                            5 = Float RTK
	*                            6 = estimated (dead reckoning) (2.3 feature)
	*                            7 = Manual input mode
	*                            8 = Simulation mode
	*  08           Number of satellites being tracked
	*  0.9          Horizontal dilution of position
	*  545.4,M      Altitude, Meters, above mean sea level
	*  46.9,M       Height of geoid (mean sea level) above WGS84
	*                   ellipsoid
	*  (empty field) time in seconds since last DGPS update
	*  (empty field) DGPS station ID number
	*  *47          the checksum data, always begins with *
	*/
	Serial.println(GPGGAstr);
	if(GPGGAstr[0] == '$'){
		tmp = getComma(1, GPGGAstr);
		hour     = (GPGGAstr[tmp + 0] - '0') * 10 + (GPGGAstr[tmp + 1] - '0');
		minute   = (GPGGAstr[tmp + 2] - '0') * 10 + (GPGGAstr[tmp + 3] - '0');
		second    = (GPGGAstr[tmp + 4] - '0') * 10 + (GPGGAstr[tmp + 5] - '0');
		
		sprintf(buff, "UTC timer %2d-%2d-%2d", hour, minute, second);
		Serial.println(buff);
		
		tmp = getComma(2, GPGGAstr);
		latitude = getDoubleNumber(&GPGGAstr[tmp]);
		int lat_D = ((int)latitude)/100;
		int lat_M = ((int)latitude)%100;
		int lat_S = (latitude - ((int)latitude))*60;
		Serial.print("lat_D = ");
		Serial.println(lat_D,BIN);
		Serial.print("lat_M = ");
		Serial.println(lat_M,BIN);
		Serial.print("lat_S = ");
		Serial.println(lat_S,BIN);
		tmp = getComma(4, GPGGAstr);
		longitude = getDoubleNumber(&GPGGAstr[tmp]);
		int lon_D = ((int)longitude)/100;
		int lon_M = ((int)longitude)%100;
		int lon_S = (longitude - ((int)longitude))*60;
//		Serial.print("lon_D = ");
//		Serial.println(lon_D);
//		Serial.print("lon_M = ");
//		Serial.println(lon_M);
//		Serial.print("lon_S = ");
//		Serial.println(lon_S);
		
		sprintf(gps_lat, "%02d.%02d%02d", lat_D, lat_M, lat_S);
		sprintf(gps_lon, "%02d.%02d%02d", lon_D, lon_M, lon_S);
		
		Serial.println("DMS format");
		Serial.println(gps_lat);
		Serial.println(gps_lon);
		
		tmp = getComma(7, GPGGAstr);
		fix_num = getIntNumber(&GPGGAstr[tmp]);    
		Serial.print("satellites number = ");
		Serial.println(fix_num); 
	}
	else{
		Serial.println("Not get data"); 
	}
}

void MyBME(){
	// example of a forced sample.  After taking the measurement the chip goes back to sleep
	BME280.writeMode(smForced);
	while (BME280.isMeasuring()) {
		Serial.println("Measuring...");
		delay(50);
	}
	Serial.println("Done!");
	
	// read out the data - must do this before calling the getxxxxx routines
	BME280.readMeasurements();
	temperature = BME280.getTemperatureMostAccurate();
	humidity = BME280.getHumidityMostAccurate();
	embient_pressure = BME280.getPressureMostAccurate();

	Serial.print("TempMostAccurate=");
	Serial.println(temperature);  // use double calculations
	Serial.print("HumidityMostAccurate=");
	Serial.println(humidity); // use double calculations
	Serial.print("PressureMostAccurate=");
	Serial.println(embient_pressure); // use double calculations
}

int Battery(){
	int batteryStatus = 0;
	int batteryLevel = LBattery.level();
	if(batteryLevel == BATTERY_FULL){
		batteryStatus = 3;
	}
	else if(batteryLevel == BATTERY_HALF){
		batteryStatus = 2;
	}
	else if(batteryLevel == BATTERY_LOW){
		batteryStatus = 1;
	}
	else{	//should be empty or ..... error XD
		batteryStatus = 0;
	}
	return batteryStatus;	
}

void LoRaBitMap(){
	/* 
	 *  You can refer to: http://labs.mediatek.com/fileMedia/download/5fed7907-b2ba-4000-bcb2-016a332a49fd
	 *  to see the bits # of different data types (is not like Arduino)
	*/
	byte batteryLv = Battery();
	Serial.print("Battery level: ");
	Serial.println(batteryLv);
	short temperatureLora = (short)((temperature+20)*10);
	Serial.println("temperature: ");
	Serial.println(temperatureLora);
	short humiditylora = (short)(humidity*10);
	Serial.println("humidity: ");
	Serial.println(humiditylora);
	short embient_pressure_int_lora = (short)embient_pressure;
	byte embient_pressure_float_lora = (embient_pressure - embient_pressure_int_lora)*100;
	Serial.println("embient pressure: ");
	Serial.print(embient_pressure_int_lora);
	Serial.print(".");
	Serial.println(embient_pressure_float_lora);
	float gps_lat_f = (float)atof(gps_lat);
	float gps_lon_f = (float)atof(gps_lon);
	Serial.print("after change to float: ");
	Serial.println(gps_lat_f,4);
	Serial.println(gps_lon_f,4);
	
	gps_lat_f += 90;
	gps_lon_f += 180;
	int gps_lat_i = (int) (gps_lat_f*10000);
	int gps_lon_i = (int) (gps_lon_f*10000);
//	Serial.print("after amplify to integer: ");
//	Serial.println(gps_lat_i);
//	Serial.println(gps_lon_i);  
	byte lat_D = (short) gps_lat_f;
//	Serial.print("what D is now: ");
//	Serial.println(lat_D);
	float temp_lat_M = (gps_lat_f - lat_D)*100;
//	Serial.print("what M is now: ");
//	Serial.println(temp_lat_M);
	byte lat_M = (short) temp_lat_M;
//	Serial.print("what M finally be: ");
//	Serial.println(lat_M,BIN);
	byte lat_S = (int) gps_lat_i%100;
//	Serial.print("what S finally be: ");
//	Serial.println(lat_S);



	
	short lon_D = (short) gps_lon_f;
	Serial.print("what D is now: ");
	Serial.println(lon_D);
	float temp_lon_M = (gps_lon_f - lon_D)*100;
	Serial.print("what M is now: ");
	Serial.println(temp_lon_M);
	byte lon_M = (short) temp_lon_M;
	Serial.print("what M finally be: ");
	Serial.println(lon_M,BIN);
	byte lon_S = (int) gps_lon_i%100;
	Serial.print("what S finally be: ");
	Serial.println(lon_S);
	byte gps_fix = fix_num;
	char lora_buff[150];
	
//	lora_trans[0] = (app_id << 4) | (temperatureLora >> 6);
	lora_trans[0] = (app_id << 6) | (batteryLv << 4) | (temperatureLora >> 6); 
	lora_trans[1] = (temperatureLora << 2) | (humiditylora >> 8);
	lora_trans[2] = humiditylora;
	// END FOR THE APP_ID, TEMPERATURE AND HUMIDITY
	lora_trans[3] =  embient_pressure_int_lora >> 3;
	lora_trans[4] = (embient_pressure_int_lora << 5)|(embient_pressure_float_lora >> 3);
	// END FOR Ambient pressure int part  
	lora_trans[5] = (embient_pressure_float_lora <<5) | (lat_D >> 3);
	lora_trans[6] = (lat_D << 5) | (lat_M >> 1);
	lora_trans[7] = (lat_M << 7) | (lat_S << 1) | (lon_D >> 8);
	lora_trans[8] = (byte)lon_D;
	lora_trans[9] = (lon_M << 2) | (lon_S >> 4);
	lora_trans[10] = (lon_S << 4) | gps_fix;
	// END FOR PM10 AND GPS
	sprintf(lora_buff, "AT+DTX=22,%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n", \
	lora_trans[0], lora_trans[1], lora_trans[2], lora_trans[3], lora_trans[4], \
	lora_trans[5], lora_trans[6], lora_trans[7], lora_trans[8], lora_trans[9], lora_trans[10]);
	Serial1.print(lora_buff);
	Serial.println(lora_buff);
}


void setup() {
	// put your setup code here, to run once:
	pinMode(0, INPUT);
	pinMode(1, OUTPUT);
	Serial.begin(115200);
	Serial1.begin(9600); //LoRa
	Wire.begin();
	LGPS.powerOn();
	Serial.println("LGPS Power on, and waiting ..."); 
	delay(3000);
	
	


	// need to read the NVM compensation parameters
	BME280.readCompensationParams();
	
	// Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
	BME280.writeOversamplingPressure(os1x);  // 1x over sampling (ie, just one sample)
	BME280.writeOversamplingTemperature(os1x);
	BME280.writeOversamplingHumidity(os1x);
	
	
	Serial.println("start");

}

void loop() {
	// put your main code here, to run repeatedly:
	Serial.println("LGPS loop"); 
	LGPS.getData(&info);
	Serial.println((char*)info.GPGGA); 
	parseGPGGA((const char*)info.GPGGA);
	MyBME();
	currentTime = millis();
	Serial.print("finished time: ");
	Serial.println(currentTime);
	
	LoRaBitMap();
	lastDataTrans = millis();
	do{
		currentTime = millis();
		pendingTime = currentTime - lastDataTrans;
	}while(pendingTime < 60200);
	
}


