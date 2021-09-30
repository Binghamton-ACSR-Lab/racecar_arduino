#include <Arduino.h>
#include <Wire.h>
//#include "AS5600.h"
//#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux


#include <Arduino_LSM9DS1.h>

#include <ros.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/String.h>
#include "acsr_hardware.h"
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/MagneticField.h>



#define LEFT_FRONT_PIN_1 D2
#define LEFT_FRONT_PIN_2 D3
#define RIGHT_FRONT_PIN_1 D6
#define RIGHT_FRONT_PIN_2 D7

#define LEFT_REAR_PIN_1 D4
#define LEFT_REAR_PIN_2 D5
#define RIGHT_REAR_PIN_1 A6
#define RIGHT_REAR_PIN_2 A7



#define SPEED_BUFFER_SIZE 16
#define SPEED_PIN_COUNT 8
#define SPEED_MAX_INCREMENT 20

//#define LEFT_STEER_PORT 4
//#define RIGHT_STEER_PORT 5

//QWIICMUX myMux;

unsigned long pulse[SPEED_PIN_COUNT][SPEED_BUFFER_SIZE];
byte pulse_pointer[SPEED_PIN_COUNT] = {0};
byte increment_count[SPEED_PIN_COUNT] = {0};
const byte speedPins[] = {LEFT_FRONT_PIN_1,LEFT_FRONT_PIN_2,RIGHT_FRONT_PIN_1,RIGHT_FRONT_PIN_2,LEFT_REAR_PIN_1,LEFT_REAR_PIN_2,RIGHT_REAR_PIN_1,RIGHT_REAR_PIN_2};

const byte forcePins[4] = {A3,A2,A1,A0};
//const byte forcePins[4] = {A0,A3,A1,A2};
//onst byte brakePins[4] = {D8,D9,D10,D11};
const byte brakePins[4] = {D11,D8,D10,D9};

//AMS_5600 as_l(0x36);
//AMS_5600 as_r(0x36);

int32_t speed_on_single_pin[SPEED_PIN_COUNT];

ros::NodeHandle_<AcsrHardware>  nh;
//ros::NodeHandle_<ArduinoHardware> nh;
//ros::NodeHandle nh;

std_msgs::Int32MultiArray speed_msg;
std_msgs::UInt16MultiArray force_msg;
//std_msgs::Float32MultiArray angle_msg;
//std_msgs::Float32MultiArray imu_raw_msg;
//sensor_msgs::MagneticField mag_msg;
geometry_msgs::PolygonStamped imu_raw_msg;


ros::Publisher speed_pub("wheel_speed", &speed_msg);
ros::Publisher force_pub("wheel_force_raw",&force_msg);
//ros::Publisher angle_pub("wheel_angle_raw",&angle_msg);
ros::Publisher imu_raw_pub("imu/data_raw", &imu_raw_msg);
//ros::Publisher mag_pub("imu/mag", &mag_msg);


float accCalib[3],gyrCalib[3],magCalib[3];
const float deg2rad=PI/180;

uint8_t led_state = false;


//std_msgs::String speed_info;
//ros::Publisher speed_info_pub("wheel_speed_info",&speed_info);

/*
void selectMultiplexerChannel(byte channel){
  byte controlRegister = 0x04; // The Control register of the Multiplexer
  controlRegister |= channel; // Bitwise OR controlRegister & channel
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write((uint8_t)I2CMultiplexer, (uint8_t)controlRegister); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }
}*/


//calc speed

void calcVelocity(){

	for(byte i=0;i<SPEED_PIN_COUNT;++i){
		if(increment_count[i]>=SPEED_MAX_INCREMENT){
			speed_on_single_pin[i]=0;
			continue;
		}
    unsigned long s=pulse[i][(pulse_pointer[i]+SPEED_BUFFER_SIZE-1)%SPEED_BUFFER_SIZE]-pulse[i][pulse_pointer[i]];
    if(s==0)speed_on_single_pin[i]=0;
	else speed_on_single_pin[i]=int32_t((SPEED_BUFFER_SIZE-1)*1000U/s);

	}		
}

//read force. read 4 times and return the average value
void readForce(){
    for(byte i=0;i<4;++i){
		int total = 0;
		for(byte j = 0;j<4;j++){
			total+=analogRead(forcePins[i]);
		}
        force_msg.data[i] = uint16_t(total/4);
	}    
}

//read angle. read 4 times and return the average value
/*
void readAngle(){  
	angle_msg.data[0] = -1.0f;
	angle_msg.data[1] = -1.0f;
	A3
	myMux.setPort(LEFT_STEER_PORT);
	if(as_r.detectMagnet())
		angle_msg.data[1] = float(as_r.getScaledAngle()*0.08789f);
	myMux.setPort(RIGHT_STEER_PORT);
	if(as_l.detectMagnet())
		angle_msg.data[0] = float(as_l.getScaledAngle()*0.08789f);
}*/

//read imu

void readIMU(){  
	while(!IMU.accelerationAvailable());
	IMU.readAcceleration(accCalib[0],accCalib[1],accCalib[2]);
	imu_raw_msg.polygon.points[0].x=accCalib[0]*9.81;
	imu_raw_msg.polygon.points[0].y=-9.81*accCalib[1];
	imu_raw_msg.polygon.points[0].z=9.81*accCalib[2];

	while(!IMU.gyroscopeAvailable());
	IMU.readGyroscope(gyrCalib[0],gyrCalib[1],gyrCalib[2]);
	imu_raw_msg.polygon.points[1].x=deg2rad*gyrCalib[0];
	imu_raw_msg.polygon.points[1].y=-deg2rad*gyrCalib[1];
	imu_raw_msg.polygon.points[1].z=deg2rad*gyrCalib[2];
	
	while(!IMU.magneticFieldAvailable());
	IMU.readMagneticField(magCalib[0],magCalib[1],magCalib[2]);
	imu_raw_msg.polygon.points[2].x=1e-6*magCalib[0];
	imu_raw_msg.polygon.points[2].y=-1e-6*magCalib[1];
	imu_raw_msg.polygon.points[2].z=1e-6*magCalib[2];
	imu_raw_msg.header.stamp = nh.now();

}

void brakeMessageCallback( const std_msgs::Float32MultiArray& msg){
    for(byte i=0;i<4;++i)
        analogWrite(brakePins[i],int(msg.data[i]*255));
}

//interrupt callback function


template<int COUNT>
void speedInteruptCallback(){    
  pulse[COUNT][pulse_pointer[COUNT]]=millis();
  ++pulse_pointer[COUNT];
  pulse_pointer[COUNT]%=SPEED_BUFFER_SIZE;
  increment_count[COUNT]=0;
}

ros::Subscriber<std_msgs::Float32MultiArray> brake_sub("/racecar/brake", &brakeMessageCallback );

void setup(void) {
	//disable interrupt
	//noInterrupts();
	//in-board led
	pinMode(13,OUTPUT);
	
	for(byte i=0;i<4;++i)
		pinMode(brakePins[i],OUTPUT);
	
	IMU.begin();
	/*
	Wire.begin();
	if (myMux.begin() == false)
	{
		Serial.println("Mux not detected. Freezing...");
		while (1)
		;
	}*/
	
	
	for(auto i=0;i<4;++i)
		analogWrite(brakePins[i],0);

	//set up speed pin mode
	for(byte i=0;i<SPEED_PIN_COUNT;++i){
		pinMode(speedPins[i], INPUT);
	}


	//fill up the buffer ring
	for(auto i=0;i<SPEED_PIN_COUNT;++i){
		for(auto j=0;j<SPEED_BUFFER_SIZE;++j){
			pulse[i][j]=millis();
		}
	}

	//attach interrupt callback function
	attachInterrupt(digitalPinToInterrupt(speedPins[0]),speedInteruptCallback<0>,RISING);
	attachInterrupt(digitalPinToInterrupt(speedPins[1]),speedInteruptCallback<1>,RISING);
	attachInterrupt(digitalPinToInterrupt(speedPins[2]),speedInteruptCallback<2>,RISING);
	attachInterrupt(digitalPinToInterrupt(speedPins[3]),speedInteruptCallback<3>,RISING);
	attachInterrupt(digitalPinToInterrupt(speedPins[4]),speedInteruptCallback<4>,RISING);
	attachInterrupt(digitalPinToInterrupt(speedPins[5]),speedInteruptCallback<5>,RISING);
	attachInterrupt(digitalPinToInterrupt(speedPins[6]),speedInteruptCallback<6>,RISING);
	attachInterrupt(digitalPinToInterrupt(speedPins[7]),speedInteruptCallback<7>,RISING);

	//Initializing ros node
	nh.initNode();
	//setup message data structure
	speed_msg.data_length = SPEED_PIN_COUNT ;
	speed_msg.data = new int32_t[SPEED_PIN_COUNT];

	force_msg.data_length = 4 ;
	force_msg.data = new uint16_t[4];

	//angle_msg.data_length = 2 ;
	//angle_msg.data = new float[2];

	imu_raw_msg.polygon.points=new geometry_msgs::Point32[3];
	imu_raw_msg.polygon.points_length = 3;

	//imu_raw_msg.data_length = 9;
	//imu_raw_msg.data = new float[9];

	imu_raw_msg.header.frame_id="imu_link";	
	//mag_msg.header.frame_id="imu_link";

	//Start advertising
	nh.advertise(speed_pub);
	nh.advertise(force_pub);
	//nh.advertise(angle_pub);
	nh.advertise(imu_raw_pub);
  	//nh.advertise(mag_pub);

	//subscribe brake message
	nh.subscribe(brake_sub);
}


 
void loop() { 
	unsigned long start = millis();

	digitalWrite(13,led_state);
	led_state=!led_state;

	for(uint8_t i=0;i<SPEED_PIN_COUNT;++i){
	if(increment_count[i]<SPEED_MAX_INCREMENT)
		++increment_count[i];      
	}


	calcVelocity();

	for(byte i=0;i<SPEED_PIN_COUNT;++i){
		speed_msg.data[i]=speed_on_single_pin[i];
	}
	speed_pub.publish(&speed_msg);

	readForce();
	//readAngle();
	force_pub.publish(&force_msg);
	//angle_pub.publish(&angle_msg);

	readIMU();
	imu_raw_pub.publish(&imu_raw_msg);

	nh.spinOnce();	

	unsigned long inteval = millis()-start;
	if(inteval<50 && inteval>0)
    	delay(50-inteval);
}
