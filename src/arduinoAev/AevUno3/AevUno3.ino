//Now

#include <PID_v1.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>

// absolute encoder 12 bits
const int  CLK = 19; // Blu Digital Pin 10
const int  DO = 7; // Grn Digital Pin 11
const int CSn = 18; // Ylw Digital Pin 12

int absolute_position;
int Degree;
int Deg = 0;
int rp = 0;
int data_angle = 0;
long int speeds_encoder;

const int analog1 = 5;
const int analog2 = 6;
const int speed_io = 3;
//________________________________PID_________________________________________
double Setpoint, Input, Output;
double Kp = 2, Ki = 0.35, Kd = 0.1;
//int offset = 60;
int offset = 32;
PID Steering(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double Setpoint2, Input2, Output2;
double Kp2 = 23, Ki2 = 3, Kd2 = 0;
PID Speed(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

long int D0 = 0;
long int D1 = 0;
long int speeds ;

//________________________________ROS_________________________________________
#define LOOPTIME 100     //Looptime in millisecond
unsigned long lastMilli = 0;
int steering_req = 0;
int speed_req = 0;
int break_req = 0;


ros::NodeHandle nh;
void steering_cmd (const std_msgs::Int16& message) {
  steering_req = message.data;
}

void speed_cmd (const std_msgs::Int16& message1) {
  speed_req = message1.data;
}

void break_cmd (const std_msgs::Int16& message1) {
  break_req = message1.data;
}

ros::Subscriber<std_msgs::Int16> sub("steering", steering_cmd);
ros::Subscriber<std_msgs::Int16> sub1("speed", speed_cmd);
ros::Subscriber<std_msgs::Int16> sub2("break", break_cmd);
geometry_msgs::Twist steering_msg;

ros::Publisher steering_pub("angle", &steering_msg);

//________________________________Canbus_________________________________________
#include <SPI.h>
#define CAN_2515
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif
#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif
#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif


//_________________________________________________________________________


void setup() {
  Serial.begin(57600);
  //_________________________________________________________________________
  pinMode(analog1, OUTPUT);
  pinMode(analog2, OUTPUT);
  pinMode(speed_io, OUTPUT);
  pinMode(16, OUTPUT);
  //_________________________________________________________________________
  pinMode(CSn, OUTPUT);// Chip select
  pinMode(CLK, OUTPUT);// Serial clock
  pinMode(DO, INPUT_PULLUP);// Serial data IN/OUT

  Steering.SetMode(AUTOMATIC);
  Steering.SetSampleTime(0.01);  // refresh rate of PID controller
  Steering.SetOutputLimits(-offset, offset);

  Speed.SetMode(AUTOMATIC);
  Speed.SetSampleTime(1);  // refresh rate of PID controller
  Speed.SetOutputLimits(0, 165);
  //_________________________________________________________________________
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(sub);                    //suscribe to ROS topic for velocity commands
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(steering_pub);                  //prepare to publish speed in ROS topic

  control_steering(0);
  delay(2000);
  SERIAL_PORT_MONITOR.begin(57600);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    //    SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
    delay(100);
  }
  //  SERIAL_PORT_MONITOR.println("CAN init ok!");
  //  nh.loginfo("Publishing 1;");
}
int input3 = 0;
void loop() {
  nh.spinOnce();
  if ((millis() - lastMilli) >= 0)
  { // enter timed loop

    lastMilli = millis();

    //___________________________________PID STEERING______________________________________
    angle();

    Input = data_angle;
    Setpoint = steering_req;
    double gap = abs(Setpoint - Input); //distance away from setpoint
    if (gap == 0)
    {
      Steering.SetTunings(1.0, 1, 0.05);
      
    }
    else if (gap < 10)
    {
      Steering.SetTunings(1.0, 1, 0.05);
    }
    else if (gap < 30)
    {
      Steering.SetTunings(1.5, 1, 0.1);
    }
    else if (gap < 40)
    {
      Steering.SetTunings(1.5, 1, 0.1);
    }
    else if (gap >= 40)
    {
      Steering.SetTunings(2.0, Ki, Kd);
    }
    Steering.Compute();
    control_steering(Output);
    //___________________________________PID SPEED______________________________________
    canbus_speed();

    //    analogWrite(speed_io, 10);
    double gap2 = abs(Setpoint2 - Input2);
    Input2 = speeds_encoder;
    Setpoint2 = speed_req;
    Speed.Compute();
    if(Setpoint2 == 0){
      Output2=0;
    }
    analogWrite(speed_io, Output2);
//    Speed.SetTunings( 18, Ki2 , Kd2);
    //    Setpoint_last = Setpoint;
    //    Serial.print(Input);
    //    Serial.print("\t");
    //       Serial.print(Setpoint);
    //       Serial.print("\t");
    //       Serial.print(Output);
    //       Serial.println("\t");

    publishSpeed(Output, Output2);
  }





  //________________________________ROS MSG STEERING______________________________________


}

void control_steering(int control) {
  analogWrite(analog1, 127 + control); // 5V/2 = 127 analog signal
  analogWrite(analog2, 127 - control);
}


void publishSpeed(int output_new, int output_new2) {
  steering_msg.linear.x = Input;    //left wheel speed (in m/s)
  steering_msg.linear.y = Setpoint;   //right wheel speed (in m/s)
  steering_msg.linear.z = output_new;       //looptime, should be the same as specified in LOOPTIME (in s)
  steering_msg.angular.x = Input2;    //left wheel speed (in m/s)
  steering_msg.angular.y = Setpoint2;   //right wheel speed (in m/s)
  steering_msg.angular.z = output_new2;       //looptime, should be the same as specified in LOOPTIME (in s)

  // steering_msg.data = steering_act;    //left wheel speed (in m/s)
  steering_pub.publish(&steering_msg);
  //  nh.spinOnce();
  //  nh.loginfo("Publishing 1;");
}

int ReadSSI(void)
{ int i, dReading;
  char Resolution = 12;
  unsigned int bitStart = 0x0800;
  dReading = 0;
  digitalWrite(CSn, LOW);
  delayMicroseconds(5);
  digitalWrite(CLK, LOW);
  for (i = (Resolution - 1); i >= 0; i--)
  { digitalWrite(CLK, HIGH);
    delayMicroseconds(5);
    if (digitalRead(DO)) dReading |= bitStart;
    digitalWrite(CLK, LOW);
    delayMicroseconds(5);
    bitStart = bitStart >> 1;
    if (i == 0)
    { digitalWrite(CLK, HIGH);
      if (digitalRead(DO)) dReading |= bitStart;
    }
  }
  digitalWrite(CSn, HIGH);
  return dReading;
}

void update_absolute_position(int encoder_position)
{
  int old_position = absolute_position;  // extract LSB
  int delta = encoder_position - old_position;
  if (delta > 320) {
    rp--;
  }
  else if (delta < -320) {
    rp++;
  }
  absolute_position += delta;
}

void angle() {
  int Data = ReadSSI();
  Degree = map(Data, 0, 4095, 0, 359);
  update_absolute_position(Degree);
  Deg = ((rp * 360) + absolute_position) - 82;
  data_angle = map(Deg, 1600, -1600, -180, 180);
  //  Serial.println(Deg);
}

void canbus_speed() {

  unsigned char len = 0;
  unsigned char buf[8];
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();
    if (canId == 534) {
      D1 = buf[1] ;
      D0 = buf[0];
      speeds = D1 << 8 | D0;
      //      speeds = D1 ;

      //      Serial.println(speeds - 30000);
      speeds_encoder = (speeds - 30000) / 100;
    }
  }
}
