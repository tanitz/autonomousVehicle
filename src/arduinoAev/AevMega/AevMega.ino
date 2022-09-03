#include <SpeedyStepper.h>
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

String       in_String         =     "";
boolean      end_input         =  false;
int32_t     freqn_pwm         =     0;

const int input_brake = 7;
const int MOTOR_STEP_PIN = 11;
const int MOTOR_DIRECTION_PIN = 10;
int brake_req = 0;
int rst_steering_req = 0;

const int estop = 41;
const int mstop = 47;

ros::NodeHandle nh;

void brake_cmd (const std_msgs::Int16& message) {

  brake_req = message.data;
}

void rst_steering_cmd (const std_msgs::Int16& message) {

  rst_steering_req = message.data;
}

ros::Subscriber<std_msgs::Int16> sub_brake("brake_AEV", brake_cmd);
ros::Subscriber<std_msgs::Int16> sub_rst("reset_eps", rst_steering_cmd);
geometry_msgs::Vector3Stamped brake_msg;
geometry_msgs::Vector3Stamped Valtage;
ros::Publisher brake_pub("break_msg", &brake_msg);
ros::Publisher val_pub("val_msg", &Valtage);


SpeedyStepper stepper;


void setup()
{
  pinMode(input_brake, OUTPUT);
  pinMode(estop, INPUT_PULLUP);
  pinMode(mstop, INPUT_PULLUP);
  pinMode(35, OUTPUT);
  Serial.begin(57600);
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(sub_brake);
  nh.subscribe(sub_rst);//suscribe to ROS topic for velocity commands
  nh.advertise(brake_pub);
  nh.advertise(val_pub);
}

int  tempr = 0;
char cmd;
int last_brake = 0;
int last_brake_req = 0 ;
int last_D1 = 0;
int last_D2 = 1;
int pos = 0;
unsigned long lastMilli = 0;
float V1 = 0.0;
float V2 = 0.0;
int analogread1 = 0;
int analogread2 = 0;
void loop()
{
  nh.spinOnce();
  //stepper.setSpeedInRevolutionsPerSecond(18000/100);
  stepper.setSpeedInRevolutionsPerSecond(25600);
  stepper.setAccelerationInStepsPerSecondPerSecond(12800);
  //  stepper.moveToPositionInSteps(brake_req);
  int D1 = digitalRead(estop);
  int D2 = digitalRead(mstop);
  if (D1 == 1 )
  {
    stepper.moveToPositionInSteps(-24000);
  }
  if (D2 == 0 ) {
    pos = pos - 14000;
    if (abs(pos) >= 20000) {
      stepper.moveToPositionInSteps(-24000);
    } else if (abs(pos) <= 20000) {
      stepper.moveToPositionInSteps(pos);
    }
  }
  if (D1 == 0 and D2 == 1)
  { pos = 0;
    stepper.moveToPositionInSteps(brake_req);
  }
  if (rst_steering_req == 1) {
    digitalWrite(35, 1);
  }
  if (rst_steering_req == 0) {
    digitalWrite(35, 0);
  }
  if ((millis() - lastMilli) >= 200)
  {
    analogread1 = analogRead(A1);
    analogread2 = analogRead(A3);
    V1 = analogread1 * (5 / 1023.0);
    V2 = analogread2 * (5 / 1023.0);
    lastMilli = millis();
  }
  last_D1 = D1;
  last_D2 = D2;
  Serial.print(V1);
  Serial.print("  ");
  Serial.println(V2);
  publishSpeed();
  publishval();
}

void publishval() {
  Valtage.header.stamp = nh.now();
  Valtage.vector.x = V1;
  Valtage.vector.y = V2;
  val_pub.publish(&Valtage);
  nh.spinOnce();
  //  nh.loginfo("Publishing 1;");
}

void publishSpeed() {
  brake_msg.header.stamp = nh.now();      //timestamp for odometry data
  brake_msg.vector.x = brake_req;    //left wheel speed (in m/s)
  brake_msg.vector.y = 0;   //right wheel speed (in m/s)
  brake_msg.vector.z = 0;       //looptime, should be the same as specified in LOOPTIME (in s)

  brake_pub.publish(&brake_msg);
  nh.spinOnce();
  //  nh.loginfo("Publishing 1;");
}
