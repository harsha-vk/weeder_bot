#include <PID_v1.h>

#include <ros.h>
#include <ros/time.h>
#include <hardware_msgs/TwistStamped.h>
#include <hardware_msgs/Twist.h>
#include <hardware_msgs/Relay.h>

#define L_ENCA 2
#define L_ENCB 4
#define R_ENCA 3
#define R_ENCB 5

#define L_DIR 8
#define L_PWM 9
#define R_PWM 10
#define R_DIR 11

#define S1 A0
#define S2 A1
#define S3 A2

#define LEFT_KP 0
#define LEFT_KI 0
#define LEFT_KD 0
#define RIGHT_KP 0
#define RIGHT_KI 0
#define RIGHT_KD 0
#define TWIST_TIMEOUT 0
#define DIA 0
#define T_PER_R 0
#define A_ (PI*DIA*1000/(T_PER_R*TWIST_TIMEOUT))

//PIDs
double l_read_vel=0, l_out_pul=0, l_set_vel=0;
PID l_pid(&l_read_vel,&l_out_pul,&l_set_vel,LEFT_KP,LEFT_KI,LEFT_KD,DIRECT);
double r_read_vel=0, r_out_pul=0, r_set_vel=0;
PID r_pid(&r_read_vel,&r_out_pul,&r_set_vel,RIGHT_KP,RIGHT_KI,RIGHT_KD,DIRECT);

long pre_millis=0;
volatile int l_tick=0, r_tick=0;
int pre_l_tick=0, pre_r_tick=0;

//Node
ros::NodeHandle nh;

//Publishers
hardware_msgs::TwistStamped tx_twist_msg;
ros::Publisher twist_pub("tx_twist",&tx_twist_msg);

//Subscribers
void twist_sub_cb(const hardware_msgs::Twist &rx_twist_msg) {
  l_set_vel=rx_twist_msg.left_wheel;
  r_set_vel=rx_twist_msg.right_wheel;
}
ros::Subscriber<hardware_msgs::Twist> twist_sub("vel",&twist_sub_cb);
void rel_sub_cb(const hardware_msgs::Relay &rel_msg) {
  digitalWrite(S1,!rel_msg.relay[0]);
  digitalWrite(S2,!rel_msg.relay[1]);
  digitalWrite(S3,!rel_msg.relay[2]);
}
ros::Subscriber<hardware_msgs::Relay> rel_sub("rel",&rel_sub_cb);

void setup() {
  //Setup outputs
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);
  digitalWrite(S1,HIGH);
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  pinMode(L_DIR,OUTPUT);
  pinMode(L_PWM,OUTPUT);
  pinMode(R_PWM,OUTPUT);
  pinMode(R_DIR,OUTPUT);
  // Setting L_PWM & R_PWM to fast PWM mode with frequency 8kHz.
  TCCR1A=0;
  TCCR1A=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);  
  TCCR1B=0;
  TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS10); 
  ICR1=1999;
  
  //Setup Node and topics
  nh.initNode();
  nh.advertise(twist_pub);
  nh.subscribe(twist_sub);
  nh.subscribe(rel_sub);

  //Setup PIDs
  l_pid.SetMode(AUTOMATIC);
  l_pid.SetSampleTime(1);
  l_pid.SetOutputLimits(-1999, 1999);
  r_pid.SetMode(AUTOMATIC);
  r_pid.SetSampleTime(1);
  r_pid.SetOutputLimits(-1999, 1999);

  //Setup Inputs
  pinMode(L_ENCA,INPUT_PULLUP);
  pinMode(L_ENCB,INPUT_PULLUP);
  pinMode(R_ENCA,INPUT_PULLUP);
  pinMode(R_ENCB,INPUT_PULLUP);
  //Rising edge of INT0 & INT1 generates interrupts.
  EICRA|=(1<<ISC00)|(1<<ISC01)|(1<<ISC10)|(1<<ISC11);
  //Enable interrupts for INT0 & INT1.
  EIMSK|=(1<<INT0)|(1<<INT1);
  //Enable global interrupts
  sei();
}

void loop() {
  if(millis()-pre_millis > TWIST_TIMEOUT){
    l_read_vel=A_*(l_tick-pre_l_tick);
    r_read_vel=A_*(r_tick-pre_r_tick);
    
    tx_twist_msg.left_wheel=l_read_vel;
    tx_twist_msg.right_wheel=r_read_vel;
    tx_twist_msg.stamp=nh.now();
    twist_pub.publish(&tx_twist_msg);
    
    pre_l_tick=l_tick;
    pre_r_tick=r_tick;
    pre_millis=millis();
  }
  l_pid.Compute();
  r_pid.Compute();
  pwm_out(l_out_pul,r_out_pul);
  nh.spinOnce();
  delayMicroseconds(1000);
}

void pwm_out(int l_pul,int r_pul) {
  if (l_pul > 0) {
    OCR1A=l_pul;
    digitalWrite(L_DIR, LOW);
  }
  else {
    OCR1A=abs(l_pul);
    digitalWrite(L_DIR, HIGH);
  }
  if (r_pul > 0) {
    OCR1B=r_pul;
    digitalWrite(R_DIR, LOW);
  }
  else {
    OCR1B=abs(r_pul);
    digitalWrite(R_DIR, HIGH);
  }
}

ISR(INT0_vect) { //Left Encoder ISR
  EIMSK &= ~(1<<INT0);  //Disable INT0
  if(PIND & 0b00010000) { //Read pin 4
    l_tick--;
  }
  else {
    l_tick++;
  }
  EIMSK |= (1<<INT0);   //Enable INT0
}
 
ISR(INT1_vect) { //Right Encoder ISR
  EIMSK &= ~(1<<INT1);  //Disable INT1
  if(PIND & 0b00100000) { //Read pin 5
    r_tick++;
  }
  else {
    r_tick--;
  }
  EIMSK |= (1<<INT1);   //Enable INT1
}