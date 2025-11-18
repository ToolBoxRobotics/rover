#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// ----- PIN DEFINITIONS -----
const uint8_t NUM_WHEELS = 6;
const uint8_t NUM_SERVOS = 4;

// Motor pins (example)
uint8_t motor_pwm_pins[NUM_WHEELS] = {3, 5, 6, 9, 10, 11};
uint8_t motor_dir_pins[NUM_WHEELS] = {22, 23, 24, 25, 26, 27};

// Encoder pins (ONLY A channel here, use interrupts if needed)
uint8_t encoder_pins[NUM_WHEELS] = {2, 18, 19, 20, 21, 7};
volatile long encoder_counts[NUM_WHEELS] = {0};

// Servo pins
uint8_t servo_pins[NUM_SERVOS] = {30, 31, 32, 33}; // FL, FR, RL, RR
Servo steering_servos[NUM_SERVOS];

// Servo calibration (map from rad to microseconds)
float servo_center[NUM_SERVOS] = {1500, 1500, 1500, 1500};
float servo_gain_us_per_rad[NUM_SERVOS] = {500, 500, 500, 500}; // ~30 deg

// ROS node
ros::NodeHandle nh;

// Subscribed PWM command
std_msgs::Int16MultiArray pwm_cmd_msg;

// Subscribed steering command (angles in radians)
std_msgs::Float32MultiArray steering_cmd_msg;

// Published encoders
std_msgs::Int32MultiArray encoder_msg;

// Encoder ISRs (example: first two wheels on hardware interrupts)
void encoder0ISR() { encoder_counts[0]++; }
void encoder1ISR() { encoder_counts[1]++; }
void encoder2ISR() { encoder_counts[2]++; }
void encoder3ISR() { encoder_counts[3]++; }
void encoder4ISR() { encoder_counts[4]++; }
void encoder5ISR() { encoder_counts[5]++; }

void pwmCmdCallback(const std_msgs::Int16MultiArray &msg) {
  for (uint8_t i = 0; i < NUM_WHEELS && i < msg.data_length; i++) {
    int16_t pwm = msg.data[i];
    bool dir = pwm >= 0;
    int16_t val = abs(pwm);
    if (val > 255) val = 255;

    digitalWrite(motor_dir_pins[i], dir ? HIGH : LOW);
    analogWrite(motor_pwm_pins[i], val);
  }
}

void steeringCmdCallback(const std_msgs::Float32MultiArray &msg) {
  for (uint8_t i = 0; i < NUM_SERVOS && i < msg.data_length; i++) {
    float angle_rad = msg.data[i];
    int us = (int)(servo_center[i] + servo_gain_us_per_rad[i] * angle_rad);
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    steering_servos[i].writeMicroseconds(us);
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub_pwm("wheel_pwm_cmd", pwmCmdCallback);
ros::Subscriber<std_msgs::Float32MultiArray> sub_steer("steering_cmd", steeringCmdCallback);
ros::Publisher pub_encoders("encoder_ticks", &encoder_msg);

unsigned long last_pub = 0;
const unsigned long PUB_INTERVAL_MS = 20; // 50 Hz

void setupEncoders() {
  pinMode(encoder_pins[0], INPUT_PULLUP);
  pinMode(encoder_pins[1], INPUT_PULLUP);
  pinMode(encoder_pins[2], INPUT_PULLUP);
  pinMode(encoder_pins[3], INPUT_PULLUP);
  pinMode(encoder_pins[4], INPUT_PULLUP);
  pinMode(encoder_pins[5], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder_pins[0]), encoder0ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_pins[1]), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_pins[2]), encoder2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_pins[3]), encoder3ISR, RISING);
  // If remaining pins have interrupts available, attach as well.
}

void setup() {
  // Motors
  for (uint8_t i = 0; i < NUM_WHEELS; i++) {
    pinMode(motor_pwm_pins[i], OUTPUT);
    pinMode(motor_dir_pins[i], OUTPUT);
    analogWrite(motor_pwm_pins[i], 0);
  }

  // Servos
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    steering_servos[i].attach(servo_pins[i]);
    steering_servos[i].writeMicroseconds(servo_center[i]);
  }

  setupEncoders();

  nh.initNode();
  nh.subscribe(sub_pwm);
  nh.subscribe(sub_steer);
  nh.advertise(pub_encoders);

  // Init messages
  encoder_msg.data_length = NUM_WHEELS;
  encoder_msg.data = (int32_t*)malloc(sizeof(int32_t) * NUM_WHEELS);
}

void loop() {
  unsigned long now = millis();
  if (now - last_pub >= PUB_INTERVAL_MS) {
    noInterrupts();
    for (uint8_t i = 0; i < NUM_WHEELS; i++) {
      encoder_msg.data[i] = encoder_counts[i];
    }
    interrupts();

    pub_encoders.publish(&encoder_msg);
    last_pub = now;
  }

  nh.spinOnce();
}
