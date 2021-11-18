// ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6

#include <Arduino_LSM6DSOX.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <TimeLib.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>

rcl_publisher_t publisher;

std_msgs__msg__Int32 msg;
sensor_msgs__msg__Imu imu_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

float Ax, Ay, Az;
float Gx, Gy, Gz;
bool is_acc =false;
bool is_gyr =false;

#define dps2rps 0.017453292
#define G 9.81

const int timeout_ms = 1000;
static int64_t time_ns;
static time_t time_seconds;
char time_str[25];

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
  }
}

void setup() {
  Serial.begin(9600);

  while(!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "imu_rp2040_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_rp2040"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // set frame_id
  imu_msg.header.frame_id.data = "imu_link";
  imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);
}

void loop() {
  
  // Synchronize time
  RCCHECK(rmw_uros_sync_session(timeout_ms));
  time_ns = rmw_uros_epoch_nanos();
  if (time_ns > 0)
  {
    time_seconds = time_ns/1000000000;
    setTime(time_seconds); 
  }
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);
    imu_msg.linear_acceleration.x = Ax*G;
    imu_msg.linear_acceleration.y = Ay*G;
    imu_msg.linear_acceleration.z = Az*G;
    is_acc =true;
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gx, Gy, Gz);
    imu_msg.angular_velocity.x = Gx*dps2rps;
    imu_msg.angular_velocity.y = Gy*dps2rps;
    imu_msg.angular_velocity.z = Gz*dps2rps;
    is_gyr =true;
  }

  if (is_acc&&is_gyr){
    // ros time stamp
//    imu_msg.header.stamp.sec =(int32_t)time_ms/1000;
//    imu_msg.header.stamp.nanosec =(uint32_t)time_ms%1000;
    imu_msg.header.stamp.sec =(int32_t)time_seconds;
    imu_msg.header.stamp.nanosec =(uint32_t)time_ns%1000000000;
    
    // publish imu topic
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    // reset flags
    is_acc =false;    
    is_gyr =false;
  }
}
