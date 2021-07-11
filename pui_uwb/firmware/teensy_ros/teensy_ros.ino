#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>


rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 range_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// set this to the hardware serial port uwb used
#define HWSERIAL Serial1

#define SOP 'm'
#define EOP '\r'

bool started = false;
bool ended = false;
uint8_t inData[20];
byte numIndex;
float meterRange[4]={0,0,0,0};

float stitchup(uint8_t hi_Bytes,uint8_t lo_Byte)
{
  return float(hi_Bytes*256+lo_Byte);
}

void handleRange()
{
   meterRange[0] = stitchup(inData[7],inData[6])/100;
   meterRange[1] = stitchup(inData[9],inData[8])/100;
   meterRange[2] = stitchup(inData[11],inData[10])/100;
   meterRange[3] = stitchup(inData[13],inData[12])/100;   
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
//{  
//  RCLC_UNUSED(last_call_time);
//  if (timer != NULL) {
//    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//    msg.data++;
//  }
//}

void setup() 
{
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(1000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "uwb_teensy_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "uwb_teensy_node_publisher"));

  msg.data = 0;
  
  HWSERIAL.begin(115200,SERIAL_8N1);

  delay(1000);
}

void loop() 
{
  // Read all serial data available, as fast as possible
  while(HWSERIAL.available() > 0)
  {
    uint8_t inChar = Serial1.read();
    if(inChar == SOP)
    {
       numIndex = 0;
       inData[numIndex] = inChar;
       started = true;
       ended = false;
       numIndex++;
    }
    else if(inChar == EOP)
    {
       inData[numIndex] = inChar;
       ended = true;
       break;
    }
    else
    {
      if(numIndex < 20)
      {
        inData[numIndex] = inChar;
        numIndex++;
      }
    }
  }

  if(started && ended)
  {
    // The end of packet marker arrived. Process the packet
    
    handleRange();

    // Serial output for debugging
    // Serial.print("a0:"); Serial.print(meterRange[0]); Serial.print(", ");
    // Serial.print("a1:"); Serial.print(meterRange[1]); Serial.print(", ");
    // Serial.print("a2:"); Serial.print(meterRange[2]); Serial.print(", ");
    // Serial.print("a3:"); Serial.print(meterRange[3]); Serial.print(", ");
    // Serial.println("------------------------");

    range_msg.data= meterRange[0];
//    range_msg.data= meterRange;
//    range_msg.data_length= 4;

    RCSOFTCHECK(rcl_publish(&publisher, &range_msg, NULL));
    
    // Reset for the next packet
    started = false;
    ended = false;
    numIndex = 0;
    inData[ numIndex] = '\0';
  }
  
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
