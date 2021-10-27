// ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6

// On Nano RP2040 Connect, RGB leds are connected to the wifi module
// The user APIs are the same, but we can't convert to int, so use defines
#if defined(ARDUINO_NANO_RP2040_CONNECT)

#include "WiFiNINA.h"
#define led1  LEDR
#define led2  LEDG
#define led3  LEDB

// On Nicla Sense ME, RGB leds are connected via an I2C module
// The user APIs are the same, but we can't convert to int, so use defines
#elif defined(ARDUINO_NICLA)

#include "Nicla_System.h"
#define led1  LEDR
#define led2  LEDG
#define led3  LEDB

#else

int led1 = LEDR;
int led2 = LEDG;
int led3 = LEDB;

#endif

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <pui_msgs/msg/multi_range.h>
pui_msgs__msg__MultiRange range_msg_;


rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// set this to the hardware serial port uwb used
//#define HWSERIAL Serial1

#define SOP 'm'
#define EOP '\r'

bool started = false;
bool ended = false;
uint8_t inData[20];
uint8_t tagID;
byte numIndex;
float meterRange[4];

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
//    digitalWrite(led1, !digitalRead(led1));
    digitalWrite(led1, HIGH);
    delay(100);
  }
}

void setup() 
{
  // Setup the 3 pins as OUTPUT
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  digitalWrite(led2, HIGH);  

  set_microros_transports();

  delay(1000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 73;
  
  // create node
  RCCHECK(rclc_node_init_with_options(&node, "uwb_micro_ros_node", "", &support, &node_ops));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(pui_msgs, msg, MultiRange),
    "uwb_range"));

  msg.data = 0;
  
//  HWSERIAL.begin(115200,SERIAL_8N1);
  Serial1.begin(115200,SERIAL_8N1);
  
  delay(1000);
}

void loop() 
{
  // Read all serial data available, as fast as possible
  while(Serial1.available())
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
    tagID = inData[3];

    range_msg_.id = tagID;
    
    range_msg_.ranges.data = (float *)calloc(10, sizeof(float));
    range_msg_.ranges.capacity = 10;
    range_msg_.ranges.data= meterRange;
    range_msg_.ranges.size= sizeof meterRange/sizeof meterRange[0];

    RCSOFTCHECK(rcl_publish(&publisher, &range_msg_, NULL));
    
    // Reset for the next packet
    started = false;
    ended = false;
    numIndex = 0;
    inData[numIndex] = '\0';
  }
  
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
