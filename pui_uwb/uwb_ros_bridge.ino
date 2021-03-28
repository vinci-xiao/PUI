#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

#include <sensor_msgs/Range.h>
#include <pui_msgs/MultiRange.h>

ros::NodeHandle nh;

pui_msgs::MultiRange range_msg;
ros::Publisher pub_range("uwb_range", &range_msg);


#define SOP 'm'
#define EOP '\r'

bool started = false;
bool ended = false;

uint8_t inData[20];
byte index;
float meterRange[3]={0,0,0};


float stitchup(uint8_t hi_Bytes,uint8_t lo_Byte)
{
  return float(hi_Bytes*256+lo_Byte);
}

void handleRange()
{
   meterRange[0] = stitchup(inData[7],inData[6])/100;
   meterRange[1] = stitchup(inData[9],inData[8])/100;
   meterRange[2] = stitchup(inData[11],inData[10])/100;
}


void setup()
{
  // ROS node
  nh.initNode();
  nh.advertise(pub_range);
  
  Serial.begin(115200);
  Serial1.begin(115200,SERIAL_8N1);

  Serial.println("Hello");
  delay(1000);
}

void loop()
{
  // Read all serial data available, as fast as possible
  while(Serial1.available() > 0)
  {
    uint8_t inChar = Serial1.read();
    if(inChar == SOP)
    {
       index = 0;
       inData[index] = inChar;
       started = true;
       ended = false;
       index++;
    }
    else if(inChar == EOP)
    {
       inData[index] = inChar;
       ended = true;
       break;
    }
    else
    {
      if(index < 20)
      {
        inData[index] = inChar;
        index++;
      }
    }
  }

  if(started && ended)
  {
    // The end of packet marker arrived. Process the packet
    
    handleRange();
    
    //range_msg.header.stamp = nh.now();
    range_msg.ranges= meterRange;
    range_msg.ranges_length= 3;

    pub_range.publish(&range_msg);  

    // Reset for the next packet
    started = false;
    ended = false;
    index = 0;
    inData[index] = '\0';
  }
  
  nh.spinOnce();

}