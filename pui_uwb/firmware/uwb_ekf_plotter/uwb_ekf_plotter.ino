#define SOP 'm'
#define EOP '\r'

bool started = false;
bool ended = false;

uint8_t inData[20];
byte index;

float meterRange[4]={0};

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


void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200,SERIAL_8N1);
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
    handleRange();

//    Serial.print("[0]:"); Serial.println(inData[0],HEX);
//    Serial.print("[1]:"); Serial.println(inData[1],HEX);
//    Serial.print("[2]:"); Serial.println(inData[2],HEX);
//    Serial.print("[3]:"); Serial.println(inData[3],HEX);
//    Serial.print("[4]:"); Serial.println(inData[4],HEX);
//    Serial.print("[5]:"); Serial.println(inData[5],HEX);
//    Serial.print("[6]:"); Serial.println(inData[6],HEX);
//    Serial.print("[7]:"); Serial.println(inData[7],HEX);
//    Serial.print("[8]:"); Serial.println(inData[8],HEX);
//    Serial.print("[9]:"); Serial.println(inData[9],HEX);
//    Serial.print("[10]:"); Serial.println(inData[10],HEX);
//    Serial.print("[11]:"); Serial.println(inData[11],HEX);
//    Serial.print("[12]:"); Serial.println(inData[12],HEX);
//    Serial.print("[13]:"); Serial.println(inData[13],HEX);
//    Serial.print("[14]:"); Serial.println(inData[14],HEX);
//    Serial.print("[15]:"); Serial.println(inData[15],HEX);
//    Serial.print("[16]:"); Serial.println(inData[16],HEX);
//    Serial.print("[17]:"); Serial.println(inData[17],HEX);
//    Serial.print("[18]:"); Serial.println(inData[18],HEX);
    
    Serial.print("a0:"); Serial.print(meterRange[0]); Serial.print(", ");
    Serial.print("a1:"); Serial.print(meterRange[1]); Serial.print(", ");
    Serial.print("a2:"); Serial.print(meterRange[2]); Serial.print(", ");
    Serial.print("a3:"); Serial.print(meterRange[3]); Serial.print(", ");

    Serial.println("------------------------");

//    Serial.print("e0:"); Serial.print(ekfRange[0]); Serial.print(", ");
//    Serial.print("e1:"); Serial.print(ekfRange[1]); Serial.print(", ");
//    Serial.print(",0,4");
    Serial.println();
    
    // Reset for the next packet
    started = false;
    ended = false;
    index = 0;
    inData[index] = '\0';
  }
  
}
