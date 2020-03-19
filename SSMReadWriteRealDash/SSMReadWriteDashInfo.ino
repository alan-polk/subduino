// =========================================================
// Program Name: SSMReadDashInfo
// Current Version Arthor: Alan Polk
// https://github.com/alan-polk/subduino
// ==========================================================
// This started as a fork of Matt Prophet's  Subduino Project.  
// But as someone who teaching themselves C++ as part of the project
// I found the arrays and memcpy hard to reverse engineer and follow
// I've simplied the code down quite a bit so that each call is spelled
// out with each parameters address easily identifiable.  The code
// in this program will not be as efficient, but should be easier
// for a begginer to work with.
// ==========================================================

#include <SoftwareSerial.h>

// Define the software serial port
SoftwareSerial kLine = SoftwareSerial(10, 11); //Rx, Tx

// =========================================================
// Defining the reads to the ECU
// The requests include the address of the data byte that we want to read from ECU memory
// Message packets all follow this structure:
// 0×80  - Header
// Destination byte  - 0x10 is the ECU, 0x18 is the TCU
// Source byte - 0x10 is the ECU, 0x18 is the TCU, 0xF0 is diagnostics tool (Arduino here)
// Data Size byte  - Amount of data being sent, not including the overhead (5 bytes of overhead)
// Command/Response byte - 0xA0 is block read, 0xA8 is address read, 0xB0 is block write, 0xB8 is address write
// Continous/Single byte - 0x00 is for one time read/write
// data… - these will be the address location we want to read on a single read, or the starting address and number of bytes on a block read
// Checksum byte - To manually calcualte the checksum, add up all of the values in your packet, and take the 8 least signifigant bits
// -----------------------------------------------
// The checksum is automatically calcualted and you can put any value you want into the packet definition.
// For a better explination on checksum look here:
// http://www.romraider.com/forum/viewtopic.php?f=7&t=10122&start=15
// =========================================================

// Define variable for debug messaging.  Set to zero to turn off messaging
byte DebugMsg = 1;



// Reads will be broken into 3 sets, fast (0.5 seconds), Normal (2 seconds), and Slow (5 seconds)

// Define the Fast Read Packet.  We are reading requesting 5 values, at 3 bytes per address, there are 6 overhead bytes, 1 checksum byte. Total of 22 bytes
// Fast Reads | RPMs (0x0E, 0x0F) | AFR (0x46) | Boost (0x24) | Coolant (0x08)
uint8_t pollECUFast[22] = {0x80, 0x10, 0xF0, 0x16, 0xA8, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x08, 0x00, 0x00, 0x24, 0x00, 0x00, 0x46, 0xCD};
// Define a buffer to put the data we recieve into.  We are reading 5 bytes
uint8_t ECUFastBuffer[5] = {0,0,0,0,0,};

// Define the Normal Read Packet.  We are reading requesting 4 values, at 3 bytes per address, there are 6 overhead bytes, 1 checksum byte. Total of 19 bytes
// Normal Reads | Velocity (0x10) | Fuel Level (0x2E)| Intake Air Temp (0x12) | Battery Voltage (0x1C) | Gear (0x4A)
uint8_t pollECUNorm[22] = {0x80, 0x10, 0xF0, 0x13, 0xA8, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x2E,0x00, 0x00, 0x12, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x4A, 0xA7}; 
// Define a buffer to put the data we recieve into.  We are reading 4 bytes
uint8_t ECUNormBuffer[5] = {0,0,0,0,0};

// Define the Slow Read Packet.  We are reading requesting 3 values, at 3 bytes per address, there are 6 overhead bytes, 1 checksum byte.  Total of 16 bytes
// Slow Reads | Odometer (0x10E = [0xFF + 0x0F], 0x10F = [0xFF + 0x10) | Cruise Speed (0x10A = [0xFF + 0x0B])
uint8_t pollECUSlow[16] = {0x80, 0x10, 0xF0, 0x10, 0xA8, 0x00, 0x00, 0xFF, 0x0F, 0x00, 0xFF, 0x10, 0x00, 0xFF, 0x0B, 0x5F };
// Define a buffer to put the data we recieve into.  We are reading 3 bytes
uint8_t ECUSlowBuffer[3] = {0,0,0};

// Define variables to track scan times for each set
unsigned long Time;
byte LastFastScan;
byte LastNormScan;
byte LastSlowScan;

// Define variables for holding values
uint8_t RPMhi;
uint8_t RPMlo;
uint8_t CTSr;
uint8_t BOOSTr;
uint8_t AFRr;
uint8_t VELr;
uint8_t FUELr;
uint8_t IATr;
uint8_t BATr;
uint8_t GEARr;
uint8_t ODOhi;
uint8_t ODOlo;
uint8_t CRSr;
uint8_t OILr; // Oil Pressure From Analog in on Arduino
bool LBLr; // Left Blinker from Digital in on Arduino
bool RBLr; // Right Blinker from Digital in on Arduino
bool HBr; // High Beams from Digital in on Arduino

// ========================================================

void setup() {
  Serial.begin(9600); //for diagnostics
  kLine.begin(4800); //SSM uses 4800 8N1 baud rate
  
}
// =======================================================

void loop() {

  Time = millis();
  if (DebugMsg == 1) {
    Serial.print("Time: "); Serial.println(Time);
    Serial.print("LastFastScan: "); Serial.println(LastFastScan);
    Serial.print("LastNormScan: "); Serial.println(LastNormScan);
    Serial.print("LastSlowScan: "); Serial.println(LastSlowScan);
  }
  
// Fast Reads
  if (Time - LastFastScan > 500) {

    if(serialCallSSM(kLine, pollECUFast, 22, ECUFastBuffer, 5, 0) < 10) // Important to update these lengths to match packet definition
   {
      LastFastScan = millis();
      RPMhi = ECUFastBuffer[0];
      RPMlo = ECUFastBuffer[1];
      CTSr = ECUFastBuffer[2];
      BOOSTr = ECUFastBuffer[3];
      AFRr = ECUFastBuffer[4];
      if (DebugMsg == 1) {
        float RPM = word(RPMhi, RPMlo)/4;
        float AFR = (AFRr/128)*14.7;
        float BOOST = (BOOSTr - 128)*.145;
        float CTS = ((CTSr-40)*1.8)+32;
    
        Serial.print(F("RPM: "));
        Serial.println(RPM);
        Serial.print("Temp: ");
        Serial.println(CTS);
        Serial.print("BOOST): ");
        Serial.println(BOOST);
        Serial.print(F("AFR: "));
        Serial.println(AFR);
      }
    }
  }

// Normal Reads
  if(Time - LastNormScan > 2000) {
    if(serialCallSSM(kLine, pollECUNorm, 19, ECUNormBuffer, 4, 0) < 10)  // Important to update these lengths to match packet definition
    {
      LastNormScan = millis();
      VELr = ECUNormBuffer[0];
      FUELr = ECUNormBuffer[1];
      IATr = ECUNormBuffer[2];
      BATr = ECUNormBuffer[3];
      GEARr = ECUNormBuffer[4];
      if (DebugMsg == 1) {
        float VEL = VELr * 0.621371192;
        float FUEL = FUELr*0.02;
        float IAT = 32+1.8*(IATr-40);
        float BAT = BATr*0.08;
        uint8_t GEAR = GEARr + 1;
         Serial.print(F("Velocity: "));
        Serial.println(VEL);
        Serial.print("Fuel Level: ");
        Serial.println(FUEL);
        Serial.print("Intake Air Temp: ");
        Serial.println(IAT);
        Serial.print(F("Battery Voltage: "));
        Serial.println(BAT);
        Serial.print(F("Gear: "));
        Serial.println(GEAR);
      }
    }
  }
  
// Slow Reads
  if(Time - LastSlowScan > 5000) {
    if(serialCallSSM(kLine, pollECUSlow, 16, ECUSlowBuffer, 3, 0) < 10)  // Important to update these lengths to match packet definition
    {
      LastSlowScan = millis();
      ODOhi = ECUSlowBuffer[0];
      ODOlo = ECUSlowBuffer[1];
      CRSr = ECUSlowBuffer[2];
      if (DebugMsg == 1) {
        float ODO = word(ODOhi, ODOlo)*1.242742384;
        float CRS = (CRSr * 0.621371192);
        Serial.print(F("ODOhi / ODOlo: "));
        Serial.print(ODOhi);
        Serial.print(F(" / "));
        Serial.println(ODOlo);
        Serial.print(F("Odometer: "));
        Serial.println(ODO);
        Serial.print(F("Cruise Speed: "));
        Serial.println(CRSr);
      }
    }
  }

// Send all data to CanBus every scan
  BuildCanFrames();
}


boolean readSSM(SoftwareSerial &serialPort, uint8_t *dataBuffer, uint16_t bufferSize)
{
  //read in an SSM serial response packet, place the data bytes in the data buffer
  //and return a boolean value indicating whether the checksum failed
  uint16_t timeout = 100;//100 millisecond timeout for response
  uint32_t timerstart = millis();//start the timer
  boolean notFinished = true;//a flag to indicate that we are done
  boolean checksumSuccess = false;
  uint8_t dataSize = 0;
  uint8_t calcSum = 0;
  uint8_t packetIndex = 0;

  Serial.print(F("Read Packet: [ "));
  while(millis() - timerstart <= timeout && notFinished)
  {
    if(serialPort.available() > 0)
    {
      uint8_t data = serialPort.read();
      Serial.print(data);
      Serial.print(F(" "));//if not printing data, be sure to delay to allow for more bytes to come in
      if(packetIndex == 0 && data == 128)
      {
        //0x80 or 128 marks the beginning of a packet
        packetIndex += 1;
        calcSum += data;
      }
      else if(packetIndex == 1 && data == 240)
      {
        //this byte indicates that the message recipient is the 'Diagnostic tool'
        packetIndex += 1;
        calcSum += data;
      }
      else if(packetIndex == 2 && data == 16)
      {
        //this byte indicates that the message sender is the ECU
        packetIndex += 1;
        calcSum += data;
      }
      else if(packetIndex == 3)
      {
        //this byte indicates the number of data bytes which follow
        dataSize = data;
        packetIndex += 1;
        calcSum += data;
      }
      else if(packetIndex == 4)
      {
        //I don't know what this byte is for
        packetIndex += 1;
        calcSum += data;
      }
      else if(packetIndex > 4 && packetIndex < (dataSize+4))
      {
        //these bytes are data
        if(packetIndex - 5 < bufferSize)//make sure it fits into the buffer
        {
          dataBuffer[packetIndex - 5] = data;
        }
        packetIndex += 1;
        calcSum += data;
      }
      else if(packetIndex == dataSize + 4)
      {
        //this is the checksum byte
        if(data == calcSum)
        {
          //checksum was successful
          checksumSuccess = true;
        }
        else
        {
          Serial.print(F("Checksum fail "));
        }
        notFinished = false;//we're done now
        break;
      }
      timerstart = millis();//reset the timeout if we're not finished
    }
  }
  Serial.println(F("]"));
  if(notFinished)
  {
    Serial.println(F("Comm. timeout"));
    delay(1000);
  }
  return checksumSuccess;
}

//writes data over the software serial port
void writeSSM(SoftwareSerial &serialPort, uint8_t *dataBuffer, uint8_t bufferSize) {
  //this function needs to catch mistakes that we make in preparing 
  dataBuffer[0] = 0x80; // Header Packet
  dataBuffer[1] = 0x10; // to the ECU
  dataBuffer[2] = 0xF0; // from diagnostics tool
  dataBuffer[3] = bufferSize - 5; //# data bytes.  Overhead data is not included in this size.
  dataBuffer[4] = 0xA8; //single address reads
  dataBuffer[5] = 0x00;// no continuous polling
  uint8_t sum = 0;
  Serial.print(F("Sending packet: [ "));
  for (uint8_t x = 0; x < bufferSize; x++) {
    if(x == bufferSize - 1 && sum != dataBuffer[x])
    {
      dataBuffer[x] = sum;//fix the checksum in the data buffer
    }
    else
    {
      sum += dataBuffer[x];//build the checksum
    }
    serialPort.write(dataBuffer[x]);
    Serial.print(dataBuffer[x]);
    Serial.print(F(" "));
  }
  Serial.println(F("]"));
}

uint8_t serialCallSSM(SoftwareSerial &serialPort, uint8_t *sendDataBuffer, uint16_t sendBufferSize, uint8_t *receiveDataBuffer, uint16_t receiveBufferSize, uint8_t attempts)
{
  // this function performs the call and response routine for 
  // exchanging serial data with the Subaru ECU via SSM
  // it sends a message and awaits a response, resending the 
  // message if we reach timeout or if the checksum failed
  // it stops trying the data exchange if 10 failures occur and 
  // it outputs the number of attempts it failed to send data
  // input: handle to software serial pins
  //  buffer with outgoing data and length of buffer
  //  buffer for received data (not the whole packet), and length of buffer
  //  the initial value for a counter of failed attempts

  writeSSM(serialPort, sendDataBuffer, sendBufferSize);//send the message
  boolean success = readSSM(serialPort, receiveDataBuffer, receiveBufferSize);//receive the response
  uint8_t newAttempt = 0;
  
  if(!success && attempts < 10)
  {
    Serial.println(F("Packet exchange failed, trying again... "));
    newAttempt = serialCallSSM(serialPort, sendDataBuffer, sendBufferSize, receiveDataBuffer, receiveBufferSize, attempts + 1);
  }
  newAttempt += attempts;
  
  return newAttempt;
}

void BuildCanFrames()
{
  byte CanBuffer[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes containing data in a manner
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values

  // build 1st CAN frame: RPMhi, RPMlo, AFR, BOOST, Coolant, Battery, Speed, IAT
  CanBuffer[0] = RPMlo;
  CanBuffer[1] = RPMhi;
  CanBuffer[2] = AFRr;
  CanBuffer[3] = BOOSTr;
  CanBuffer[4] = CTSr;
  CanBuffer[5] = FUELr;
  CanBuffer[6] = VELr;
  CanBuffer[7] = IATr;

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, CanBuffer);

  // build 2nd CAN frame: Fuel, OdometerHi, OdometerLo, Cruise Set Speed
  CanBuffer[0] = FUELr;
  CanBuffer[1] = ODOlo;
  CanBuffer[2] = ODOhi;
  CanBuffer[3] = CRSr;
  CanBuffer[4] = OILr;
  CanBuffer[5] = LBLr; // will be used for analog inputs later
  CanBuffer[6] = RBLr; // will be used for analog inputs later
  CanBuffer[7] = HBr; // will be used for analog inputs later
  
  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3201, CanBuffer);
}


void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  Serial.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial.write(frameData, 8);
}
