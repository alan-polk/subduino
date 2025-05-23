// =========================================================
// Program Name: SSMReadDashInfo
// Current Version Arthor: Alan Polk
// https://github.com/alan-polk/subduino
// ==========================================================
// This started as a fork of Matt Prophet's  Subduino Project.  
// But as someone who teaching themselves C++ as part of the project
// I found the arrays, pointers, and memcpy hard to reverse engineer and follow
// I've simplied the code down quite a bit so that each call is spelled
// out with each parameters address easily identifiable.  The code
// in this program will not be as efficient, but should be easier
// for a begginer to work with.
// ==========================================================

#include <SoftwareSerial.h>  // For SSM Line
int ProgRev = 15;

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
// Command/Response byte - 0xA0 is block read (not available for live data), 0xA8 is address read, 0xB0 is block write, 0xB8 is address write
// Continous/Single byte - 0x00 is for one time read/write, 0x01 will cause the ECU to continously respond to a query until interupted
// data… - these will be the address location we want to read on a single read, or the starting address and number of bytes on a block read
// Checksum byte - To manually calcualte the checksum, add up all of the values in your packet, and take the 8 least signifigant bits
// -----------------------------------------------
// The checksum is automatically calcualted and you can put any value you want into the packet definition.
// For a better explination on checksum look here:
// http://www.romraider.com/forum/viewtopic.php?f=7&t=10122&start=15
// =========================================================

// Define variable for debug messaging.  Comment out to turn off messaging
#define Debug;

/// Define variables to track scan times 
unsigned long Time;
byte LastRequest;
byte LastResponse;
byte LastBroadcast;

// Define variables for holding values
#ifdef Debug
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
  uint8_t MAFhi;
  uint8_t MAFlo;
  uint8_t MAPr;
  uint8_t IGNr;
  uint8_t THROTr;
#endif

uint8_t OILr; // Oil Pressure From Analog in on Arduino
bool LBLr; // Left Blinker from Digital in on Arduino
bool RBLr; // Right Blinker from Digital in on Arduino
bool HBr; // High Beams from Digital in on Arduino
bool MALFr; // Check Engine Light from Digital in on Arduino
bool CruiseOnr; // Cruise on will be calculated from SSM Data
bool eBrakeStat; // eBrake status from brake controller
int SpeedSignal; //speed signal to brake controller


// Defin pin modes for physical IO
int LBLPin = 43; //Left Blinker on Pin 43
int RBLPin = 45; // Right Blinker on Pin 45
int HBPin = 47; // High Beams on Pin 47
int MALFPin = 49; // Check Engine (MALF) on Pin 49
int OILPin = A8; // Oil Pressure on Pin 8
int eBrakePin = 41; // eBrake status on pin 51
int SpeedPin  = A5; // Speed signal to brake controller on pin A5



// Define a variable for read mode (continous or single)
byte SSMResponseType = 0x00; // Change to 0x01 for continous response, 0x00 for single response

// Define the Read Packet.

// We are reading requesting 18 values, at 3 bytes per address, there are 6 overhead bytes, 1 checksum byte. Total of 63 bytes
byte RequestPacketSize = 63;  
uint8_t ResponsePacketSize = 18;

// Build the RequestPacket
uint8_t pollECUPacket[63] = {
  0x80, // header
  0x10, // to ECU
  0xF0, // from diagnostics tool
  0x16, // payload size  /// must be updated if you add or remove parameters
  0xA8, // read predefined addresses
  0x00, // One Response
  0x00, // RPM lo 
  0x00, // RPM lo 
  0x0E, // RPM lo Response byte 0
  0x00, // RPM hi
  0x00, // RPM hi
  0x0F, // RPM hi Response byte 1
  0x00, // Boost
  0x00, // Boost
  0x24, // Boost Response byte 2
  0x00, // Speed
  0x00, // Speed
  0x10, // Speed Response byte 3
  0x00, // Coolant
  0x00, // Coolant
  0x08, // Coolant Response byte 4
  0x00, // Volts
  0x00, // Volts
  0x1C, // Volts Response byte 5
  0x00, // IAT
  0x00, // IAT
  0x12, // IAT Response byte 6
  0x00, // Cruise Speed
  0xFF, // Cruise Speed
  0x0B, // Cruise Speed Response byte 7
  0x00, // AFR
  0x00, // AFR
  0x46, // AFR Response byte 8
  0x00, // Fuel Level
  0x00, // Fuel Level
  0x2E, // Fuel Level Response byte 9
  0x00, // Gear
  0x00, // Gear
  0x4A, // Gear Response byte 10
  0x00, // Odometer lo
  0xFF, // Odometer lo
  0x0F, // Odometer lo Response byte 11
  0x00, // Odometer hi
  0xFF, // Odometer hi
  0x10, // Odometer hi Response byte 12
  0x00, // MAF lo
  0x00, // MAF lo
  0x13, //MAF lo Response byte 13
  0x00, //MAF hi
  0x00, //MAF hi
  0x14, //MAF hi Response byte 14
  0x00, // MAP
  0x00, // MAP
  0x0D, // MAP Response byte 15
  0x00, // Ignition Advance
  0x00, // Ignition Advance
  0x11, // Ignition Advance Response byte 16
  0x00, // Throttle Position
  0x00, // Throttle Position
  0x15, // Throttle Position Response byte 17
  0xCD};

// Define a buffer to put the data we recieve into.  We are reading 18 bytes
uint8_t ECUBuffer[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void setup() {
  Serial.begin(115200); //for diagnostics
  Serial1.begin(115200); // for TTL to Dash
  kLine.begin(4800); //SSM uses 4800 8N1 baud rate

  pinMode(LBLPin, INPUT); // Left Blinker will be 5+ when activated
  pinMode(RBLPin, INPUT); // Right Blinker will be 5+ when activated
  pinMode(HBPin, INPUT); // High Beams will be 5+ when activated
  pinMode(MALFPin, INPUT); //Check engine light will be 5+ when activated
  pinMode(eBrakePin, INPUT) ;// 5+ from brake controller when brake is active.
}
// =======================================================

void loop() {

  Serial.print("Program Version:"); Serial.println(ProgRev);
  
  Time = millis();
  #ifdef Debug
    Serial.print("Time: "); Serial.println(Time);
    Serial.print("LastRequest: "); Serial.println(LastRequest);
    Serial.print("LastResponse: "); Serial.println(LastResponse);
    Serial.print("LastBroadcast: "); Serial.println(LastBroadcast);
  #endif
  
// Get new SSM Data every 20 milliseconds
  if (Time - LastRequest > 20) {

    if(serialCallSSM(kLine, pollECUPacket, RequestPacketSize, ECUBuffer, ResponsePacketSize, 0) < 3)
   {
      LastRequest = millis();
      // if debug is turned on place all of the read data in variables for printing
      #ifdef Debug
        RPMhi = ECUBuffer[0];
        RPMlo = ECUBuffer[1];
        CTSr = ECUBuffer[4];
        BOOSTr = ECUBuffer[2];
        AFRr = ECUBuffer[8];
        VELr = ECUBuffer[3];
        FUELr = ECUBuffer[9];
        IATr = ECUBuffer[6];
        BATr = ECUBuffer[5];
        GEARr = ECUBuffer[10];
        ODOhi = ECUBuffer[11];
        ODOlo = ECUBuffer[12];
        CRSr = ECUBuffer[7];
        MAFhi = ECUBuffer[13];
        MAFlo = ECUBuffer[14];
        MAPr = ECUBuffer[15];
        IGNr = ECUBuffer[16];
        THROTr = ECUBuffer[17];
        
        float RPM = word(RPMhi, RPMlo)/4;
        float AFR = (AFRr/128)*14.7;
        float BOOST = (BOOSTr - 128)*.145;
        float CTS = ((CTSr-40)*1.8)+32;
        float VEL = VELr * 0.621371192;
        float FUEL = FUELr*0.02;
        float IAT = 32+1.8*(IATr-40);
        float BAT = BATr*0.08;
        uint8_t GEAR = GEARr + 1;
        float ODO = word(ODOhi, ODOlo)*1.242742384;
        float CRS = (CRSr * 0.621371192);
        float MAF =word(MAFhi, MAFlo)/100;
        float MAP = MAPr * 0.1451;
        float IGN = (IGNr-128)/2;
        float THROT = THROTr * 0.3922;
        
        Serial.print("RPM: ");
        Serial.println(RPM);
        Serial.print("Temp: ");
        Serial.println(CTS);
        Serial.print("BOOST): ");
        Serial.println(BOOST);
        Serial.print("AFR: ");
        Serial.println(AFR);
        Serial.print("Velocity: ");
        Serial.println(VEL);
        Serial.print("Fuel Level: ");
        Serial.println(FUEL);
        Serial.print("Intake Air Temp: ");
        Serial.println(IAT);
        Serial.print("Battery Voltage: ");
        Serial.println(BAT);
        Serial.print("Gear: ");
        Serial.println(GEAR);
        Serial.print("ODOhi / ODOlo: ");
        Serial.print(ODOhi);
        Serial.print(F(" / "));
        Serial.println(ODOlo);
        Serial.print("Odometer: ");
        Serial.println(ODO);
        Serial.print("Cruise Speed: ");
        Serial.println(CRSr);
        Serial.print("MAF: ");
        Serial.println(MAF);
        Serial.print("MAP: ");
        Serial.println(MAP);
        Serial.print("IGN: ");
        Serial.println(IGN);
        Serial.print("THROT: ");
        Serial.println(THROT);
      #endif
   }
  }

// Go read the ardiuno IO
  ReadIOPins();
  
// Send Data to CanBus every 23 millisecons
if (Time - LastBroadcast > 23) {
  LastBroadcast = millis();
  BuildCanFrames();
  }


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
        //ECU Response Command (not relevant)
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
  dataBuffer[5] = SSMResponseType; // single or continuous polling
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
  
  if(!success && attempts < 3)
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

  // build 1st CAN frame: RPMhi, RPMlo, Left Blinker, Right Blinker, Boost, Speed, Coolant, Oil Pressure
  CanBuffer[0] = ECUBuffer[1];
  CanBuffer[1] = ECUBuffer[0];
  CanBuffer[2] = LBLr;
  CanBuffer[3] = RBLr;
  CanBuffer[4] = ECUBuffer[2];
  CanBuffer[5] = ECUBuffer[3];
  CanBuffer[6] = ECUBuffer[4];
  CanBuffer[7] = 0; // Moved oil pressure to 4th frame, appears to be issues with slot 7

  // write first CAN frame to serial
  SendCANFrameToSerial(3200, CanBuffer);

  // build 2nd CAN frame: Volts, IAT, Cruise Speed, AFR, Fuel Level, Gear, Cruise Light, Hi Beams
  CanBuffer[0] = ECUBuffer[5];
  CanBuffer[1] = ECUBuffer[6];
  CanBuffer[2] = ECUBuffer[7];
  CanBuffer[3] = ECUBuffer[8];
  CanBuffer[4] = ECUBuffer[9];
  CanBuffer[5] = ECUBuffer[10];
  CanBuffer[6] = CruiseOnr;
  CanBuffer[7] = 0; // Moved High Beams to 4th frame, appeasr to be issues with slot 7
  
  // write 2nd CAN frame to serial
  SendCANFrameToSerial(3201, CanBuffer);

 // build 3rd CAN frame: ODOhi, ODOlo, Check Engine, MAFhi, MAFlo, MAP, IgnTiming, Throttle
  CanBuffer[0] = ECUBuffer[11];
  CanBuffer[1] = ECUBuffer[12];
  CanBuffer[2] = MALFr;
  CanBuffer[3] = ECUBuffer[14];
  CanBuffer[4] = ECUBuffer[13];
  CanBuffer[5] = ECUBuffer[15];
  CanBuffer[6] = ECUBuffer[16];
  CanBuffer[7] = ECUBuffer[17];

  // write 3rd CAN frame to serial
  SendCANFrameToSerial(3202, CanBuffer);

 // build 4th CAN frame: HighBeams, ParkingBrake, OilPressure
  CanBuffer[0] = HBr;
  CanBuffer[1] = eBrakeStat;
  CanBuffer[2] = OILr;
  CanBuffer[3] = 0;
  CanBuffer[4] = 0;
  CanBuffer[5] = 0;
  CanBuffer[6] = 0;
  CanBuffer[7] = 0;

  // write 4th CAN frame to serial
  SendCANFrameToSerial(3203, CanBuffer);
}


void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial1.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  Serial1.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial1.write(frameData, 8);
}

void ReadIOPins(){
  LBLr = digitalRead(LBLPin);
  RBLr = digitalRead(RBLPin);
  HBr = digitalRead(HBPin);
  MALFr = digitalRead(MALFPin);
  OILr = analogRead(OILPin);
  eBrakeStat = digitalRead(eBrakePin);

  #ifdef Debug
        int LBL = LBLr;
        int RBL = RBLr;
        int HB = HBr;
        int MALF = MALFr;
        int OIL = OILr;
        int eBrakeStatus = eBrakeStat;
        
        Serial.print("LBL: ");
        Serial.println(LBL);
        Serial.print("RBL: ");
        Serial.println(RBL);
        Serial.print("HB: ");
        Serial.println(HB);
        Serial.print("MALF: ");
        Serial.println(MALF);
        Serial.print("OIL: ");
        Serial.println(OIL);
        Serial.print("eBrake: ");
        Serial.println(eBrakeStatus);
      #endif
}


 
