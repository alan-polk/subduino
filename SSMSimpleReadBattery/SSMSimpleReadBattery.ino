// A fork of Matt-Prophet's Subduino Project
// https://github.com/alan-polk/subduino

// This is a simple sketch to read the battery volrate
// form a subaru ECU via SSM using MC33660 chip
// Usful for testing that all hardware and wiring is correct
// When loaded you should be able to see your battery voltage in serial monitor


#include <SoftwareSerial.h>

SoftwareSerial kLine = SoftwareSerial(10, 11); //Rx, Tx


//the requests include the 3-byte address of the data byte that we want to read from ECU memory
//msg format = {header,to,from,# data bytes,read type flag(?),continuous response request flag,addr part 1,addr part 2, addr part 3,..., checksum}
uint8_t pollECU_BattVolt[10] = {128, 16, 240, 5, 168, 0, 0x00, 0x00, 0x1C, 0x49}; //response [X]
uint8_t ECUResponseBuffer[2] = {0, 0};//the longest number of bytes we get from the ECU in response to our queries is 2 bytes

void setup() {
  Serial.begin(115200); //for diagnostics
  kLine.begin(4800); //SSM uses 4800 8N1 baud rate
  
}

void loop() {
  
  //these are buffers for displayable data
  double parameter = 0;
  String unit = "";
  String title = "";


  //battery voltage
  if(serialCallSSM(kLine, pollECU_BattVolt, 10, ECUResponseBuffer, 2, 0) < 10)
  {
    //the battery voltage calc looks like: x*8/100 for units of Volts
    double volts = ECUResponseBuffer[0] * 8.00;
    volts = volts/100.00;
    Serial.print(F("Battery Voltage: "));
    Serial.println(volts);
   
    //update the buffers for displayable data
    parameter = volts;
    unit = F(" V");
    title = F("Battery voltage");
    delay(5000);  

}
  Serial.println();
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
        //this byte is data
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
  //this function needs to catch mistakes that I make in preparing 
  //the packets -> {128, 16, 240, #data, 168, cont. response flag, addr1,addr2,addr3,checksum} = 10 bytes, 5 of which are data
  dataBuffer[0] = 128;//begin packet
  dataBuffer[1] = 16;//to subaru ECU
  dataBuffer[2] = 240;//from diagnostic tool
  //number of data bytes include 3*(number of addresses), cont. response flag, and mode (address read)
  //this means that the header, to, from, #data, and checksum are not part.
  dataBuffer[3] = bufferSize - 5;//# data bytes
  dataBuffer[4] = 168;//single address read
  dataBuffer[5] = 0;//no continuous polling
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
  //this function performs the call and response routine for 
  //exchanging serial data with the Subaru ECU via SSM
  //it sends a message and awaits a response, resending the 
  //message if we reach timeout or if the checksum failed
  //it stops trying the data exchange if 10 failures occur and 
  //it outputs the number of attempts it failed to send data
  //input: handle to software serial pins
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
