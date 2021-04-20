#include <Arduino.h>
#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <neotimer.h>
#include <SerialCommands.h>
// This defines ssid and password for the wifi configuration
#include "c:\Users\jesse\Documents\Arduino\config.h"

Neotimer mbtimer = Neotimer(500);


// instantiate ModbusMaster object
ModbusMaster node;

HardwareSerial Mb(1);

uint8_t result = 0, running = 0,direction = 0,ready = 0,fault =  0; 

void readReg(uint16_t addr,uint16_t &val){
  //ensure we wait a bit
  delay(10);
  result = node.readHoldingRegisters(addr,1);
  if(result == node.ku8MBSuccess){
    val = node.getResponseBuffer(0);
    Serial.print("readReg bin");
    Serial.print(val,BIN);
    Serial.println();
  }else{
    Serial.printf("could not read addr: %d\n",val);
  }
}


uint16_t current = 0;
uint16_t frequency_command = 0;
uint16_t output_frequency = 0;
uint16_t output_voltage = 0;

void read_monitor( ){
  // hack to avoid swamping the controller
  delay(10);

  uint16_t  current = 0;
  result = node.readHoldingRegisters(0x2527, 1);
 
  // If we have data to read.
  if (result == node.ku8MBSuccess) { 
    current = node.getResponseBuffer(0);
  }else{
    Serial.println("\t\tInvalid result: current");
  }
  readReg(0x2527,frequency_command);
  readReg(0x2524,output_frequency);
  readReg(0x2525,output_voltage);

  Serial.printf("\tCurrent: %d Frequency Command: %d Out Frequency: %d Out Voltage %d\n",current,frequency_command,output_frequency,output_voltage);
}


bool read_status() {
   
  uint16_t buff = 0;

  result = node.readHoldingRegisters(0x2520 , 1);
  bool status = false;
  // If we have data to read.
  if (result == node.ku8MBSuccess) { 
    buff = node.getResponseBuffer(0);
    running = bitRead(buff,0);
    direction = bitRead(buff,1);
    ready = bitRead(buff,2);
    fault = bitRead(buff,3);
    Serial.printf("\t\tRunning: %d Direction: %d Ready: %d fault %d\n",running,direction,ready,fault);
    if(ready == 1){
      status = true;
    }
  }else{
    Serial.println("\t\tInvalid result: speed");
  }
  return status;
}

void preTransmission()
{
  
  Mb.flush();
  // this isn't needed on this HW-0519 modbus to ttl converter
  //digitalWrite(SSerialTxControl, HIGH);
  Serial.print(".");
}

void postTransmission()
{
  //delay(10);
  // this isn't needed on this HW-0519 modbus to ttl converter
  //digitalWrite(SSerialTxControl, LOW);
  Serial.print(",");
}

// setup serial commands
char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

// handle unknown command
void cmd_unrecognized(SerialCommands* sender, const char* cmd)
{
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

uint16_t writeRegisterBit(uint16_t regNum, int bitNum, bool bitValue) {
    uint16_t value;
    readReg(regNum, value);
    Serial.print("\n\t got: 0x2501: ");
    Serial.println(value,BIN);
    value &= ~(1 << bitNum);
    value |= (bitValue << bitNum);
    return value;
}

void op_signal(uint16_t the_bit, bool val){

  // This is not correct, we need to write the bit and the bit value not a whole register
  //result = node.writeSingleRegister(0x2501,c);
  uint16_t new_value = writeRegisterBit(0x2501,the_bit,val);
  result = node.writeSingleRegister(0x2501,new_value);
  if(result == node.ku8MBSuccess){
    //sender->GetSerial()->println("cmd start succeded");
    Serial.println("cmd good");
  }else{
    //sender->GetSerial()->println("cmd start failed");
    Serial.println("cmd bad");
    Serial.println(result,BIN);
  }
  node.clearTransmitBuffer();
}

void cmd_start(SerialCommands* sender){
  op_signal(0,true);
}
SerialCommand cmd_start_("+",cmd_start,true);

void cmd_stop(SerialCommands* sender){
  op_signal(0,false);
}
SerialCommand cmd_stop_("-",cmd_stop,true);

void cmd_f(SerialCommands* sender){
  // send frequency to vfd
  char* f_str = sender->Next();
  if(f_str == NULL){
    Serial.println("Error: need to add the frequency");
    return;
  }
  uint16_t f_val = atoi(f_str);
  Serial.printf("cmd_f got: %d",f_val);
  //result = node.writeSingleRegister(0x2502,0x2710);
  result = node.writeSingleRegister(0x2502,f_val);
  if(result == node.ku8MBSuccess){
    //sender->GetSerial()->println("cmd start succeded");
    Serial.println("cmd good");
  }else{
    //sender->GetSerial()->println("cmd start failed");
    Serial.println("cmd bad");
    Serial.println(result,BIN);
  }
  node.clearTransmitBuffer();
}
SerialCommand cmd_f_("F",cmd_f);


void setup_ota(){
    WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

}


void setup() {
  Serial.begin(115200);
  Serial.begin(115200);
  Serial.println("Booting");

  // setup OTA
  setup_ota();

  // Modbus stuff
  
  // startup uart1 on esp32
  Mb.begin(9600,SERIAL_8N1,16,17);
  Serial.println("Hello world, now listening"); 
  // communicate with Modbus slave ID 1 over Serial device Mb 
  node.begin(1, Mb);
  
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // setup serial commands
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&cmd_start_);
  serial_commands_.AddCommand(&cmd_stop_);
  serial_commands_.AddCommand(&cmd_f_ );
}

void loop() {
  serial_commands_.ReadSerial();
  ArduinoOTA.handle();
  if(mbtimer.repeat()){
    if(read_status()){
      read_monitor();
    }else{
      Serial.println("not ready or communication error");
    }
  }
}
