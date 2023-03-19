#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <stdio.h>
#include "network_defines.hpp"
#include <WebSocketsClient.h>

//WiFiClient client;
WebSocketsClient client;

const unsigned long duration = 5000;
unsigned long timeLatestCycle = 0;
boolean ledStatus;

//const uint16_t port = 1234;
//const char *host = "10.0.0.187";


//  Example data for current node_state: (even though it is strings, when it is sent to the server, it is received as a byte array)
  String X = "2.980000";
  String Y = "103.5000";
  String Velocity = "5.145000";
  String Heading = "30.64000";
  String ts_ms = "10.00000";
  String State = "1";
  String x_exp = "2.980000";
  String y_exp = "103.5000";
  String velocity_exp = "5.145000";
  String heading_exp = "30.64000";
  
  String data = X + Y + Velocity + Heading + ts_ms + State + x_exp + y_exp + velocity_exp + heading_exp;
  //use client.print() to send a string to server

char state = 'I';
int i=0;
String message;
String packet;
char buffer[75]; //Actual amount is 73

//void getMessage(int len){
////    char buffer[len];
//
//       while(i<(len-1)){
//          char c = client.read();
//          buffer[i] = c;
//          i++;
//       }
//       i=0;
//       Serial.println(buffer);
//
////    return buffer;
//}

void setup() {
  Serial.begin(115200); //baud rate for serial communication

  Serial.setDebugOutput(true);
  delay(1000);
  Serial.println();
  pinMode(A0,INPUT);
 
  for(uint8_t t = 3; t > 0; t--) { //wait to begin connection & transmission
    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
    Serial.flush(); //Waits for the transmission of outgoing serial data to complete
    delay(1000);
  }

  Serial.printf("[SETUP] CONNECTING \n");
  
  WiFi.persistent(false); // do not allow wifi configuration to persist in flash memory
  WiFi.begin("BELL864","F7EAE5311517");
  
  while(WiFi.status() != WL_CONNECTED) { //wait until we are connected to the wifi
    delay(500);
    Serial.print(".");
  }
      
  Serial.printf("[SETUP] Connected to Wifi \n");

  state = 'I'; // set to idle state by default
        
}


void loop() {

//    check to make sure we are connected to the server otherwise wait and try again
  
    if (!client.connect("192.168.2.23", 1234))
    {
        Serial.println("Connection to host failed, Retrying...");
        delay(1000);
        return;
    }
    if (client){
      if(client.connected()){
        Serial.printf("Connected to Server \n");
      }
    }

  /*
  FSM Definitions:
  C - CONNECT STATE
  I - IDLE STATE
  S - SEND STATE
  A - PATH ALLOCATION STATE
  P - PATH RECEIVE STATE
  R - RESPOND_CHECK STATE
  */

//    packet_type = {  # dictionary for packet type
//        "who_am_i": '0',
//        "node": '1',
//        "path": '2'
//    }


  // FSM
  switch (state){

    case 'I': // IDLE STATE
      Serial.printf("[FSM] IDLE STATE \n");
      getMessage(1);
      message = buffer;
      if(message.compareTo("1")==0){
        state = 'S'; // go to send_state
        break;
      }
        
      else if(message.compareTo("2")==0){
        state = 'A'; //go to path allocation state
        break;
      }

      else {
        state = 'I';
        break;
      }

    case 'S': // SEND STATE
      Serial.printf("[FSM] SEND STATE \n");
      // TODO - add tx code to send current state to server
      client.print(data);
      state = 'I'; // go back to idle state
      break;

    case 'A': // PATH ALLOCATION STATE
      Serial.printf("[FSM] PATH ALLOCATION STATE \n");
      client.print("ready");
      state = 'P'; // go to path receive state
      break;

    case 'P': // PATH RECEIVE STATE
      Serial.printf("[FSM] PATH RECEIVE STATE \n");
      getMessage(40); //TODO - update this based on decided packet size
      packet = buffer;
      // TODO - add code for converting received data 
      parsedata(packet);
      packet_path_point path_packet;
      packet_from_buff(&path_packet,buffer,sizeof(buffer));
      state = 'R'; // go to respond check state
      break;

    case 'R': // RESPOND_CHECK STATE
      Serial.printf("[FSM] RESPOND_CHECK STATE \n");
      if(packet.length() == 40){ // TODO - Update this based on decided packet
        client.print("g"); // send g for good
        
        // TODO - Add code to transmit processed data over UART
        //Serial.print(path_packet);
        Serial.print(packet);
        state = 'I';
        break;
      }
      else{
        client.print("n"); // send n for nope
        state = 'A';
        break;
      }
      
      state = 'I'; // default state is idle
      break;

  }

}

void parsedata(String data_in){
  // 4 doubles + 1 uint64 + 1 char + 4 double = 73 total bytes
  X = data_in.substring(0,8);
  Y = data_in.substring(8,16);
  Velocity = data_in.substring(16,24);
  Heading = data_in.substring(24,32);
  ts_ms = data_in.substring(32,40);
  State = data_in.substring(40,41);
  x_exp = data_in.substring(41,49);
  y_exp = data_in.substring(49,57);
  velocity_exp = data_in.substring(57,65);
  heading_exp = data_in.substring(65,73);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

  switch(type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED: {
      USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

      // send message to server when Connected
      webSocket.sendTXT("Connected");
    }
      break;
    case WStype_TEXT:
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);

      // send message to server
      // webSocket.sendTXT("message here");
      break;
    case WStype_BIN:
      USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
      hexdump(payload, length);

      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
        case WStype_PING:
            // pong will be send automatically
            USE_SERIAL.printf("[WSc] get ping\n");
            break;
        case WStype_PONG:
            // answer to a ping we send
            USE_SERIAL.printf("[WSc] get pong\n");
            break;
    }

}
