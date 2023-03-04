#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <stdio.h>
//#include <WebSocketsClient.h>

WiFiClient client;

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

void getMessage(int len){
//    char buffer[len];

    if(client.available()){
       while(i<(len-1)){
          char c = client.read();
          buffer[i] = c;
          i++;
       }
       i=0;
       Serial.println(buffer);
    }

//    return buffer;
}

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

  // FSM
  switch (state){

    case 'I': // IDLE STATE
      Serial.printf("[FSM] IDLE STATE \n");
      getMessage(4);
      message = buffer;
      if(message.compareTo("node")==0){
        state = 'S'; // go to send_state
        break;
      }
        
      else if(message.compareTo("path")==0){
        state = 'A'; //go to path allocation state
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
      getMessage(40);
      packet = buffer;
      // TODO - add code for converting received data 
      parsedata(packet);
      state = 'R'; // go to respond check state
      break;

    case 'R': // RESPOND_CHECK STATE
      Serial.printf("[FSM] RESPOND_CHECK STATE \n");
      getMessage(4);
      message = buffer;
      if(message.compareTo("good")==0){
        state = 'I'; // go back to idle
        break;
      }
        
      else if(message.compareTo("nope")==0){
        state = 'A'; //go back to allocation state
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
