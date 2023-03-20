#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <Hash.h>
#include <stdio.h>
#include "network_defines.hpp"
#include <iostream>
#include <string.h>

WebSocketsClient webSocket;

String message;
String *packet;
char buffer[100]; 


//node_packet = &node;
//path_packet = &path;
//diag_packet = &diag;


#define USE_SERIAL Serial

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
  String *pdata = &data;
  //use client.print() to send a string to server


void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  packet_node_state *nodePacketPtr, nodePacket;
  packet_path_point *pathPacketPtr, pathPacket;
  diagnostic_node_state *diagPacketPtr, diagPacket;
  
  switch(type) {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      break;
      
    case WStype_CONNECTED: {
      USE_SERIAL.printf("[WSc] Connected!");
      // send message to server when Connected
      webSocket.sendTXT("Client Connected!");
      //webSocket.sendBIN(pdata, sizeof(data));
    }
      break;
      
    case WStype_TEXT:
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);
      // send message to server
      USE_SERIAL.println((char*)payload);
      webSocket.sendTXT(payload);

      
      if(payload[0] == '2'){ // path packet
//        USE_SERIAL.printf("Index Worked! \n");
//        USE_SERIAL.println((char)payload[0]); // 2
//        USE_SERIAL.println((char)payload[1]); // 2
//        payload++;
//        USE_SERIAL.println((char)payload[0]); // 1
//        USE_SERIAL.printf("test: ");
//        USE_SERIAL.println(sizeof(packet_path_point)); // 40
//        USE_SERIAL.println(sizeof(path)); // 40
//        USE_SERIAL.println(sizeof(payload)); // 4 - bc its just a uint8 pointer
//        message = converter(payload); //convert payload to a string
//        message = message.substring(1);
//        USE_SERIAL.println(sizeof(message)); // 12 - not sure why
//        *packet = message.substring(1);
        payload++; // increment pointer to get rid of the pcket code digit
        buff_from_packet(&pathPacketPtr,payload,sizeof(packet_path_point));
        USE_SERIAL.println(pathPacket.x);     
        USE_SERIAL.printf("x: %f",pathPacket.x); // check if values copied in correctly
//        USE_SERIAL.printf("xd: %d \n",nodePacket.x); // check if values copied in correctly
//        USE_SERIAL.print(nodePacket.x); // check if values copied in correctly
//        USE_SERIAL.printf("y: %f \n",nodePacket.y); // check if values copied in correctly
//        USE_SERIAL.printf("ts_ms: %f \n",nodePacket.ts_ms); // check if values copied in correctly
//        USE_SERIAL.printf("y: %f",pathPacketPtr->y);
        webSocket.sendTXT("done");
      }
      if(payload[0] == '1'){ // send current node state packet
//        buff_from_packet((void*)node_packet,payload,sizeof(node_packet));
        webSocket.sendTXT("done");
      }
      if(payload[0] == '6'){ // send diagnostic node state packet
//        buff_from_packet((void*)diag_packet,payload,sizeof(diag_packet));
        webSocket.sendTXT("done");
      }

      
      break;
      
    case WStype_BIN:
      USE_SERIAL.printf("[WSc] get binary length: %s\n", String(length));
      hexdump(payload, length); //  print out the contents of a buffer (to serial) in hexadecimal format. The function takes in two arguments - the first is a pointer to the buffer that contains the data to be printed, and the second is the length of the buffer.
      uint8_t firstByte = *payload;
      Serial.println(" ");
      Serial.print("First byte: ");
      Serial.println(firstByte, HEX);


      if(payload[0] == '2'){ // path packet
//      parsedata(payload); // may need to index payload OR ignore the first bit in parse function
        payload++; // increment the pointer to skip the packet code when copying into buffer
        USE_SERIAL.printf("Binary Index Worked! \n");
//        buff_from_packet(&pathPacketPtr,payload,sizeof(packet_path_point));
        message = converter(payload); //convert payload to a string
//        path_parsedata(message, pathPacketPtr);
        pathPacket.x = message.substring(0,8).toDouble();
        pathPacket.y = message.substring(8,16).toDouble();
        pathPacket.v = message.substring(16,24).toDouble();
        pathPacket.theta = message.substring(24,32).toDouble();
        pathPacket.ts_ms = message.substring(32,40).toInt();
//        USE_SERIAL.printf("x: %f \n",pathPacket.x);
//        USE_SERIAL.printf("y: %f \n",pathPacket.y);
//        USE_SERIAL.printf("v: %f \n",pathPacket.v);
//        USE_SERIAL.printf("theta: %f \n",pathPacket.theta);
//        USE_SERIAL.printf("ts_ms: %i \n",pathPacket.ts_ms);
// TODO - HOW TO send data to pico from here
        payload--;
        USE_SERIAL.write(payload)
        webSocket.sendTXT("done");
        break;
      }
      if(payload[0] == '1'){ // send current node state packet
//        payload++; // increment the pointer to skip the packet code when copying into buffer
        // QUESTION - how will PICO know to send here? Do we just tell it over serial what we want?
        USE_SERIAL.readBytes(buffer,sizeof(packet_node_state)); // receive current node state packet from Pico
//        packet_from_buff((void*)node_packet,payload,sizeof(node_packet));
        webSocket.sendTXT(buffer);
        break;
      }
      if(payload[0] == '5'){ // send diagnostic node state packet
        payload++; // increment the pointer to skip the packet code when copying into buffer
        //        TODO - HOW TO GET DATA HERE FROM PICO
        message = converter(payload); //convert payload to a string
        diagPacket.ts_ms = message.substring(0,8).toInt();
        diagPacket.v_right = message.substring(8,16).toDouble();
        diagPacket.d_right = message.substring(16,24).toDouble();
        diagPacket.v_left = message.substring(24,32).toDouble();
        diagPacket.d_left = message.substring(32,40).toDouble();
//        buff_from_packet((void*)diag_packet,payload,sizeof(diag_packet));
        USE_SERIAL.readBytes(buffer,sizeof(packet_node_state));
        webSocket.sendTXT(buffer);
        break;
      }
      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
      
//    case WStype_PING:
//      // pong will be send automatically
//      USE_SERIAL.printf("[WSc] get ping\n");
//      break;
//    
//    case WStype_PONG:
//      // answer to a ping we send
//      USE_SERIAL.printf("[WSc] get pong\n");
//      break;
      
    }

}

void setup() {
    
  USE_SERIAL.begin(115200); //baud rate for serial communication

  USE_SERIAL.setDebugOutput(false);

  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();

  for(uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }
   WiFi.begin("JM Pixel 7 Pro", "Julian1499");
//  WiFi.begin("BELL864", "F7EAE5311517");
//  WiFi.begin("OilLock", "oillock-capstone");
//    WiFi.begin("Crocodile", "Crazyhorse65");


  while(WiFi.status() != WL_CONNECTED) { //wait until we are connected to the wifi
    delay(200);
    Serial.print(".");
  }

  // server address, port and URL
  webSocket.begin("192.168.244.243", 1234, "/");
//  webSocket.begin("192.168.2.23", 1234, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // use HTTP Basic Authorization this is optional remove if not needed
  //webSocket.setAuthorization("user", "Password");

  // try ever 5000 again if connection has failed
  webSocket.setReconnectInterval(5000);
  
  // start heartbeat (optional)
  // ping server every 15000 ms
  // expect pong from server within 3000 ms
  // consider connection disconnected if pong is not received 2 times
  //webSocket.enableHeartbeat(15000, 3000, 2);

}

void loop() {
  webSocket.loop(); //Keep websocket alive
}


void path_parsedata(String data_in, packet_path_point *path){
  // 4 doubles + 1 uint64 + 1 char + 4 double = 73 total bytes
  (*path).x = data_in.substring(0,8).toDouble();
  (*path).y = data_in.substring(8,16).toDouble();
  (*path).v = data_in.substring(16,24).toDouble();
  (*path).theta = data_in.substring(24,32).toDouble();
  (*path).ts_ms = data_in.substring(32,40).toInt();
}

String converter(uint8_t *str){
    return String((char *)str);
}
