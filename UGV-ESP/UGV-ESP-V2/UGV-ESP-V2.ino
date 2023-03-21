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
uint8_t node_state_buffer[sizeof(packet_node_state)+1];
uint8_t path_packet_buffer[sizeof(packet_path_point)+1];
uint8_t diag_packet_buffer[sizeof(diagnostic_node_state)+1];


#define USE_SERIAL Serial

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
//      USE_SERIAL.println((char*)payload);
//      webSocket.sendTXT(payload);
      break;
      
    case WStype_BIN:
//      USE_SERIAL.printf("[WSc] get binary length: %s\n", String(length));
      if(payload[0] == '1'){ // send current node state packet
        USE_SERIAL.write(1); // Send 1 over serial to Pico, so that PICO responds with current node state packet
        USE_SERIAL.readBytes(node_state_buffer,sizeof(packet_node_state));
        webSocket.sendBIN(node_state_buffer,sizeof(node_state_buffer)); // Forward the node state packet binary stream to the Server
        break;
      }
      
      if(payload[0] == '2'){ // path packet
        USE_SERIAL.write(payload, length); // write entire payload to Pico, it will detect the packet code and store the stream accordingly
        break;
      }

      if(payload[0] == '5'){ // send diagnostic node state packet
        USE_SERIAL.write(5); // Send 5 over serial to Pico, so that PICO responds with diagnostic state packet
        USE_SERIAL.readBytes(diag_packet_buffer,sizeof(diagnostic_node_state));
        webSocket.sendBIN(diag_packet_buffer,sizeof(diag_packet_buffer)); // Forward the diag state packet binary stream to the Server
        break;
      }

      break;
          
    }

}

void setup() {
    
  USE_SERIAL.begin(115200); //baud rate for serial communication

  USE_SERIAL.setDebugOutput(false);

//  USE_SERIAL.println();
//  USE_SERIAL.println();
//  USE_SERIAL.println();

  for(uint8_t t = 4; t > 0; t--) {
//    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
//    USE_SERIAL.flush();
    delay(1000);
  }
   WiFi.begin("JM Pixel 7 Pro", "Julian1499");
//  WiFi.begin("BELL864", "F7EAE5311517");
//  WiFi.begin("OilLock", "oillock-capstone");
//  WiFi.begin();


  while(WiFi.status() != WL_CONNECTED) { //wait until we are connected to the wifi
    delay(200);
//    Serial.print(".");
  }

  // server address, port and URL
  webSocket.begin("192.168.244.243", 1234, "/");
//  webSocket.begin("192.168.2.23", 1234, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);

  // try every 1000 again if connection has failed
  webSocket.setReconnectInterval(1000);
  
}

void loop() {
  webSocket.loop(); //Keep websocket alive
}


String converter(uint8_t *str){
    return String((char *)str);
}
