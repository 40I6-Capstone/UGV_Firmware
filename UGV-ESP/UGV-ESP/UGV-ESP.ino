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
// String *packet;
uint8_t node_state_buffer[sizeof(packet_node_state) + 1];
uint8_t path_packet_buffer[sizeof(packet_path_point) + 1];
uint8_t diag_packet_buffer[sizeof(packet_diag_node_state) + 1];


#define USE_SERIAL Serial

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  packet_node_state *nodePacketPtr, nodePacket;
  packet_path_point *pathPacketPtr, pathPacket;
  packet_diag_node_state *diagPacketPtr, diagPacket;

  switch (type) {
    case WStype_DISCONNECTED:
      //      USE_SERIAL.printf("[WSc] Disconnected!\n");
      break;

    case WStype_CONNECTED: {
        //        USE_SERIAL.print("Connected");
        // send message to server when Connected
        //      webSocket.sendTXT("Client Connected!");
        //webSocket.sendBIN(pdata, sizeof(data));
      }
      break;

    case WStype_TEXT:
      //      USE_SERIAL.print("[WSc] get text: %s\n", payload);
      // send message to server
      //      USE_SERIAL.println((char*)payload);
      //      webSocket.sendTXT(payload);
      break;

    case WStype_BIN:
      //      USE_SERIAL.printf("[WSc] get binary length: %s\n", String(length));
      if (payload[0] == '1') { // send current node state packet
        USE_SERIAL.write(1); // Send 1 over serial to Pico, so that PICO responds with current node state packet
        USE_SERIAL.readBytes(node_state_buffer, sizeof(packet_node_state));
        webSocket.sendBIN(node_state_buffer, sizeof(packet_node_state)); // Forward the node state packet binary stream to the Server
        break;
      }

      if (payload[0] == '2') { // path packet
        // USE_SERIAL.write(payload, length);
        USE_SERIAL.write(payload, sizeof(packet_path_point)); // write entire payload to Pico, it will detect the packet code and store the stream accordingly
        // TODO - ADD acknowledgement?
        break;
      }

      if (payload[0] == '3') { // STOP
        USE_SERIAL.write(3);
        // TODO - ADD acknowledgement?
        break;
      }

      if (payload[0] == '4') { // GO
        USE_SERIAL.write(4);
        // TODO - ADD acknowledgement?
        break;
      }

      if (payload[0] == '5') { // send diagnostic node state packet
        USE_SERIAL.write(5); // Send 5 over serial to Pico, so that PICO responds with diagnostic state packet
        USE_SERIAL.readBytes(diag_packet_buffer, sizeof(packet_diag_node_state));
        webSocket.sendBIN(diag_packet_buffer, sizeof(packet_diag_node_state)); // Forward the diag state packet binary stream to the Server
        break;
      }

      if (payload[0] == '6'){
        USE_SERIAL.write(6);
        USE_SERIAL.write(payload,sizeof(packet_rebase));
      }

      break;

  }

}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  USE_SERIAL.begin(115200); //baud rate for serial communication

  USE_SERIAL.setDebugOutput(false);

  //  USE_SERIAL.println();
  //  USE_SERIAL.println();
  //  USE_SERIAL.println();

  //  for(uint8_t t = 4; t > 0; t--) {
  ////    USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
  ////    USE_SERIAL.flush();
  //    delay(1000);
  //  }
  //   WiFi.begin("JM Pixel 7 Pro", "Julian1499");
  //  WiFi.begin("BELL864", "F7EAE5311517");
  //    WiFi.begin("OilLock", "oillock-capstone");
  //  USE_SERIAL.print("Trying to connect to WiFi...\n");
  WiFi.begin("Pixel_Shaq", "shaq1234");
  while ( WiFi.status() != WL_CONNECTED ) {
    //    USE_SERIAL.print("WiFi failed, trying again\n");
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
  }
  //  USE_SERIAL.print("Connected to WIFI at ");
  //  USE_SERIAL.println(WiFi.localIP());
  digitalWrite(LED_BUILTIN, LOW);


  //  WiFi.begin();

  //
  //  while(WiFi.status() != WL_CONNECTED) { //wait until we are connected to the wifi
  //    delay(200);
  //    Serial.print(".");
  //  }

  // server address, port and URL
  //  webSocket.begin("192.168.244.243", 1234, "/");
  //  webSocket.begin("192.168.2.23", 1234, "/");
  webSocket.begin("192.168.0.95", 63734, "/");



  // event handler
  webSocket.onEvent(webSocketEvent);

  // try every 1000 again if connection has failed
  webSocket.setReconnectInterval(1000);

  //  USE_SERIAL.print("ESP BOOTED\n");



}


void loop() {
  // webSocket.loop(); //Keep websocket alive?
  static unsigned long ts =  millis();
  static double xVal = 0.0;

  if ((millis() - ts) > 1000UL) {
    // USE_SERIAL.write(testChar++);
    // if(testChar > 'z') testChar = 'a';
    
    packet_path_point testPoint= {
      .x = xVal,
      .y = 0
    };
    xVal += 1.0;

    char * testPacket = (char*)malloc(sizeof(packet_path_point));
    buff_from_packet(testPacket, &testPoint, sizeof(packet_path_point));
    USE_SERIAL.write(testPacket, sizeof(packet_path_point));
    ts = millis();
  }

}
