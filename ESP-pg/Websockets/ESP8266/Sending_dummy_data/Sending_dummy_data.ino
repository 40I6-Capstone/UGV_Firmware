#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsClient.h>
#include <Hash.h>
#include <ArduinoOTA.h>

ESP8266WiFiMulti WiFiMulti; // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
WebSocketsClient webSocket; //create a websocket 


const unsigned long duration = 5000;
unsigned long timeLatestCycle = 0;
boolean ledStatus;

//IP and port for websocket connection, these will change based on your machine / network
//String IP = "192.168.0.46";
//int PORT = 7890;
// char connection[15]; //for some reason having this uncommented breaks the watchdog timer and causes the board to repeatedly reboot
int data = 0;


void setup() {
	Serial.begin(115200); //baud rate for serial communication

	//Serial.setDebugOutput(true);
	Serial.setDebugOutput(true);
  delay(1000);
	Serial.println();
	Serial.println("Start WebSocket Client");
	Serial.println();

  pinMode(A0,INPUT);
	
	//ota(); //function to Upload sketch over network to Arduino board with WiFi or Ethernet librarie we dont need this

	for(uint8_t t = 4; t > 0; t--) { //wait to begin connection & transmission
		Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
		Serial.flush(); //Waits for the transmission of outgoing serial data to complete
		delay(1000);
	}

	WiFiMulti.addAP("COGECO-CD3A0", "DF9CECOGECO"); //ssid and password for wifi network

	//WiFi.disconnect();
	while(WiFiMulti.run() != WL_CONNECTED) { //wait until we are connected to the wifi
		delay(100);
	}

	// server address, port and URL
	webSocket.begin("192.168.0.46", 7890, "/");//connect to websocket server - This IP will change based on your machine

	// event handler
	webSocket.onEvent(webSocketEvent); //fires any time anything happens with the websocket server.

	// try again every 5 seconds if connection has failed
	webSocket.setReconnectInterval(5000);

  // Serial.println("Connection established!");
  // sprintf(connection,"Connected to IP: 192.168.0.46 and Port: %i",PORT); 
  // Serial.println(connection);
  // Serial.print("ESP8266 IP address:\t");
  // Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
}

void loop() {
	//ArduinoOTA.handle();
	
	webSocket.loop();

	if(Cycle()){
	  char p[32];
	  sprintf(p, "Test Data %i", data);
	  webSocket.sendTXT(p);
	}
}

boolean Cycle(){ //ensures that the data is only sent every 'duration' seconds
	boolean cumple = false;
	unsigned long time = millis(); // millis() Returns the number of milliseconds passed since the Arduino board began running the current program
	
	if(millis() - timeLatestCycle >= duration){ // run the cycle every 'duration' seconds
	  timeLatestCycle = millis();
	  cumple = true;
    data += 1; //increment test data number
	}
	return cumple;
}


void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) { //fires any time anything happens with the websocket server.

	switch(type) {
	  case WStype_DISCONNECTED:
	    Serial.printf("[WSc] Disconnected!\n");
	    break;
	  case WStype_CONNECTED: {
	    Serial.printf("[WSc] Connected to url: %s\n", payload);

	    // send message to server when Connected
	    webSocket.sendTXT("Connected");
	    break;
	  }
	  case WStype_TEXT:
	    Serial.printf("[WSc] get text: %s\n", payload);

	    // send message to server
	    //webSocket.sendTXT("I, client, received messsge and send this back to server");
	    break;
	  case WStype_BIN:
	    Serial.printf("[WSc] get binary length: %u\n", length);
	    hexdump(payload, length);

	    // send data to server
	    // webSocket.sendBIN(payload, length);
	    break;
	}

}

// void ota(){

// // Port defaults to 8266
// // ArduinoOTA.setPort(8266);

// // Hostname defaults to esp8266-[ChipID]
// // ArduinoOTA.setHostname("myesp8266");

// // No authentication by default
// // ArduinoOTA.setPassword((const char *)"123");

// 	ArduinoOTA.onStart([]() {
// 	  Serial.println("Start");
// 	});
// 	ArduinoOTA.onEnd([]() {
// 	  Serial.println("\nEnd");
// 	});
// 	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
// 	  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
// 	});
// 	ArduinoOTA.onError([](ota_error_t error) {
// 	  Serial.printf("Error[%u]: ", error);
// 	  if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
// 	  else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
// 	  else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
// 	  else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
// 	  else if (error == OTA_END_ERROR) Serial.println("End Failed");
// 	});
// 	ArduinoOTA.begin();
// 	Serial.println("OTA ready");
// 	//Serial.print("IP address: ");
// 	//Serial.println(WiFi.localIP());
// }