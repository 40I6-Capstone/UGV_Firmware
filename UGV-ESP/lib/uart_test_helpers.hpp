#include "network_defines.h"

// Print out node state struct
void print_node_state(packet_node_state state)
{
    Serial.print(F("x: "));
    Serial.println(state.x);
    Serial.print(F("y: "));
    Serial.println(state.y);
    Serial.print(F("v: "));
    Serial.println(state.v);
    Serial.print(F("theta: "));
    Serial.println(state.theta);
    Serial.print(F("ts_ms: "));
    Serial.println(state.ts_ms);
    Serial.print(F("state: "));
    Serial.println(state.state);
}

// Test packet to send
packet_path_point test_path_point = 
{
    .x = 1.0,
    .y = 2.0,
    .v = 10.0,
    .theta = PI/2,
    .ts_ms = 4000
};