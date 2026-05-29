#include "PacketPush.h"
#include <Arduino.h>

void sendSerialPacket(const SensorPacket& data) {
    Serial.print('S');
    
    // 1. FL: Front Left (VL índice 0)
    Serial.print(data.vl[0]);
    Serial.print(',');
    
    // 2. FC: Front Center (US índice 0)
    Serial.print(data.us[0]);
    Serial.print(',');
    
    // 3. FR: Front Right (VL índice 1)
    Serial.print(data.vl[1]);
    Serial.print(',');
    
    // 4. BR: Back Right (VL índice 2)
    Serial.print(data.vl[2]);
    Serial.print(',');
    
    // 5. BC: Back Center (US índice 1)
    Serial.print(data.us[1]);
    Serial.print(',');
    
    // 6. BL: Back Left (VL índice 3)
    Serial.print(data.vl[3]);
    
    Serial.println('E');
}