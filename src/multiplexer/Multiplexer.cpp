#include "Multiplexer.h"

#include "GlobalVars.h"
#include "status/Status.h"

#include <Wire.h>
#define TCA9548A_I2C_ADDRESS 0x70
#define TCA9548A_CHANNEL_0 0
#define TCA9548A_CHANNEL_1 1
#define TCA9548A_CHANNEL_2 2
#define TCA9548A_CHANNEL_3 3
#define TCA9548A_CHANNEL_4 4
#define TCA9548A_CHANNEL_5 5

void Multiplexer::setup()
{
    // Wire.begin();
    setTCAChannel(TCA9548A_CHANNEL_0);
}

// void Multiplexer::setTCAChannel(byte i)
// {
//     Wire.beginTransmission(TCA9548A_I2C_ADDRESS);
//     Wire.write(1 << i);
//     Wire.endTransmission();
// }

void Multiplexer::setTCAChannel(int i) {
  uint8_t channelBits = 1 << i;
  
  Wire.beginTransmission(0x70);  // TCA9548A I2C address
  Wire.write(channelBits);       // Write the control byte (channel selection)
  Wire.endTransmission();
}

