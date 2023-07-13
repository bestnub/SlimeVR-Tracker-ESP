/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2022 TheDevMinerTV

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "SensorManager.h"
#include <i2cscan.h>
#include "network/network.h"
#include "bno055sensor.h"
#include "bno080sensor.h"
#include "mpu9250sensor.h"
#include "mpu6050sensor.h"
#include "bmi160sensor.h"
#include "icm20948sensor.h"
#include "ErroneousSensor.h"
#include "consts.h"
#include "Multiplexer.h"
#include "Multiplexer.cpp"

namespace SlimeVR
{
    namespace Sensors
    {
        void SensorManager::setup()
        {
            uint8_t firstIMUAddress = 0;
            uint8_t secondIMUAddress = 0;
            uint8_t n3IMUAddress = 0;
            uint8_t n4IMUAddress = 0;
            uint8_t n5IMUAddress = 0;
            uint8_t n6IMUAddress = 0;
            uint8_t n7IMUAddress = 0;
            uint8_t n8IMUAddress = 0;
            uint8_t n9IMUAddress = 0;
            uint8_t n10IMUAddress = 0;
            uint8_t n11IMUAddress = 0;

            {
                Multiplexer::setTCAChannel(0);
#if IMU == IMU_BNO080 || IMU == IMU_BNO085 || IMU == IMU_BNO086
                firstIMUAddress = 0x4A; // BNO Address 1
#elif IMU == IMU_BNO055
                firstIMUAddress = I2CSCAN::pickDevice(0x29, 0x28, true);
#elif IMU == IMU_MPU9250 || IMU == IMU_BMI160 || IMU == IMU_MPU6500 || IMU == IMU_MPU6050 || IMU == IMU_ICM20948
                firstIMUAddress = I2CSCAN::pickDevice(0x68, 0x69, true);
#else
#error Unsupported primary IMU
#endif

                if (!I2CSCAN::isI2CExist(firstIMUAddress))
                {
                    m_Sensor1 = new ErroneousSensor(0, IMU);
                }
                else
                {
                    m_Logger.trace("Primary IMU found at address 0x%02X", firstIMUAddress);

#if IMU == IMU_BNO080 || IMU == IMU_BNO085 || IMU == IMU_BNO086
                    m_Sensor1 = new BNO080Sensor(0, IMU, firstIMUAddress, IMU_ROTATION, PIN_IMU_INT);
#elif IMU == IMU_BNO055
                    m_Sensor1 = new BNO055Sensor(0, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_MPU9250
                    m_Sensor1 = new MPU9250Sensor(0, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_BMI160
                    m_Sensor1 = new BMI160Sensor(0, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_MPU6500 || IMU == IMU_MPU6050
                    m_Sensor1 = new MPU6050Sensor(0, IMU, firstIMUAddress, IMU_ROTATION);
#elif IMU == IMU_ICM20948
                    m_Sensor1 = new ICM20948Sensor(0, firstIMUAddress, IMU_ROTATION);
#endif
                }

                m_Sensor1->motionSetup();
            }

            {
#if SECOND_IMU == IMU_BNO080 || SECOND_IMU == IMU_BNO085 || SECOND_IMU == IMU_BNO086
                secondIMUAddress = 0x4B; // BNO Address 2
#elif SECOND_IMU == IMU_BNO055
                secondIMUAddress = I2CSCAN::pickDevice(0x28, 0x29, false);
#elif SECOND_IMU == IMU_MPU9250 || SECOND_IMU == IMU_BMI160 || SECOND_IMU == IMU_MPU6500 || SECOND_IMU == IMU_MPU6050 || SECOND_IMU == IMU_ICM20948
                secondIMUAddress = I2CSCAN::pickDevice(0x69, 0x68, false);
#else
#error Unsupported secondary IMU
#endif

                if (secondIMUAddress == firstIMUAddress)
                {
                    m_Logger.debug("No secondary IMU connected");
                }
                else if (!I2CSCAN::isI2CExist(secondIMUAddress))
                {
                    m_Sensor2 = new ErroneousSensor(1, SECOND_IMU);
                }
                else
                {
                    m_Logger.trace("Secondary IMU found at address 0x%02X", secondIMUAddress);

#if SECOND_IMU == IMU_BNO080 || SECOND_IMU == IMU_BNO085 || SECOND_IMU == IMU_BNO086
                    m_Sensor2 = new BNO080Sensor(1, SECOND_IMU, secondIMUAddress, SECOND_IMU_ROTATION, PIN_IMU_INT_2);
#elif SECOND_IMU == IMU_BNO055
                    m_Sensor2 = new BNO055Sensor(1, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_MPU9250
                    m_Sensor2 = new MPU9250Sensor(1, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_BMI160
                    m_Sensor2 = new BMI160Sensor(1, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_MPU6500 || SECOND_IMU == IMU_MPU6050
                    m_Sensor2 = new MPU6050Sensor(1, SECOND_IMU, secondIMUAddress, SECOND_IMU_ROTATION);
#elif SECOND_IMU == IMU_ICM20948
                    m_Sensor2 = new ICM20948Sensor(1, secondIMUAddress, SECOND_IMU_ROTATION);
#endif
                }

                m_Sensor2->motionSetup();
            }
            Multiplexer::setTCAChannel(1);
            {

                // IMU 3
                n3IMUAddress = 0x4A; // BNO Address 1
                if (!I2CSCAN::isI2CExist(n3IMUAddress))
                {
                    m_Sensor3 = new ErroneousSensor(2, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor3 = new BNO080Sensor(2, IMU_BNO085, n3IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 3 found at address 0x%02X", n3IMUAddress);
                }
                m_Sensor3->motionSetup();
            }
            {
                // IMU 4
                n4IMUAddress = 0x4B; // BNO Address 2
                if (!I2CSCAN::isI2CExist(n4IMUAddress))
                {
                    m_Sensor4 = new ErroneousSensor(3, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor4 = new BNO080Sensor(3, IMU_BNO085, n4IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 4 found at address 0x%02X", n4IMUAddress);
                }
                m_Sensor4->motionSetup();
            }
            Multiplexer::setTCAChannel(2);
            {
                // IMU 5
                n5IMUAddress = 0x4A; // BNO Address 1
                if (!I2CSCAN::isI2CExist(n5IMUAddress))
                {
                    m_Sensor5 = new ErroneousSensor(4, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor5 = new BNO080Sensor(4, IMU_BNO085, n5IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 5 found at address 0x%02X", n5IMUAddress);
                }
                m_Sensor5->motionSetup();
            }
            {
                // IMU 6
                n6IMUAddress = 0x4B; // BNO Address 2
                if (!I2CSCAN::isI2CExist(n6IMUAddress))
                {
                    m_Sensor6 = new ErroneousSensor(5, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor6 = new BNO080Sensor(5, IMU_BNO085, n6IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 6 found at address 0x%02X", n6IMUAddress);
                }
                m_Sensor6->motionSetup();
            }
            Multiplexer::setTCAChannel(3);
            {
                // IMU 7
                n7IMUAddress = 0x4A; // BNO Address 1
                if (!I2CSCAN::isI2CExist(n7IMUAddress))
                {
                    m_Sensor7 = new ErroneousSensor(6, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor7 = new BNO080Sensor(6, IMU_BNO085, n7IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 7 found at address 0x%02X", n7IMUAddress);
                }
                m_Sensor7->motionSetup();
            }
            {
                // IMU 8
                n8IMUAddress = 0x4B; // BNO Address 2
                if (!I2CSCAN::isI2CExist(n8IMUAddress))
                {
                    m_Sensor8 = new ErroneousSensor(7, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor8 = new BNO080Sensor(7, IMU_BNO085, n8IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 8 found at address 0x%02X", n8IMUAddress);
                }
                m_Sensor8->motionSetup();
            }
            Multiplexer::setTCAChannel(4);
            {
                // IMU 9
                n9IMUAddress = 0x4A; // BNO Address 1
                if (!I2CSCAN::isI2CExist(n9IMUAddress))
                {
                    m_Sensor9 = new ErroneousSensor(8, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor9 = new BNO080Sensor(8, IMU_BNO085, n9IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 9 found at address 0x%02X", n9IMUAddress);
                }
                m_Sensor9->motionSetup();
            }
            {
                // IMU 10
                n10IMUAddress = 0x4B; // BNO Address 2
                if (!I2CSCAN::isI2CExist(n10IMUAddress))
                {
                    m_Sensor10 = new ErroneousSensor(9, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor10 = new BNO080Sensor(9, IMU_BNO085, n10IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 10 found at address 0x%02X", n10IMUAddress);
                }
                m_Sensor10->motionSetup();
            }
            Multiplexer::setTCAChannel(5);
            {
                // IMU 11
                n11IMUAddress = 0x4A; // BNO Address 1
                if (!I2CSCAN::isI2CExist(n11IMUAddress))
                {
                    m_Sensor11 = new ErroneousSensor(10, IMU);
                }
                else
                {
                    // Id, IMU, IMU Address, Rotation, Int Pin
                    m_Sensor11 = new BNO080Sensor(10, IMU_BNO085, n11IMUAddress, DEG_0, 24);
                    m_Logger.trace("IMU 11 found at address 0x%02X", n11IMUAddress);
                }
                m_Sensor11->motionSetup();
            }
        }

        void SensorManager::postSetup()
        {
            m_Sensor1->postSetup();
            m_Sensor2->postSetup();
            m_Sensor3->postSetup();
            m_Sensor4->postSetup();
            m_Sensor5->postSetup();
            m_Sensor6->postSetup();
            m_Sensor7->postSetup();
            m_Sensor8->postSetup();
            m_Sensor9->postSetup();
            m_Sensor10->postSetup();
            m_Sensor11->postSetup();
        }

        void SensorManager::update()
        {
            // Gather IMU data
            Multiplexer::setTCAChannel(0);
            m_Sensor1->motionLoop();
            m_Sensor2->motionLoop();
            Multiplexer::setTCAChannel(1);
            m_Sensor3->motionLoop();
            m_Sensor4->motionLoop();
            Multiplexer::setTCAChannel(2);
            m_Sensor5->motionLoop();
            m_Sensor6->motionLoop();
            Multiplexer::setTCAChannel(3);
            m_Sensor7->motionLoop();
            m_Sensor8->motionLoop();
            Multiplexer::setTCAChannel(4);
            m_Sensor9->motionLoop();
            m_Sensor10->motionLoop();
            Multiplexer::setTCAChannel(5);
            m_Sensor11->motionLoop();

            if (!ServerConnection::isConnected())
            {
                return;
            }

            // Send updates
            m_Sensor1->sendData();
            m_Sensor2->sendData();
            m_Sensor3->sendData();
            m_Sensor4->sendData();
            m_Sensor5->sendData();
            m_Sensor6->sendData();
            m_Sensor7->sendData();
            m_Sensor8->sendData();
            m_Sensor9->sendData();
            m_Sensor10->sendData();
            m_Sensor11->sendData();
        }
    }
}
