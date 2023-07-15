/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

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
#include "network.h"

bool lastWifiConnected = false;

void Network::setUp() {
    WiFiNetwork::setUp();
}

void Network::update(Sensor * const sensor, Sensor * const sensor2, Sensor * const sensor3) { // , Sensor * const sensor4, Sensor * const sensor5, Sensor * const sensor6, Sensor * const sensor7, Sensor * const sensor8, Sensor * const sensor9, Sensor * const sensor10, Sensor * const sensor11
    WiFiNetwork::upkeep();
    if(WiFiNetwork::isConnected()) {
        if(lastWifiConnected == false) {
            lastWifiConnected = true;
            ServerConnection::resetConnection(); // WiFi was reconnected, reconnect to the server
        }
        ServerConnection::update(sensor, sensor2, sensor3); // , sensor4, sensor5, sensor6, sensor7, sensor8, sensor9, sensor10, sensor11
    } else {
        lastWifiConnected = false;
    }
}
