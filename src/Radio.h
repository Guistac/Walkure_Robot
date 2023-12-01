#pragma once

#include <Arduino.h>
#include <RH_RF95.h>

class Radio{
private:

    uint8_t enable_pin = 35; //not used ???
    uint8_t interrupt_pin = 34;
    uint8_t reset_pin = 33;
    uint8_t chipSelect_pin = 10;
    float frequencyMHz = 434.0; //434.0->470.0Mhz

public:

    Radio(){
        rf95 = new RH_RF95(chipSelect_pin, interrupt_pin);
    }

    bool initialize(){        
        pinMode(reset_pin, OUTPUT);
        digitalWrite(reset_pin, LOW);
        delay(10);
        digitalWrite(reset_pin, HIGH);
        delay(10);

        if(!rf95->init()) {
            Serial.println("Unable to initialize radio.");
            return false;
        }


        // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
        //these parameters are less interesting
        rf95->setTxPower(23, false);        //23dBm is max
        rf95->setPayloadCRC(false);         //we'll do our own CRC manually
        rf95->setFrequency(frequencyMHz);   //400-460 MHz

        rf95->setCodingRate4(5);            //5->8, default==5 lower is faster, higher is better for range
        rf95->setSpreadingFactor(7);        //6->12 default==7 lower is faster, higher is better for range
        rf95->setSignalBandwidth(125000);   //smaller bandwidths are better for range, larger bandwidths are better for speed
        
        /*
        available bandwidths
        7.8 KHZ  
        10.4 KHZ 
        15.6 KHZ 
        20.8 KHZ 
        31.25 KHZ
        41.7 KHZ 
        62.5 KHZ 
        125 KHZ  
        250 KHZ  
        500 KHZ  
        */

        Serial.println("Initialized Radio.");

        return true;
    }

    bool send(uint8_t* buffer, uint8_t length){
        rf95->send(buffer, length);
    }

    bool receive(uint8_t* buffer, uint8_t length){
        uint8_t receivedLength = length;
        if(rf95->recv(buffer, &receivedLength)){
            if(receivedLength != length) {
                Serial.printf("Frame of wrong length received. (%i instead of %i)\n", receivedLength, length);
                return false;
            }else return true;
        }
        return false;
    }

    int16_t getSignalStrength(){
        return rf95->lastRssi();
    }

private:
    RH_RF95* rf95;
};