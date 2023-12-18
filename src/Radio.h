#pragma once

#include <Arduino.h>
#include "RH_RF69.h"
#include "CRC.h"
#include "EEPROM.h"

class Radio{
public:

    enum class TranscieverType{
        MASTER,
        SLAVE
    };

    Radio(TranscieverType type, uint8_t resetPin, uint8_t chipSelectPin, uint8_t interruptPin){
        transcieverType = type;
        reset_pin = resetPin;
        rf69 = new RH_RF69(chipSelectPin, interruptPin);
    }

    bool setFrequency(uint32_t newFrequencyHz){
        if(newFrequencyHz > maxFrequency) newFrequencyHz = maxFrequency;
        else if(newFrequencyHz < minFrequency) newFrequencyHz = minFrequency;

        if(rf69->setFrequency(float(frequencyHz) / 1000000.0)) {
            frequencyHz = newFrequencyHz;
            nodeID = (frequencyHz / 100000) % 255;
            return true;
        }

        return false;
    }

    uint32_t getFrequency(){ return frequencyHz; }

    bool saveFrequency(){
        if(savedFrequencyHz != frequencyHz){
            EEPROM.write(0x30, uint8_t(frequencyHz & 0xFF));
            EEPROM.write(0x31, uint8_t((frequencyHz >> 8) & 0xFF));
            EEPROM.write(0x32, uint8_t((frequencyHz >> 16) & 0xFF));
            EEPROM.write(0x33, uint8_t((frequencyHz >> 24) & 0xFF));
            savedFrequencyHz = frequencyHz;
            return true;
        }
        return false;
    }

    uint32_t loadFrequency(){
        uint8_t data[4];
        data[0] = EEPROM.read(0x30);
        data[1] = EEPROM.read(0x31);
        data[2] = EEPROM.read(0x32);
        data[3] = EEPROM.read(0x33);
        uint32_t loadedFrequencyHz = *(uint32_t*)data;
        return loadedFrequencyHz;
    }

    bool initialize(uint32_t frequency = 0){

        if(frequency == 0){
            frequencyHz = loadFrequency();
            savedFrequencyHz = frequencyHz;
            Serial.printf("Loaded radio frequency: %iHz\n", frequencyHz);
        }else{
            frequencyHz = frequency;
            savedFrequencyHz = frequency;
        }

        pinMode(reset_pin, OUTPUT);
        digitalWrite(reset_pin, HIGH);
        delay(10);
        digitalWrite(reset_pin, LOW);
        delay(10);

        if(!rf69->init()) {
            Serial.println("Unable to initialize radio.");
            return false;
        }else Serial.println("Initialized radio.");

        if(!rf69->setModemConfig(RH_RF69::ModemConfigChoice::GFSK_Rb38_4Fd76_8)){
            Serial.println("Could not set radio modem parameters.");
            return false;
        }else Serial.println("Configured radio modem parameters.");

        if(!setFrequency(frequencyHz)){
            Serial.printf("Could not set radio frequency to %.3fHz\n", float(frequencyHz) / 1000000.0);
            return false;
        }else Serial.printf("Set radio frequency to %.3fMHz.\n", float(frequencyHz) / 1000000.0);

        rf69->setTxPower(20, true);
        rf69->setEncryptionKey(nullptr);

        rf69->setPromiscuous(true); //disable radiohead library code related to header and transmission control

        Serial.println("Successfully configured radio !");

        return true;
    }

    bool send(uint8_t* buffer, uint8_t length){

        if(length == 0){
            Serial.printf("Cannot send messages with 0 length. %i\n", millis());
            return false;
        }

        //construct the payload buffer by appending, a 2 byte crc, a 1 byte nodeID and a 1 byte messageCounter at the start
        //the crc covers all bytes that follow it
        //[2b:CRC][1b:nodeID][1b:messageCounter][lb:UserPayload]

        uint8_t crcPayloadSize = length + 2;
        uint8_t crcPayloadBuffer[crcPayloadSize];
        crcPayloadBuffer[0] = nodeID;
        switch(transcieverType){
            case TranscieverType::MASTER:
                //in master mode, we increment the sent message counter
                //we also store the last transmission time
                //the slave will reply with the last receive counter
                //this way we can compute the round trip time
                sentMessageCounter++;
                crcPayloadBuffer[1] = sentMessageCounter; 
                lastMessageSendTime_micros = micros();
                break;
            case TranscieverType::SLAVE:
                //in slave mode, send back the message counter received from the master
                //this way the master can compute the round trip time
                crcPayloadBuffer[1] = sentMessageCounter;
                break;
        }
        memccpy(crcPayloadBuffer+2, buffer, length, length);
        uint16_t crc = calcCRC16(crcPayloadBuffer, crcPayloadSize);

        //radiohead forces a 4 bytes header to be sent, this header is used for basic network addressing
        //we don't want to use this feature and we don't want to lose data rate
        //so we fill this 4 byte header with our CRC, nodeID and messageCounter
        //this way we don't have to modify the library code and we avoid wasting bitrate
        rf69->setHeaderTo((crc >> 8) & 0xFF);
        rf69->setHeaderFrom(crc & 0xFF);
        rf69->setHeaderId(crcPayloadBuffer[0]);
        rf69->setHeaderFlags(crcPayloadBuffer[1], 255);

        bool b_success = rf69->send(crcPayloadBuffer+2, crcPayloadSize-2);

        if(false){
            Serial.print("Frame: ");
            for(int i = crcPayloadSize-1; i >= 0; i--){
                bool b0 = crcPayloadBuffer[i] & 0x1;
                bool b1 = crcPayloadBuffer[i] & 0x2;
                bool b2 = crcPayloadBuffer[i] & 0x4;
                bool b3 = crcPayloadBuffer[i] & 0x8;
                bool b4 = crcPayloadBuffer[i] & 0x10;
                bool b5 = crcPayloadBuffer[i] & 0x20;
                bool b6 = crcPayloadBuffer[i] & 0x40;
                bool b7 = crcPayloadBuffer[i] & 0x80;
                Serial.printf("%i%i%i%i%i%i%i%i ", b7, b6, b5, b4, b3, b2, b1, b0);
            }
            Serial.println("");
        }

        if(false){
            uint32_t startMicros = micros();
            rf69->waitPacketSent();
            uint32_t time = micros() - startMicros;
            float time_ms = float(time) / 1000.0;
            Serial.printf("Send time: %.2fms (%i bytes)\n", time_ms, length);
        }
        
        return b_success;
    }



    enum class ReceptionResult{
        NOTHING_RECEIVED,
        BAD_CRC,
        BAD_NODEID,
        GOOD_RECEPTION
    };

    ReceptionResult receive(uint8_t* buffer, uint8_t& length){

        if(!rf69->available()) return ReceptionResult::NOTHING_RECEIVED;

        uint8_t crcPayloadSize = length + 2;
        uint8_t crcPayloadBuffer[crcPayloadSize];

        uint8_t payloadSize = crcPayloadSize - 2;
        uint8_t* payloadBuffer = crcPayloadBuffer + 2;

        if(!rf69->recv(payloadBuffer, &payloadSize)) return ReceptionResult::NOTHING_RECEIVED;
        length = payloadSize;

        crcPayloadBuffer[0] = rf69->headerId();     //nodeID
        crcPayloadBuffer[1] = rf69->headerFlags();  //messageCounter
        uint16_t calculatedCRC = calcCRC16(crcPayloadBuffer, crcPayloadSize);
        uint16_t receivedCRC = (rf69->headerTo() << 8) | rf69->headerFrom();

        if(true){
            Serial.print("Frame: ");
            for(int i = crcPayloadSize-1; i >= 0; i--){
                bool b0 = crcPayloadBuffer[i] & 0x1;
                bool b1 = crcPayloadBuffer[i] & 0x2;
                bool b2 = crcPayloadBuffer[i] & 0x4;
                bool b3 = crcPayloadBuffer[i] & 0x8;
                bool b4 = crcPayloadBuffer[i] & 0x10;
                bool b5 = crcPayloadBuffer[i] & 0x20;
                bool b6 = crcPayloadBuffer[i] & 0x40;
                bool b7 = crcPayloadBuffer[i] & 0x80;
                Serial.printf("%i%i%i%i%i%i%i%i ", b7, b6, b5, b4, b3, b2, b1, b0);
            }
            Serial.println("");
        }

        if(calculatedCRC != receivedCRC) return ReceptionResult::BAD_CRC;

        uint8_t receivedNodeID = crcPayloadBuffer[0];
        if(receivedNodeID != nodeID) return ReceptionResult::BAD_NODEID;

        switch(transcieverType){
            case TranscieverType::MASTER:
                //the slave retransmits the last counter it received
                //if this matches the last message counter that was sent out
                //then the master can compute the round trip time
                if(crcPayloadBuffer[1] == sentMessageCounter){
                    lastRoundTripTime_ms = float(micros() - lastMessageSendTime_micros) / 1000.0;
                    //Serial.printf("Message %i round trip time: %.2fms\n", sentMessageCounter, lastRoundTripTime_ms);
                }
                break;
            case TranscieverType::SLAVE:
                //Slave Behavior, store the last receive message counter and reply with the same one
                sentMessageCounter = crcPayloadBuffer[1];
                break;
        }

        memccpy(buffer, payloadBuffer, length, length);
        return ReceptionResult::GOOD_RECEPTION;
    }

    int16_t getSignalStrength(){ return rf69->lastRssi(); }

    float getLastRoundTripTime_ms(){ return lastRoundTripTime_ms; }

private:

    RH_RF69* rf69;
    TranscieverType transcieverType;
    uint32_t frequencyHz;       //read by eeprom on startup
    uint32_t savedFrequencyHz;  //to check if the current frequency is saved
    uint8_t nodeID;             //set by frequency
    uint8_t sentMessageCounter = 0; //incremented on each send
    uint32_t lastMessageSendTime_micros = 0;
    uint8_t reset_pin = 7;
    const uint32_t minFrequency = 424000000;
    const uint32_t maxFrequency = 510000000;
    float lastRoundTripTime_ms = 0.0;

};
