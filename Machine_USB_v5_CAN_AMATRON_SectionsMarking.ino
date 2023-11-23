

// Code copied from CAN BUS Shield loovee, 2014-6-13
// Read Amazone Amatron CAN Bus and check sections for AGO, at 1st step send Sections Info to AGO with keyboard Hotkeys, 2st step use section control and send section info from AGO to Amatron
// Alexander Beck (Support Valentin) Nov 2023

#include <SPI.h>
#include "mcp2515_can.h"
#define CAN_2515

const uint8_t LOOP_TIME = 200; //5hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

uint8_t send_AOG_sections [] = {0x80,0x81, 0x7B, 0xEA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
    byte send_AOG_sectionsbyte9_on = 0b00000000; //needed for section 1-8
    byte send_AOG_sectionsbyte10_off = 0b00000000;
                         
void setup() {
    Serial.begin(38400);
    //SERIAL_PORT_MONITOR.begin(115200);
    
    while (CAN_OK != CAN.begin(CAN_250KBPS)) {                      // init can bus : baudrate = 250k bei Amazone/mueller elektronik
        //SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");
    CAN.init_Mask(0,1,0xFFFFFFFF);        // Init first mask... allow all Bits to filtered
    CAN.init_Filt(0,1,0x1CE72687);       // Init first filter... check for specific sections ID --> 1CE72687 is ID for the status of the sections in Amatron
    CAN.init_Mask(1,0,0xFFFFFFFF);       // Init second mask... second mask has to be set otherwise it will not work
}

void loop() 
{


          
    unsigned char len = 0;
    unsigned char buf[8];

    if (CAN_MSGAVAIL == CAN.checkReceive())  // check if data coming
    {         
        CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        //unsigned long canId = CAN.getCanId();
        //SERIAL_PORT_MONITOR.println("-----------------------------");
        //SERIAL_PORT_MONITOR.print("Get data from ID: 0x");
        
        //SERIAL_PORT_MONITOR.print(canId, HEX);
        //SERIAL_PORT_MONITOR.print("_");
        //SERIAL_PORT_MONITOR.print("\t");
//                
//        for (int i = 0; i < len; i++)     // print the data
//          {
//            SERIAL_PORT_MONITOR.print(buf[i], HEX);
//            SERIAL_PORT_MONITOR.print("\t");
//          }
//        SERIAL_PORT_MONITOR.println(); 

     

        if (buf[3]==1)      // ID for sections info is 1CE7268. The data according to this ID defines which section is on/off buf[3] delivers on/off 1/0 and buf[1] delivers which section
          {  
            switch(buf[1]) 
            {
              case 247:
                Serial.println("Section 1 on");
                  bitWrite(send_AOG_sectionsbyte9_on, 0, 1);
                  bitWrite(send_AOG_sectionsbyte10_off, 0, 0);
                break;
              case 249:
                Serial.println("Section 2 on");
                  bitWrite(send_AOG_sectionsbyte9_on, 1, 1);
                  bitWrite(send_AOG_sectionsbyte10_off, 1, 0);
                break;
              case 251:
                Serial.println("Section 3 on");
                  bitWrite(send_AOG_sectionsbyte9_on, 2, 1);
                  bitWrite(send_AOG_sectionsbyte10_off, 2, 0);
                break;
              case 253:
                Serial.println("Section 4 on");
                  bitWrite(send_AOG_sectionsbyte9_on, 3, 1);
                  bitWrite(send_AOG_sectionsbyte10_off, 3, 0);
                break;
              case 255:
                Serial.println("Section 5 on");
                  bitWrite(send_AOG_sectionsbyte9_on, 4, 1);
                  bitWrite(send_AOG_sectionsbyte10_off, 4, 0);
                break;
              case 1:
                Serial.println("Section 6 on");
                  bitWrite(send_AOG_sectionsbyte9_on, 5, 1);
                  bitWrite(send_AOG_sectionsbyte10_off, 5, 0);
                break;
              case 3:
                Serial.println("Section 7 on");
                  bitWrite(send_AOG_sectionsbyte9_on, 6, 1);
                  bitWrite(send_AOG_sectionsbyte10_off, 6, 0);
                break;
            }
          }
          
         if (buf[3]==0) 
          {  
            switch(buf[1]) {
              case 247:
                Serial.println("Section 1 off");
                  bitWrite(send_AOG_sectionsbyte9_on, 0, 0);
                  bitWrite(send_AOG_sectionsbyte10_off, 0, 1);
                break;
              case 249:
                Serial.println("Section 2 off");
                bitWrite(send_AOG_sectionsbyte9_on, 1, 0);
                  bitWrite(send_AOG_sectionsbyte10_off, 1, 1);
                break;
              case 251:
                Serial.println("Section 3 off");
                bitWrite(send_AOG_sectionsbyte9_on, 2, 0);
                  bitWrite(send_AOG_sectionsbyte10_off, 2, 1);
                break;
              case 253:
                Serial.println("Section 4 off");
                bitWrite(send_AOG_sectionsbyte9_on, 3, 0);
                  bitWrite(send_AOG_sectionsbyte10_off, 3, 1);
                break;
              case 255:
                Serial.println("Section 5 off");
                bitWrite(send_AOG_sectionsbyte9_on, 4, 0);
                  bitWrite(send_AOG_sectionsbyte10_off, 4, 1);
                break;
              case 1:
                Serial.println("Section 6 off");
                bitWrite(send_AOG_sectionsbyte9_on, 5, 0);
                  bitWrite(send_AOG_sectionsbyte10_off, 5, 1);
                break;
              case 3:
                Serial.println("Section 7 off");
                bitWrite(send_AOG_sectionsbyte9_on, 6, 0);
                  bitWrite(send_AOG_sectionsbyte10_off, 6, 1);
                break; 
            }
          } 
       
    }

      currentTime = millis();
  if (currentTime - lastTime >= LOOP_TIME)
      {
          lastTime = currentTime;
        send_AOG_sections[9]= send_AOG_sectionsbyte9_on,HEX;
        send_AOG_sections[10]= send_AOG_sectionsbyte10_off,HEX;
        sendSectionsupdate();

      }

}


//
  void sendSectionsupdate(void)
  {
    //add the checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < sizeof(send_AOG_sections) - 1; i++)
          {
              CK_A = (CK_A + send_AOG_sections[i]);
          }
          send_AOG_sections[sizeof(send_AOG_sections) - 1] = CK_A;

          Serial.write(send_AOG_sections, sizeof(send_AOG_sections));
          Serial.flush();   // flush out buffer
  }

/*********************************************************************************************************
    END FILE  Version 1.0    Beck Alexander     11_14_2023
*********************************************************************************************************/
