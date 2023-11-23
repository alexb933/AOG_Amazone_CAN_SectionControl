
   /*
    * This program only turns the relays for section control
    * On and Off. Connect to the Relay Port in AgOpenGPS
    * 
    * Hydraulic Raise D4
    * Hydraulic Lower D3
    * 
    * Tram Right Side D5
    * Tram left Side D6
    * 
    * Section 0 to 5 -- D7 to D12
    * edited for Amatron CAN BUS by Alexander Beck/Valentin /Nov 2023
    */
    
  //loop time variables in microseconds

  #include <EEPROM.h> 
  #define EEP_Ident 0x5400
  //------------added because of CAN Amatron-------
  #include <SPI.h>
  #include "mcp2515_can.h"  
  // Set SPI CS Pin according to your hardware !!
  // Arduino MCP2515 Hat: the cs pin of the version after v1.1 is default to D9 //v0.9b and v1.0 is default D10
  const int SPI_CS_PIN = 9;   //ATTENTION !! PIN9 is also used already in Machine USB but is PIN9 or 10 ar needed for CAN Shield. Therefore i replaced the PIN9 in the existing Machine code
  mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
  
  // Set Amatron Claim adress to send data
  byte AmaClick_addressClaim[8] = {0x28, 0xEC, 0x44, 0x0C, 0x00, 0x80, 0x1A, 0x20};
  //set Amatron message to change Sections
  uint8_t AmaClick_data[8] = {0x21, 0xFA, 0x00, 0x00, 0x00, 0x00, 0x01, 0x09};
  byte AmaClick_data_byte2 = 0b00000000;
  byte AmaClick_data_byte3 = 0b00000000;  
  //----------------------

    //Program counter reset
    void(* resetFunc) (void) = 0;

  //Variables for config - 0 is false  
    struct Config {
        uint8_t raiseTime = 2;
        uint8_t lowerTime = 4;
        uint8_t enableToolLift = 0;
        uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

        uint8_t user1 = 0; //user defined values set in machine tab
        uint8_t user2 = 0;
        uint8_t user3 = 0;
        uint8_t user4 = 0;

    };  Config aogConfig;   //4 bytes

    /*
    * Functions as below assigned to pins
    0: -
    1 thru 16: Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section 7,Section 8, 
                Section 9, Section 10, Section 11, Section 12, Section 13, Section 14, Section 15, Section 16, 
    17,18    Hyd Up, Hyd Down, 
    19 Tramline, 
    20: Geo Stop
    21,22,23 - unused so far
    */

    //24 possible pins assigned to these functions
    uint8_t pin[] = { 1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

    //read value from Machine data and set 1 or zero according to list
    uint8_t relayState[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

  const uint8_t LOOP_TIME = 200; //5hz
  uint32_t lastTime = LOOP_TIME;
  uint32_t currentTime = LOOP_TIME;
  uint32_t fifthTime = 0;
  uint16_t count = 0;

  //Comm checks
  uint8_t watchdogTimer = 0; //make sure we are talking to AOG
  uint8_t serialResetTimer = 0; //if serial buffer is getting full, empty it

  bool isRaise = false;
  bool isLower = false;
  
   //Communication with AgOpenGPS
  int16_t temp, EEread = 0;

   //Parsing PGN
  bool isPGNFound = false, isHeaderFound = false;
  uint8_t pgn = 0, dataLength = 0, idx = 0;
  int16_t tempHeader = 0;
  
  uint8_t AOG[] = {0x80,0x81, 0x7f, 0xED, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };

  //The variables used for storage
  uint8_t relayHi=0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0, geoStop = 0;
  float gpsSpeed;
  
  uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;
 
  void setup()
  {
      //set the baud rate
      Serial.begin(38400);
      //while (!Serial) { ; } // wait for serial port to connect. Needed for native USB

      EEPROM.get(0, EEread);              // read identifier

      if (EEread != EEP_Ident)   // check on first start and write EEPROM
      {
          EEPROM.put(0, EEP_Ident);
          EEPROM.put(6, aogConfig);
          EEPROM.put(20, pin);
      }
      else
      {
          EEPROM.get(6, aogConfig);
          EEPROM.get(20, pin);
      }

      //set the pins to be outputs (pin numbers)
      pinMode(2, OUTPUT);
      pinMode(3, OUTPUT);

      pinMode(4, OUTPUT);
      pinMode(5, OUTPUT);
      pinMode(6, OUTPUT);
      pinMode(7, OUTPUT);
      pinMode(8, OUTPUT);
      //pinMode(9, OUTPUT); //comment because PIN9 is needed for Can Shield
      pinMode(10, OUTPUT);
      pinMode(11, OUTPUT);
      pinMode(12, OUTPUT);
      pinMode(13, OUTPUT);

      //------------added because of CAN Amatron-------

        
      while (CAN_OK != CAN.begin(CAN_250KBPS)) {        //init can bus: Amatron baudrate = 250k 
          Serial.println("CAN init fail, retry...");
          delay(100);
      }
      Serial.println("CAN init ok!");

      //set CAN Filter and Mask to look for sending data from AMACLICK (if AMACLICK is activ and sending we don´t want to send CAN)
      //Therfore you can chose either use manually AMACLICK or automatic Section Control
      CAN.init_Mask(0,1,0xFFFFFFFF);        // Init first mask... allow all Bits to filtered
      CAN.init_Filt(0,1,0x18E6FFCE);       // Init first filter... check for specific sections ID --> 0x18E6FFCE is AMACLICK Messages ID
      CAN.init_Mask(1,0,0xFFFFFFFF);       // Init second mask... second mask has to be set otherwise it will not work
      
      //send first Claim Adress to Amatron to send future data
      CAN.sendMsgBuf(0x18EEFFCE, 1, 8,AmaClick_addressClaim); //input from Valentin
      //Serial.println("Address Claimed");  
      //-----------------------------------------------
      
  }

  void loop()
  {
      //Loop triggers every 200 msec and sends back gyro heading, and roll, steer angle etc

      currentTime = millis();

      if (currentTime - lastTime >= LOOP_TIME)
      {
          lastTime = currentTime;

          //If connection lost to AgOpenGPS, the watchdog will count up 
          if (watchdogTimer++ > 250) watchdogTimer = 12;

          //clean out serial buffer to prevent buffer overflow
          if (serialResetTimer++ > 20)
          {
              while (Serial.available() > 0) Serial.read();
              serialResetTimer = 0;
          }

          if (watchdogTimer > 12)
          {
            if (aogConfig.isRelayActiveHigh) {
                relayLo = 255;
                relayHi = 255;
              } else {
                relayLo = 0;
                relayHi = 0;        
              }          
          }

          //hydraulic lift

          if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2))
          {
              lastTrigger = hydLift;
              lowerTimer = 0;
              raiseTimer = 0;

              //200 msec per frame so 5 per second
              switch (hydLift)
              {
                  //lower
              case 1:
                  lowerTimer = aogConfig.lowerTime * 5;
                  break;

                  //raise
              case 2:
                  raiseTimer = aogConfig.raiseTime * 5;
                  break;
              }
          }

          //countdown if not zero, make sure up only
          if (raiseTimer)
          {
              raiseTimer--;
              lowerTimer = 0;
          }
          if (lowerTimer) lowerTimer--;

          //if anything wrong, shut off hydraulics, reset last
          if ((hydLift != 1 && hydLift != 2) || watchdogTimer > 10) //|| gpsSpeed < 2)
          {
              lowerTimer = 0;
              raiseTimer = 0;
              lastTrigger = 0;
          }

          if (aogConfig.isRelayActiveHigh)
          {
              isLower = isRaise = false;
              if (lowerTimer) isLower = true;
              if (raiseTimer) isRaise = true;
          }
          else
          {
              isLower = isRaise = true;
              if (lowerTimer) isLower = false;
              if (raiseTimer) isRaise = false;
          }

          //section relays
          SetRelays();

          AOG[5] = pin[0];
          AOG[6] = pin[1];
          AOG[7] = (uint8_t)tramline;


          //add the checksum
          int16_t CK_A = 0;
          for (uint8_t i = 2; i < sizeof(AOG) - 1; i++)
          {
              CK_A = (CK_A + AOG[i]);
          }
          AOG[sizeof(AOG) - 1] = CK_A;

          Serial.write(AOG, sizeof(AOG));
          Serial.flush();   // flush out buffer

      } //end of timed loop

      // Serial Receive
      //Do we have a match with 0x8081?    
      if (Serial.available() > 4 && !isHeaderFound && !isPGNFound)
      {
          uint8_t temp = Serial.read();
          if (tempHeader == 0x80 && temp == 0x81)
          {
              isHeaderFound = true;
              tempHeader = 0;
          }
          else
          {
              tempHeader = temp;     //save for next time
              return;
          }
      }

      //Find Source, PGN, and Length
      if (Serial.available() > 2 && isHeaderFound && !isPGNFound)
      {
          Serial.read(); //The 7F or less
          pgn = Serial.read();
          dataLength = Serial.read();
          isPGNFound = true;
          idx = 0;
      }

      //The data package
      if (Serial.available() > dataLength && isHeaderFound && isPGNFound)
      {
          if (pgn == 239) // EF Machine Data
          {
              uTurn = Serial.read();
              gpsSpeed = (float)Serial.read();//actual speed times 4, single uint8_t

              hydLift = Serial.read();
              tramline = Serial.read();  //bit 0 is right bit 1 is left

              //just get the rest of bytes
              Serial.read();   //high,low bytes   
              Serial.read();

              relayLo = Serial.read();          // read relay control from AgOpenGPS
              relayHi = Serial.read();

              if (aogConfig.isRelayActiveHigh)
              {
                  tramline = 255 - tramline;
                  relayLo = 255 - relayLo;
              }

              //Bit 13 CRC
              Serial.read();

              //reset watchdog
              watchdogTimer = 0;

              //Reset serial Watchdog  
              serialResetTimer = 0;

              //reset for next pgn sentence
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }

          else if (pgn == 238) //EE Machine Settings 
          {
              aogConfig.raiseTime = Serial.read();
              aogConfig.lowerTime = Serial.read();
              aogConfig.enableToolLift = Serial.read();

              //set1 
              uint8_t sett = Serial.read();  //setting0     
              if (bitRead(sett, 0)) aogConfig.isRelayActiveHigh = 1; else aogConfig.isRelayActiveHigh = 0;

              aogConfig.user1 = Serial.read();
              aogConfig.user2 = Serial.read();
              aogConfig.user3 = Serial.read();
              aogConfig.user4 = Serial.read();

              //crc
              //udpData[13];        //crc
              Serial.read();

              //save in EEPROM and restart
              EEPROM.put(6, aogConfig);
              resetFunc();

              //reset for next pgn sentence
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }

          else if (pgn == 236) //EC Relay Pin Settings 
          {
              for (uint8_t i = 0; i < 24; i++)
              {
                  pin[i] = Serial.read();
              }

              //save in EEPROM and restart
              EEPROM.put(20, pin);
              //resetFunc();

              //reset for next pgn sentence
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }

          else //nothing found, clean up
          {
              isHeaderFound = isPGNFound = false;
              pgn = dataLength = 0;
          }
      }
  }

  void SetRelays(void)
  {
      //pin, rate, duration  130 pp meter, 3.6 kmh = 1 m/sec or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
      //gpsSpeed is 10x actual speed so 3.61111
      gpsSpeed *= 3.61111;
      //tone(13, gpsSpeed);
      
      //Load the current pgn relay state - Sections
      for (uint8_t i = 0; i < 8; i++)
      {
          relayState[i] = bitRead(relayLo, i);
      }
      
      for (uint8_t i = 0; i < 8; i++)
      {
          relayState[i + 8] = bitRead(relayHi, i);
      }

      // Hydraulics
      relayState[16] = isLower;
      relayState[17] = isRaise;

      //Tram
      relayState[18] = bitRead(tramline, 0); //right
      relayState[19] = bitRead(tramline, 1); //left

      //GeoStop
      relayState[20] =  (geoStop == 0) ? 0 : 1;

//      if (pin[0]) digitalWrite(13, relayState[pin[0]-1]);
//      if (pin[1]) digitalWrite(5, relayState[pin[1]-1]);
//      if (pin[2]) digitalWrite(6, relayState[pin[2]-1]);
//      if (pin[3]) digitalWrite(7, relayState[pin[3]-1]);
//
//      if (pin[4]) digitalWrite(8, relayState[pin[4]-1]);
//      //if (pin[5]) digitalWrite(9, relayState[pin[5]-1]); //comment because PIN9 is needed for CAN shield
//      if (pin[6]) digitalWrite(10, relayState[pin[6]-1]);
//      if (pin[7]) digitalWrite(11, relayState[pin[7]-1]);
//
//      if (pin[8]) digitalWrite(12, relayState[pin[8]-1]);
//      if (pin[9]) digitalWrite(4, relayState[pin[9]-1]);

      //if (pin[10]) digitalWrite(IO#Here, relayState[pin[10]-1]);
      //if (pin[11]) digitalWrite(IO#Here, relayState[pin[11]-1]);
      //if (pin[12]) digitalWrite(IO#Here, relayState[pin[12]-1]);
      //if (pin[13]) digitalWrite(IO#Here, relayState[pin[13]-1]);
      //if (pin[14]) digitalWrite(IO#Here, relayState[pin[14]-1]);
      //if (pin[15]) digitalWrite(IO#Here, relayState[pin[15]-1]);
      //if (pin[16]) digitalWrite(IO#Here, relayState[pin[16]-1]);
      //if (pin[17]) digitalWrite(IO#Here, relayState[pin[17]-1]);
      //if (pin[18]) digitalWrite(IO#Here, relayState[pin[18]-1]);
      //if (pin[19]) digitalWrite(IO#Here, relayState[pin[19]-1]);


      //------------added because of CAN Amatron-------------------------------------
      //define data for message to send to control Amatron sections  
      //See ExcelFile or als Valentin for mor details according to Send Message
      
      //check if minimum one of the section relay is on/activ -> Main switch in Amatron on/off
      //16 relays are defined in relay[0-15]
      int isOnerelayPositiv = 0;
      for (uint8_t i = 0; i < 16; i++)
      {
          isOnerelayPositiv = isOnerelayPositiv + relayState[i];
      }
      if (isOnerelayPositiv != 0) bitWrite(AmaClick_data_byte3, 7, 1); //byte 3, bit 7 defines MainSwitch
      if (isOnerelayPositiv == 0) bitWrite(AmaClick_data_byte3, 7, 0);
      
      //write relayState into CAN message
      if (relayState[0]==HIGH) bitWrite(AmaClick_data_byte2, 0, 1);   // byte 2 and part of byte3 defines sections
      if (relayState[1]==HIGH) bitWrite(AmaClick_data_byte2, 1, 1);
      if (relayState[2]==HIGH) bitWrite(AmaClick_data_byte2, 2, 1);
      if (relayState[3]==HIGH) bitWrite(AmaClick_data_byte2, 3, 1);
      if (relayState[4]==HIGH) bitWrite(AmaClick_data_byte2, 4, 1);
      if (relayState[5]==HIGH) bitWrite(AmaClick_data_byte2, 5, 1);
      if (relayState[6]==HIGH) bitWrite(AmaClick_data_byte2, 6, 1);
      if (relayState[7]==HIGH) bitWrite(AmaClick_data_byte2, 7, 1);
      if (relayState[8]==HIGH) bitWrite(AmaClick_data_byte3, 0, 1);
      if (relayState[9]==HIGH) bitWrite(AmaClick_data_byte3, 1, 1);
      if (relayState[10]==HIGH) bitWrite(AmaClick_data_byte3, 2, 1);
      if (relayState[11]==HIGH) bitWrite(AmaClick_data_byte3, 3, 1);
      if (relayState[12]==HIGH) bitWrite(AmaClick_data_byte3, 4, 1);
      //AMATRON supports max 13 sections
      
      if (relayState[0]==LOW) bitWrite(AmaClick_data_byte2, 0, 0);
      if (relayState[1]==LOW) bitWrite(AmaClick_data_byte2, 1, 0);
      if (relayState[2]==LOW) bitWrite(AmaClick_data_byte2, 2, 0);
      if (relayState[3]==LOW) bitWrite(AmaClick_data_byte2, 3, 0);
      if (relayState[4]==LOW) bitWrite(AmaClick_data_byte2, 4, 0);
      if (relayState[5]==LOW) bitWrite(AmaClick_data_byte2, 5, 0);
      if (relayState[6]==LOW) bitWrite(AmaClick_data_byte2, 6, 0);
      if (relayState[7]==LOW) bitWrite(AmaClick_data_byte2, 7, 0);
      if (relayState[8]==LOW) bitWrite(AmaClick_data_byte3, 0, 0);
      if (relayState[9]==LOW) bitWrite(AmaClick_data_byte3, 1, 0);
      if (relayState[10]==LOW) bitWrite(AmaClick_data_byte3, 2, 0);
      if (relayState[11]==LOW) bitWrite(AmaClick_data_byte3, 3, 0);
      if (relayState[12]==LOW) bitWrite(AmaClick_data_byte3, 4, 0);

      //assign byte2 and 3 to the CAN message
      AmaClick_data[2]= AmaClick_data_byte2,HEX;
      AmaClick_data[3]= AmaClick_data_byte3,HEX;

      //check if AMACLICK is active if not then send CAN message
      unsigned char len = 0;
      unsigned char buf[8];
      if (watchdogTimer < 10){  //when connection to AGO is lost then send nothing
      CAN.readMsgBuf(&len, buf); //read CAN with Filter and Mask for AMACLICK ID
      if (buf[6]!= 1) CAN.sendMsgBuf(0x18E6FFCE, 1, 8, AmaClick_data);
      }
        
  }
