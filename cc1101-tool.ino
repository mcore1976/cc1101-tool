//
// CC1101 interactive terminal tool
// allows for sending / receiving data over serial port
// on selected radio channel, modulation, ..
//
// (C) Adam Loboda '2023
//
// based on great SmartRC library by Little_S@tan
// Please download ZIP from 
// https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
// and attach it as ZIP library for Arduino
//
// Also uses Arduino Command Line interpreter by Edgar Bonet
// from https://gist.github.com/edgar-bonet/607b387260388be77e96
//
// This code will ONLY work with Arduino Pro Micro 3.3V 8MHz
//



#include <ELECHOUSE_CC1101_SRC_DRV.h>


#define CCBUFFERSIZE 64

#include <avr/pgmspace.h>

#define BUF_LENGTH 256  /* Buffer for the incoming command. */

static bool do_echo = true;

// buffer for receiving  CC1101
byte ccreceivingbuffer[CCBUFFERSIZE] = {0};

// buffer for sending  CC1101
byte ccsendingbuffer[CCBUFFERSIZE] = {0};

// buffer for recording and replaying of single frame
byte ccrecordingbuffer[CCBUFFERSIZE] = {0};

// buffer for hex to ascii conversions 
byte textbuffer[256];


// The RX LED has a defined Arduino Pro Micro pin
int RXLED = 17; 

// check if CLI receiving mode enabled
int receivingmode = 0; 

// check if CLI jamming mode enabled
int jammingmode = 0; 

// check if CLI recording mode enabled
int recordingmode = 0; 


// convert char table to string with hex numbers

void asciitohex(byte *ascii_ptr, byte *hex_ptr,int len)
{
    byte i,j,k;
    for(i = 0; i < len; i++)
    {
      // high byte first
      j = ascii_ptr[i] / 16;
      if (j>9) 
         { k = j - 10 + 65; }
      else 
         { k = j + 48; }
      hex_ptr[2*i] = k ;
      // low byte second
      j = ascii_ptr[i] % 16;
      if (j>9) 
         { k = j - 10 + 65; }
      else
         { k = j + 48; }
      hex_ptr[2*i+1] = k ; 
    };
    hex_ptr[2*i+2] = 0 ; 
}


// convert string with hex numbers to array of bytes

void  hextoascii(byte *ascii_ptr, byte *hex_ptr,int len)
{
    byte i,j;
    for(i = 0; i < (len/2); i++)
     { 
     j = hex_ptr[i*2];
     if ((j>47) && (j<58))  ascii_ptr[i] = (j - 48) * 16;
     if ((j>64) && (j<71))  ascii_ptr[i] = (j - 55) * 16;
     if ((j>96) && (j<103)) ascii_ptr[i] = (j - 87) * 16;
     j = hex_ptr[i*2+1];
     if ((j>47) && (j<58))  ascii_ptr[i] = ascii_ptr[i]  + (j - 48);
     if ((j>64) && (j<71))  ascii_ptr[i] = ascii_ptr[i]  + (j - 55);
     if ((j>96) && (j<103)) ascii_ptr[i] = ascii_ptr[i]  + (j - 87);
     };
    ascii_ptr[i++] = 0;
}

/*
void  hextoascii(char *ascii_ptr, char *hex_ptr,int len)
{
    int final_len = len / 2;
    int i,j;
    for (i=0, j=0; j<final_len; i+=2, j++)
        ascii_ptr[j] = (hex_ptr[i] % 32 + 9) % 25 * 16 + (hex_ptr[i+1] % 32 + 9) % 25;
    ascii_ptr[final_len] = '\0';
}
*/
/* Execute a complete CC1101 command. */

static void exec(char *cmdline)
{ 
        
    char *command = strsep(&cmdline, " ");

  // identification of the command & actions
      
    if (strcmp_P(command, PSTR("help")) == 0) {
        Serial.println(F(
          "setmodulation <mode>         // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.\r\n\r\n"
          "setmhz <frequency>           // Here you can set your basic frequency. default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.\r\n\r\n"
          "setdeviation <deviation>     // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.\r\n\r\n"
          "setchannel <channel>         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.\r\n\r\n"
         ));
        Serial.println(F(
          "setchsp <spacing>            // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz. \r\n\r\n"
          "setrxbw <Receive bandwidh>   // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.\r\n\r\n"
          "setdrate <datarate>          // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!\r\n\r\n"
          "setpa <power value>          // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!\r\n\r\n"
          "setsyncmode  <sync mode>     // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.\r\n\r\n"
          "setsyncword <LOW, HIGH>      // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)\r\n\r\n"
         ));
        Serial.println(F(
          "setadrchk <address check>    // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.\r\n\r\n"
          "setaddr <address>            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).\r\n\r\n"
          "setwhitedata <whitening>     // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.\r\n\r\n"
          "setpktformat <pkt format>    // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.\r\n\r\n"
          "setlengthconfig <mode>       // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved \r\n\r\n"
          "setpacketlength <mode>       // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.\r\n\r\n"
         ));
        Serial.println(F(
          "setcrc <mode>                // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.\r\n\r\n"
          "setcrcaf <mode>             // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.\r\n\r\n"
          "setdcfilteroff <mode>        // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).\r\n\r\n"
          "setmanchester <mode>         // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.\r\n\r\n"
          "setfec <mode>                // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.\r\n\r\n"
          "setpre <mode>                // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24\r\n\r\n"
          "setpqt <mode>                // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.\r\n\r\n"
           ));
        Serial.println(F(
         "setappendstatus <mode>       // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.\r\n\r\n"
         "receive <mode>               // Enable or disable printing of received RF packets on serial terminal. 1 = enabled, 0 = disabled\r\n\r\n"
         "transmit <times> <hex-vals>  // Send the same packet of 64 hex values over RF \r\n\r\n"
         "jamming <mode>               // Enable or disable continous jamming on selected band. 1 = enabled, 0 = disabled\r\n\r\n"
         "record <mode>                // Enable or disable recording of single frame. 1 = enabled, 0 = disabled\r\n\r\n"
         "replay <number>              // Replay previously recorded frame on selected band number of times\r\n\r\n"
         "echo <mode>                  // Enable or disable Echo on serial terminal. 1 = enabled, 0 = disabled\r\n\r\n"
         ));
            
    } else if (strcmp_P(command, PSTR("setmodulation")) == 0) {
        int modulation = atoi(cmdline);
        ELECHOUSE_cc1101.setModulation(modulation);
        Serial.print("\r\nModulation set to ");
        Serial.print(modulation);
        Serial.print(" \r\n");
        
    } else if (strcmp_P(command, PSTR("setmhz")) == 0) {
        float frequency = atof(cmdline);
        ELECHOUSE_cc1101.setMHZ(frequency);
        Serial.print("\r\nFrequency set to ");
        Serial.print(frequency);
        Serial.print(" MHz \r\n");
        
    } else if (strcmp_P(command, PSTR("setdeviation")) == 0) {
        float deviation = atof(cmdline);
        ELECHOUSE_cc1101.setDeviation(deviation);
        Serial.print("\r\nDeviation set to ");
        Serial.print(deviation);
        Serial.print(" KHz \r\n");        
        
    } else if (strcmp_P(command, PSTR("setchannel")) == 0) {
        int channelnumber = atoi(cmdline);
        ELECHOUSE_cc1101.setChannel(channelnumber);
        Serial.print("\r\nChannel number set to ");
        Serial.print(channelnumber);
        Serial.print("\r\n");        
        
    } else if (strcmp_P(command, PSTR("setchsp")) == 0) {
        float spacing = atof(cmdline);
        ELECHOUSE_cc1101.setChsp(spacing);
        Serial.print("\r\nChannel spacing set to ");
        Serial.print(spacing);
        Serial.print(" kHz \r\n");  
        
    } else if (strcmp_P(command, PSTR("setrxbw")) == 0) {
        float rxbandwidth = atof(cmdline);
        ELECHOUSE_cc1101.setRxBW(rxbandwidth);
        Serial.print("\r\nReceive bandwidth set to ");
        Serial.print(rxbandwidth);
        Serial.print(" kHz \r\n");  
        
    } else if (strcmp_P(command, PSTR("setdrate")) == 0) {
        float datarate = atof(cmdline);
        ELECHOUSE_cc1101.setDRate(datarate);
        Serial.print("\r\nData rate set to ");
        Serial.print(datarate);
        Serial.print(" kilo bauds \r\n");  

    } else if (strcmp_P(command, PSTR("setpa")) == 0) {
        int txpower = atoi(cmdline);
        ELECHOUSE_cc1101.setPA(txpower);
        Serial.print("\r\nTransmission power set to ");
        Serial.print(txpower);
        Serial.print(" \r\n");  
        
    } else if (strcmp_P(command, PSTR("setsyncmode")) == 0) {
        int syncmode = atoi(cmdline);
        ELECHOUSE_cc1101.setSyncMode(syncmode);
        Serial.print("\r\nSynchronization mode set to ");
        Serial.print(syncmode);
        Serial.print(" \r\n");  
        
    } else if (strcmp_P(command, PSTR("setsyncword")) == 0) {
        int lowword = atoi(strsep(&cmdline, " "));
        int highword = atoi(cmdline);
        ELECHOUSE_cc1101.setSyncWord(lowword, highword);
        Serial.print("\r\nSynchronization mode set : \r\n");
        Serial.print("high word = ");
        Serial.print(highword);
        Serial.print("\r\nlow word = ");
        Serial.print(lowword);
        Serial.print(" \r\n");  

    } else if (strcmp_P(command, PSTR("setadrchk")) == 0) {
        int adrcheck = atoi(cmdline);
        ELECHOUSE_cc1101.setAdrChk(adrcheck);
        Serial.print("\r\nAddress checking mode set to ");
        Serial.print(adrcheck);
        Serial.print(" \r\n");  
        
    } else if (strcmp_P(command, PSTR("setaddr")) == 0) {
        int address = atoi(cmdline);
        ELECHOUSE_cc1101.setAddr(address);
        Serial.print("\r\nAddress set to ");
        Serial.print(address);
        Serial.print(" \r\n");  

    } else if (strcmp_P(command, PSTR("setwhitedata")) == 0) {
        int whitening = atoi(cmdline);
        ELECHOUSE_cc1101.setWhiteData(whitening);
        Serial.print("\r\nWhitening configuration set to ");
        Serial.print(whitening);
        Serial.print(" \r\n");  
        
    } else if (strcmp_P(command, PSTR("setpktformat")) == 0) {
        int pktformat = atoi(cmdline);
        ELECHOUSE_cc1101.setPktFormat(pktformat);
        Serial.print("\r\nPacket format set to ");
        Serial.print(pktformat);
        Serial.print(" \r\n");  
  
    } else if (strcmp_P(command, PSTR("setlengthconfig")) == 0) {
        int lengthconfig = atoi(cmdline);
        ELECHOUSE_cc1101.setLengthConfig(lengthconfig);
        Serial.print("\r\nPacket length mode set to ");
        Serial.print(lengthconfig);
        Serial.print(" \r\n");  
  
    } else if (strcmp_P(command, PSTR("setpacketlength")) == 0) {
        int pktlength = atoi(cmdline);
        ELECHOUSE_cc1101.setPacketLength(pktlength);
        Serial.print("\r\nPacket length set to ");
        Serial.print(pktlength);
        Serial.print(" \r\n");  
        
    } else if (strcmp_P(command, PSTR("setcrc")) == 0) {
        int pktcrc = atoi(cmdline);
        ELECHOUSE_cc1101.setCrc(pktcrc);
        Serial.print("\r\nCRC setting changed to ");
        Serial.print(pktcrc);
        Serial.print(" \r\n"); 
        
    } else if (strcmp_P(command, PSTR("setcrcaf")) == 0) {
        int pktcrcaf = atoi(cmdline);
        ELECHOUSE_cc1101.setCRC_AF(pktcrcaf);
        Serial.print("\r\nCRC Autoflush setting changed to ");
        Serial.print(pktcrcaf);
        Serial.print(" \r\n"); 
        
     } else if (strcmp_P(command, PSTR("setdcfilteroff")) == 0) {
        int dcfilter = atoi(cmdline);
        ELECHOUSE_cc1101.setDcFilterOff(dcfilter);
        Serial.print("\r\nDC filter setting changed to ");
        Serial.print(dcfilter);
        Serial.print(" \r\n"); 

     } else if (strcmp_P(command, PSTR("setmanchester")) == 0) {
        int manchset = atoi(cmdline);
        ELECHOUSE_cc1101.setManchester(manchset);
        Serial.print("\r\nManchester coding setting changed to ");
        Serial.print(manchset);
        Serial.print(" \r\n"); 

     } else if (strcmp_P(command, PSTR("setfec")) == 0) {
        int fecset = atoi(cmdline);
        ELECHOUSE_cc1101.setFEC(fecset);
        Serial.print("\r\nForward Error Correction setting changed to ");
        Serial.print(fecset);
        Serial.print(" \r\n"); 

     } else if (strcmp_P(command, PSTR("setpre")) == 0) {
        int preambleset = atoi(cmdline);
        ELECHOUSE_cc1101.setPRE(preambleset);
        Serial.print("\r\nMinimum preamble length changed to ");
        Serial.print(preambleset);
        Serial.print(" \r\n"); 

      } else if (strcmp_P(command, PSTR("setpqt")) == 0) {
        int preamblequal = atoi(cmdline);
        ELECHOUSE_cc1101.setPQT(preamblequal);
        Serial.print("\r\nPreamble quality estimator changed to ");
        Serial.print(preamblequal);
        Serial.print(" \r\n"); 

       } else if (strcmp_P(command, PSTR("setappendstatus")) == 0) {
        int appendstatus = atoi(cmdline);
        ELECHOUSE_cc1101.setAppendStatus(appendstatus);
        Serial.print("\r\nStatus bytes appending changed to ");
        Serial.print(appendstatus);
        Serial.print(" \r\n"); 

       } else if (strcmp_P(command, PSTR("receive")) == 0) {
        receivingmode = atoi(cmdline);
        Serial.print("\r\nReceiving and printing RF packet changed to ");
        Serial.print(receivingmode);
        Serial.print(" \r\n"); 

       } else if (strcmp_P(command, PSTR("jamming")) == 0) {
        jammingmode = atoi(cmdline);
        Serial.print("\r\nJamming changed to ");
        Serial.print(jammingmode);
        Serial.print(" \r\n"); 



       } else if (strcmp_P(command, PSTR("transmit")) == 0) {
        int numberofpackets = atoi(strsep(&cmdline, " "));
        // convert hex array to set of bytes
        hextoascii((byte *)textbuffer, cmdline, strlen(cmdline));        
        memcpy(ccsendingbuffer, textbuffer, strlen(cmdline)/2 );
        ccsendingbuffer[strlen(cmdline)/2 + 1] = 0x00;       
        Serial.print("\r\nTransmitting RF packets... Please wait...\r\n ");
        // blink LED RX - only for Arduino Pro Micro
        digitalWrite(RXLED, LOW);   // set the RX LED ON
        for (int i=0; i<numberofpackets; i++)  
             {
               // send these data to radio over CC1101
               ELECHOUSE_cc1101.SendData(ccsendingbuffer);
              };
        // blink LED RX - only for Arduino Pro Micro
        digitalWrite(RXLED, HIGH);   // set the RX LED OFF    
        // for DEBUG only
        asciitohex((byte *)ccsendingbuffer, (byte *)textbuffer,  strlen(cmdline)/2 );
        Serial.print("Sent payload: ");
        Serial.print((char *)textbuffer);
        Serial.print("\r\n");        

    } else if (strcmp_P(command, PSTR("record")) == 0) {
        recordingmode = atoi(cmdline);
        Serial.print("\r\nRecording mode set to ");
        Serial.print(recordingmode);
        Serial.print(" \r\n"); 

       } else if (strcmp_P(command, PSTR("replay")) == 0) {
        int numberofpackets = atoi(strsep(&cmdline, " ")); 
        // copy previously recorded frame to sending buffer buffer for replay
        memcpy(ccsendingbuffer, ccrecordingbuffer, CCBUFFERSIZE );
        Serial.print("\r\nReplaying recorded frame.\r\n ");
        // blink LED RX - only for Arduino Pro Micro
        digitalWrite(RXLED, LOW);   // set the RX LED ON
        for (int i=0; i<numberofpackets; i++)  
             {
               // send these data to radio over CC1101
               ELECHOUSE_cc1101.SendData(ccsendingbuffer);
              };
        // blink LED RX - only for Arduino Pro Micro
        digitalWrite(RXLED, HIGH);   // set the RX LED OFF    
        Serial.print("Done.\r\n");

    } else if (strcmp_P(command, PSTR("echo")) == 0) {
        do_echo = atoi(cmdline);
        
    } else {
        Serial.print(F("Error: Unknown command: "));
        Serial.println(command);
    }
}


void setup() {

     // initialize USB Serial Port CDC
     Serial.begin(9600);

     while (!Serial) {
        ; // wait until USB CDC port connects... Needed for Leonardo only
                     }
     Serial.println("CC1101 terminal tool connected, use 'help' for list of commands...\n\r");
     Serial.println("(C) Adam Loboda 2023\n\r ");

     Serial.println();  // print CRLF

     // Arduino Pro Micro - RXLED diode will be used for debug blinking
     pinMode(RXLED, OUTPUT);  // Set RX LED as an output

     // Following section enables SmartRC CC1101 library 
     // to work with Arduino Pro Micro
     // if using different board, please remove it
     // defining PINs set for Arduino Pro Micro setup
     byte sck = 15;   
     byte miso = 14;
     byte mosi = 16;
     byte ss = 10;
     int gdo0 = 3;
     // initializing library with custom pins selected
     ELECHOUSE_cc1101.setSpiPin(sck, miso, mosi, ss);

    // Main part to tune CC1101 with proper frequency, modulation and encoding    
    ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
    ELECHOUSE_cc1101.setGDO0(gdo0);         // set lib internal gdo pin (gdo0). Gdo2 not use for this example.
    ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode.
    ELECHOUSE_cc1101.setModulation(1);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setMHZ(433.92);        // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    ELECHOUSE_cc1101.setDeviation(47.60);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    ELECHOUSE_cc1101.setChannel(0);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
    ELECHOUSE_cc1101.setChsp(199.95);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
    ELECHOUSE_cc1101.setRxBW(812.50);       // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
    ELECHOUSE_cc1101.setDRate(1.2);         // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
    ELECHOUSE_cc1101.setPA(10);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
    ELECHOUSE_cc1101.setSyncMode(2);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
    ELECHOUSE_cc1101.setSyncWord(211, 145); // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
    ELECHOUSE_cc1101.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
    ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
    ELECHOUSE_cc1101.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
    ELECHOUSE_cc1101.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
    ELECHOUSE_cc1101.setLengthConfig(1);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
    ELECHOUSE_cc1101.setPacketLength(0);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
    ELECHOUSE_cc1101.setCrc(1);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
    ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
    ELECHOUSE_cc1101.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
    ELECHOUSE_cc1101.setManchester(1);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setPRE(3);             // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
    ELECHOUSE_cc1101.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
    ELECHOUSE_cc1101.setAppendStatus(0);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

}


void loop() {

  // index for serial port characters
  int i = 0;

    /* Process incoming commands. */
    while (Serial.available()) {
        static char buffer[BUF_LENGTH];
        static int length = 0;

        int data = Serial.read();
        if (data == '\b' || data == '\177') {  // BS and DEL
            if (length) {
                length--;
                if (do_echo) Serial.write("\b \b");
            }
        }
        else if (data == '\r') {
            if (do_echo) Serial.write("\r\n");    // output CRLF
            buffer[length] = '\0';
            if (length) exec(buffer);
            length = 0;
        }
        else if (length < BUF_LENGTH - 1) {
            buffer[length++] = data;
            if (do_echo) Serial.write(data);
        }
    };

    /* Process RF received packets */
   
   //Checks whether something has been received.
  if (ELECHOUSE_cc1101.CheckReceiveFlag() && (receivingmode == 1 || recordingmode ==1) )
      {
       // blink LED RX - only for Arduino Pro Micro
       digitalWrite(RXLED, LOW);   // set the RX LED ON

       //CRC Check. If "setCrc(false)" crc returns always OK!
       if (ELECHOUSE_cc1101.CheckCRC())
          { 
            //Get received Data and calculate length
            int len = ELECHOUSE_cc1101.ReceiveData(ccreceivingbuffer);

            if ( receivingmode == 1 )
               {
                   // put NULL at the end of char buffer
                   ccreceivingbuffer[len] = '\0';
                   //Print received packet as set of hex values
                   asciitohex((byte *)ccreceivingbuffer, (byte *)textbuffer,  len);
                   Serial.print("Received payload: ");
                   Serial.print((char *)textbuffer);
                   Serial.print("\r\n");
                   // Serial.print((char *) ccreceivingbuffer);
               };   // end of handling receiving mode 
               
            if ( recordingmode == 1 )
               { 
                   recordingmode = 0;  // clear recording flag
                   // copy the frame from receiving buffer for replay
                   memcpy(ccrecordingbuffer, ccreceivingbuffer, CCBUFFERSIZE );
                   // display info about recorded frame
                   // put NULL at the end of char buffer
                   ccreceivingbuffer[len] = '\0';
                   asciitohex((byte *)ccreceivingbuffer, (byte *)textbuffer,  len);
                   Serial.print("Recorded frame with  payload: ");
                   Serial.print((char *)textbuffer);
                   Serial.print("\r\n");
               };   // end of handling frame recording mode 
 
          };   // end of CRC check IF

       // blink LED RX - only for Arduino Pro Micro
       digitalWrite(RXLED, HIGH);   // set the RX LED OFF

      };   // end of Check receive flag if

      // if jamming mode activate continously send something over RF...
      if ( jammingmode == 1)
      { 
        memcpy(ccsendingbuffer, "ABCDEFGHIJKLMNOP", 16 );
        ccsendingbuffer[17] = 0x00;       
        // blink LED RX - only for Arduino Pro Micro
        digitalWrite(RXLED, LOW);   // set the RX LED ON
        // send these data to radio over CC1101
        ELECHOUSE_cc1101.SendData(ccsendingbuffer);
        // blink LED RX - only for Arduino Pro Micro
        digitalWrite(RXLED, HIGH);   // set the RX LED OFF    
      };
 
}  // end of main LOOP
