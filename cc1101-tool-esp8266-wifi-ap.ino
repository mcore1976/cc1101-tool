//
// CC1101 interactive terminal tool
// allows for sending / receiving data over serial port
// on selected radio channel, modulation, ..
//
// (C) Adam Loboda '2023 , adam.loboda@wp.pl
//  
// based on great SmartRC library by Little_S@tan
// Please download ZIP from 
// https://github.com/LSatan/SmartRC-CC1101-Driver-Lib
// and attach it as ZIP library for Arduino
//
// Also uses Arduino Command Line interpreter by Edgar Bonet
// from https://gist.github.com/edgar-bonet/607b387260388be77e96
//
// This code will ONLY work with ESP8266 board, WIFI version
//

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>

#define CCBUFFERSIZE 64
#define RECORDINGBUFFERSIZE 4096   // Buffer for recording the frames
#define EPROMSIZE 4096              // Size of EEPROM in your Arduino chip. For  ESP8266 size is 4096
#define BUF_LENGTH 128             // Buffer for the incoming command.

// defining PINs set for ESP8266 - WEMOS D1 MINI module
byte sck = 14;     // GPIO 14
byte miso = 12;  // GPIO 12
byte mosi = 13;  // GPIO 13
byte ss = 15;      // GPIO 15
int gdo0 = 5;     // GPIO 5
int gdo2 = 4;     // GPIO 4

// You need to attach ESP8266 board to your own WIFI router
#define LED_BUILTIN 2
#define TCP_PORT (23)                       // Choose any port you want
WiFiServer tcpserver(TCP_PORT);
WiFiClient tcpclient ;                      // class for handling incoming telnet connection

IPAddress ip(192, 168, 1, 100);             // Local Static IP address
IPAddress gateway(192, 168, 1, 1);        // Gateway IP address
IPAddress subnet(255, 255, 255, 0);         // Subnet Mask
const char ssid[] = "cc1101";              // Change to your ESP8266 AP SSID
const char password[] = "cc1101";          // Change to your ESP8266 AP Password



// position in big recording buffer
int bigrecordingbufferpos = 0; 

// number of frames in big recording buffer
int framesinbigrecordingbuffer = 0; 

// check if CLI receiving mode enabled
int receivingmode = 0; 

// check if CLI jamming mode enabled
int jammingmode = 0; 

// check if CLI recording mode enabled
int recordingmode = 0; 

// check if CLI chat mode enabled
int chatmode = 0; 

static bool do_echo = true;

// buffer for receiving  CC1101
byte ccreceivingbuffer[CCBUFFERSIZE] = {0};

// buffer for sending  CC1101
byte ccsendingbuffer[CCBUFFERSIZE] = {0};
//char ccsendingbuffer[CCBUFFERSIZE] = {0};

// buffer for recording and replaying of many frames
byte bigrecordingbuffer[RECORDINGBUFFERSIZE] = {0};

// buffer for hex to ascii conversions 
byte textbuffer[BUF_LENGTH];
//char textbuffer[BUF_LENGTH];


// convert bytes in table to string with hex numbers
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
      hex_ptr[(2*i)+1] = k ; 
    };
    hex_ptr[(2*i)+2] = '\0' ; 
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
    ascii_ptr[i++] = '\0' ;
}

// Initialize CC1101 board with default settings, you may change your preferences here
static void cc1101initialize(void)
{
    // initializing library with custom pins selected
     ELECHOUSE_cc1101.setSpiPin(sck, miso, mosi, ss);
     ELECHOUSE_cc1101.setGDO(gdo0, gdo2);

    // Main part to tune CC1101 with proper frequency, modulation and encoding    
    ELECHOUSE_cc1101.Init();                // must be set to initialize the cc1101!
    ELECHOUSE_cc1101.setGDO0(gdo0);         // set lib internal gdo pin (gdo0). Gdo2 not use for this example.
    ELECHOUSE_cc1101.setCCMode(1);          // set config for internal transmission mode. value 0 is for RAW recording/replaying
    ELECHOUSE_cc1101.setModulation(2);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
    ELECHOUSE_cc1101.setMHZ(433.92);        // Here you can set your basic frequency. The lib calculates the frequency automatically (default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
    ELECHOUSE_cc1101.setDeviation(47.60);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    ELECHOUSE_cc1101.setChannel(0);         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
    ELECHOUSE_cc1101.setChsp(199.95);       // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz.
    ELECHOUSE_cc1101.setRxBW(812.50);       // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
    ELECHOUSE_cc1101.setDRate(9.6);         // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
    ELECHOUSE_cc1101.setPA(10);             // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
    ELECHOUSE_cc1101.setSyncMode(2);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
    ELECHOUSE_cc1101.setSyncWord(211, 145); // Set sync word. Must be the same for the transmitter and receiver. Default is 211,145 (Syncword high, Syncword low)
    ELECHOUSE_cc1101.setAdrChk(0);          // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
    ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
    ELECHOUSE_cc1101.setWhiteData(0);       // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
    ELECHOUSE_cc1101.setPktFormat(0);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
    ELECHOUSE_cc1101.setLengthConfig(1);    // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved
    ELECHOUSE_cc1101.setPacketLength(0);    // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
    ELECHOUSE_cc1101.setCrc(0);             // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
    ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
    ELECHOUSE_cc1101.setDcFilterOff(0);     // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
    ELECHOUSE_cc1101.setManchester(0);      // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setFEC(0);             // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.
    ELECHOUSE_cc1101.setPRE(0);             // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
    ELECHOUSE_cc1101.setPQT(0);             // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
    ELECHOUSE_cc1101.setAppendStatus(0);    // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.
}


// Execute a complete CC1101 command.

static void exec(char *cmdline)
{ 
        
    char *command = strsep(&cmdline, " ");
    char destination[8];
    int setting, setting2, len;
    byte j, k;
    uint16_t brute, poweroftwo;   
    float settingf1;
    float settingf2;
    // variables for frequency scanner
    float freq;
    long compare_freq;
    float mark_freq;
    int rssi;
    int mark_rssi=-100; 

  // identification of the command & actions
      
    if (strcmp_P(command, PSTR("help")) == 0) {
        tcpclient.write(
          "setmodulation <mode> : Set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.\r\n\r\n"
          "setmhz <frequency>   : Here you can set your basic frequency. default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.\r\n\r\n"
          "setdeviation <deviation> : Set the Frequency deviation in kHz. Value from 1.58 to 380.85.\r\n\r\n"
          "setchannel <channel> : Set the Channelnumber from 0 to 255. Default is cahnnel 0.\r\n\r\n"
          "setchsp <spacing>  :  The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. \r\n\r\n"
          "setrxbw <Receive bndwth> : Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. \r\n\r\n"
          "setdrate <datarate> : Set the Data Rate in kBaud. Value from 0.02 to 1621.83.\r\n\r\n"
          "setpa <power value> : Set RF transmission power. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!\r\n\r\n"
          "setsyncmode  <sync mode> : Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.\r\n"
         );
          yield();       
        tcpclient.write(
          "setsyncword <decimal LOW, decimal HIGH> : Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low) Default is 211,145\r\n\r\n"
          "setadrchk <address chk> : Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.\r\n\r\n"
          "setaddr <address> : Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).\r\n\r\n"
          "setwhitedata <whitening> : Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.\r\n\r\n"
          "setpktformat <pktformat> : Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator.  3 = Asynchronous serial mode\r\n\r\n"
          "setlengthconfig <mode> : Set packet Length mode : 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved \r\n\r\n"
          "setpacketlength <mode> : Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.\r\n\r\n"
          "setcrc <mode> : Switches on/of CRC calculation and check. 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.\r\n\r\n"
          "setcrcaf <mode> : Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.\r\n"
         );
          yield();
        tcpclient.write(
          "setdcfilteroff <mode> : Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).\r\n\r\n"
          "setmanchester <mode> : Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.\r\n\r\n"
          "setfec <mode> : Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.\r\n\r\n"
          "setpre <mode> : Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24\r\n\r\n"
          "setpqt <mode> : Preamble quality estimator threshold. \r\n\r\n"
          "setappendstatus <mode> : When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.\r\n\r\n"
          "getrssi : Display quality information about last received frames over RF\r\n\r\n"
          "scan <start> <stop> : Scan frequency range for the highest signal.\r\n\r\n"         
          "chat :  Enable chat mode between many devices. No exit available, disconnect device to quit\r\n"
         );
          yield();
         tcpclient.write(
          "rx : Sniffer. Enable or disable printing of received RF packets on serial terminal.\r\n\r\n"
          "tx <hex-vals> : Send packet of max 60 bytes <hex values> over RF\r\n\r\n"
          "jam : Enable or disable continous jamming on selected band.\r\n\r\n"
          "brute <microseconds> <number-of-bits> : Brute force garage gate with <nb-of-bits> keyword where symbol time is <usec>.\r\n\r\n"
          "rec : Enable or disable recording frames in the buffer.\r\n\r\n"
          "add <hex-vals> : Manually add single frame payload (max 64 hex values) to the buffer so it can be replayed\r\n\r\n"
          "show : Show content of recording buffer\r\n\r\n"
          "flush : Clear the recording buffer\r\n\r\n"
          "play <N> : Replay 0 = all frames or N-th recorded frame previously stored in the buffer.\r\n\r\n"
          "rxraw <microseconds> : Sniffs radio by sampling with <microsecond> interval and prints received bytes in hex.\r\n\r\n"
          "recraw <microseconds> : Recording RAW RF data with <microsecond> sampling interval.\r\n"
            );
          yield();
        tcpclient.write(
          "addraw <hex-vals> : Manually add chunks (max 60 hex values) to the buffer so they can be further replayed.\r\n\r\n"        
          "showraw : Showing content of recording buffer in RAW format.\r\n\r\n"
          "playraw <microseconds> : Replaying previously recorded RAW RF data with <microsecond> sampling interval.\r\n\r\n"
          "showbit : Showing content of recording buffer in RAW format as a stream of bits.\r\n\r\n"
          "save : Store recording buffer content in non-volatile memory\r\n\r\n"
          "load : Load the content from non-volatile memory to the recording buffer\r\n\r\n"
          "echo <mode> : Enable or disable Echo on serial terminal. 1 = enabled, 0 = disabled\r\n\r\n"
          "x : Stops jamming/receiving/recording packets.\r\n\r\n"
          "init : Restarts CC1101 board with default parameters\r\n\r\n"
            );
          yield();

    // Handling SETMODULATION command 
    } else if (strcmp_P(command, PSTR("setmodulation")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setModulation(setting);
        tcpclient.write("\r\nModulation: ");
        if (setting == 0) { tcpclient.write("2-FSK"); }
        else if (setting == 1) { tcpclient.write("GFSK"); }
        else if (setting == 2) { tcpclient.write("ASK/OOK"); }
        else if (setting == 3) { tcpclient.write("4-FSK"); }
        else if (setting == 4) { tcpclient.write("MSK"); };  
        tcpclient.write(" \r\n");
        yield();

    // Handling SETMHZ command 
    } else if (strcmp_P(command, PSTR("setmhz")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setMHZ(settingf1);
        tcpclient.write("\r\nFrequency: ");
        dtostrf(settingf1, 6,2 , destination);         
        tcpclient.write(destination);
        tcpclient.write(" MHz\r\n");
        yield();
        
    // Handling SETDEVIATION command 
    } else if (strcmp_P(command, PSTR("setdeviation")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setDeviation(settingf1);
        tcpclient.write("\r\nDeviation: ");
        dtostrf(settingf1, 6,2 , destination);         
        tcpclient.write(destination);
        tcpclient.write(" KHz\r\n");        
        yield();

    // Handling SETCHANNEL command       
    } else if (strcmp_P(command, PSTR("setchannel")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setChannel(setting);
        tcpclient.write("\r\nChannel:");
        itoa(setting,destination,10);
        tcpclient.write(destination);
        tcpclient.write("\r\n");        
        yield();

    // Handling SETCHSP command 
    } else if (strcmp_P(command, PSTR("setchsp")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setChsp(settingf1);
        tcpclient.write("\r\nChann spacing: ");
        dtostrf(settingf1, 6,2 , destination);         
        tcpclient.write(destination);
        tcpclient.write(" kHz\r\n");  
        yield();

    // Handling SETRXBW command         
    } else if (strcmp_P(command, PSTR("setrxbw")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setRxBW(settingf1);
        tcpclient.write("\r\nRX bandwidth: ");
        dtostrf(settingf1, 6,2 , destination);         
        tcpclient.write(destination);
        tcpclient.write(" kHz \r\n");  
        yield();

    // Handling SETDRATE command         
    } else if (strcmp_P(command, PSTR("setdrate")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setDRate(settingf1);
        tcpclient.write("\r\nDatarate: ");
        dtostrf(settingf1, 6,2 , destination);         
        tcpclient.write(destination);
        tcpclient.write(" kbaud\r\n");  
        yield();

    // Handling SETPA command         
    } else if (strcmp_P(command, PSTR("setpa")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPA(setting);
        tcpclient.write("\r\nTX PWR: ");
        itoa(setting,destination,10);
        tcpclient.write(destination);
        tcpclient.write(" dBm\r\n");  
        yield();
        
    // Handling SETSYNCMODE command         
    } else if (strcmp_P(command, PSTR("setsyncmode")) == 0) {
        int setting = atoi(cmdline);
        ELECHOUSE_cc1101.setSyncMode(setting);
        tcpclient.write("\r\nSynchronization: ");
        if (setting == 0) { tcpclient.write("No preamble"); }
        else if (setting == 1) { tcpclient.write("16 sync bits"); }
        else if (setting == 2) { tcpclient.write("16/16 sync bits"); }
        else if (setting == 3) { tcpclient.write("30/32 sync bits"); }
        else if (setting == 4) { tcpclient.write("No preamble/sync, carrier-sense"); }
        else if (setting == 5) { tcpclient.write("15/16 + carrier-sense"); }
        else if (setting == 6) { tcpclient.write("16/16 + carrier-sense"); }
        else if (setting == 7) { tcpclient.write("30/32 + carrier-sense"); };
        tcpclient.write("\r\n");  
        yield();
        
    // Handling SETSYNCWORD command         
    } else if (strcmp_P(command, PSTR("setsyncword")) == 0) {
        setting = atoi(strsep(&cmdline, " "));
        setting2 = atoi(cmdline);
        ELECHOUSE_cc1101.setSyncWord(setting2, setting);
        tcpclient.write("\r\nSynchronization:\r\n");
        tcpclient.write("high = ");
        itoa(setting,destination,10);
        tcpclient.write(destination);
        tcpclient.write("\r\nlow = ");
        itoa(setting2,destination,10);
        tcpclient.write(destination);
        tcpclient.write("\r\n");  
        yield();
    
    // Handling SETADRCHK command         
    } else if (strcmp_P(command, PSTR("setadrchk")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAdrChk(setting);
        tcpclient.write("\r\nAddress checking:");
        if (setting == 0) { tcpclient.write("No adr chk"); }
        else if (setting == 1) { tcpclient.write("Adr chk, no bcast"); }
        else if (setting == 2) { tcpclient.write("Adr chk and 0 bcast"); }
        else if (setting == 3) { tcpclient.write("Adr chk and 0 and FF bcast"); };
        tcpclient.write("\r\n");  
        yield();
        
    // Handling SETADDR command         
    } else if (strcmp_P(command, PSTR("setaddr")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAddr(setting);
        tcpclient.write("\r\nAddress: ");
        itoa(setting,destination,10);
        tcpclient.write(destination);
        tcpclient.write("\r\n");  
        yield();

    // Handling SETWHITEDATA command         
    } else if (strcmp_P(command, PSTR("setwhitedata")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setWhiteData(setting);
        tcpclient.write("\r\nWhitening ");
        if (setting == 0) { tcpclient.write("OFF"); }
        else if (setting == 1) { tcpclient.write("ON"); }
        tcpclient.write("\r\n");  
        yield();
        
    // Handling SETPKTFORMAT command         
    } else if (strcmp_P(command, PSTR("setpktformat")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPktFormat(setting);
        tcpclient.write("\r\nPacket format: ");
        if (setting == 0) { tcpclient.write("Normal mode"); }
        else if (setting == 1) { tcpclient.write("Synchronous serial mode"); }
        else if (setting == 2) { tcpclient.write("Random TX mode"); }
        else if (setting == 3) { tcpclient.write("Asynchronous serial mode"); };
        tcpclient.write("\r\n");  
        yield();
  
    // Handling SETLENGTHCONFIG command         
    } else if (strcmp_P(command, PSTR("setlengthconfig")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setLengthConfig(setting);
        tcpclient.write("\r\nPkt length mode: ");
        if (setting == 0) { tcpclient.write("Fixed"); }
        else if (setting == 1) { tcpclient.write("Variable"); }
        else if (setting == 2) { tcpclient.write("Infinite"); }
        else if (setting == 3) { tcpclient.write("Reserved"); };
        tcpclient.write("\r\n");  
  
    // Handling SETPACKETLENGTH command         
    } else if (strcmp_P(command, PSTR("setpacketlength")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPacketLength(setting);
        tcpclient.write("\r\nPkt length: ");
        itoa(setting,destination,10);
        tcpclient.write(destination);
        tcpclient.write(" bytes\r\n");  
        yield();
        
    // Handling SETCRC command         
    } else if (strcmp_P(command, PSTR("setcrc")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setCrc(setting);
        tcpclient.write("\r\nCRC checking: ");
        if (setting == 0) { tcpclient.write("Disabled"); }
        else if (setting == 1) { tcpclient.write("Enabled"); };
        tcpclient.write("\r\n"); 
        yield();
        
    // Handling SETCRCAF command         
    } else if (strcmp_P(command, PSTR("setcrcaf")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setCRC_AF(setting);
        tcpclient.write("\r\nCRC Autoflush: ");
        if (setting == 0) { tcpclient.write("Disabled"); }
        else if (setting == 1) { tcpclient.write("Enabled"); };
         tcpclient.write("\r\n"); 
        
    // Handling SETDCFILTEROFF command         
     } else if (strcmp_P(command, PSTR("setdcfilteroff")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setDcFilterOff(setting);
        tcpclient.write("\r\nDC filter: ");
        if (setting == 0) { tcpclient.write("Enabled"); }
        else if (setting == 1) { tcpclient.write("Disabled"); };
        tcpclient.write("\r\n"); 
        yield();

    // Handling SETMANCHESTER command         
     } else if (strcmp_P(command, PSTR("setmanchester")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setManchester(setting);
        tcpclient.write("\r\nManchester coding: ");
        if (setting == 0) { tcpclient.write("Disabled"); }
        else if (setting == 1) { tcpclient.write("Enabled"); };
        tcpclient.write("\r\n"); 
        yield();

    // Handling SETFEC command         
     } else if (strcmp_P(command, PSTR("setfec")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setFEC(setting);
        tcpclient.write("\r\nForward Error Correction: ");
        if (setting == 0) { tcpclient.write("Disabled"); }
        else if (setting == 1) { tcpclient.write("Enabled"); };
        tcpclient.write("\r\n"); 
        yield();
        
    // Handling SETPRE command         
     } else if (strcmp_P(command, PSTR("setpre")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPRE(setting);
        tcpclient.write("\r\nMinimum preamble bytes:");
        itoa(setting,destination,10);
        tcpclient.write(destination);
        tcpclient.write(" means 0 = 2 bytes, 1 = 3b, 2 = 4b, 3 = 6b, 4 = 8b, 5 = 12b, 6 = 16b, 7 = 24 bytes\r\n"); 
        tcpclient.write("\r\n"); 
        yield();

  
    // Handling SETPQT command         
      } else if (strcmp_P(command, PSTR("setpqt")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPQT(setting);
        tcpclient.write("\r\nPQT: ");
        itoa(setting,destination,10);
        tcpclient.write(destination);
        tcpclient.write("\r\n"); 
        yield();

    // Handling SETAPPENDSTATUS command         
       } else if (strcmp_P(command, PSTR("setappendstatus")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAppendStatus(setting);
        tcpclient.write("\r\nStatus bytes appending: ");
        if (setting == 0) { tcpclient.write("Enabled"); }
        else if (setting == 1) { tcpclient.write("Disabled"); };
        tcpclient.write("\r\n"); 
        yield();

    // Handling GETRSSI command         
      } else if (strcmp_P(command, PSTR("getrssi")) == 0) {
        //Rssi Level in dBm
        tcpclient.write("Rssi: ");
        setting = ELECHOUSE_cc1101.getRssi();
        itoa(setting,destination,10);
        tcpclient.write(destination);        
        //Link Quality Indicator
        tcpclient.write(" LQI: ");
        setting = ELECHOUSE_cc1101.getLqi();        
        itoa(setting,destination,10);
        tcpclient.write(destination);        
        tcpclient.write("\r\n"); 
        yield();


    // Handling SCAN command - frequency scanner by Little S@tan !
    } else if (strcmp_P(command, PSTR("scan")) == 0) {
        settingf1 = atof(strsep(&cmdline, " "));
        settingf2 = atof(cmdline);
        tcpclient.write("\r\nScanning frequency range from : ");
        // dtostrf(floatValue, minStringWidth, numAfterDecimal, charBuf_to_store_string);                            
        dtostrf(settingf1, 6,2 , destination);          
        tcpclient.write(destination);
        tcpclient.write(" MHz to ");
        dtostrf(settingf2, 6,2 , destination);          
        tcpclient.write(destination);
        tcpclient.write(" MHz, press any key for stop or wait...\r\n");  
        // initialize parameters for scanning
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setRxBW(58);
        ELECHOUSE_cc1101.SetRx();
        // Do scanning until some key pressed
        freq = settingf1;  // start frequency for scanning
        mark_rssi=-100;   
      
        while (tcpclient.read()<0)        
          {
            // feed the watchdog
            ESP.wdtFeed();
            yield();

            ELECHOUSE_cc1101.setMHZ(freq);
            rssi = ELECHOUSE_cc1101.getRssi();
            if (rssi>-75)
               {
                    if (rssi > mark_rssi)
                    {
                          mark_rssi = rssi;  
                          mark_freq = freq;
                    };
              };

           freq+=0.01;

           if (freq > settingf2)
              {
                   freq = settingf1;

                   if (mark_rssi>-75)
                    {
                      long fr = mark_freq*100;
                      if (fr == compare_freq)
                          {
                            tcpclient.write("\r\nSignal found at  ");
                            tcpclient.write("Freq: ");
                            // dtostrf(floatValue, minStringWidth, numAfterDecimal, charBuf_to_store_string);                            
                            dtostrf(mark_freq, 6,2 , destination);                            
                            tcpclient.write(destination);                          
                            tcpclient.write(" Rssi: "); 
                            itoa(mark_rssi,destination,10);
                            tcpclient.write(destination);
                            mark_rssi=-100;
                            compare_freq = 0;
                            mark_freq = 0;
                          }
                      else
                          {
                            compare_freq = mark_freq*100;
                            freq = mark_freq -0.10;
                            mark_freq=0;
                            mark_rssi=-100;
                          };
                    };
                    
              }; // end of IF freq>stop frequency 
              
          };  // End of While 


    // handling SAVE command
    } else if (strcmp_P(command, PSTR("save")) == 0) {
        //start saving recording buffer content into EEPROM non-volatile memory 
        tcpclient.write("\r\nSaving recording buffer content into the non-volatile memory...\r\n");
        
        for (setting=0; setting<EPROMSIZE ; setting++)  
           {  // copying byte after byte from SRAM to EEPROM
            EEPROM.write(setting, bigrecordingbuffer[setting] );
           };
        // following command is required for ESP32
        EEPROM.commit();   
        // print confirmation
        tcpclient.write("\r\nSaving complete.\r\n\r\n");
        yield();
        
                 
    // handling LOAD command
    } else if (strcmp_P(command, PSTR("load")) == 0) {     
         // first flushing bigrecordingbuffer with zeros and rewinding all the pointers 
        for (setting = 0; setting<RECORDINGBUFFERSIZE; setting++)  bigrecordingbuffer[setting] = 0;  
        // and rewinding all the pointers to the recording buffer
        bigrecordingbufferpos = 0;
        framesinbigrecordingbuffer = 0;     
        //start loading EEPROM non-volatile memory content into recording buffer
        tcpclient.write("\r\nLoading content from the non-volatile memory into the recording buffer...\r\n");
        
        for (setting=0; setting<EPROMSIZE ; setting++)  
           { // copying byte after byte from EEPROM to SRAM 
            bigrecordingbuffer[setting] = EEPROM.read(setting);
           }
        tcpclient.write("\r\nLoading complete. Enter 'show' or 'showraw' to see the buffer content.\r\n\r\n");
        yield();
                  


    // Handling RX command         
       } else if (strcmp_P(command, PSTR("rx")) == 0) {
        tcpclient.write("\r\nReceiving and printing RF packet changed to ");
        if (receivingmode == 1) {
          receivingmode = 0;
          tcpclient.write("Disabled"); }
        else if (receivingmode == 0)
               { ELECHOUSE_cc1101.SetRx();
                 tcpclient.write("Enabled"); 
                 receivingmode = 1;
                 jammingmode = 0; 
                 recordingmode = 0;
               };
        tcpclient.write("\r\n"); 
        yield();
 

    // Handling CHAT command         
       } else if (strcmp_P(command, PSTR("chat")) == 0) {

       // feed the watchdog
       ESP.wdtFeed();
       // needed for ESP8266   
       yield(); 
       delay(200);
           
        tcpclient.write("\r\nEntering chat mode:\r\n\r\n");
        //
        if (chatmode == 0) 
           { 
             chatmode = 1;
             jammingmode = 0;
             receivingmode = 0;
             recordingmode = 0;
           };
    

    // Handling JAM command         
       } else if (strcmp_P(command, PSTR("jam")) == 0) {
        tcpclient.write("\r\nJamming changed to ");
        if (jammingmode == 1) 
           { tcpclient.write("Disabled"); 
             jammingmode = 0;
           }
        else if (jammingmode == 0) 
               { 
                 tcpclient.write("Enabled"); 
                 jammingmode = 1;
                 receivingmode = 0; };
        tcpclient.write("\r\n"); 
        yield();
    
    // handling BRUTE command
    } else if (strcmp_P(command, PSTR("brute")) == 0) {
      
        // take interval period for sampling
        setting = atoi(strsep(&cmdline, " "));
        // take number of bits for brute forcing
        setting2 = atoi(cmdline);
        // calculate power of 2 upon setting
        poweroftwo = 1 << setting2;
               
        if (setting>0)
        {        
        // setup async mode on CC1101 and go into TX mode
        // with GDO0 pin processing
        ELECHOUSE_cc1101.setCCMode(0); 
        ELECHOUSE_cc1101.setPktFormat(3);
        ELECHOUSE_cc1101.SetTx();
        
        //start playing RF with setting GDO0 bit state with bitbanging
        tcpclient.write("\r\nStarting Brute Forcing press any key to stop...\r\n");
        pinMode(gdo0, OUTPUT);
     
        for (brute = 0; brute < poweroftwo ; brute++)  
           { 
             // blink ESP8266 led - turn it on
             digitalWrite(LED_BUILTIN, HIGH);
             for(int j = (setting2 -1); j > -1; j--)  // j bits in a value brute
               {
                 digitalWrite(gdo0, bitRead(brute, j)); // Set GDO0 according to actual brute force value
                 delayMicroseconds(setting);            // delay for selected sampling interval
               }; 
             // checking if key pressed
             if (tcpclient.read()>0) break;           
             // blink ESP8266 led - turn it off
             digitalWrite(LED_BUILTIN, LOW);             
             // watchdog
             yield(); 
             // feed the watchdog in ESP8266
             ESP.wdtFeed();                
           };

        tcpclient.write("\r\nBrute forcing complete.\r\n\r\n");
        
        // setting normal pkt format again
        ELECHOUSE_cc1101.setCCMode(1); 
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.SetTx();
        // pinMode(gdo0pin, INPUT);
        } // end of IF
        
        else { tcpclient.write("Wrong parameters.\r\n"); };

  

    // Handling TX command         
       } else if (strcmp_P(command, PSTR("tx")) == 0) {
        // convert hex array to set of bytes
        if ((strlen(cmdline)<=120) && (strlen(cmdline)>0) )
        { 
                hextoascii(textbuffer,(byte *)cmdline, strlen(cmdline));        
                memcpy(ccsendingbuffer, textbuffer, strlen(cmdline)/2 );
                ccsendingbuffer[strlen(cmdline)/2] = 0x00;       
                tcpclient.write("\r\nTransmitting RF packets.\r\n");
                // blink ESP8266 led - turn it on
                digitalWrite(LED_BUILTIN, HIGH);
                // send these data to radio over CC1101
                ELECHOUSE_cc1101.SendData(ccsendingbuffer, (byte)(strlen(cmdline)/2));
                // blink ESP8266 led - turn it off
                digitalWrite(LED_BUILTIN, LOW);                
                // for DEBUG only
                asciitohex(ccsendingbuffer, textbuffer,  strlen(cmdline)/2 );
                tcpclient.write("Sent frame: ");
                tcpclient.write((char *)textbuffer);
                tcpclient.write("\r\n"); }
         else { tcpclient.write("Wrong parameters.\r\n"); };
        yield();



    // handling RECRAW command
    } else if (strcmp_P(command, PSTR("recraw")) == 0) {
        // take interval period for samplink
        setting = atoi(cmdline);
        if (setting>0)
        {
        // setup async mode on CC1101 with GDO0 pin processing
        ELECHOUSE_cc1101.setCCMode(0); 
        ELECHOUSE_cc1101.setPktFormat(3);
        ELECHOUSE_cc1101.SetRx();

        //start recording to the buffer with bitbanging of GDO0 pin state
        tcpclient.write("\r\nWaiting for radio signal to start RAW recording...\r\n");
        pinMode(gdo0, INPUT);
        // feed the watchdog
        ESP.wdtFeed();
        // needed for ESP8266
        yield();        

        // this is only for ESP32 boards because they are getting some noise on the beginning
        setting2 = digitalRead(gdo0);
        delay(1);  
            
        // waiting for some data first
        // feed the watchdog while waiting for the RF signal    
        while ( digitalRead(gdo0) == LOW ) 
                {  yield(); 
                   // feed the watchdog in ESP8266
                   ESP.wdtFeed();                  
                };
                   
            
        //start recording to the buffer with bitbanging of GDO0 pin state
        tcpclient.write("\r\nStarting RAW recording to the buffer...\r\n");
        pinMode(gdo0, INPUT);

        // temporarly disable WDT for the time of recording
        // ESP.wdtDisable();
        // start recording RF signal        
        for (int i=0; i<RECORDINGBUFFERSIZE ; i++)  
           { 
             byte receivedbyte = 0;
             for(int j=7; j > -1; j--)  // 8 bits in a byte
               {
                 bitWrite(receivedbyte, j, digitalRead(gdo0)); // Capture GDO0 state into the byte
                 delayMicroseconds(setting);                   // delay for selected sampling interval
               }; 
                 // store the output into recording buffer
             bigrecordingbuffer[i] = receivedbyte;
             // feed the watchdog in ESP8266
             ESP.wdtFeed();  
             // needed for ESP8266
             yield();            
           }
        // enable WDT 
        // ESP.wdtEnable(5000);
        
        tcpclient.write("\r\nRecording RAW data complete.\r\n\r\n");
        // setting normal pkt format again
        ELECHOUSE_cc1101.setCCMode(1); 
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.SetRx();
        // feed the watchdog
        ESP.wdtFeed();        
        // needed for ESP8266   
        yield();            
        }
        else { tcpclient.write("Wrong parameters.\r\n"); };

   // handling RXRAW command - sniffer
    } else if (strcmp_P(command, PSTR("rxraw")) == 0) {
        // take interval period for samplink
        setting = atoi(cmdline);
        if (setting>0)
        {
        // setup async mode on CC1101 with GDO0 pin processing
        ELECHOUSE_cc1101.setCCMode(0); 
        ELECHOUSE_cc1101.setPktFormat(3);
        ELECHOUSE_cc1101.SetRx();
        //start recording to the buffer with bitbanging of GDO0 pin state
        tcpclient.write("\r\nSniffer enabled...\r\n");
        pinMode(gdo0, INPUT);    
        // feed the watchdog
        ESP.wdtFeed();
        // needed for ESP8266
        yield();          

        // temporarly disable WDT for the time of recording
        // ESP.wdtDisable();       
       // Any received char over Serial port stops printing  RF received bytes
        while (tcpclient.read()<0)         
           {  
             
             // we have to use the buffer not to introduce delays
             for (int i=0; i<RECORDINGBUFFERSIZE ; i++)  
                { 
                  byte receivedbyte = 0;
                  for(int j=7; j > -1; j--)  // 8 bits in a byte
                    {
                       bitWrite(receivedbyte, j, digitalRead(gdo0));  // Capture GDO0 state into the byte
                       delayMicroseconds(setting);                    // delay for selected sampling interval
                    }; 
                    // store the output into recording buffer
                    bigrecordingbuffer[i] = receivedbyte;
                    // feed the watchdog
                    ESP.wdtFeed();
                    // needed for ESP8266;
                    yield();
                  }; 
             // enable WDT 
             // ESP.wdtEnable(5000);        
             // feed the watchdog
             ESP.wdtFeed();        
             // needed for ESP8266   
             yield();      
                  
             // when buffer full print the ouptput to serial port
             for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)  
                    { 
                       asciitohex(&bigrecordingbuffer[i], textbuffer,  32);
                       tcpclient.write((char *)textbuffer);
                       // feed the watchdog
                       ESP.wdtFeed();
                       // needed foe ESP8266                    
                       yield();
                    };
                    
 
           }; // end of While loop
           
        tcpclient.write("\r\nStopping the sniffer.\n\r\n");
        
        // setting normal pkt format again
        ELECHOUSE_cc1101.setCCMode(1); 
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.SetRx();
        }
        else { tcpclient.write("Wrong parameters.\r\n"); };


    // handling PLAYRAW command
    } else if (strcmp_P(command, PSTR("playraw")) == 0) {
        // take interval period for sampling
        setting = atoi(cmdline);
        if (setting>0)
        {
        // setup async mode on CC1101 and go into TX mode
        // with GDO0 pin processing
        ELECHOUSE_cc1101.setCCMode(0); 
        ELECHOUSE_cc1101.setPktFormat(3);
        ELECHOUSE_cc1101.SetTx();
        // blink ESP8266 led - turn it on
         digitalWrite(LED_BUILTIN, HIGH);        
        //start replaying GDO0 bit state from data in the buffer with bitbanging 
        tcpclient.write("\r\nReplaying RAW data from the buffer...\r\n");        
        pinMode(gdo0, OUTPUT);
        // feed the watchdog
        ESP.wdtFeed();
        // needed for ESP8266
        yield();        

        // temporarly disable WDT for the time of recording
        // ESP.wdtDisable();
        // start RF replay
        for (int i=1; i<RECORDINGBUFFERSIZE ; i++)  
           { 
             byte receivedbyte = bigrecordingbuffer[i];
             for(int j=7; j > -1; j--)  // 8 bits in a byte
               {
                 digitalWrite(gdo0, bitRead(receivedbyte, j)); // Set GDO0 according to recorded byte
                 delayMicroseconds(setting);                      // delay for selected sampling interval
               }; 
              // feed the watchdog
              ESP.wdtFeed();
              // needed for ESP8266
              yield();
           };
        // Enable WDT 
        // ESP.wdtEnable(5000);
        
        // blink ESP8266 led - turn it off
         digitalWrite(LED_BUILTIN, LOW);           
        tcpclient.write("\r\nReplaying RAW data complete.\r\n\r\n");
        // setting normal pkt format again
        ELECHOUSE_cc1101.setCCMode(1); 
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.SetTx();
        // pinMode(gdo0pin, INPUT);
        // feed the watchdog
        ESP.wdtFeed();        
        // needed for ESP8266   
        yield();      
        }
        else { tcpclient.write("Wrong parameters.\r\n"); };

    // handling SHOWRAW command
    } else if (strcmp_P(command, PSTR("showraw")) == 0) {
    // show the content of recorded RAW signal as hex numbers
       tcpclient.write("\r\nRecorded RAW data:\r\n");
       for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)  
           { 
                    asciitohex(&bigrecordingbuffer[i], textbuffer,  32);
                    tcpclient.write((char *)textbuffer);
                    // feed the watchdog
                    ESP.wdtFeed();
                    // needed for ESP8266   
                    yield();                      
           };
       tcpclient.write("\r\n\r\n");
       // feed the watchdog
       ESP.wdtFeed();
       // needed for ESP8266   
       yield();      
      


    // handling SHOWBIT command
    } else if (strcmp_P(command, PSTR("showbit")) == 0) {
    // show the content of recorded RAW signal as hex numbers
       tcpclient.write("\r\nRecorded RAW data as bit stream:\r\n");
       for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)  
           {        // first convert to hex numbers
                    asciitohex((byte *)&bigrecordingbuffer[i], (byte *)textbuffer,  32);
                    // now decode as binary and print
                    for (setting = 0; setting < 64 ; setting++)
                        {
                        setting2 = textbuffer[setting];
                        switch( setting2 )
                              {
                              case '0':
                              tcpclient.write("____");
                              break;
   
                              case '1':
                              tcpclient.write("___-");
                              break;
   
                              case '2':
                              tcpclient.write("__-_");
                              break;

                              case '3':
                              tcpclient.write("__--");
                              break;

                              case '4':
                              tcpclient.write("_-__");
                              break;

                              case '5':
                              tcpclient.write("_-_-");
                              break;

                              case '6':
                              tcpclient.write("_--_");
                              break;

                              case '7':
                              tcpclient.write("_---");
                              break;

                              case '8':
                              tcpclient.write("-___");
                              break;

                              case '9':
                              tcpclient.write("-__-");
                              break;

                              case 'A':
                              tcpclient.write("-_-_");
                              break;

                              case 'B':
                              tcpclient.write("-_--");
                              break;

                              case 'C':
                              tcpclient.write("--__");
                              break;

                              case 'D':
                              tcpclient.write("--_-");
                              break;

                              case 'E':
                              tcpclient.write("---_");
                              break;

                              case 'F':
                              tcpclient.write("----");
                              break;
                              
                              }; // end of switch
                              
                        }; // end of for
              // feed the watchdog
              ESP.wdtFeed();
              // needed for ESP8266   
              yield();      
   
              } // end of for
              tcpclient.write("\r\n\r\n");


    // Handling ADDRAW command         
       } else if (strcmp_P(command, PSTR("addraw")) == 0) {
        // getting hex numbers - the content of the  frame 
        len = strlen(cmdline);
        // convert hex array to set of bytes
        if ((len<=120) && (len>0) )
        { 
                // convert the hex content to array of bytes
                hextoascii(textbuffer, (byte *)cmdline, len);        
                len = len /2;
                // check if the frame fits into the buffer and store it
                if (( bigrecordingbufferpos + len) < RECORDINGBUFFERSIZE) 
                     { // copy current frame and increase pointer for next frames
                      memcpy(&bigrecordingbuffer[bigrecordingbufferpos], &textbuffer, len );
                      // increase position in big recording buffer for next frame
                      bigrecordingbufferpos = bigrecordingbufferpos + len; 
                      tcpclient.write("\r\nChunk added to recording buffer\r\n\r\n");
                    }   
               else                  
                   {   
                     tcpclient.write("\r\nBuffer is full. The frame does not fit.\r\n ");
                   };
        }  
        else { tcpclient.write("Wrong parameters.\r\n"); };
        // needed for ESP8266   
        yield();      


        
    // Handling REC command         
    } else if (strcmp_P(command, PSTR("rec")) == 0) {
        tcpclient.write("\r\nRecording mode set to ");
        if (recordingmode == 1) 
            { 
               tcpclient.write("Disabled"); 
               bigrecordingbufferpos = 0; 
               recordingmode = 0;
            }
        else if (recordingmode == 0)
            {  ELECHOUSE_cc1101.SetRx(); 
               tcpclient.write("Enabled");
               bigrecordingbufferpos = 0;
               // flush buffer for recording 
               for (int i = 0; i < RECORDINGBUFFERSIZE; i++)
                    { bigrecordingbuffer[RECORDINGBUFFERSIZE] = 0; };
               recordingmode = 1;
               jammingmode = 0;
               receivingmode = 0;
               // start counting frames in the buffer
               framesinbigrecordingbuffer = 0;
               };
        tcpclient.write("\r\n"); 
        // needed for ESP8266   
        yield();      
 

    // Handling PLAY command         
       } else if (strcmp_P(command, PSTR("play")) == 0) {
        setting = atoi(strsep(&cmdline, " ")); 
        // if number of played frames is 0 it means play all frames
        if (setting <= framesinbigrecordingbuffer)
        {
          tcpclient.write("\r\nReplaying recorded frames.\r\n ");
          
          // rewind recording buffer position to the beginning
          bigrecordingbufferpos = 0;
          if (framesinbigrecordingbuffer >0)
          {
            // start reading and sending frames from the buffer : FIFO
            for (int i=1; i<=framesinbigrecordingbuffer ; i++)  
               { 
                 // blink ESP8266 led - turn it on
                 digitalWrite(LED_BUILTIN, HIGH);                
                 // read length of the recorded frame first from the buffer
                 len = bigrecordingbuffer[bigrecordingbufferpos];
                 if ( ((len<=60) and (len>0)) and ((i == setting) or (setting == 0))  )
                 { 
                    // take next frame from the buffer  for replay
                    memcpy(ccsendingbuffer, &bigrecordingbuffer[bigrecordingbufferpos + 1], len );      
                    // send these data to radio over CC1101
                    ELECHOUSE_cc1101.SendData(ccsendingbuffer, (byte)len);
                 };
                  // increase position to the buffer and check exception
                  bigrecordingbufferpos = bigrecordingbufferpos + 1 + len;
                  if ( bigrecordingbufferpos > RECORDINGBUFFERSIZE) break;
                 // 
                 // blink ESP8266 led - turn it off
                 digitalWrite(LED_BUILTIN, LOW);
                 // feed the watchdog
                 ESP.wdtFeed();
                 // needed for ESP8266   
                 yield();                  
               };
                   
             }; // end of IF framesinrecordingbuffer  
        
          // rewind buffer position
          bigrecordingbufferpos = 0;
          tcpclient.write("Done.\r\n");       
        }
         else { tcpclient.write("Wrong parameters.\r\n"); };


    // Handling ADD command         
       } else if (strcmp_P(command, PSTR("add")) == 0) {
        // getting hex numbers - the content of the  frame 
        len = strlen(cmdline);
        // convert hex array to set of bytes
        if ((len<=120) && (len>0) )
        { 
                // convert the hex content to array of bytes
                hextoascii(textbuffer,(byte *)cmdline, len);        
                len = len /2;
                // check if the frame fits into the buffer and store it
                if (( bigrecordingbufferpos + len + 1) < RECORDINGBUFFERSIZE) 
                     { // put info about number of bytes
                      bigrecordingbuffer[bigrecordingbufferpos] = len; 
                      bigrecordingbufferpos++;
                      // next - copy current frame and increase 
                      memcpy(&bigrecordingbuffer[bigrecordingbufferpos], &textbuffer, len );
                      // increase position in big recording buffer for next frame
                      bigrecordingbufferpos = bigrecordingbufferpos + len; 
                      // increase counter of frames stored
                      framesinbigrecordingbuffer++;
                      tcpclient.write("\r\nAdded frame number ");
                      itoa(framesinbigrecordingbuffer,destination,10);
                      tcpclient.write(destination);                       
                      tcpclient.write("\r\n");                  
                    }   
               else                  
                   {   
                     tcpclient.write("\r\nBuffer is full. The frame does not fit.\r\n ");
                   };
        }  
        else { tcpclient.write("Wrong parameters.\r\n"); };
        // needed for ESP8266   
        yield();      
       

    // Handling SHOW command         
       } else if (strcmp_P(command, PSTR("show")) == 0) {
         if (framesinbigrecordingbuffer>0)
        {
          tcpclient.write("\r\nFrames stored in the recording buffer:\r\n ");
          // rewind recording buffer position to the beginning
          bigrecordingbufferpos = 0;
          // start reading and sending frames from the buffer : FIFO
          for (setting=1; setting<=framesinbigrecordingbuffer; setting++)  
               { 
                 // read length of the recorded frame first from the buffer
                 len = bigrecordingbuffer[bigrecordingbufferpos];
                 if ((len<=60) and (len>0))
                 { 
                    // take next frame from the buffer  for replay
                    // flush textbuffer
                    for (setting2 = 0; setting2 < BUF_LENGTH; setting2++)
                        { textbuffer[setting2] = 0; };           
                    asciitohex(&bigrecordingbuffer[bigrecordingbufferpos + 1], textbuffer,  len);
                    tcpclient.write("\r\nFrame ");
                    itoa(setting,destination,10);
                    tcpclient.write(destination);                    
                    tcpclient.write(" : ");                                       
                    tcpclient.write((char *)textbuffer);
                    tcpclient.write("\r\n");
                 };
                    // increase position to the buffer and check exception
                    bigrecordingbufferpos = bigrecordingbufferpos + 1 + len;
                    if ( bigrecordingbufferpos > RECORDINGBUFFERSIZE) break;
                    // feed the watchdog
                    ESP.wdtFeed();
                 // 
               };
          // rewind buffer position
          // bigrecordingbufferpos = 0;
          tcpclient.write("\r\n"); 
        }
         else { tcpclient.write("Wrong parameters.\r\n"); };
        // needed for ESP8266   
        yield();      


    // Handling FLUSH command         
    } else if (strcmp_P(command, PSTR("flush")) == 0) {
        // flushing bigrecordingbuffer with zeros and rewinding all the pointers 
        for (setting = 0; setting<RECORDINGBUFFERSIZE; setting++)  bigrecordingbuffer[setting] = 0;  
        // and rewinding all the pointers to the recording buffer
        bigrecordingbufferpos = 0;
        framesinbigrecordingbuffer = 0;
        tcpclient.write("\r\nRecording buffer cleared.\r\n");
        // needed for ESP8266   
        yield();      
          
       
    // Handling ECHO command         
    } else if (strcmp_P(command, PSTR("echo")) == 0) {
        do_echo = atoi(cmdline);

    // Handling X command         
    // command 'x' stops jamming, receiveing, recording...
    } else if (strcmp_P(command, PSTR("x")) == 0) {
        receivingmode = 0;
        jammingmode = 0;
        recordingmode = 0;
        tcpclient.write("\r\n");
        // needed for ESP8266   
        yield();      

    // Handling INIT command         
    // command 'init' initializes board with default settings
    } else if (strcmp_P(command, PSTR("init")) == 0) {
         // blink ESP8266 led - turn it on
         digitalWrite(LED_BUILTIN, HIGH);
        // init cc1101
        cc1101initialize();
        // give feedback
        tcpclient.write("CC1101 initialized\r\n");
        // blink ESP8266 led - turn it off
         digitalWrite(LED_BUILTIN, LOW);          
    } else {
        tcpclient.write("Error: Unknown command: ");
        tcpclient.write(command);
        tcpclient.write("\r\n");
        // needed for ESP8266   
        yield();      
    }
}


void setup() {

    // initializing WIFI
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED on
   
    // connect ESP8266 to an access point
    /*
    WiFi.setAutoReconnect(true);
    WiFi.mode(WIFI_STA);    
    WiFi.config(ip, gateway, subnet);
    delay(100);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
         digitalWrite(LED_BUILTIN, HIGH);
         delay(200);
         digitalWrite(LED_BUILTIN, LOW);
         delay(200);
         ESP.wdtFeed();
         yield();          
      };
    */
    // Setup an eccess point in ESP8266 with preconfigured values
    digitalWrite(LED_BUILTIN, HIGH);
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig (ip, gateway, subnet);
    WiFi.softAP(ssid);
    delay(10000);
    digitalWrite(LED_BUILTIN, LOW);
      
     // Start TCP listener on port TCP_PORT
     tcpserver.begin();
     tcpserver.setNoDelay(true);

     // bind to wifi client
     // tcpclient = tcpserver.available();

     /*
     tcpclient.write("CC1101 terminal tool connected, use 'help' for list of commands...\n\r");
     tcpclient.write("(C) Adam Loboda 2023\n\r  ");
     tcpclient.write("\n\r");  // print CRLF
     */
     
     //Init EEPROM - for ESP32 based boards only
     EEPROM.begin(EPROMSIZE);
  
     // initialize CC1101 module with preffered parameters
     cc1101initialize();

     /*
     if (ELECHOUSE_cc1101.getCC1101()) {  // Check the CC1101 Spi connection.
     tcpclient.write("cc1101 initialized. Connection OK\n\r");
     } else {
     tcpclient.write("cc1101 connection error! check the wiring.\n\r");
     };
     */
    
      // setup variables
     bigrecordingbufferpos = 0;

     // Enable software watchdog in ESP8266 chip with 3 second fuse in case something goes wrong...
     ESP.wdtEnable(5000);
  
     // disable temporarly software watchdog in ESP8266 chip
     // ESP.wdtDisable();   

      
}


void loop() {

  // index, buffers for port characters and conversions to variables
   int i = 0;
   static char buffer[BUF_LENGTH];
   static int length = 0;
   char destination[8];

   // feed the watchdog in ESP8266
   ESP.wdtFeed(); 

   // bind to wifi client
   tcpclient = tcpserver.available();
   
    // check if WIFI initialized and there is tcpclient connected over WIFI
    if (tcpclient)
    { 
     // process incoming characters from WIFI
    while(tcpclient.connected()){      
        // read data from the connected tcpclient
      {
        // read if there is any character received from telnet
        int data = tcpclient.read();

    // handling CHAT MODE     
    if ( (chatmode == 1) and (data>0) ) 
       { 
            // clear serial port buffer index
            i = 0;

            // something was received over serial port put it into radio sending buffer
            while (data>0 and (i<(CCBUFFERSIZE-1)) ) 
             {
               // feed the watchdog
               ESP.wdtFeed();
               // needed for ESP8266   
               yield();    
                          
              // read single character from TCP stream         
              ccsendingbuffer[i] = data;

              // also put it as ECHO back 
              tcpclient.write(ccsendingbuffer[i]);

              // if CR was received add also LF character and display it on Serial port
              if (ccsendingbuffer[i] == 0x0d )
                  {  
                    tcpclient.write( 0x0a );
                    i++;
                    ccsendingbuffer[i] = 0x0a;
                  }
              //
              
              // increase CC1101 TX buffer position
              i++;   
              // read next character from TCP stream
              data = tcpclient.read();
             };

            // put NULL at the end of CC transmission buffer
            ccsendingbuffer[i] = '\0';

            // blink ESP8266 led - turn it on
            digitalWrite(LED_BUILTIN, HIGH);         
            // send these data to radio over CC1101
            ELECHOUSE_cc1101.SendData((char *)ccsendingbuffer);
           // blink ESP8266 led - turn it off
            digitalWrite(LED_BUILTIN, LOW);
            
            
            // feed the watchdog
            ESP.wdtFeed();
            // needed for ESP8266   
            yield();      
                           
       } // end of chatmode
       
    // handling CLI commands processing
    else
      {   
        // int data = tcpclient.read();
        if (data == '\b' || data == '\177') {  // BS and DEL
            if (length) {
                length--;
                if (do_echo) tcpclient.write("\b \b");
                // feed the watchdog
                ESP.wdtFeed();
                // needed for ESP8266   
                yield();              
            }
        }
        else if (data == '\r' || data == '\n' ) {
            if (do_echo) tcpclient.write("\r\n");    // output CRLF
            buffer[length] = '\0';
            if (length)  {
                         // tcpclient.write(buffer);
                         exec(buffer); };            
            length = 0;
            // feed the watchdog
            ESP.wdtFeed();
            // needed for ESP8266   
            yield();            
        }

        else if ( (data > 0) && (length < BUF_LENGTH - 1) ) {
            buffer[length++] = data;
            if (do_echo) tcpclient.write(data);
            // feed the watchdog
            ESP.wdtFeed();
            // needed for ESP8266   
            yield();            
        }
        
       };  
      // end of handling CLI processing
        
    };

  /* Process RF received packets */
   
   //Checks whether something has been received.
  if (ELECHOUSE_cc1101.CheckReceiveFlag() && (receivingmode == 1 || recordingmode == 1 || chatmode == 1) )
      {

       //CRC Check. If "setCrc(false)" crc returns always OK!
       if (ELECHOUSE_cc1101.CheckCRC())
          { 
            // feed the watchdog
            ESP.wdtFeed();

            //Get received Data and calculate length
            int len = ELECHOUSE_cc1101.ReceiveData(ccreceivingbuffer);

            // Actions for CHAT MODE
            if ( ( chatmode == 1) && (len < CCBUFFERSIZE ) )
               {
                // put NULL at the end of char buffer
                ccreceivingbuffer[len] = '\0';
                //Print received in char format.
                tcpclient.write((char *) ccreceivingbuffer);
                // feed the watchdog
                ESP.wdtFeed();
                // needed for ESP8266   
                yield();      

               };  // end of handling Chat mode

            // Actions for RECEIVNG MODE
            if ( ((receivingmode == 1) && (recordingmode == 0))&& (len < CCBUFFERSIZE ) )
               {
                   // put NULL at the end of char buffer
                   ccreceivingbuffer[len] = '\0';
                   // flush textbuffer
                   for (int i = 0; i < BUF_LENGTH; i++)
                    { textbuffer[i] = 0; };
                   
                   //Print received packet as set of hex values directly 
                   // not to loose any data in buffer
                   // asciitohex((byte *)ccreceivingbuffer, (byte *)textbuffer,  len);
                   asciitohex(ccreceivingbuffer, textbuffer,  len);
                   tcpclient.write((char *)textbuffer);
                   // set RX  mode again
                   ELECHOUSE_cc1101.SetRx();
                   // feed the watchdog
                   ESP.wdtFeed();
                   // needed for ESP8266   
                   yield();                        
                };   // end of handling receiving mode 

            // Actions for RECORDING MODE               
            if ( ((recordingmode == 1) && (receivingmode == 0) )&& (len < CCBUFFERSIZE ) )
               { 
                // copy the frame from receiving buffer for replay - only if it fits
                if (( bigrecordingbufferpos + len + 1) < RECORDINGBUFFERSIZE) 
                     {
                      // put info about number of bytes
                      bigrecordingbuffer[bigrecordingbufferpos] = len; 
                      bigrecordingbufferpos++;
                      // next - copy current frame and increase 
                      memcpy(&bigrecordingbuffer[bigrecordingbufferpos], ccreceivingbuffer, len );
                      // increase position in big recording buffer for next frame
                      bigrecordingbufferpos = bigrecordingbufferpos + len; 
                      // increase counter of frames stored
                      framesinbigrecordingbuffer++;
                      // set RX  mode again
                      ELECHOUSE_cc1101.SetRx();
                      // feed the watchdog
                      ESP.wdtFeed();
                      // needed for ESP8266   
                      yield();                            
                     }
                     
                else {
                    tcpclient.write("Recording buffer full! Stopping..\r\nFrames stored: ");
                    itoa(framesinbigrecordingbuffer,destination,10);
                    tcpclient.write(destination);                     
                    tcpclient.write("\r\n");
                    bigrecordingbufferpos = 0;
                    recordingmode = 0;
                    // feed the watchdog
                    ESP.wdtFeed();
                    // needed for ESP8266   
                    yield();                          
                     };
                
               };   // end of handling frame recording mode 
 
          };   // end of CRC check IF


      };   // end of Check receive flag if

      // if jamming mode activate continously send something over RF...
      if ( jammingmode == 1)
      { 
        // populate cc1101 sending buffer with random values
        randomSeed(analogRead(0));
        
        for (i = 0; i<60; i++)
           { ccsendingbuffer[i] = (byte)random(255); 
             // feed the watchdog
             ESP.wdtFeed();
             // needed for ESP8266   
             yield();                   
           };        
        // blink ESP8266 led - turn it on
         digitalWrite(LED_BUILTIN, HIGH);
        // send these data to radio over CC1101
        ELECHOUSE_cc1101.SendData(ccsendingbuffer,60);
        // blink ESP8266 led - turn it off
         digitalWrite(LED_BUILTIN, LOW);
        // feed the watchdog
        ESP.wdtFeed();
        // needed for ESP8266   
        yield();              
      };

   // give control for other procedures in ESP8266
   yield(); 

    }; // end of while

   }; // end of "tcpclient" if
 
}  // end of main LOOP
