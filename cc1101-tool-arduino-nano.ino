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
// This code will ONLY work with Arduino Nano board
//

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>

#define CCBUFFERSIZE 64
#define RECORDINGBUFFERSIZE 1024   // Buffer for recording the frames. ATMEGA328 has only 2048 SRAM storage
#define EPROMSIZE 1024             // Size of EEPROM in your Arduino chip. ATMEGA328 has 1024
#define BUF_LENGTH 128             // Buffer for the incoming command.

// defining PINs for Arduino NANO
byte sck = 16; // D13
byte miso = 15; // D12
byte mosi = 14; // D11
byte ss = 13; // D10
byte gdo0 = 9; // D6
byte gdo2 = 5; // D2

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

// buffer for recording and replaying of many frames
byte bigrecordingbuffer[RECORDINGBUFFERSIZE] = {0};

// buffer for hex to ascii conversions 
byte textbuffer[BUF_LENGTH];



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
    int setting, setting2, len;
    uint16_t brute, poweroftwo;    
    byte j, k;
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
        Serial.println(F(
          "setmodulation <mode> : Set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.\r\n\r\n"
          "setmhz <frequency>   : Here you can set your basic frequency. default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.\r\n\r\n"
          "setdeviation <deviation> : Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.\r\n\r\n"
          "setchannel <channel> : Set the Channelnumber from 0 to 255. Default is cahnnel 0.\r\n\r\n"
          "setchsp <spacing>  :  The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz. \r\n\r\n"
          "setrxbw <Receive bndwth> : Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.\r\n"
          "setdrate <datarate> : Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!\r\n\r\n"
          "setpa <power value> : Set RF transmission power. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!\r\n\r\n"
          "setsyncmode  <sync mode> : Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.\r\n"
         ));
        Serial.println(F(
          "setsyncword <decimal LOW, decimal HIGH> : Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low) Default is 211,145\r\n\r\n"
          "setadrchk <address chk> : Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.\r\n\r\n"
          "setaddr <address> : Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).\r\n\r\n"
          "setwhitedata <whitening> : Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.\r\n\r\n"
          "setpktformat <pktformat> : Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator.  3 = Asynchronous serial mode\r\n\r\n"
          "setlengthconfig <mode> : Set packet Length mode : 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved \r\n\r\n"
          "setpacketlength <mode> : Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.\r\n\r\n"
          "setcrc <mode> : Switches on/of CRC calculation and check. 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.\r\n\r\n"
          "setcrcaf <mode> : Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.\r\n"
         ));
        Serial.println(F(
          "setdcfilteroff <mode> : Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).\r\n\r\n"
          "setmanchester <mode> : Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.\r\n\r\n"
          "setfec <mode> : Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.\r\n\r\n"
          "setpre <mode> : Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24\r\n\r\n"
          "setpqt <mode> : Preamble quality estimator threshold. \r\n\r\n"
          "setappendstatus <mode> : When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.\r\n\r\n"
          "getrssi : Display quality information about last received frames over RF\r\n\r\n"
          "scan <start> <stop> : Scan frequency range for the highest signal.\r\n\r\n"         
          "chat :  Enable chat mode between many devices. No exit available, disconnect device to quit\r\n"
         ));
        Serial.println(F(
          "rx : Sniffer. Enable or disable printing of received RF packets on serial terminal.\r\n\r\n"
          "tx <hex-vals> : Send packet of max 60 bytes <hex values> over RF\r\n"
          "jam : Enable or disable continous jamming on selected band.\r\n\r\n"
          "brute <microseconds> <number-of-bits> : Brute force garage gate with <nb-of-bits> keyword where symbol time is <usec>.\r\n\r\n"            
          "rec : Enable or disable recording frames in the buffer.\r\n\r\n"
          "add <hex-vals> : Manually add single frame payload (max 64 hex values) to the buffer so it can be replayed\r\n\r\n"
          "show : Show content of recording buffer\r\n\r\n"
          "flush : Clear the recording buffer\r\n\r\n"
          "play <N> : Replay 0 = all frames or N-th recorded frame previously stored in the buffer.\r\n\r\n"
          "rxraw <microseconds> : Sniffs radio by sampling with <microsecond> interval and prints received bytes in hex.\r\n\r\n"
          "recraw <microseconds> : Recording RAW RF data with <microsecond> sampling interval.\r\n"
            ));
        Serial.println(F(
          "addraw <hex-vals> : Manually add chunks (max 60 hex values) to the buffer so they can be further replayed.\r\n\r\n"        
          "showraw : Showing content of recording buffer in RAW format.\r\n\r\n"
          "playraw <microseconds> : Replaying previously recorded RAW RF data with <microsecond> sampling interval.\r\n\r\n"
          "showbit : Showing content of recording buffer in RAW format as a stream of bits.\r\n\r\n"
          "save : Store recording buffer content in non-volatile memory\r\n\r\n"
          "load : Load the content from non-volatile memory to the recording buffer\r\n\r\n"
          "echo <mode> : Enable or disable Echo on serial terminal. 1 = enabled, 0 = disabled\r\n\r\n"
          "x : Stops jamming/receiving/recording packets.\r\n\r\n"
          "init : Restarts CC1101 board with default parameters\r\n\r\n"
        ));

    // Handling SETMODULATION command 
    } else if (strcmp_P(command, PSTR("setmodulation")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setModulation(setting);
        Serial.print(F("\r\nModulation: "));
        if (setting == 0) { Serial.print(F("2-FSK")); }
        else if (setting == 1) { Serial.print(F("GFSK")); }
        else if (setting == 2) { Serial.print(F("ASK/OOK")); }
        else if (setting == 3) { Serial.print(F("4-FSK")); }
        else if (setting == 4) { Serial.print(F("MSK")); };  
        Serial.print(F(" \r\n"));

    // Handling SETMHZ command 
    } else if (strcmp_P(command, PSTR("setmhz")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setMHZ(settingf1);
        Serial.print(F("\r\nFrequency: "));
        Serial.print(settingf1);
        Serial.print(F(" MHz\r\n"));
        
    // Handling SETDEVIATION command 
    } else if (strcmp_P(command, PSTR("setdeviation")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setDeviation(settingf1);
        Serial.print(F("\r\nDeviation: "));
        Serial.print(settingf1);
        Serial.print(F(" KHz\r\n"));        

    // Handling SETCHANNEL command       
    } else if (strcmp_P(command, PSTR("setchannel")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setChannel(setting);
        Serial.print(F("\r\nChannel:"));
        Serial.print(setting);
        Serial.print(F("\r\n"));        

    // Handling SETCHSP command 
    } else if (strcmp_P(command, PSTR("setchsp")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setChsp(settingf1);
        Serial.print(F("\r\nChann spacing: "));
        Serial.print(settingf1);
        Serial.print(F(" kHz\r\n"));  

    // Handling SETRXBW command         
    } else if (strcmp_P(command, PSTR("setrxbw")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setRxBW(settingf1);
        Serial.print(F("\r\nRX bandwidth: "));
        Serial.print(settingf1);
        Serial.print(F(" kHz \r\n"));  

    // Handling SETDRATE command         
    } else if (strcmp_P(command, PSTR("setdrate")) == 0) {
        settingf1 = atof(cmdline);
        ELECHOUSE_cc1101.setDRate(settingf1);
        Serial.print(F("\r\nDatarate: "));
        Serial.print(settingf1);
        Serial.print(F(" kbaud\r\n"));  

    // Handling SETPA command         
    } else if (strcmp_P(command, PSTR("setpa")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPA(setting);
        Serial.print(F("\r\nTX PWR: "));
        Serial.print(setting);
        Serial.print(F(" dBm\r\n"));  
        
    // Handling SETSYNCMODE command         
    } else if (strcmp_P(command, PSTR("setsyncmode")) == 0) {
        int setting = atoi(cmdline);
        ELECHOUSE_cc1101.setSyncMode(setting);
        Serial.print(F("\r\nSynchronization: "));
        if (setting == 0) { Serial.print(F("No preamble")); }
        else if (setting == 1) { Serial.print(F("16 sync bits")); }
        else if (setting == 2) { Serial.print(F("16/16 sync bits")); }
        else if (setting == 3) { Serial.print(F("30/32 sync bits")); }
        else if (setting == 4) { Serial.print(F("No preamble/sync, carrier-sense")); }
        else if (setting == 5) { Serial.print(F("15/16 + carrier-sense")); }
        else if (setting == 6) { Serial.print(F("16/16 + carrier-sense")); }
        else if (setting == 7) { Serial.print(F("30/32 + carrier-sense")); };
        Serial.print(F("\r\n"));  
        
    // Handling SETSYNCWORD command         
    } else if (strcmp_P(command, PSTR("setsyncword")) == 0) {
        setting = atoi(strsep(&cmdline, " "));
        setting2 = atoi(cmdline);
        ELECHOUSE_cc1101.setSyncWord(setting2, setting);
        Serial.print(F("\r\nSynchronization:\r\n"));
        Serial.print(F("high = "));
        Serial.print(setting);
        Serial.print(F("\r\nlow = "));
        Serial.print(setting2);
        Serial.print(F("\r\n"));  

    
    // Handling SETADRCHK command         
    } else if (strcmp_P(command, PSTR("setadrchk")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAdrChk(setting);
        Serial.print(F("\r\nAddress checking:"));
        if (setting == 0) { Serial.print(F("No adr chk")); }
        else if (setting == 1) { Serial.print(F("Adr chk, no bcast")); }
        else if (setting == 2) { Serial.print(F("Adr chk and 0 bcast")); }
        else if (setting == 3) { Serial.print(F("Adr chk and 0 and FF bcast")); };
        Serial.print(F("\r\n"));  
        
    // Handling SETADDR command         
    } else if (strcmp_P(command, PSTR("setaddr")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAddr(setting);
        Serial.print(F("\r\nAddress: "));
        Serial.print(setting);
        Serial.print(F("\r\n"));  

    // Handling SETWHITEDATA command         
    } else if (strcmp_P(command, PSTR("setwhitedata")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setWhiteData(setting);
        Serial.print(F("\r\nWhitening "));
        if (setting == 0) { Serial.print(F("OFF")); }
        else if (setting == 1) { Serial.print(F("ON")); }
        Serial.print(F("\r\n"));  
        
    // Handling SETPKTFORMAT command         
    } else if (strcmp_P(command, PSTR("setpktformat")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPktFormat(setting);
        Serial.print(F("\r\nPacket format: "));
        if (setting == 0) { Serial.print(F("Normal mode")); }
        else if (setting == 1) { Serial.print(F("Synchronous serial mode")); }
        else if (setting == 2) { Serial.print(F("Random TX mode")); }
        else if (setting == 3) { Serial.print(F("Asynchronous serial mode")); };
        Serial.print(F("\r\n"));  
  
    // Handling SETLENGTHCONFIG command         
    } else if (strcmp_P(command, PSTR("setlengthconfig")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setLengthConfig(setting);
        Serial.print(F("\r\nPkt length mode: "));
        if (setting == 0) { Serial.print(F("Fixed")); }
        else if (setting == 1) { Serial.print(F("Variable")); }
        else if (setting == 2) { Serial.print(F("Infinite")); }
        else if (setting == 3) { Serial.print(F("Reserved")); };
        Serial.print(F("\r\n"));  
  
    // Handling SETPACKETLENGTH command         
    } else if (strcmp_P(command, PSTR("setpacketlength")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPacketLength(setting);
        Serial.print(F("\r\nPkt length: "));
        Serial.print(setting);
        Serial.print(F(" bytes\r\n"));  
        
    // Handling SETCRC command         
    } else if (strcmp_P(command, PSTR("setcrc")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setCrc(setting);
        Serial.print(F("\r\nCRC checking: "));
        if (setting == 0) { Serial.print(F("Disabled")); }
        else if (setting == 1) { Serial.print(F("Enabled")); };
        Serial.print(F("\r\n")); 
        
    // Handling SETCRCAF command         
    } else if (strcmp_P(command, PSTR("setcrcaf")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setCRC_AF(setting);
        Serial.print(F("\r\nCRC Autoflush: "));
        if (setting == 0) { Serial.print(F("Disabled")); }
        else if (setting == 1) { Serial.print(F("Enabled")); };
         Serial.print(F("\r\n")); 
        
    // Handling SETDCFILTEROFF command         
     } else if (strcmp_P(command, PSTR("setdcfilteroff")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setDcFilterOff(setting);
        Serial.print(F("\r\nDC filter: "));
        if (setting == 0) { Serial.print(F("Enabled")); }
        else if (setting == 1) { Serial.print(F("Disabled")); };
        Serial.print(F("\r\n")); 

    // Handling SETMANCHESTER command         
     } else if (strcmp_P(command, PSTR("setmanchester")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setManchester(setting);
        Serial.print(F("\r\nManchester coding: "));
        if (setting == 0) { Serial.print(F("Disabled")); }
        else if (setting == 1) { Serial.print(F("Enabled")); };
        Serial.print(F("\r\n")); 

    // Handling SETFEC command         
     } else if (strcmp_P(command, PSTR("setfec")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setFEC(setting);
        Serial.print(F("\r\nForward Error Correction: "));
        if (setting == 0) { Serial.print(F("Disabled")); }
        else if (setting == 1) { Serial.print(F("Enabled")); };
        Serial.print(F("\r\n")); 
        
    // Handling SETPRE command         
     } else if (strcmp_P(command, PSTR("setpre")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPRE(setting);
        Serial.print(F("\r\nMinimum preamble bytes:"));
        Serial.print(setting);
        Serial.print(F(" means 0 = 2 bytes, 1 = 3b, 2 = 4b, 3 = 6b, 4 = 8b, 5 = 12b, 6 = 16b, 7 = 24 bytes\r\n")); 
        Serial.print(F("\r\n")); 

  
    // Handling SETPQT command         
      } else if (strcmp_P(command, PSTR("setpqt")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setPQT(setting);
        Serial.print(F("\r\nPQT: "));
        Serial.print(setting);
        Serial.print(F("\r\n")); 

    // Handling SETAPPENDSTATUS command         
       } else if (strcmp_P(command, PSTR("setappendstatus")) == 0) {
        setting = atoi(cmdline);
        ELECHOUSE_cc1101.setAppendStatus(setting);
        Serial.print(F("\r\nStatus bytes appending: "));
        if (setting == 0) { Serial.print(F("Enabled")); }
        else if (setting == 1) { Serial.print(F("Disabled")); };
        Serial.print(F("\r\n")); 

    // Handling GETRSSI command         
      } else if (strcmp_P(command, PSTR("getrssi")) == 0) {
        //Rssi Level in dBm
        Serial.print(F("Rssi: "));
        Serial.println(ELECHOUSE_cc1101.getRssi());
        //Link Quality Indicator
        Serial.print(F(" LQI: "));
        Serial.println(ELECHOUSE_cc1101.getLqi());        
        Serial.print(F("\r\n")); 

    // Handling RX command         
       } else if (strcmp_P(command, PSTR("rx")) == 0) {
        Serial.print(F("\r\nReceiving and printing RF packet changed to "));
        if (receivingmode == 1) {
          receivingmode = 0;
          Serial.print(F("Disabled")); }
        else if (receivingmode == 0)
               { ELECHOUSE_cc1101.SetRx();
                 Serial.print(F("Enabled")); 
                 receivingmode = 1;
                 jammingmode = 0; 
                 recordingmode = 0;
               };
        Serial.print(F("\r\n")); 
 

    // Handling CHAT command         
       } else if (strcmp_P(command, PSTR("chat")) == 0) {
        Serial.print(F("\r\nEntering chat mode:\r\n\r\n"));
        if (chatmode == 0) 
           { 
             chatmode = 1;
             jammingmode = 0;
             receivingmode = 0;
             recordingmode = 0;
           };
 

    // Handling JAM command         
       } else if (strcmp_P(command, PSTR("jam")) == 0) {
        Serial.print(F("\r\nJamming changed to "));
        if (jammingmode == 1) 
           { Serial.print(F("Disabled")); 
             jammingmode = 0;
           }
        else if (jammingmode == 0) 
               { 
                 Serial.print(F("Enabled")); 
                 jammingmode = 1;
                 receivingmode = 0; };
        Serial.print(F("\r\n")); 

        
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
        Serial.print(F("\r\nStarting Brute Forcing press any key to stop...\r\n"));
        pinMode(gdo0, OUTPUT);
     
        for (brute = 0; brute < poweroftwo ; brute++)  
           { 
           for(int k = 0; k <  5; k++)  // sending 5 times each code
             {
             for(int j = (setting2 - 1); j > -1; j--)  // j bits in a value brute
               {
                 digitalWrite(gdo0, bitRead(brute, j)); // Set GDO0 according to actual brute force value
                 delayMicroseconds(setting);            // delay for selected sampling interval
               }; // end of J loop
             };  // end of K loop
             // checking if key pressed
             if (Serial.available()) break;
           };


        Serial.print(F("\r\nBrute forcing complete.\r\n\r\n"));
        
        // setting normal pkt format again
        ELECHOUSE_cc1101.setCCMode(1); 
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.SetTx();
        // pinMode(gdo0pin, INPUT);
        } // end of IF
        else { Serial.print(F("Wrong parameters.\r\n")); };


    // Handling TX command         
       } else if (strcmp_P(command, PSTR("tx")) == 0) {
        // convert hex array to set of bytes
        if ((strlen(cmdline)<=120) && (strlen(cmdline)>0) )
        { 
                hextoascii((byte *)textbuffer, cmdline, strlen(cmdline));        
                memcpy(ccsendingbuffer, textbuffer, strlen(cmdline)/2 );
                ccsendingbuffer[strlen(cmdline)/2] = 0x00;       
                Serial.print("\r\nTransmitting RF packets.\r\n");
                // send these data to radio over CC1101
                ELECHOUSE_cc1101.SendData(ccsendingbuffer, (byte)(strlen(cmdline)/2));
                // for DEBUG only
                asciitohex((byte *)ccsendingbuffer, (byte *)textbuffer,  strlen(cmdline)/2 );
                Serial.print(F("Sent frame: "));
                Serial.print((char *)textbuffer);
                Serial.print(F("\r\n")); }
         else { Serial.print(F("Wrong parameters.\r\n")); };



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
        Serial.print(F("\r\nWaiting for radio signal to start RAW recording...\r\n"));
        pinMode(gdo0, INPUT);

        // waiting for some data first or serial port signal
        // while (!Serial.available() ||  (digitalRead(gdo0) == LOW) ); 
        while ( digitalRead(gdo0) == LOW ); 

        // start capturing
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
           }
        Serial.print(F("\r\nRecording RAW data complete.\r\n\r\n"));
        // setting normal pkt format again
        ELECHOUSE_cc1101.setCCMode(1); 
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.SetRx();
        }
        else { Serial.print(F("Wrong parameters.\r\n")); };

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
        Serial.print(F("\r\nSniffer enabled...\r\n"));
        pinMode(gdo0, INPUT);

        
        
       // Any received char over Serial port stops printing  RF received bytes
        while (!Serial.available()) 
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
                }; 
             // when buffer full print the ouptput to serial port
             for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)  
                    { 
                       asciitohex((byte *)&bigrecordingbuffer[i], (byte *)textbuffer,  32);
                       Serial.print((char *)textbuffer);
                    };
                    
            
           }; // end of While loop
           
        Serial.print(F("\r\nStopping the sniffer.\n\r\n"));
        
        // setting normal pkt format again
        ELECHOUSE_cc1101.setCCMode(1); 
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.SetRx();
        }
        else { Serial.print(F("Wrong parameters.\r\n")); };


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
        //start replaying GDO0 bit state from data in the buffer with bitbanging 
        Serial.print(F("\r\nReplaying RAW data from the buffer...\r\n"));
        pinMode(gdo0, OUTPUT);

        
        for (int i=1; i<RECORDINGBUFFERSIZE ; i++)  
           { 
             byte receivedbyte = bigrecordingbuffer[i];
             for(int j=7; j > -1; j--)  // 8 bits in a byte
               {
                 digitalWrite(gdo0, bitRead(receivedbyte, j)); // Set GDO0 according to recorded byte
                 delayMicroseconds(setting);                      // delay for selected sampling interval
               }; 
           }

        
        Serial.print(F("\r\nReplaying RAW data complete.\r\n\r\n"));
        // setting normal pkt format again
        ELECHOUSE_cc1101.setCCMode(1); 
        ELECHOUSE_cc1101.setPktFormat(0);
        ELECHOUSE_cc1101.SetTx();
        // pinMode(gdo0pin, INPUT);
        }
        else { Serial.print(F("Wrong parameters.\r\n")); };

    // handling SHOWRAW command
    } else if (strcmp_P(command, PSTR("showraw")) == 0) {
    // show the content of recorded RAW signal as hex numbers
       Serial.print(F("\r\nRecorded RAW data:\r\n"));
       for (int i = 0; i < RECORDINGBUFFERSIZE ; i = i + 32)  
           { 
                    asciitohex((byte *)&bigrecordingbuffer[i], (byte *)textbuffer,  32);
                    Serial.print((char *)textbuffer);
           }
       Serial.print(F("\r\n\r\n"));


    // handling SHOWBIT command
    } else if (strcmp_P(command, PSTR("showbit")) == 0) {
    // show the content of recorded RAW signal as hex numbers
       Serial.print(F("\r\nRecorded RAW data as bit stream:\r\n"));
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
                              Serial.print(F("____"));
                              break;
   
                              case '1':
                              Serial.print(F("___-"));
                              break;
   
                              case '2':
                              Serial.print(F("__-_"));
                              break;

                              case '3':
                              Serial.print(F("__--"));
                              break;

                              case '4':
                              Serial.print(F("_-__"));
                              break;

                              case '5':
                              Serial.print(F("_-_-"));
                              break;

                              case '6':
                              Serial.print(F("_--_"));
                              break;

                              case '7':
                              Serial.print(F("_---"));
                              break;

                              case '8':
                              Serial.print(F("-___"));
                              break;

                              case '9':
                              Serial.print(F("-__-"));
                              break;

                              case 'A':
                              Serial.print(F("-_-_"));
                              break;

                              case 'B':
                              Serial.print(F("-_--"));
                              break;

                              case 'C':
                              Serial.print(F("--__"));
                              break;

                              case 'D':
                              Serial.print(F("--_-"));
                              break;

                              case 'E':
                              Serial.print(F("---_"));
                              break;

                              case 'F':
                              Serial.print(F("----"));
                              break;
                              
                              }; // end of switch
                              
                        }; // end of for
 
              } // end of for
              Serial.print(F("\r\n\r\n"));



    // Handling ADDRAW command         
       } else if (strcmp_P(command, PSTR("addraw")) == 0) {
        // getting hex numbers - the content of the  frame 
        len = strlen(cmdline);
        // convert hex array to set of bytes
        if ((len<=120) && (len>0) )
        { 
                // convert the hex content to array of bytes
                hextoascii((byte *)textbuffer, cmdline, len);        
                len = len /2;
                // check if the frame fits into the buffer and store it
                if (( bigrecordingbufferpos + len) < RECORDINGBUFFERSIZE) 
                     { // copy current frame and increase pointer for next frames
                      memcpy(&bigrecordingbuffer[bigrecordingbufferpos], &textbuffer, len );
                      // increase position in big recording buffer for next frame
                      bigrecordingbufferpos = bigrecordingbufferpos + len; 
                      Serial.print(F("\r\nChunk added to recording buffer\r\n\r\n"));
                    }   
               else                  
                   {   
                     Serial.print(F("\r\nBuffer is full. The frame does not fit.\r\n "));
                   };
        }  
        else { Serial.print(F("Wrong parameters.\r\n")); };



    // Handling REC command         
    } else if (strcmp_P(command, PSTR("rec")) == 0) {
        Serial.print(F("\r\nRecording mode set to "));
        if (recordingmode == 1) 
            { 
               Serial.print(F("Disabled")); 
               bigrecordingbufferpos = 0; 
               recordingmode = 0;
            }
        else if (recordingmode == 0)
            {  ELECHOUSE_cc1101.SetRx(); 
               Serial.print(F("Enabled"));
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
        Serial.print(F("\r\n")); 
 

    // Handling PLAY command         
       } else if (strcmp_P(command, PSTR("play")) == 0) {
        setting = atoi(strsep(&cmdline, " ")); 
        // if number of played frames is 0 it means play all frames
        if (setting <= framesinbigrecordingbuffer)
        {
          Serial.print(F("\r\nReplaying recorded frames.\r\n "));
          // rewind recording buffer position to the beginning
          bigrecordingbufferpos = 0;
          if (framesinbigrecordingbuffer >0)
          {

            // start reading and sending frames from the buffer : FIFO
            for (int i=1; i<=framesinbigrecordingbuffer ; i++)  
               { 
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
               };
             }; // end of IF framesinrecordingbuffer  
        
          // rewind buffer position
          bigrecordingbufferpos = 0;
          Serial.print(F("Done.\r\n"));       
        }
         else { Serial.print(F("Wrong parameters.\r\n")); };


    // Handling ADD command         
       } else if (strcmp_P(command, PSTR("add")) == 0) {
        // getting hex numbers - the content of the  frame 
        len = strlen(cmdline);
        // convert hex array to set of bytes
        if ((len<=120) && (len>0) )
        { 
                // convert the hex content to array of bytes
                hextoascii((byte *)textbuffer, cmdline, len);        
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
                      Serial.print(F("\r\nAdded frame number "));
                      Serial.print(framesinbigrecordingbuffer);
                      Serial.print(F("\r\n"));                  
                    }   
               else                  
                   {   
                     Serial.print(F("\r\nBuffer is full. The frame does not fit.\r\n "));
                   };
        }  
        else { Serial.print(F("Wrong parameters.\r\n")); };



    // Handling SHOW command         
       } else if (strcmp_P(command, PSTR("show")) == 0) {
         if (framesinbigrecordingbuffer>0)
        {
          Serial.print(F("\r\nFrames stored in the recording buffer:\r\n "));
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
                    asciitohex((byte *)&bigrecordingbuffer[bigrecordingbufferpos + 1], (byte *)textbuffer,  len);
                    Serial.print(F("\r\nFrame "));
                    Serial.print(setting);
                    Serial.print(F(" : "));                     
                    Serial.print((char *)textbuffer);
                    Serial.print(F("\r\n"));
                 };
                    // increase position to the buffer and check exception
                    bigrecordingbufferpos = bigrecordingbufferpos + 1 + len;
                    if ( bigrecordingbufferpos > RECORDINGBUFFERSIZE) break;
                 // 
               };
          // rewind buffer position
          // bigrecordingbufferpos = 0;
          Serial.print(F("\r\n")); 
        }
         else { Serial.print(F("Wrong parameters.\r\n")); };


    // Handling FLUSH command         
    } else if (strcmp_P(command, PSTR("flush")) == 0) {
        // flushing bigrecordingbuffer with zeros and rewinding all the pointers 
        for (setting = 0; setting<RECORDINGBUFFERSIZE; setting++)  bigrecordingbuffer[setting] = 0;  
        // and rewinding all the pointers to the recording buffer
        bigrecordingbufferpos = 0;
        framesinbigrecordingbuffer = 0;
        Serial.print(F("\r\nRecording buffer cleared.\r\n"));


    // Handling SCAN command - frequency scanner by Little S@tan !
    } else if (strcmp_P(command, PSTR("scan")) == 0) {
        settingf1 = atof(strsep(&cmdline, " "));
        settingf2 = atof(cmdline);
        Serial.print(F("\r\nScanning frequency range from : "));
        Serial.print(settingf1);
        Serial.print(F(" MHz to "));
        Serial.print(settingf2);
        Serial.print(F(" MHz, press any key for stop or wait...\r\n"));  
        // initialize parameters for scanning
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setRxBW(58);
        ELECHOUSE_cc1101.SetRx();
        // Do scanning until some key pressed
        freq = settingf1;  // start frequency for scanning
        while (!Serial.available())        
          {
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
                            Serial.print(F("Signal found at  "));
                            Serial.print(F("Freq: "));
                            Serial.print(mark_freq);
                            Serial.print(F(" Rssi: "));
                            Serial.println(mark_rssi);
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
        Serial.print(F("\r\nSaving recording buffer content into the non-volatile memory...\r\n"));
        
        for (setting=0; setting<EPROMSIZE ; setting++)  
           {  // copying byte after byte from SRAM to EEPROM
            EEPROM.write(setting, bigrecordingbuffer[setting] );
           }
        Serial.print(F("\r\nSaving complete.\r\n\r\n"));

                 
    // handling LOAD command
    } else if (strcmp_P(command, PSTR("load")) == 0) {     
         // first flushing bigrecordingbuffer with zeros and rewinding all the pointers 
        for (setting = 0; setting<RECORDINGBUFFERSIZE; setting++)  bigrecordingbuffer[setting] = 0;  
        // and rewinding all the pointers to the recording buffer
        bigrecordingbufferpos = 0;
        framesinbigrecordingbuffer = 0;     
        //start loading EEPROM non-volatile memory content into recording buffer
        Serial.print(F("\r\nLoading content from the non-volatile memory into the recording buffer...\r\n"));
        for (setting=0; setting<EPROMSIZE ; setting++)  
           { // copying byte after byte from EEPROM to SRAM 
            bigrecordingbuffer[setting] = EEPROM.read(setting);
           }
        Serial.print(F("\r\nLoading complete. Enter 'show' or 'showraw' to see the buffer content.\r\n\r\n"));
                  
       
    // Handling ECHO command         
    } else if (strcmp_P(command, PSTR("echo")) == 0) {
        do_echo = atoi(cmdline);

    // Handling X command         
    // command 'x' stops jamming, receiveing, recording...
    } else if (strcmp_P(command, PSTR("x")) == 0) {
        receivingmode = 0;
        jammingmode = 0;
        recordingmode = 0;
        Serial.print(F("\r\n"));

    // Handling INIT command         
    // command 'init' initializes board with default settings
    } else if (strcmp_P(command, PSTR("init")) == 0) {
        // init cc1101
        cc1101initialize();
        // give feedback
        Serial.print(F("CC1101 initialized\r\n"));
          
    } else {
        Serial.print(F("Error: Unknown command: "));
        Serial.println(command);
        //  debug only
        // asciitohex(command, (byte *)textbuffer,  strlen(command));
        // Serial.print(F("\r\n"));
        // Serial.print((char *)textbuffer);
        // Serial.print(F("\r\n"));
    }
}


void setup() {

     // initialize USB Serial Port CDC
     Serial.begin(115200);

     Serial.println(F("CC1101 terminal tool connected, use 'help' for list of commands...\n\r"));
     Serial.println(F("(C) Adam Loboda 2023\n\r  "));

     Serial.println();  // print CRLF


     // initialize CC1101 module with preffered parameters
     cc1101initialize();

     if (ELECHOUSE_cc1101.getCC1101()) {  // Check the CC1101 Spi connection.
     Serial.println(F("cc1101 initialized. Connection OK\n\r"));
       } 
     else {
     Serial.println(F("cc1101 connection error! check the wiring.\n\r"));
         };
    
     // setup variables
     bigrecordingbufferpos = 0;
}


void loop() {

  // index for serial port characters
  int i = 0;

    /* Process incoming commands. */
    while (Serial.available()) {
        static char buffer[BUF_LENGTH];
        static int length = 0;

    // handling CHAT MODE     
    if (chatmode == 1) 
       { 
            
            // clear serial port buffer index
            i = 0;

            // something was received over serial port put it into radio sending buffer
            while (Serial.available() and (i<(CCBUFFERSIZE-1)) ) 
             {
              // read single character from Serial port         
              ccsendingbuffer[i] = Serial.read();

              // also put it as ECHO back to serial port
              Serial.write(ccsendingbuffer[i]);
               
              // if CR was received add also LF character and display it on Serial port
              if (ccsendingbuffer[i] == 0x0d )
                  {  
                    Serial.write( 0x0a );
                    i++;
                    ccsendingbuffer[i] = 0x0a;
                  }
              //
              
              // increase CC1101 TX buffer position
              i++;   
             };

            // put NULL at the end of CC transmission buffer
            ccsendingbuffer[i] = '\0';

            // send these data to radio over CC1101
            ELECHOUSE_cc1101.SendData(ccsendingbuffer);

                
       }
    // handling CLI commands processing
    else
      {   
        int data = Serial.read();
        if (data == '\b' || data == '\177') {  // BS and DEL
            if (length) {
                length--;
                if (do_echo) Serial.write("\b \b");
            }
        }
        else if (data == '\r' || data == '\n' ) {
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
      // end of handling CLI processing
        
    };

  /* Process RF received packets */
   
   //Checks whether something has been received.
  if (ELECHOUSE_cc1101.CheckReceiveFlag() && (receivingmode == 1 || recordingmode == 1 || chatmode == 1) )
      {

       //CRC Check. If "setCrc(false)" crc returns always OK!
       if (ELECHOUSE_cc1101.CheckCRC())
          { 
            //Get received Data and calculate length
            int len = ELECHOUSE_cc1101.ReceiveData(ccreceivingbuffer);

            // Actions for CHAT MODE
            if ( ( chatmode == 1) && (len < CCBUFFERSIZE ) )
               {
                // put NULL at the end of char buffer
                ccreceivingbuffer[len] = '\0';
                //Print received in char format.
                Serial.print((char *) ccreceivingbuffer);
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
                   asciitohex((byte *)ccreceivingbuffer, (byte *)textbuffer,  len);
                   Serial.print((char *)textbuffer);
                   // set RX  mode again
                   ELECHOUSE_cc1101.SetRx();
                };   // end of handling receiving mode 

            // Actions for RECORDING MODE               
            if ( ((recordingmode == 1) && (receivingmode == 0) )&& (len < CCBUFFERSIZE ) )
               { 
                // copy the frame from receiving buffer for replay - only if it fits
                if (( bigrecordingbufferpos + len + 1) < RECORDINGBUFFERSIZE) 
                     { // put info about number of bytes
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
                     }
                     
                else {
                    Serial.print(F("Recording buffer full! Stopping..\r\nFrames stored: "));
                    Serial.print(framesinbigrecordingbuffer); 
                    Serial.print(F("\r\n"));
                    bigrecordingbufferpos = 0;
                    recordingmode = 0;
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
           { ccsendingbuffer[i] = (byte)random(255);  };        
        // send these data to radio over CC1101
        ELECHOUSE_cc1101.SendData(ccsendingbuffer,60);
      };
 
}  // end of main LOOP
