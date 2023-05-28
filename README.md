# cc1101-tool
RF tool based on CC1101 module and Arduino Pro Micro 8VMHz/3.3V. Allows using CLI to control CC1101 board over USB interface. Putty or any other serial terminal can be used.

You simply connect your Arduino Pro Micro (Arduino Leonardo clone from Sparkfun) to USB port of your PC and launch Putty terminal to communicate with CC1101 module over USB Serial port ( /dev/ttyACM0 port in Linux, COMxx in Windows). Also you may connect this device to Android OTG USB port in your smartphone and use USB Serial Terminal application ( set option CDC driver).

Following commands are available :

    setModulation <mode>         // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK. 
    
    setMHZ <frequency>           // Here you can set your basic frequency. default = 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ.
    
    setDeviation <deviation>     // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
    
    setChannel <channel>         // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
    
    setChsp <spacing>            // The channel spacing is multiplied by the channel number CHAN and added to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz. 
    
    setRxBW <Receive bandwidh>   // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
    
    setDRate <datarate>          // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
    
    setPA <power value>          // Set TxPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
    
    setSyncMode  <sync mode>     // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
    
    setSyncWord <LOW, HIGH>      // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
    
    setAdrChk <address check>    // Controls address check configuration of received packages. 0 = No address check. 1 = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 = Address check and 0 (0x00) and 255 (0xFF) broadcast.
    
    setAddr <address>            // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).

    setWhiteData <whitening>     // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
    
    setPktFormat <pkt format>    // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.
    
    setLengthConfig <mode>       // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite packet length mode. 3 = Reserved 
    
    setPacketLength <mode>       // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed.
    
    setCrc <mode>                // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.

    setCRC_AF <mode>             // Enable automatic flush of RX FIFO when CRC is not OK. This requires that only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.

    setDcFilterOff <mode>        // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud The recommended IF frequency changes when the DC blocking is disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).

    setManchester <mode>         // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.

    setFEC <mode>                // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. 0 = Disable. 1 = Enable.

    setPRE <mode>                // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 : 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24

    setPQT <mode>                // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.

    setAppendStatus <mode>       // When enabled, two status bytes will be appended to the payload of the packet. The status bytes contain RSSI and LQI values, as well as CRC OK.

    receive <mode>               // Enable or disable printing of received RF packets on serial terminal. 1 = enabled, 0 = disabled

    transmit <times> <hex-vals>  // Send the same packet of 64 hex values over RF 

    echo <mode>                  // Enable or disable Echo on serial terminal. 1 = enabled, 0 = disabled
 
 
The code uses SmartRC library (modified Electrohouse library by Little_S@tan) which allows to customize ALL transmission parameters in human readable format without using SmartRF studio from TI (CC1101 parameter customization tool). To use it please download following ZIP library from following github link https://github.com/LSatan/SmartRC-CC1101-Driver-Lib and attach it to the script in Arduino IDE.

Arduino Pro Micro board ( ATMEGA32U4 chip ) must support 3.3Volt VCC and 3.3V TTL logic because this is required by CC1101 board, otherwise you will fry CC1101 chip. Please follow this guide to setup your Arduino environment for Arduino Pro Micro board : https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/all

If you are having issues with uploading the code from Arduino IDE to the board, after pressing "Upload" in Arduino you have to immediatelly short GND+RST pins two times in few seconds. Then bootloader in Arduino Pro Micro will start (common issue) and upload will begin.

Connections to be made :

ARDUINO PRO MICRO 3.3V / 8MHz <-> CC1101 BOARD

DIGITAL PIN 3 ( PD0 / INT0 ) <-> CC1101 GDO0
DIGITAL PIN 9 ( PB5 ) <-> CC1101 GDO2
DIGITAL PIN 10 ( PB6 ) <-> CC1101 CSN / CS / SS
DIGITAL PIN 16 ( PB2 / MOSI ) <-> CC1101 MOSI
DIGITAL PIN 14 ( PB3 / MISO ) <-> CC1101 MISO
DIGITAL PIN 15 ( PB1 / SCK ) <-> CC1101 SCLK / CLK
VCC 3.3V  <-> CC1101 VCC
GND <-> CC1101 GND
