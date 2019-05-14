/*************************************************************************************************************************
**
** - Balloon GPS PPS WSPR integration test 20190407
**
** - currently quite crude and inelegant
** - currently transmits WSPR every other fram i.e. 00, 04, 08, 12, 16, 18, (etc...)
**
** - uses pin change interupt for PPS
** - uses neoSoftwareSerial for debug serial port through MOSI and MISO of six pin ISP connector
** - uses neoGPS for gps decodig
** - uses Ron Carr's K1URC  WSPR program as a start
**   https://github.com/roncarr880/QRP_LABS_WSPR/blob/master/QRP_LABS_WSPR.ino
**
** - parts of OE1CGS example sketch from QrpLabs web site:
**
**   https://www.qrp-labs.com/ocxokit/si5351ademo.html
**
**   https://playground.arduino.cc/Main/PinChangeInterrupt
**
** - uncomment this line:   #define NEOSWSERIAL_EXTERNAL_PCINT                               // uncomment to use your own PCINT ISRs
**
**   in NeoSWSerial.h in order to use custom pin change interupt handler
**
**  //#if defined(PCINT1_vect)
**  //ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));                                            //-- gc for balloon 
**  //#endif
**
*************************************************************************************************************************/

#include <NMEAGPS.h>
#include <TimeLib.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <int.h>
#include <string.h>

// Mode defines
#define JT9_TONE_SPACING        174          // ~1.74 Hz
#define JT65_TONE_SPACING       269          // ~2.69 Hz
#define JT4_TONE_SPACING        437          // ~4.37 Hz
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
#define FSQ_TONE_SPACING        879          // ~8.79 Hz
#define FT8_TONE_SPACING        625          // ~6.25 Hz

#define JT9_DELAY               576          // Delay value for JT9-1
#define JT65_DELAY              371          // Delay in ms for JT65A
#define JT4_DELAY               229          // Delay value for JT4A
#define WSPR_DELAY              683          // Delay value for WSPR
#define FSQ_2_DELAY             500          // Delay value for 2 baud FSQ
#define FSQ_3_DELAY             333          // Delay value for 3 baud FSQ
#define FSQ_4_5_DELAY           222          // Delay value for 4.5 baud FSQ
#define FSQ_6_DELAY             167          // Delay value for 6 baud FSQ
#define FT8_DELAY               159          // Delay value for FT8

/*
**  WSPR frequencies
** =================================================
**  Band Dial freq (MHz)     Tx freq (MHz)
** =================================================
**  160m   1.836600      1.838000 -   1.838200
**   80m   3.592600      3.594000 -   3.594200
**   60m   5.287200      5.288600 -   5.288800
**   40m   7.038600      7.040000 -   7.040200
**   30m  10.138700     10.140100 -  10.140300
**   20m  14.095600     14.097000 -  14.097200
**   17m  18.104600     18.106000 -  18.106200
**   15m  21.094600     21.096000 -  21.096200
**   12m  24.924600     24.926000 -  24.926200
**   10m  28.124600     28.126000 -  28.126200
**    6m  50.293000     50.294400 -  50.294600
**    2m 144.488500    144.489900 - 144.490100
**/

#define JT9_DEFAULT_FREQ        14078700UL
#define JT65_DEFAULT_FREQ       14078300UL
#define JT4_DEFAULT_FREQ        14078500UL
//#define WSPR_DEFAULT_FREQ        7038720UL     // dial frequency ... pluse 120 Hz
#define WSPR_DEFAULT_FREQ       10138740UL     // dial frequency ... 10138800 pluse 100 Hz  10.140273  - 10138650  10138800  10138740
#define FSQ_DEFAULT_FREQ         7105350UL     // Base freq is 1350 Hz higher than dial freq in USB
#define FT8_DEFAULT_FREQ        14075000UL

#define DEFAULT_MODE            MODE_WSPR

/*************************************************************************************************************************
** 
**  - setup WSPR defines and variables
**
*************************************************************************************************************************/
#define si5351_7BITADDR   0x60                                                                   // si5351a i2c address
#define PLLA 26                                                                                  // register address offsets for PLL's
#define PLLB 34
#define CLK0_EN    1
#define CLK1_EN    2
#define CLK2_EN    4
#define DIV       14                                                                             // starting divider
#define RDIV       2                                                                             // starting sub divider ( 13 meg breakpoint for div = 1 )

/*************************************************************************************************************************
** 
**  use even dividers between 6 and 254 for lower jitter
**  freq range 2 to 150 without using the post dividers
**  we are using the post dividers
**  vco 600 to 900
**
*************************************************************************************************************************/

uint64_t clock_freq = 2500000000;                                                               // 2700465300;// * 100 to enable setting fractional frequency
//uint32_t freq = FREQ;                                                                           // ssb vfo freq
const uint32_t cal_freq = 3000000;                                                              // calibrate frequency
const uint32_t cal_divider = 200;
uint32_t divider = DIV;                                                                         //  7 mhz with Rdiv of 8, 28 mhz with Rdiv of 2
uint32_t audio_freq = 1500;                                                                     // wspr 1400 to 1600 offset from base vfo freq 
uint8_t  Rdiv = RDIV; 

// Hardware defines
#define LED_PIN                 13

// Enumerations
enum mode {MODE_JT9, MODE_JT65, MODE_JT4, MODE_WSPR, MODE_FSQ_2, MODE_FSQ_3,
  MODE_FSQ_4_5, MODE_FSQ_6, MODE_FT8};

// Class instantiation
JTEncode jtencode;

// Global variables
unsigned long freq;
char message[] = "VE3GHM FN25";
char call[] = "VE3GHM";
char loc[] = "FN25";
uint8_t dbm = 27;
uint8_t tx_buffer[255];
enum mode cur_mode = DEFAULT_MODE;
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;

time_t t            = 0;
time_t previousTime = 0;
time_t lastTimeSync = 0;                                                                     // last time the time was synced

unsigned long previousMillis = 0;

/*************************************************************************************************************************
**
**        Program: NMEA.ino
**      
**        Description:  This program uses the fix-oriented methods available() and
**          read() to handle complete fix structures.
**      
**          When the last character of the LAST_SENTENCE_IN_INTERVAL (see NMEAGPS_cfg.h)
**          is decoded, a completed fix structure becomes available and is returned
**          from read().  The new fix is saved the 'fix' structure, and can be used
**          anywhere, at any time.
**      
**          If no messages are enabled in NMEAGPS_cfg.h, or
**          no 'gps_fix' members are enabled in GPSfix_cfg.h, no information will be
**          parsed, copied or printed.
**      
**        Prerequisites:
**           1) Your GPS device has been correctly powered.
**                Be careful when connecting 3.3V devices.
**           2) Your GPS device is correctly connected to an Arduino serial port.
**                See GPSport.h for the default connections.
**           3) You know the default baud rate of your GPS device.
**                If 9600 does not work, use NMEAdiagnostic.ino to
**                scan for the correct baud rate.
**           4) LAST_SENTENCE_IN_INTERVAL is defined to be the sentence that is
**                sent *last* in each update interval (usually once per second).
**                The default is NMEAGPS::NMEA_RMC (see NMEAGPS_cfg.h).  Other
**                programs may need to use the sentence identified by NMEAorder.ino.
**           5) NMEAGPS_RECOGNIZE_ALL is defined in NMEAGPS_cfg.h
**      
**        'Serial' is for debug output to the Serial Monitor window.
**      
**        License:
**          Copyright (C) 2014-2017, SlashDevin
**      
**          This file is part of NeoGPS
**      
**          NeoGPS is free software: you can redistribute it and/or modify
**          it under the terms of the GNU General Public License as published by
**          the Free Software Foundation, either version 3 of the License, or
**          (at your option) any later version.
**      
**          NeoGPS is distributed in the hope that it will be useful,
**          but WITHOUT ANY WARRANTY; without even the implied warranty of
**          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**          GNU General Public License for more details.
**      
**          You should have received a copy of the GNU General Public License
**          along with NeoGPS.  If not, see <http:**      www.gnu.org/licenses/>.
**      
**      ======================================================================
**        The GPSport.h include file tries to choose a default serial port
**        for the GPS device.  If you know which serial port you want to use,
**        edit the GPSport.h file.
**      
**      #include <GPSport.h>
**      
**       or comment out #include <GPSport.h> and use these values inline
**
*************************************************************************************************************************/

#define gpsPort Serial
#define GPS_PORT_NAME "Serial"                                                                   //#warning Using Serial for GPS connection.

/*************************************************************************************************************************
**
** - setup SoftI2CMaster for I2C big bang communications
**
*************************************************************************************************************************/

#define SCL_PIN 1                                                                                // PB1 digital pin 1
#define SCL_PORT PORTB
#define SDA_PIN 2                                                                                // PD2 digital pin 2
#define SDA_PORT PORTD
#include <SoftI2CMaster.h>

/*************************************************************************************************************************
**
** - setup NeoSWSerial for software serical debug interface using MISO (pin 12) for RXD, and MOSI (pin 11) for TXD
**
*************************************************************************************************************************/
#include <NeoSWSerial.h>

NeoSWSerial debugSerial(12, 11);                                                                 // RX, TX  -- MISO MOSI of six pin ICSP header 

/*************************************************************************************************************************
**   For the NeoGPS example programs, "Streamers" is common set of printing and formatting routines for GPS data, in a
**   Comma-Separated Values text format (aka CSV).  The CSV data will be printed to the "debug output device".
**   If you don't need these formatters, simply delete this section.
***********************************************************************************************************************/

#include <Streamers.h>

/*************************************************************************************************************************
**   This object parses received characters into the gps.fix() data structure
*************************************************************************************************************************/

static NMEAGPS  gps;

/*************************************************************************************************************************
**   Define a set of GPS fix information. It will hold on to the various pieces as they are received from an RMC sentence.
**   It can be used anywhere in your sketch.
*************************************************************************************************************************/

static gps_fix  fix;

/*************************************************************************************************************************
** 
** - set up pinChangeInterupt
**
** - uncomment this line:   #define NEOSWSERIAL_EXTERNAL_PCINT // uncomment to use your own PCINT ISRs
**
**   in NeoSWSerial.h in order to use custom pin change interupt handler
**
**  //#if defined(PCINT1_vect)
**  //ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));  //-- gc for balloon 
**  //#endif
**
*************************************************************************************************************************/

volatile int pps = 0;                                                                            // 1 = 1pps signal has been received

const byte pinChangeInteruptPin = A5;
 
void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));                                    // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin));                                                     // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin));                                                     // enable interrupt for the group
  }

/*************************************************************************************************************************
** 
**  - handles pin change interrupt for A0 to A5
**  - this routine gets called for EVERY pin change i.e. low to high AND high to low
**
**  - if (digitalRead(A5)) =  1 then interupt is on rising edge of PPS, if 0 then on falling edge
**
**
*************************************************************************************************************************/

ISR (PCINT1_vect) {
  //debugSerial.println (digitalRead(A5));                                                       
  if (digitalRead(A5)) {                                                                         // if 1 then interupt is on rising edge of PPS, if 0 then on falling
    digitalWrite( LED_BUILTIN, !digitalRead(LED_BUILTIN) );                                      // toggle the LED and something useful here
    }
  }  


/*************************************************************************************************************************
**  
** - setup
**
*************************************************************************************************************************/

void setup()
{

  int i;

  pinMode(LED_BUILTIN, OUTPUT);                                                                  // initialize digital pin LED_BUILTIN as an output.
  digitalWrite(LED_BUILTIN, LOW); 

  pinMode(pinChangeInteruptPin, INPUT_PULLUP);                                                   // pin A5 for pinChangeInterupt, set to INPUT with PULLUP

  pciSetup(pinChangeInteruptPin);

  debugSerial.begin(19200);

  debugSerial.println( F("" ));
  debugSerial.println( F(" Balloon GPS WSPR integration 2019-04-07" ));
  debugSerial.println( F("" ));
  
  debugSerial.print( F("NMEA.INO: started\n") );
  debugSerial.print( F("  fix object size = ") );
  debugSerial.println( sizeof(gps.fix()) );
  debugSerial.print( F("  gps object size = ") );
  debugSerial.println( sizeof(gps) );
  debugSerial.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  // Set the mode to use
  cur_mode = MODE_WSPR;

  // Set the proper frequency, tone spacing, symbol count, and
  // tone delay depending on mode
  switch(cur_mode)
  {
  case MODE_JT9:
    freq = JT9_DEFAULT_FREQ;
    symbol_count = JT9_SYMBOL_COUNT;                                                              // From the library defines
    tone_spacing = JT9_TONE_SPACING;
    tone_delay = JT9_DELAY;
    break;
  case MODE_JT65:
    freq = JT65_DEFAULT_FREQ;
    symbol_count = JT65_SYMBOL_COUNT;                                                             // From the library defines
    tone_spacing = JT65_TONE_SPACING;
    tone_delay = JT65_DELAY;
    break;
  case MODE_JT4:
    freq = JT4_DEFAULT_FREQ;
    symbol_count = JT4_SYMBOL_COUNT;                                                              // From the library defines
    tone_spacing = JT4_TONE_SPACING;
    tone_delay = JT4_DELAY;
    break;
  case MODE_WSPR:
    freq = WSPR_DEFAULT_FREQ;
    symbol_count = WSPR_SYMBOL_COUNT;                                                             // From the library defines
    tone_spacing = WSPR_TONE_SPACING;
    tone_delay = WSPR_DELAY;
    break;
  case MODE_FT8:
    freq = FT8_DEFAULT_FREQ;
    symbol_count = FT8_SYMBOL_COUNT;                                                              // From the library defines
    tone_spacing = FT8_TONE_SPACING;
    tone_delay = FT8_DELAY;
    break;
  case MODE_FSQ_2:
    freq = FSQ_DEFAULT_FREQ;
    tone_spacing = FSQ_TONE_SPACING;
    tone_delay = FSQ_2_DELAY;
    break;
  case MODE_FSQ_3:
    freq = FSQ_DEFAULT_FREQ;
    tone_spacing = FSQ_TONE_SPACING;
    tone_delay = FSQ_3_DELAY;
    break;
  case MODE_FSQ_4_5:
    freq = FSQ_DEFAULT_FREQ;
    tone_spacing = FSQ_TONE_SPACING;
    tone_delay = FSQ_4_5_DELAY;
    break;
  case MODE_FSQ_6:
    freq = FSQ_DEFAULT_FREQ;
    tone_spacing = FSQ_TONE_SPACING;
    tone_delay = FSQ_6_DELAY;
    break;
  }

  #ifndef NMEAGPS_RECOGNIZE_ALL
    #error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
  #endif

  #ifdef NMEAGPS_INTERRUPT_PROCESSING
    #error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
  #endif

  #if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
      !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
      !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
      !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

    debugSerial.println( F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed.") );

  #else
    if (gps.merging == NMEAGPS::NO_MERGING) {
      debugSerial.print  ( F("\nWARNING: displaying data from ") );
      debugSerial.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      debugSerial.print  ( F(" sentences ONLY, and only if ") );
      debugSerial.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
      debugSerial.println( F(" is enabled.\n"
                            "  Other sentences may be parsed, but their data will not be displayed.") );
    }
  #endif

  debugSerial.print  ( F("\nGPS quiet time is assumed to begin after a ") );
  debugSerial.print  ( gps.string_for( LAST_SENTENCE_IN_INTERVAL ) );
  debugSerial.println( F(" sentence is received.\n"
                        "  You should confirm this with NMEAorder.ino\n") );


/*************************************************************************************************************************
**  
** - begin setting up si5351a regestiers etc
**
*************************************************************************************************************************/
  //debugSerial.println( F(" - begin setup() function, of course debugSerial must already be started by now ;)" ));
  
  //debugSerial.println( F(" - write clock 0, PLLA" ));
  si5351a_Write( 16, 0x4f);

  //debugSerial.println( F( " - write clock 1, PLLA" ));
  si5351a_Write( 17, 0x4f);

  debugSerial.println( F(" - write clock 2, PLLB" ));
  si5351a_Write( 18, 0x6f);

  //debugSerial.println( F(" - set some divider registers that will never change" ));
  for(i = 0; i < 3; ++i ){
    si5351a_Write( 42+8*i, 0);
    si5351a_Write( 43+8*i, 1);
    si5351a_Write( 47+8*i, 0);
    si5351a_Write( 48+8*i, 0);
    si5351a_Write( 49+8*i, 0);
  }

  //debugSerial.println( F(" - calibrate frequency on clock 2" ));
  si_pll_x(PLLB,cal_freq, cal_divider, 0);                                                       // calibrate frequency on clock 2
  si_load_divider(cal_divider, 2, 0, 1);

  //debugSerial.println( F(" - TX clock 1/4th of the RX clock" ));
  si_load_divider(divider, 0, 0, Rdiv * 4);                                                      // TX clock 1/4th of the RX clock

  //debugSerial.println( F(" - load divider for clock 1 and reset pll's" ));
  si_load_divider(divider, 1, 1, Rdiv);                                                          // load divider for clock 1 and reset pll's
  
  TX_OFF();

  //debugSerial.println( F(" - done setup() function" ));  
  //debugSerial.println( F("" )); 
  //debugSerial.println( F(" - begin loop() function " ));   

  trace_header( debugSerial );
  debugSerial.flush();

  //gpsPort.begin( 9600 );

  gpsPort.begin( 9600 );

  // Encode the message in the transmit buffer
  // This is RAM intensive and should be done separately from other subroutines
  set_tx_buffer();
}

/*************************************************************************************************************************
**  
**  begin main loop
**
**  re sync time every 15 minutes  900 seconds
**
**  timeStatus();                     // indicates if time has been set and recently synchronized
**                                    // returns one of the following enumerations:
**  timeNotSet                        // the time has never been set, the clock started on Jan 1, 1970
**  timeNeedsSync                     // the time had been set but a sync attempt did not succeed
**  timeSet                           // the time is set and is synced
**
**  Time and Date values are not valid if the status is timeNotSet. Otherwise, values can be used but the returned time may 
**  have drifted if the status is timeNeedsSync.
*************************************************************************************************************************/

void loop() {

  while (gps.available( gpsPort )) {
    fix = gps.read();

    trace_all( debugSerial, gps, fix );

    if ( fix.valid.location ) {
      if ( timeStatus() == timeNotSet ) {
        setTime(fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds, fix.dateTime.date, fix.dateTime.month, fix.dateTime.year);
        lastTimeSync = now();
        debugSerial.println( F("* setting date-time....." ));
        }

    if ( now() - lastTimeSync >= 903 ) {
        setTime(fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds, fix.dateTime.date, fix.dateTime.month, fix.dateTime.year);
        lastTimeSync = now();
        debugSerial.println( F("* re-sync date-time....." ));
        }
      }
    }

  t = now();
  
  if ( timeStatus() != timeNotSet ) {

    switch( second(t) )
      {
      case 1 ... 3:
        switch( minute(t) )
            {
            case 0:
                encode();
                break;
 
            case 4:
                encode();
                break;
 
            case 8:
                encode();
                break;
 
            case 12:
                encode();
                break;
 
            case 16:
                encode();
                break;
 
            case 20:
                encode();
                break;
 
            case 24:
                encode();
                break;
 
            case 28:
                encode();
                break;
 
            case 32:
                encode();
                break;
 
            case 36:
                encode();
                break;
 
            case 40:
                encode();
                break;
 
            case 44:
                encode();
                break;
 
            case 48:
                encode();
                break;
 
            case 52:
                encode();
                break;
 
            case 56:
                encode();
                break;
 
            }
      }
  }

}

/*************************************************************************************************************************
**  
**   si5351a_Write( byteRegister, byteValue )
**  
**   - writes byteValue to byteRegister using I2C
**  
**   - uses I2C address defined in si5351_7BITADDR
**  
*************************************************************************************************************************/

void si5351a_Write( byte byteRegister, byte byteValue ){

  if (!i2c_start((si5351_7BITADDR<<1)|I2C_WRITE)) {                                              // Starts transmission as master to slave 0x60 96 decimal which is 
        debugSerial.println(F("I2C device busy"));                                                  // the I2C address of the Si5351a (see Si5351a datasheet)
        return;
    }

   i2c_write( byteRegister );
   i2c_write( byteValue );
   
   i2c_stop();

}

/*************************************************************************************************************************
**   
**   TX_ON()
**   
**    -  Enables output on CLK0 and disables Park Mode on CLK1
**   
**
*************************************************************************************************************************/

void TX_ON() {
  debugSerial.println(F("TX ON"));
  si5351a_Write (17, 128);                                                                         // Disable output CLK1
  si5351a_Write (16, 79);                                                                          // Enable output CLK0, set crystal as source and Integer Mode on PLLA
  SetPower(4);
}

/*************************************************************************************************************************
**   
**   TX_OFF()
**   
**    -  Disables output on CLK0 and enters Park Mode on CLK1
**   
**
*************************************************************************************************************************/

void TX_OFF() {
  debugSerial.println(F("TX OFF"));
  si5351a_Write (16, 128);                                                                        // Disable output CLK0
  si5351a_Write (17, 111);                                                                        // Enable output CLK1, set crystal as source and Integer Mode on PLLB
}

/*************************************************************************************************************************
**   
**   SetPower( byte power )
**   
**    -  valid power values are 0 (25%), 1 (50%), 2 (75%) or 3 (100%)
**   
**    - level = 0 = CLK0 drive strength = 2mA; power level approximately  -8dB
**    - level = 1 = CLK0 drive strength = 4mA; power level approximately  -3dB
**    - level = 2 = CLK0 drive strength = 6mA; power level approximately  -1dB
**    - level = 3 = CLK0 drive strength = 8mA; power level approximately   0dB
**
*************************************************************************************************************************/

void SetPower (byte power){
  if (power == 0 || power > 4){power = 0;}                                                         // valid power values are 0 (25%), 1 (50%), 2 (75%) or 3 (100%)
  switch (power){
    case 1:
      si5351a_Write (16, 76);                                                                      // CLK0 drive strength = 2mA; power level approximately -8dB
      break;
    case 2:
      si5351a_Write (16, 77);                                                                      // CLK0 drive strength = 4mA; power level approximately -3dB
      break;
    case 3:
      si5351a_Write (16, 78);                                                                      // CLK0 drive strength = 6mA; power level approximately -1dB
      break;
    case 4:
      si5351a_Write (16, 79);                                                                      // CLK0 drive strength = 8mA; power level approximately 0dB
      break;
  }
}

/*************************************************************************************************************************
**   
**   SI5351  functions
**
*************************************************************************************************************************/

void  si_pll_x(unsigned char pll, uint32_t freq, uint32_t out_divider, uint32_t fraction ){
  
   uint64_t a,b,c;
   uint64_t bc128;                                                                                 // floor 128 * b/c term of equations
   uint64_t pll_freq;
   uint32_t P1;                                                                                    // PLL config register P1
   uint32_t P2;                                                                                    // PLL config register P2
   uint32_t P3;                                                                                    // PLL config register P3
   uint64_t r;

   c = 1000000;                                                                                    // max 1048575
   pll_freq = 100ULL * (uint64_t)freq + fraction;                                                  // allow fractional frequency for wspr
   pll_freq = pll_freq * out_divider;
   a = pll_freq / clock_freq ;
   r = pll_freq - a * clock_freq ;
   b = ( c * r ) / clock_freq;
   bc128 =  (128 * r)/ clock_freq;
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 > c ) P2 = 0;                                                                            // ? avoid negative numbers 
   P3 = c;

   si5351a_Write( pll + 0, (P3 & 0x0000FF00) >> 8);
   si5351a_Write( pll + 1, (P3 & 0x000000FF));
   si5351a_Write( pll + 2, (P1 & 0x00030000) >> 16);
   si5351a_Write( pll + 3, (P1 & 0x0000FF00) >> 8);
   si5351a_Write( pll + 4, (P1 & 0x000000FF));
   si5351a_Write( pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   si5351a_Write( pll + 6, (P2 & 0x0000FF00) >> 8);
   si5351a_Write( pll + 7, (P2 & 0x000000FF));
   
 //  si5351a_Write( 177, 0xAC );                                                                   // PLLA PLLB soft reset  
}

/*************************************************************************************************************************
**   
**   - load new divider for specified clock, reset PLLA and PLLB if desired
**
*************************************************************************************************************************/

void si_load_divider( uint32_t val, uint8_t clk , uint8_t rst, uint8_t Rdiv){                    

   uint8_t R;

   R = 0;
   Rdiv >>= 1;                                                                                     // calc what goes in the R divisor field
   while( Rdiv ){
      ++R;
      Rdiv >>= 1;
   }
   
  R <<= 4;     
   
  val = 128 * val - 512;
  
  si5351a_Write( 44+8*clk, ((val >> 16 ) & 3) | R );
  si5351a_Write( 45+8*clk, ( val >> 8 ) & 0xff );
  si5351a_Write( 46+8*clk, val & 0xff ); 
     
  if( rst ) {
    si5351a_Write( 177, 0xAC );                                                                   // PLLA PLLB soft reset needed
  }
}

// Loop through the string, transmitting one character at a time.
void encode()
{
  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  TX_ON();
  digitalWrite(LED_PIN, HIGH);

  // Now transmit the channel symbols
  if(cur_mode == MODE_FSQ_2 || cur_mode == MODE_FSQ_3 || cur_mode == MODE_FSQ_4_5 || cur_mode == MODE_FSQ_6)
  {
    uint8_t j = 0;

    while(tx_buffer[j++] != 0xff);

    symbol_count = j - 1;
  }

  for(i = 0; i < symbol_count; i++) {

   si_pll_x(PLLA, Rdiv * 4 * ( freq + audio_freq ), divider, Rdiv * 4 * tone_spacing * (tx_buffer[i]) );

   previousMillis = millis();

   while ( millis() - previousMillis < tone_delay ) {
    // keep looping
    }
  }
   
  // Turn off the output
  TX_OFF();
  digitalWrite(LED_PIN, LOW);
}


void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
  case MODE_JT9:
    jtencode.jt9_encode(message, tx_buffer);
    break;
  case MODE_JT65:
    jtencode.jt65_encode(message, tx_buffer);
    break;
  case MODE_JT4:
    jtencode.jt4_encode(message, tx_buffer);
    break;
  case MODE_WSPR:
    jtencode.wspr_encode(call, loc, dbm, tx_buffer);
    break;
  case MODE_FT8:
    jtencode.ft8_encode(message, tx_buffer);
    break;
  case MODE_FSQ_2:
  case MODE_FSQ_3:
  case MODE_FSQ_4_5:
  case MODE_FSQ_6:
    jtencode.fsq_dir_encode(call, "n0call", ' ', "hello world", tx_buffer);
    break;
  }
}

/*************************************************************************************************************************
**   
**   - that's all folks!!!!!
**
*************************************************************************************************************************/
