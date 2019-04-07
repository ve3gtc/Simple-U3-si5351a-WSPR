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
**
**  https://playground.arduino.cc/Main/PinChangeInterrupt
**
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

time_t prevDisplay = 0;                                                                      // when the digital clock was displayed

time_t lastTimeSync = 0;                                                                     // last time the time was synced


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
**  This function gets called about once per second, during the GPS quiet time.  It's the best place to do anything that 
**  might take a while: print a bunch of things, write to SD, send an SMS, etc.
**
**  By doing the "hard" work during the quiet time, the CPU can get back to reading the GPS chars as they come in, so that
**  no chars are lost.
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
**
*************************************************************************************************************************/

static void doSomeWork()
{
  // Print all the things!

  trace_all( debugSerial, gps, fix );

  if ( ( fix.valid.location && timeStatus() == timeNotSet ) || ( fix.valid.location && ( now() - lastTimeSync) >= 900 ) ) {
    //debugSerial.println( now() );
    //debugSerial.println( lastTimeSync );
    //debugSerial.println( "setting date-time....." );
    lastTimeSync = now();
    setTime(fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds, fix.dateTime.date, fix.dateTime.month, fix.dateTime.year);
    }

} 

/*************************************************************************************************************************
**   This is the main GPS parsing loop.
*************************************************************************************************************************/

static void GPSloop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    doSomeWork();
  }

} 

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
**  - setup WSPR defines and variables
**
*************************************************************************************************************************/
#define si5351_7BITADDR   0x60                                                                   // si5351a i2c address
#define PLLA 26                                                                                  // register address offsets for PLL's
#define PLLB 34
#define CLK0_EN   1
#define CLK1_EN   2
#define CLK2_EN   4
#define FREQ 7038720                                                                             // 7038600
#define DIV   14                                                                                 // starting divider
#define RDIV   2                                                                                 // starting sub divider ( 13 meg breakpoint for div = 1 )

#define FRAME_MODE 1                                                                             // or self timed frame (stand alone mode)

/*************************************************************************************************************************
** 
**  use even dividers between 6 and 254 for lower jitter
**  freq range 2 to 150 without using the post dividers
**  we are using the post dividers
**  vco 600 to 900
**
*************************************************************************************************************************/

uint64_t clock_freq = 2500000000;                                                               // 2700465300;// * 100 to enable setting fractional frequency
uint32_t freq = FREQ;                                                                           // ssb vfo freq
const uint32_t cal_freq = 3000000;                                                              // calibrate frequency
const uint32_t cal_divider = 200;
uint32_t divider = DIV;                                                                         //  7 mhz with Rdiv of 8, 28 mhz with Rdiv of 2
uint32_t audio_freq = 1500;                                                                     // wspr 1400 to 1600 offset from base vfo freq 
uint8_t  Rdiv = RDIV; 

uint8_t  operate_mode = FRAME_MODE;                                                             // start in stand alone timing mode
uint8_t wspr_tx_enable;
uint8_t cal_enable;

/*************************************************************************************************************************
**  
**     Download WSPRcode.exe from  http://physics.princeton.edu/pulsar/K1JT/WSPRcode.exe  and run it in a dos window 
**  
**     Type (for example):   WSPRcode "K1ABC FN33 37"    37 is 5 watts, 30 is 1 watt, 33 is 2 watts, 27 is 1/2 watt
**     - use capital letters in your call and locator when typing in the message string.  
**     - No extra spaces!
**     - using the editing features of the dos window, mark and copy the last group of numbers.
**     - paste into notepad and replace all 3 with "3,"  all 2 with "2," all 1 with "1," all 0 with "0,".
**     - remove the comma on the end.
**  
**     - the goal of course, is to create a list like that found below in wspr_msg[]
**  
**     - the current message is   "VE3GHM FN25 23"
**  
*************************************************************************************************************************/

const char wspr_msg[] = { 
3, 3, 2, 2, 0, 2, 2, 2, 1, 2, 2, 2, 3, 1, 3, 2, 2, 0, 3, 0, 2, 3, 0, 3, 3, 1, 1, 2, 0, 0,
0, 2, 0, 0, 3, 0, 2, 1, 0, 3, 0, 2, 0, 0, 0, 2, 1, 0, 3, 3, 2, 0, 1, 3, 2, 1, 2, 2, 2, 3,
3, 2, 1, 2, 2, 0, 0, 1, 1, 2, 1, 2, 1, 2, 1, 2, 3, 2, 0, 3, 2, 2, 1, 0, 3, 1, 0, 2, 0, 1,
1, 0, 3, 2, 3, 0, 0, 0, 3, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 3, 1, 0, 1, 3, 0, 0, 1, 1,
0, 1, 0, 2, 2, 3, 3, 3, 2, 2, 2, 0, 0, 3, 0, 1, 2, 0, 3, 3, 2, 0, 2, 0, 0, 0, 0, 3, 3, 2,
3, 2, 1, 1, 2, 0, 0, 3, 1, 0, 0, 2
 };

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

  debugSerial.println( "" );
  debugSerial.println( " Balloon GPS WSPR integration 2019-04-07" );
  debugSerial.println( "" );
  
  debugSerial.print( F("NMEA.INO: started\n") );
  debugSerial.print( F("  fix object size = ") );
  debugSerial.println( sizeof(gps.fix()) );
  debugSerial.print( F("  gps object size = ") );
  debugSerial.println( sizeof(gps) );
  debugSerial.println( F("Looking for GPS device on " GPS_PORT_NAME) );

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

  trace_header( debugSerial );
  debugSerial.flush();

  gpsPort.begin( 9600 );

/*************************************************************************************************************************
**  
** - begin setting up si5351a regestiers etc
**
*************************************************************************************************************************/
  debugSerial.println( " - begin setup() function, of course debugSerial must already be started by now ;)" );
  
  debugSerial.println( " - write clock 0, PLLA" );
  si5351a_Write( 16, 0x4f);

  debugSerial.println( " - write clock 1, PLLA" );
  si5351a_Write( 17, 0x4f);

  debugSerial.println( " - write clock 2, PLLB" );
  si5351a_Write( 18, 0x6f);

  debugSerial.println( " - set some divider registers that will never change" );
  for(i = 0; i < 3; ++i ){
    si5351a_Write( 42+8*i, 0);
    si5351a_Write( 43+8*i, 1);
    si5351a_Write( 47+8*i, 0);
    si5351a_Write( 48+8*i, 0);
    si5351a_Write( 49+8*i, 0);
  }

  debugSerial.println( " - calibrate frequency on clock 2" );

  si_pll_x(PLLB,cal_freq, cal_divider, 0);                                                       // calibrate frequency on clock 2
  si_load_divider(cal_divider, 2, 0, 1);

  //debugSerial.println( " - receiver 4x clock" );

  //si_pll_x(PLLA,Rdiv*4*freq, divider, 0);                                                      // receiver 4x clock

  debugSerial.println( " - TX clock 1/4th of the RX clock" );
  si_load_divider(divider, 0, 0, Rdiv * 4);                                                      // TX clock 1/4th of the RX clock

  debugSerial.println( " - load divider for clock 1 and reset pll's" );
  si_load_divider(divider, 1, 1, Rdiv);                                                          // load divider for clock 1 and reset pll's
  
  debugSerial.println( " - done setup() function" );  
  debugSerial.println( "" ); 
  debugSerial.println( " - begin loop() function " ); 

  TX_OFF();
  
}

//--------------------------

void loop()
{
  static unsigned long ms;
  
  GPSloop();

  if (now() != prevDisplay) {                                                                 //update the display only if time has changed
    prevDisplay = now();
    debugSerial.print(year()); 
    debugSerial.print("-");
    if(month() < 10)
      debugSerial.print('0');
    debugSerial.print(month());
    debugSerial.print("-");
    if(day() < 10)
      debugSerial.print('0');
    debugSerial.print(day());
    debugSerial.print(" ");
    debugSerial.print(hour());
    debugSerial.print(":");
    if(minute() < 10)
      debugSerial.print('0');
    debugSerial.print(minute());
    debugSerial.print(":");
    if(second() < 10)
      debugSerial.print('0');
    debugSerial.print(second());
    debugSerial.println(); 
    }

  // very crude  and inelegant but Ok for now
  
  if      ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) ==  0 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) ==  4 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) ==  8 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 12 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 16 ) {
    wspr_tx_enable = 1;
    }
  if      ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 20 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 24 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 28 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 32 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 36 ) {
    wspr_tx_enable = 1;
    }
  if      ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 40 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 44 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 48 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 52 ) {
    wspr_tx_enable = 1;
    }
  else if ( ( fix.valid.location && timeStatus() == timeSet ) && minute(now()) == 56 ) {
    wspr_tx_enable = 1;
    }     
    
  if( wspr_tx_enable ) {
      wspr_tx(millis());
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
  // direct register writes.  A possible speed up could be realized if one were
  // to use the auto register inc feature of the SI5351

  // debugSerial.println(" - begin si5351a_write routine)");
  // debugSerial.print("   writing to register :");
  // debugSerial.print( byteRegister );
  // debugSerial.print("   value :");
  // debugSerial.println( byteValue );

  if (!i2c_start((si5351_7BITADDR<<1)|I2C_WRITE)) {                                              // Starts transmission as master to slave 0x60 96 decimal which is 
        debugSerial.println("I2C device busy");                                                  // the I2C address of the Si5351a (see Si5351a datasheet)
        return;
    }

   i2c_write( byteRegister );
   i2c_write( byteValue );
   
   i2c_stop();

}


/*************************************************************************************************************************
**   
**   wspr_tx( t ))
**   
**    - t is unsigned long
**   
**
*************************************************************************************************************************/

void wspr_tx( unsigned long t ){
static int i;
static unsigned long timer;
static uint8_t mod;

   debugSerial.print(" - begin wspr_tx routine  t = ");
   debugSerial.println( t );
      
   if( i != 0 && (t - timer) < 683 ) return;                                                      // baud time is 682.66666666 ms
   timer = t;
   ++mod;   mod &= 3;
   if( mod == 0 ) ++timer;                                                                        // delay 683, 683, 682, etc.

   if( i == 162 ){
      TX_OFF();
      i = 0;                                                                                      // setup for next time to begin at zero index
      wspr_tx_enable = 0;                                                                         // flag done
      return;
   }
   // set the frequency
   debugSerial.print(" wspr_msg(index) index = ");
   debugSerial.println( i );
   debugSerial.print(" wspr_msg(index) contents = ");
   debugSerial.println( wspr_msg[i] );
   debugSerial.print(" divider = ");
   debugSerial.println( divider );
   debugSerial.print(" Rdiv * 4 * 146 * wspr_msg[i] = ");                                         // 146
   debugSerial.println( Rdiv * 4 * 146 * wspr_msg[i] );

   si_pll_x(PLLA, Rdiv * 4 * ( freq + audio_freq ), divider, Rdiv * 4 * 146 * (wspr_msg[i] + 1) );
   if( i == 0 ) TX_ON();
   ++i; 
}

/********************************************************************************************************************************
** - frame timer currently not used, replaced with logic based on time in the main loop
********************************************************************************************************************************/
void frame_timer( unsigned long t ){   
static int msec;
static unsigned long old_t;
static uint8_t slot;
static uint8_t sec;
static int time_adjust;   
// 16mhz clock measured at 16001111.  Will gain 1ms in approx 14401 ms.  Or 1 second in 4 hours.

   msec += ( t - old_t );
    time_adjust += (t - old_t);
    if( time_adjust >= 14401 && msec != 0 ) time_adjust = 0, --msec;
   old_t = t;
   if( msec >= 1000 ){
      msec -= 1000;
      if( ++sec >= 120 ){     // 2 minute slot time
        sec -= 120;
        if( ++slot >= 10 ) slot = 0;   // 10 slots is a 20 minute frame
        
        debugSerial.print(F("Slot ")); 
        debugSerial.println(slot);
        
        if( slot == 1 && operate_mode == FRAME_MODE ) wspr_tx_enable = 1;

        if( slot == 3 && operate_mode == FRAME_MODE ) wspr_tx_enable = 1;

        if( slot == 5 && operate_mode == FRAME_MODE ) wspr_tx_enable = 1;

        if( slot == 7 && operate_mode == FRAME_MODE ) wspr_tx_enable = 1;

        if( slot == 9 && operate_mode == FRAME_MODE ) wspr_tx_enable = 1;

        
        // enable other modes in different slots
        // if( slot != 1 ) cal_enable = 1;
      }
   } 
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
  debugSerial.println("TX ON");
  si5351a_Write (17, 128);                                                                         // Disable output CLK1
  si5351a_Write (16, 79);                                                                          // Enable output CLK0, set crystal as source and Integer Mode on PLLA
  SetPower(4);
  digitalWrite(LED_BUILTIN, HIGH);
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
  debugSerial.println("TX OFF");
  si5351a_Write (16, 128);                                                                        // Disable output CLK0
  si5351a_Write (17, 111);                                                                        // Enable output CLK1, set crystal as source and Integer Mode on PLLB
  digitalWrite(LED_BUILTIN, LOW);
}

/*************************************************************************************************************************
**   
**   TX_OFF()
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
 uint64_t bc128;                                                                                   // floor 128 * b/c term of equations
 uint64_t pll_freq;

 uint32_t P1;                                                                                      // PLL config register P1
 uint32_t P2;                                                                                      // PLL config register P2
 uint32_t P3;                                                                                      // PLL config register P3
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

   debugSerial.print(" - setting frequency: ");
   debugSerial.println( freq );

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

/*************************************************************************************************************************
**   
**   - that's all folkd!!!!!
**
*************************************************************************************************************************/
