//----------------------------------------------------------------------
// Power monitor for ESP32
//
// This will use a Current Transformer to measure current through a wire
// and log the time and amount of current that is drawn over time.
//
// To make it battery capable it will use the deep-sleep mode of the ESP32
// to sleep most of the time, turning on periodically to take a measurement
// and log the data, and then sleeping again.
//
// At some interval, it will try to transfer it's logged data to an external
// server over WiFi.
//
// We are monitoring a well water pump, so it goes on and off in time
// frames measured in minutes.   So we don't have to sample real often.
//
// We're going to watch for it to go from OFF to ON, then record the time-on
// as a time-stamp.   Then we'll wait for it to go from ON to OFF and
// so forth.  We'll record the on/off event with a time stamp.
//
// This is using a current transformer sensor that clamps around the hot line
// of the AC supply.   When current is flowing through the hot line, there
// is an induced current in the sensor.   This is current flows through
// a burden resistor in parallel with the sensor to generate a voltage.
//
// This voltage is AC and it's RMS value is proportional to the current
// in the hot wire.   By measuring the AC voltage (RMS) of across the
// sensor/burden resistor, we can measure the current flowing through
// the hot wire that the sensor is clamped around.
//
// Since the ADC can only read positive voltages, we use a voltage
// divider to offset the voltage across the sensor/burden resistor
// so we read 1.65V +/- the AC voltage from the sensor.   The RMS
// measurement routine takes this into account.
//
// Circuit diagram for sensor.
//----------------------------------------------------------------
//
//     o +3.3v (GPIO17 set to 3.3V)
//     |
//     |
//     \
//     /              current txfrmr
//     \  1K        o--UUUUU---o
//     /            |          |
//     |  3.3v/2    |          |
//     o------------o---/\/\---o------------o ADC A7 (ADC1_CH7) = GPIO 35
//     |            |  burden  |              (1.65v +- AC sensor voltage)
//     \            | resistor |
//     /  1K        |          |
//     \            o---|(-----o
//     /               filter cap 1uf
//     |
//     |
//     o  GND
//
//----------------------------------------------------------------------
// TODO list
// - Make upload period configurable in config.ini file
//

// Board: WEMOS LOLIN32 Lite
// Partition: Default

#include <Arduino.h>
#include <ESP32Time.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ESP32_FTPClient.h>    // FTP client to upload photos

// This RTC_DATA_ATTR puts the data into the RTC memory which
// is preserved across deep-sleep mode.   Any other data in ESP32
// regular memory will be lost after waking from a deep sleep.

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int needNtp = true;
RTC_DATA_ATTR int lastHr=-1;
RTC_DATA_ATTR int pumpState=0; // assume it's initially OFF (0), ON is (1)
// Information from configuration file
RTC_DATA_ATTR unsigned int uploadPort;
RTC_DATA_ATTR float ampsForOnMeasurement;

RTC_DATA_ATTR char ssid[128];
RTC_DATA_ATTR char password[128];
RTC_DATA_ATTR char ftpHost[128];
RTC_DATA_ATTR char ftpUsername[64];
RTC_DATA_ATTR char ftpPassword[64];
RTC_DATA_ATTR char ftpPath[128];
RTC_DATA_ATTR char ftpFile[128];
RTC_DATA_ATTR long timeToSleepMs;

// we will log to Flash, then periodically upload flash data to an FTP server
// appending to a log file

#define ms_TO_uS_FACTOR 1000  /* Conversion factor for micro seconds to milli seconds */

#define WANTSERIAL (1)
#define SIGNON "\nIOT Power Monitor 1.0, 1/21/2024"

char wifiIsConnected=0; // 1 means connected

//-----------------------------------------------------------------
char msgbuf[256]; // for printing messages with sprintf
void print(String msg)
{
#ifdef WANTSERIAL
  Serial.println(msg);
#endif
}

void print(char* msg)
{
#ifdef WANTSERIAL
  Serial.println(msg);
#endif
}

//-----------------------------------------------------------------
// real time clock (software based, not backed up for power failures
ESP32Time rtc(-8*3600);  // -8 from GMT by default

//============================= RMS POWER MEASUREMENT =========================
// NOTES:
// Test using ADC to read a voltage (60 Hz) and measure
// RMS.
// Dean Gienger, Nov 10, 2022
//
// Objective: Use a current transformer to convert current to
// voltage.  Use ESP32 with ADC to measure the voltage and
// calculate the RMS value.
//
// CT used is 100A -> 50MA (rms), with 33.3 ohm burden resistor
// 1.67 vrms max on a 33.3 ohm burden resistor
// (4.70 vptp)
//
// We're using a 3.3V max ADC so that means 1.17 vrms or 70A max measurement
// (3.3/2.82)/33.3 * (100/0.05) = 70.28 A
//
// Some notes:
// 1- For 60Hz sine-wave shaped signals, RMS is Vptp/2*sqrt(2)
//    Mainly for scaling, we will use the ADC which will see
//    Vptp (peak-to-peak voltage).
// 2- The ADC measures only positive voltages 0 to +3.3 volts
//    So we have a voltage divider to 3.3/2 = 1.65 volts and
//    connect the current transformer so that we will measure
//    1.65 volts +- the 60 Hz signal.   This 1.65 volts is not
//    related to the RMS of the signal we're measuring - it's
//    just a scaling offset to accomdate the ADC range.
//
// We will try to do the following
// a) Sample at 16x the signal frequency (60Hz) so sampling
//    at frequency of 16*60 or 960Hz.  1 sample every 1041.67 us
// b) Sample for like 10 cycles = 16*60*10 = 9600 samples
// c) To compute the RMS, we will
//    - Compute the mean of the 9600 samples (mu)
//    - Subtract the mean from all the samples (x[i]-mu)-> y[i]
//    - Sum the squares of y[i], divide by 9600 then take the square root
//
// d) Fixed point scaling, the samples are 10 bits, so square them we get
//    20 bits, then sum 160 of them we get 8 bits more so we get 28 bits
//    needed for max resolution.  So it's safe to use 32 bit integers for
//    this without fearing overflow
//
//
// Testing, 10 Nov 2022, Dean
// Hairdryer measured with WattsUp meter
// Fan only, 60W
// Medium, 1070W
// High, 1590W
//
// Measured RMS voltages with 33 ohm burden resistor
//
// OFF (no current) 0.01 +/- 0.05 - quite noisy
// 60W (0.52A) 0.018 - still quite noisy
// 1070W (9.3A) 0.122
// 1590W (13.8A) 0.198
//
// The noise means it's going to be impossible to measure less than about 250W
//

// ADC pin A7 (ADC1_CH7) = GPIO 35
#define ADCPIN (35) // reading analog input from current transformer sensor A7
#define LED (5)     // built-in LED - blink on when we wake up
#define VPPPIN (17) // Set high to power voltage divider for measurement

// 100 cycles of 60 Hz at 16 OSR
#define SIGNALRATE (60)
#define OSR (16)
#define CYCLES (20)
#define NSAMPLES (OSR*CYCLES)

volatile int sampleCount = NSAMPLES;
volatile int samples[NSAMPLES]; // ISR will put samples here when sampleCount < 

hw_timer_t *My_timer = NULL; // handling a timer on the ESP32 chip with interrupt

//---------------------------------------------------------------------
// Timer Interrupt Service Routine (ISR)
//
// Software is using a timer with ISR
// to read the ADC and store samples in a buffer.
// Then in mainline code we will do the RMS calculation after all the
// samples are available.
void IRAM_ATTR onTimer()
{
  // the ISR is always active, but only runs the ADC when a sampling
  // is triggered.
  // To trigger the sampling interval, mainline code sets sampleCount=0
  // Then on the next ISR execution, it will start collecting samples
  // and storing them in the samples[] buffer.   When NSAMPLES have been
  // stored, the ISR stops reading the ADC until mainline code again
  // sets sampleCount=0
  //
  if ((sampleCount>=0) && (sampleCount < NSAMPLES))
  {
    samples[sampleCount++] = analogRead(ADCPIN);
  }
}

//--------------------------------------------------------------------
// Initiate a read of analog samples from the ADC
void readAnalogSamples()
{
  int dly=17*CYCLES;
  sampleCount = 0; // this triggers the ISR to start reading the samples
  
  // This should cause the ISR to read samples for next CYCLES of 60Hz (16.67 ms per cycle)
  delay(dly); // we're delaying for 17 cycles, by then the ISR should be finished reading the samples
  if (sampleCount!=NSAMPLES)
  {
    print("ADC processing is not working");
  }
  //for (int i = 0; i < NSAMPLES; i++)
  //{
  //  sprintf(msgbuf,"%d",samples[i]); print(msgbuf);
  //}
  timerWrite(My_timer,0); // disable timer, we're done with interrupts
}

//--------------------------------------------------------------------
// Measure the RMS value of the samples recorded
float measureRms(int* samples, int nsamples)
{
  // this is tricky because of noise and because of the cyclic nature of the signal...
  //
  // first calculate the mean of the samples, or use something fixed
  // this is because the ADC measures positive voltages only (0 to +3.3 v)
  // so if we have a signal like a sine wave, we "bias" it to 3.3/2 volts
  // using an RC divider so that we can measure +- 1.65 volts in a 0 to 3.3v
  // scale.
  int32_t sum=0; // 32 bit sum
  for (int i = 0; i < nsamples; i++) sum += samples[i];
  int mean = (int)(sum/(int32_t)(nsamples));


  // RMS is root-mean-square 
  // so compute sum of squares, divide by nsamples, take square root
  // now compute sum of (x-mean)^2
  sum=0;
  for (int i = 0; i < nsamples; i++)
  {
    int32_t y = (samples[i] - mean);
    sum += y*y;
  }
  float ym = (float)sum/(float)nsamples;
  float rms = sqrt(ym);
  rms = rms * 3.3/4096.0; // scale to volts, 3.3v (MAX) is a count of 4095
#ifdef WANTSERIAL
  //sprintf(msgbuf,"mean=%d",mean); print(msgbuf);
  //sprintf(msgbuf,"meansq=%ld",sum); print(msgbuf);
  //sprintf(msgbuf,"rmssamples=%f",rms); print(msgbuf);
  //sprintf(msgbuf,"rmsvolts=%f",rms); print(msgbuf);
#endif  
  //for (;;) ; // temp - hang here forever
  return rms;
}

//--------------------------------------------------------------------
// convert measured vrms to amps using some primitive calibration data
// and linear interpolation
float cvtRmsToAmps(float vrms)
{
  // This converts the RMS reading (in volts) to an amperage reading (in amps)
  //
  // The nature of real life is (signal + noise + non-linearity)=measured value
  // We have tried to take out some of the noise component by averaging over
  // a few dozen cycles of the sine wave.
  // This routine will attempt to address the non-linearity portion by applying
  // a calibration.   We did a calibration by reading the RMS voltage with some
  // known loads with a "good" meter.   So we know the actual measured rms, the
  // actual watts, and we can compute a factor to convert between the two.
  //
  // calibration data from WattsUp watt meter, assuming 115Vrms
  //   0W  ->  0.00A -> measured 0.010 noise
  //   60W ->  0.52A -> measured 0.018 vrms      0.52/(0.018-0.01) = 65
  // 1070W ->  9.30A -> measured 0.122 vrms      9.30/0.122 = 76.2
  // 1590W -> 13.82A -> measured 0.198 vrms     13.82/0.198 = 69.8

  // expected with 50ma=100A primary current, and 33.3 ohm burden resistor (100/0.05)/33.3 = 60.06
  // so the ideal, linear, spherical constant is 60.06
  // We measured things like 65, 76.2, and 69.8 using known loads
  // So we're in the right ball-park.
  //
  // We'll constuct a piece-wise linear interpolation curve to take the actual
  // measured RMS of an unknown load and convert it to some calibrated current
  // value.
  //
  if (vrms < 0.01) return 0.0;
  if (vrms < .122) return vrms*(65.0+((vrms-0.01)/.122)*(76.2-65.0));
  if (vrms < .198) return vrms*(76.2+(vrms/.198)*(69.8-76.2));
  return vrms*(60.0+(vrms/1.67)*(60-69.8));
}

#define CLOCKRATE 80000000 /* Hz */
#define TIMERDIVIDER 4


//----------------------------------------------------------------------
void setupMeasurement()
{
  // initialize ADC - no setup initialization is required apparently
  //analogReadResolution(12);
  //analogSetWidth(12);
  //analogSetClockDiv(1);
  //analogSetAttenuation(ADC_DB11);
  //analogSetPinAttenuation(ADCPIN, ADC_DB0);

  // zero out the buffer
  //for (int i = 0; i < NSAMPLES; i++) samples[i] = 0;

  // initialize timer for interrupt every 1041.6666 is as close as we can get
  // (1000000us/(16*60) us
  // timer 1 - set up to generate periodic interrupts for reading the ADC
  My_timer = timerBegin(1, // Timer 1
                        TIMERDIVIDER, // prescaler, 80MHz/4 = 20 MHz tick rate 
                        true);  // true means count up

  timerAttachInterrupt(My_timer, &onTimer, true); // attach the ISR to read analog samples

  // we're trying to measure a sine wave (noisy one, but kind of a sine wave)
  // of a voltage coming off of the current transformer sensor
  //
  // We measure at a rate to get exactly 16 samples for every sine wave
  //
  float measIntervalSec = 1.0/(60.0*OSR); // for 16x osr, 1041.67 us
  
  int count = (int)(measIntervalSec*CLOCKRATE/TIMERDIVIDER + 0.5); // round to nearest integer
  
  timerAlarmWrite(My_timer, count, true); // timer for interrupts
  timerAlarmEnable(My_timer); // and finally, Enable the dang interrupt
}

//----------------------------------------------------------------------
float makeMeasurement()
{
  // measure the current using the current transformer sensor
  // It generates some voltage that get's fed into the ESP32's Analog->Digital converter
  // Then we do some math to make that into current (amps)
  //
  // remember: measured signal = (real signal + noise + non-linearity)
  //
  float rms;
  float amps = -1.0;
  readAnalogSamples();  // read many chcles of the sine wave at like 16 samples/cycle
    
  if (sampleCount==NSAMPLES)
  {
    rms = measureRms((int*)samples, NSAMPLES);  // convert samples to an RMS voltage
    amps = cvtRmsToAmps(rms); // convert RMS voltage to amps
#ifdef WANTSERIAL
    //sprintf(msgbuf,"Measured=%f volts, Amps %f",rms, amps); print(msgbuf);
#endif
  }
  return amps;
}


// File names in on-board SPIFFS
#define LOGFN "/iotdata.log"
#define CONFIGFN "/config.ini"

//-------------------------------------------------------------
// append a line to the log file
void appendToLogFile(String msg)
{
  print(msg);

  // append to log file
  File fout = SPIFFS.open(LOGFN, FILE_APPEND);
  if (!fout)
  {
    print("Unable to append to log file");
  }
  else
  {
    fout.print(msg);
    fout.print("\n");
    fout.close();
  }
}

//----------------------------------------------------------------------
// Compose a log line with a leading time-tag then append to log file
void addLogLine(char* msg)
{
  char buf[128];
  String ttag = rtc.getTime("%Y/%m/%d,%H:%M:%S"); // "2022/11/16,18:22:01"
  sprintf(buf,"%s,%s",ttag.c_str(), msg);
  appendToLogFile(buf); 
}

//----------------------------------------------------------
// read line from input text file
int readln(File finp, uint8_t* buf, int maxlen)
{
  // return true on successful read, false on EOF
  // 10 or 13 (LF, CR) or both are EOL indicators
  int len=0;
  int eof=false;

  buf[0]=0;
  while (len<(maxlen-1))
  {
    if (!finp.available())
    {
      eof=true;
      break;
    }
    char c = finp.read();
    if (c < 0) 
    {
      eof=true;
      break; // EOF
    }
    if (c==13) continue; // ignore CR
    if (c==10) break; // end-of-line
    buf[len++]=c;
  }
  buf[len]=0; // null terminate
  return !eof;
}

//----------------------------------------------------------
// retrieve a value for a key in the config file
void readKey(char* configFn, char* key, char* outbuf, int maxlen)
{
  outbuf[0] = 0; // returning null string on error 
  //
  // Config file is key=value format
  // SSID=mywifi
  // PASSWORD=mypassword
  // TIMEZONE=-8
  // OFFSET=123590 
  //
  // pass in key with trailing = sign!!! 
  // readKey("/test.cfg","MYKEY=", outbuf, 127);

  File finp = SPIFFS.open(CONFIGFN, FILE_READ);
  if (!finp)
  {
    print("Unable to read config file");
    return;
  }
  // scan file and look for key
  char buf[128];
  int n = strlen(key);
  while (readln(finp, (uint8_t*) buf, 127))
  {
    if (strncmp(buf,key,n) == 0) // found
    { 
      print(buf);
      strncpy(outbuf,&buf[n],maxlen);
      break;
    }
  }
  finp.close();
 
}

int wifiWaitCounter=0;
String ipAddress = String("unknown");

//------------------------------------------------------------------------
// convert IP address to string
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}

//----------------------------------------------------------------------
// Try to connect to WiFi AP
int connectToWiFi()
{
 // Connect to Wi-Fi, 1=connected, 0=not connected

  WiFi.begin(ssid, password);
  wifiWaitCounter=0;
  wifiIsConnected=0;
  
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    print("Connecting to WiFi..");
    if (wifiWaitCounter++ > 30)
    {
      addLogLine("WIFI Connect Failed");
      return 0; // not connected
    }
  }

  // Print ESP32 Local IP Address
  //ipAddress = "WIFICONNECT,"+IpAddress2String(WiFi.localIP());
  //addLogLine((char*)ipAddress.c_str());
  print("WiFi connected, "+IpAddress2String(WiFi.localIP()));
  wifiIsConnected=1;
  return wifiIsConnected;
}

//----------------------------------------------------------------------
// Turn off the WiFi to save power
void turnOffWiFi()
{
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF); 
    wifiIsConnected = 0;
}

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

//----------------------------------------------------------------------
// Try to contact an NTP server to see what the current date/time is
void getNtpTime()
{
  if (wifiIsConnected==0) return; // can't do it if no wifi connected.
  // NTP Client
  timeClient.begin();
  timeClient.setTimeOffset(0);
  int waitingForNtp = 12; // seconds max wait for NTP
  while (waitingForNtp--)
  {
    if (!timeClient.update())
    {
      timeClient.forceUpdate();
      delay(1000);
    }
  }

  if (waitingForNtp > 0)
  {
    unsigned long epochTime = timeClient.getEpochTime();
    print("NTP Time: ");
    print(timeClient.getFormattedDate());
    addLogLine((char*)("NTP,"+timeClient.getFormattedDate()+","+IpAddress2String(WiFi.localIP())).c_str());
    rtc.setTime(epochTime);
  }
}

//----------------------------------------------------------------------
// Process a current measurement and log ON/OFF events with time tag
void logMeasurement(float amps)
{
  // We only log a measurement if the pump changed state since our
  // last measurement. (went from OFF to ON, or ON to OFF)
  char buf[128];
  //sprintf(buf,"%s,%f amps",ttag.c_str(), amps);
  //print(buf);
  if (amps>=ampsForOnMeasurement) // we count it as ON
  {
    if (pumpState==0) // was off?
    {
      // yes
      pumpState=1; // mark it as on
      // we have an ON event
      sprintf(buf,"ON,%f",amps);
      addLogLine(buf);
    }
  }
  else // amps < 6
  {
    if (pumpState==1) // was on?
    {
      // yes
      pumpState=0;
      // we have an OFF event
      sprintf(buf,"OFF,%f",amps);
      addLogLine(buf);
    }
  }
}

//----------------------------------------------------------------------
// Use FTP to try to append any locally logged data to a log file
// on the FTP server.
void pushDataToServer()
{
  int success = false;
  char buf[128];

  // upload any data recorded in the log file to the server
  File finp = SPIFFS.open(LOGFN, FILE_READ);
  if (!finp)
  {
    print("Unable to read config file");
    return; // no file to upload
  }


  // contact FTP server and upload data, appending it to a file
  // specified in the config.ini  
  if (wifiIsConnected == 0) return; // no wifi means no client action can happen

  print("------ uploading log data to server -------");

  ESP32_FTPClient ftp(ftpHost,ftpUsername,ftpPassword, 5000, 2); // FTP object
  
  ftp.OpenConnection();
  ftp.InitFile("Type I");
  ftp.ChangeWorkDir(ftpPath);
  ftp.AppendFile(ftpFile);
  // copy any log file contents to append to file on the FTP server
  while (readln(finp, (uint8_t*) buf, 127))
  {
    strcat(buf,"\n");
    ftp.WriteData( (unsigned char*)buf, strlen(buf) );
  }
  finp.close();

  ftp.CloseFile();
  ftp.CloseConnection();
  success = (strcmp(ftp.GetStatus(),"OK") == 0);

  if (success)
    // delete the log file, since we've uploaded it's data
    SPIFFS.remove(LOGFN);
}

//----------------------------------------------------------------------
void setup() 
{
  // --- deep sleep mode note ---
  // Since this application puts the ESP32 into deep sleep mode most of
  // the time, waking from deep sleep means the contents of memory are
  // unknown, so the entire program is read back into memory from flash
  // and we execute the setup() routine.   All the application logic
  // happens here, then we go back to sleep.
  // 
  // So the normal loop() function is never actually executed!
  //
  char buf[32];
  esp_sleep_wakeup_cause_t wakeup_reason;
#ifdef WANTSERIAL
  Serial.begin(115200);
#endif
  print(SIGNON);

  // see why we woke up 
  wakeup_reason = esp_sleep_get_wakeup_cause(); // see why we woke up - cold boot or deep sleep
  
  setCpuFrequencyMhz(80); // take it easy on CPU speed to reduce power consumption

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0     : print("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1     : print("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER    : print("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : print("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP      : print("Wakeup caused by ULP program"); break;
    default : print("Wakeup was not caused by deep sleep"); break;
  }

  // Initialize SPIFFS (file system)
  if(!SPIFFS.begin(true))
  {
    print("An Error has occurred while mounting SPIFFS");
    //return; what to do here?  We can't do much without the file system
  }

  pinMode(LED,OUTPUT);
  pinMode(VPPPIN,OUTPUT);

  digitalWrite(LED,HIGH); // use this as 3.3v src for measurement voltage divider
  digitalWrite(VPPPIN,HIGH); // power the current tx sensor for a little bit

  // Stuff to do on an initial boot from power on or reset (not done on wakeup)
  if (bootCount == 0)
  {
    // read needed data from the config file - stored in RTC memory so it survives deep sleep
    readKey(CONFIGFN,"SSID=",ssid,127); 
    readKey(CONFIGFN,"PASSWORD=",password,127);
    readKey(CONFIGFN,"FTPHOST=",ftpHost,127);
    readKey(CONFIGFN,"FTPUSER=",ftpUsername,63);
    readKey(CONFIGFN,"FTPPASSWORD=",ftpPassword,63);
    readKey(CONFIGFN,"FTPFILE=",ftpFile,127);
    readKey(CONFIGFN,"FTPPATH=",ftpPath,127);
    readKey(CONFIGFN,"AMPSON=",buf,31);
    ampsForOnMeasurement = atof(buf);
    readKey(CONFIGFN,"MEASUREMENTINTERVALMS=",buf,31);
    timeToSleepMs = atol(buf);
    Serial.print("Meaurement Interval (ms) ");
    Serial.println(timeToSleepMs);
    if (timeToSleepMs < 2000) timeToSleepMs = 2000;
    if (timeToSleepMs > 120*1000) timeToSleepMs = 120*1000;

    // initialize the RTC
    rtc.setTime(0,0,0,1,1,2024); // default time 00:00:00 1/1/2024
    if (connectToWiFi() == 1)
    {
      needNtp = true;
      getNtpTime(); // get NTP time if possible
    }
    turnOffWiFi(); // now we can turn off the WiFi modem to save power
  }
  
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)
  {
    // most of the time, the app wakes up from deep sleep and makes
    // a current measurement.   If the current is above some threshold
    // value (specified in config.ini file) then we consider the load (pump)
    // is ON, otherwise it's OFF.
    // If it changed state (OFF->ON) or (ON->OFF) we log that in the flash
    // file system
    // Probably the pump will only go one once per day, so most of the time
    // not much happens and we just go back to sleep.
    //
    // However, every so often (currently 6 hr intervals) we turn on WiFi and
    // try to upload any data to a remote server.
    //
    float amps;
    
    String ttag = rtc.getTime("---Time=%Y/%m/%d,%H:%M:%S");
    print(ttag.c_str());
    
    // do stuff on wakeup here
    setupMeasurement();
    amps = makeMeasurement();
    sprintf(buf,"Amps=%f",amps);
    print(buf);
    logMeasurement(amps);

    // do hourly stuff here
    int hr = rtc.getHour();
    if (lastHr == -1)
    {
      lastHr = hr;
    }
    else if (lastHr != hr)
    {
      lastHr = hr;
      if ((hr % 6) == 0) // every 6 hours
      {
        // hourly tasks
        connectToWiFi(); // first connect this ESP32 to the WIFI network
        pushDataToServer();
        getNtpTime(); // resync time
        turnOffWiFi(); // turn off when done
      }
    }
  }

  // and go back to sleep here
  ++bootCount; // count this boot-up sequencerm
  digitalWrite(LED,LOW); // turn of comfort LED
  digitalWrite(VPPPIN,LOW); // set voltage divider output pin to 0v
  sprintf(buf,"Awake for %d ms",millis());
  print(buf);
  
  long tts = (timeToSleepMs - millis()) * ms_TO_uS_FACTOR;
  esp_sleep_enable_timer_wakeup(tts);
  esp_deep_sleep_start();

  // and, the ESP32 never executes any code past the deep_sleep_start!
}

//----------------------------------------------------------------------
void loop() 
{
  // nothing goes here since this app is using the deep-sleep mode
  // and the ESP32 will never really execute any code here.
}
