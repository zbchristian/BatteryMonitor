
/*
BATTERY MONITOR WITH CURRENT SPLIT CORE WITH HALL SENSOR
========================================================
Sensor model YHDC HSTS016L or a shunt resistor

ADC: ADS1115 module via I2C
- Powered by 5V
- Voltage range 4V (Hall sensor) or 256mV (shunt resistor)

- Channel A3 - GND
- Channel A2 - 12V battery voltage (divided by (10+30)/10=4.0) = approx 3V
- Channel A1 - reference voltage of the sensor (nom. 2.5V) 
- Channel A0 - Sensor reading typically 2.5V +- 0.625V or +-75mV (shunt)

Precision: noise of hall sensor allows for about 50mA precision

Values are transmitted via Low Energy Bluetooth to a smartphone app
- all values in uint16 format
- voltage (V) * 100
- current (A) * 100
- power (W) * 10
- nominal and current battery capacity (Ah) * 10
- percentage available (%) * 10

Nominal and current capacity are stored in FLASH memory every 30 seconds

Commands accepted over BLE (string)
===================================
s100 - set current capacity to 100% (full)
s000 - set current capacity to 0% (empty)
scxxxxxx - set nominal capacity to xxxxxx/10 Ah
s%xxxx - set percentage of the current capacity to xxxx/10% (e.g. s%785 -> 78.5%)
soxxxx - set the current offset in milli Amps
sixxxx - set the pre-shared pass phrase used for the hash value of the sign on message (max 9 characters)
ghxxxx - get history data (xxxx = time window in secs, if 0 : send all data) 


THE BATTERY MONITOR DEBUG INFOS
===============================
Connect the ESP32 via USB to a computer and open a terminal program and connect to 
the corresponding serial interface (e.g. COM port)

CZ July 2020/2023

*/

// set the sensor type
//#define CURRENT_SHUNT
#define CURRENT_HALL

#include <stdlib.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "esp32-hal-cpu.h"

#include "mbedtls/md.h"

/*
extern "C" {
  #include "soc/syscon_struct.h"
}
#include "soc/sens_struct.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
*/

#include<ADS1115_WE.h> 
#include<Wire.h>
#define I2C_ADDRESS 0x48

#include <Preferences.h>
Preferences persistentStore;  // provides access to the Flash memory


#ifndef CURRENT_HALL
#define CURRENT_CURRENT
#endif
#ifndef CURRENT_SHUNT
#define CURRENT_HALL
#endif

int16_t scale16(float ,float );
void clearStats(void);
void calcStats();

BLEServer *pServer=NULL;
BLEService *pService=NULL;
BLECharacteristic *pCharacteristic_rx;
BLECharacteristic *pCharacteristic_tx;

bool deviceConnected = false;

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

const ADS1115_MUX adc12V        = ADS1115_COMP_2_3; // Diff channel 2 and 3: Battery Voltage
const ADS1115_MUX adcSensor     = ADS1115_COMP_0_1; // Diff channel 0 and 1 : Sensor signal

const ADS1115_RANGE RangeV12    = ADS1115_RANGE_4096;
#ifdef CURRENT_HALL
const ADS1115_RANGE RangeSensor = ADS1115_RANGE_4096;
#else
const ADS1115_RANGE RangeSensor = ADS1115_RANGE_0256;
#endif

#define LED     LED_BUILTIN
#define BUTTON  GPIO_NUM_0   // this is the boot button - NOT AVAILABLE in ESP32 mini modules

hw_timer_t * msTimer = NULL;

DRAM_ATTR unsigned long msTime;
unsigned long deltaTime=0;
unsigned long msLast=0;

#define DefaultSignOnMsg  "123456"   // pre-shared message (max 9 characters) for which a sha256 hash is send by the client
#define maxSignonLen      10

#define currentBin        0.05       // current only displayed in units of 50mA

#define R12V              0.25      // measured - ideal 10/(10+30)=1/4

#define chargeEff		      0.95	    // charging efficiency - AGM/GEL: ~95% , standard lead acid: ~80%

// define time (ms) between the execution of certains functional codes
#define doSave            30000 // ms to save data to EPPROM 
#define doADC             300   // ms to read data from ADC
#define doLED             350   // ms to toggle the LED 
#define doBLE             2000  // ms to send data over BLE
#define doDelay           10    // ms to sleep in LOOP
#define waitSignon        2000  // ms to wait for the signon message

#define timeNoSignon 60000 // ms to wait for a connection without signon message (starts when button is pressed or after reset)

DRAM_ATTR boolean   isSave=false;
DRAM_ATTR boolean   isADC=false;
DRAM_ATTR boolean   isLED=false;
DRAM_ATTR boolean   isBLE=false;
DRAM_ATTR boolean   isAVR=false;
DRAM_ATTR boolean   isValidClient;
DRAM_ATTR int       countSignon;
DRAM_ATTR long      countNoSignon;
DRAM_ATTR boolean   isNoSignon;
DRAM_ATTR int       ncharSignon;
DRAM_ATTR boolean   isReceiveSignon;

float  fCPU;  // CPU frequency in MHz

double filtered12V = -1.0;
double filteredSensor= -1.0;
double BatCapNominal=0.0;
double BatCap  = 0.0;   
double BatIoff = 0.0;
double VBat    = 0.0;
double AmpsBat = 0.0;

const unsigned int timeHistory = 7*24*60*60*1000; // length of history in milli seconds - 7 days
const unsigned int histBin     = 6*60*1000;       // store average values for every n ms - 6 min
const unsigned int nhistory    = timeHistory/histBin; // # of values to store for the timeHistory

float *histAmps;
float *histAh;
float *histVolts;
float avrAmps = 0.0;
float avrAh   = 0.0;
float avrVolts= 0.0;
float statAh[4]   = {-10000.0,-10000.0,-10000.0,-10000.0};  // Ah store statistic for 1h, 6h, 12h and 24h
float statWh[4]   = {-10000.0,-10000.0,-10000.0,-10000.0};  // energy (Wh) store statistic for 1h, 6h, 12h and 24h
unsigned int  histPos = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb" // UART service UUID
#define CHARACTERISTIC_UUID_TX "0000ffe1-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_RX "0000ffe2-0000-1000-8000-00805f9b34fb"

char txt[100];

char *signOnText;
#define lenSignOn  100

double Vcal12(double v)         { return v-0.05; } // calibrated voltage of ADC 
double get12V(double v)         { return (Vcal12(v))/R12V; }

#ifdef CURRENT_HALL
// The Hall sensor requires calibration
// - check the reference voltage (serial debug port) at 0 Amps and correct VcalSensor 
double VcalSensor(double v)     { return v-0.0027; } // Hall - calibrated voltage of Sensor measurement
// Voltage to current might not be completely linear, but this is ignored here
double V2Amps(double dV)        { return 26.73*dV - 0.0; } // Hall sensor - calibrated voltage to current function

#else
// The Shunt sensor requires calibration
// - check the reference voltage (serial debug port) at 0 Amps and correct VcalSensor 
double VcalSensor(double v)     { return v+0.00014; } // Shunt - calibrated voltage of Sensor measurement
// Voltage to current should be linear. Check marking on the shunt, e.g. 85mV @ 100A 
double V2Amps(double dV)        { return 100/0.085*dV; } // Shunt - calibrated voltage to current function
#endif
double getCurrent(double Vdiff) { return V2Amps(VcalSensor(Vdiff)); }

#define sign(x) ((x>=0) ? 1 : -1)

// nominal and current battery capacity is retrieved/stored from/in Flash memory 
void persistentBatCapacity(boolean isGet=true) {
  double val;
  if(isGet) {
    BatCapNominal = persistentStore.getDouble("CapNominal", 100.0);
    BatCap = persistentStore.getDouble("CapCurrent", 10.0);
    BatIoff = persistentStore.getDouble("Ioffset", 0.0);
  } else {
    // limit the precision to reduce FLASH write operations
    val=BatCapNominal;
    val = (long)(val/0.01+0.5)*0.01; // 2 decimal digits of precision
    if (fabs(persistentStore.getDouble("CapNominal", 100.0)-val) > 0.005) persistentStore.putDouble("CapNominal", val);
    val=BatCap;
    val = (long)(val/0.001+0.5)*0.001;  // 3 decimal digits of precision
    if (fabs(persistentStore.getDouble("CapCurrent",10.0)-val) > 0.0005) persistentStore.putDouble("CapCurrent", val);
    val=BatIoff;
    val = (long)(val/0.001+0.5)*0.001;  // 3 decimal digits of precision
    if (fabs(persistentStore.getDouble("Ioffset",0.0)-val) > 0.0005) persistentStore.putDouble("Ioffset", val);
  }
}

mbedtls_md_context_t ctx;
mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
byte sha256Res[32];
char sha256hash[65];

// calculate the sha256 hash and return the hash as an array of 32 bytes 
void hashSHA256(char *payload, int len, byte *result) {
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
  mbedtls_md_starts(&ctx);
  mbedtls_md_update(&ctx, (const unsigned char *) payload, len);
  mbedtls_md_finish(&ctx, result);
  mbedtls_md_free(&ctx);
 }

long long timemillisecs=0;

// signon message is a sha256 hash of the predefined sign on message with the appended time in ms since 1970 of the client
// The time string is send as clear text "time hash" (salt)  
bool checkSignon() {
  int posBlank = strcspn(signOnText," ");
  if (posBlank < 0 || posBlank >= 20) return false;
  char strTime[20];
  strncpy(strTime,signOnText,posBlank);
  strTime[posBlank] = '\0';
  long long timems = atoll(strTime);
  if (timems <= timemillisecs) return false;
  char *strhash = &signOnText[posBlank+1];
  if(strlen(strhash) < 64) return false; 
  int lenSignon=persistentStore.getString("signon").length();
  int len = strlen(strTime)+lenSignon;
  if(len > 120) return false;
  char *txt2hash = (char*)malloc(len+1);
  strncpy(txt2hash,persistentStore.getString("signon").c_str(),lenSignon);
  strncpy(&txt2hash[lenSignon],strTime,strlen(strTime));
  txt2hash[len]='\0';
  byte result[32];
  hashSHA256(txt2hash,len,sha256Res);
  for(int i= 0; i<32; i++) sprintf(&sha256hash[i*2], "%02x", (int)sha256Res[i]);
  sha256hash[64]='\0';
  Serial.println();
  Serial.println(strhash);
  Serial.println(sha256hash);
  if (strcmp(strhash,sha256hash)==0) {
    timemillisecs = timems;
    return true;  
  } 
  return false;
}

const int16_t SignonOK[2]={2,0};
void receiveSignon(char *txt, int txtlen) {
  if (!isReceiveSignon || txtlen <=0) return;
  int dlen=strlen(signOnText);
  int len = min(txtlen,lenSignOn-dlen);
  strncpy(&signOnText[dlen],txt,len);
  signOnText[dlen+len]='\0';
  isReceiveSignon = strlen(signOnText)<ncharSignon;
  if(strlen(signOnText) == ncharSignon) { // we received the whole signon message
    // save the message
     persistentStore.putString("signon",signOnText);
    Serial.println();
    Serial.print("New Signon message -");
    Serial.print(signOnText);
    Serial.println("- done");
    isNoSignon=false;
    
    pCharacteristic_tx->setValue((uint8_t*)SignonOK,sizeof(SignonOK)*2); // acknowledge reception 
    pCharacteristic_tx->notify(); 
  }
}

void BLEAdvertise(bool isStart) {
  if (isStart) {
    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting for a client connection ...");
  } else {
    pServer->getAdvertising()->stop();
    Serial.println("Stopped BLE advertisment ...");
  }
  isNoSignon=false;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Client connects");
      deviceConnected = true;
      signOnText[0]='\0';
      isValidClient = isNoSignon;
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Client disconnects");
      deviceConnected = false;
      isValidClient = false;
      BLEAdvertise(true);
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rx = pCharacteristic->getValue();

      if (deviceConnected && rx.length() >=1) {
        Serial.println();
        Serial.print("Received Value: ");
        Serial.println(rx.c_str());
        if (!isValidClient) { // still collecting the signon message
          int dlen=strlen(signOnText);
          int len = min((int)rx.length(),lenSignOn-dlen);
          strncpy(&signOnText[dlen],rx.c_str(),len);
          signOnText[dlen+len]='\0';
          isValidClient = checkSignon();
        } else if (isReceiveSignon && isNoSignon) { // collecting a new signon message
          receiveSignon((char*)rx.substr(0).c_str(),(int)rx.length());
        } else {  // valid connection          
          long val;
          if(rx.compare(0,1,"s") == 0 || rx.compare(0,1,"g") == 0 ) {  // its a set/get command
            // retrieve the value
            val = atoi(rx.substr(2).c_str());
            Serial.print("Received set command ");
            Serial.print(rx.substr(0,2).c_str());
            Serial.print(" value = ");
            Serial.println(val);
            double xval = val/10.;
            if(rx.compare(0,2,"sc") == 0 && xval >= 0.0 && xval < 10000.0) {  // received set capacity command
               BatCapNominal = xval;
               persistentBatCapacity(false);            
               clearStats();              
            } else if(rx.compare(0,2,"s%") == 0 && xval >= 0.0 && xval <= 100.0) { // received set % level command
               BatCap = BatCapNominal*xval/100.0;
               persistentBatCapacity(false);
               clearStats();              
            } else if(rx.compare(0,2,"s1") == 0) { // received set to full command
               BatCap = BatCapNominal;
               persistentBatCapacity(false);
               clearStats();              
            } else if(rx.compare(0,2,"s0") == 0) { // received set to empty command
               BatCap = 0.0;
               persistentBatCapacity(false);
               clearStats();              
            } else if(rx.compare(0,2,"so") == 0 && val >= -500 && val <= 500) { // received set current offset command
               BatIoff=val/1000.;  // current offset given in mA
               persistentBatCapacity(false);
               Serial.print(" BatIoff = ");
               Serial.println(BatIoff);
            } else if(rx.compare(0,2,"gh") == 0) { // received get history command
               sendHistory();
            } else if(rx.compare(0,2,"si") == 0 ) { // received set new signon message 
               ncharSignon = atoi(rx.substr(2,1).c_str());  // 1 digit to set the length of the message
               isReceiveSignon = ncharSignon > 0 && ncharSignon < maxSignonLen;
               signOnText[0] = '\0';
               Serial.print("Store new signon message of length ");
               Serial.println(ncharSignon);
               receiveSignon((char*)rx.substr(3).c_str(), (int)rx.length()-3);
            }
          }
        }
      }
    }
};

void setup() {

  setCpuFrequencyMhz(80); //Set CPU clock to 80MHz to save power

  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  pinMode(BUTTON, INPUT);
  countNoSignon=ncharSignon;
  isNoSignon=isReceiveSignon=false; 
    
  Serial.print("CPU Freq "); Serial.println(fCPU=getCpuFrequencyMhz()); //Get CPU clock


  msTime = 0;
  signOnText= (char*)malloc(lenSignOn);
  signOnText[0] = '\0';
  persistentStore.begin("batmon",false);
  persistentBatCapacity();  // retrieve stored values of nominal and current battery capacity
  if(persistentStore.getString("signon").length() == 0) persistentStore.putString("signon",DefaultSignOnMsg);
    
  // init ADC
  Wire.begin();
  if(!adc.init()){
    Serial.println("No ADC found - ADS1115 not connected!");
  }
  adc.setVoltageRange_mV(RangeSensor);
  adc.setCompareChannels(ADS1115_COMP_0_1);
  adc.setConvRate(ADS1115_16_SPS);     // set samples per second
  adc.setMeasureMode(ADS1115_SINGLE); // ADC running in single acquisition mode

  filtered12V=filteredSensor=0.0;
  setupTimer();
  Serial.println("Timer setup done");

  Serial.print("History bins : ");
  Serial.println(nhistory);
  histAmps  = (float*) malloc(nhistory*sizeof(float));
  histAh    = (float*) malloc(nhistory*sizeof(float));
  histVolts = (float*) malloc(nhistory*sizeof(float));

  clearStats();
  
  // Create the BLE Device
  BLEDevice::init("CZ_Battery_Monitor"); // Give it a name

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic_tx = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic_tx->addDescriptor(new BLE2902());

  pCharacteristic_rx = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic_rx->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertise(true);
  countNoSignon=timeNoSignon;
  isNoSignon=true;

  Serial.println("Setup done ...");

}

void BlinkLED() {
  if( isLED ) {
    if(deviceConnected || isNoSignon) {
      if(digitalRead(LED) == HIGH) digitalWrite(LED, LOW); 
      else digitalWrite(LED, HIGH);
    } 
    else digitalWrite(LED, LOW);
    isLED = false;  
  }
}

// routine called once every 1ms 
void IRAM_ATTR SetTime() { 
  ++msTime; 
  // check time
  isSave  = isSave || (msTime%doSave) == 0;
  isADC   = isADC  || (msTime%doADC) == 0;
  if(isNoSignon) isLED   = isLED  || (msTime%(doLED*2)) == 0;
  else           isLED   = isLED  || (msTime%doLED) == 0;
  if (isLED) BlinkLED();
  isBLE   = isBLE  || (msTime%doBLE) == 0;
  isAVR   = isAVR  || (msTime%histBin) == 0;
  if (deviceConnected && !isValidClient) ++countSignon;
  else countSignon = 0;

  isNoSignon = isNoSignon && (--countNoSignon>0);
  // check button state
  if (!digitalRead(BUTTON) && !isNoSignon) { // button pressed and no device connected -> ignore signon message for some time
    countNoSignon=timeNoSignon;
    isNoSignon=true;
  }
}

void setupTimer() {
  noInterrupts();           // disable all interrupts

// timer to set the timer in milli seconds
  msTimer = timerBegin(1, fCPU, true);    // timer 1 - 1usec resolution
  timerAttachInterrupt(msTimer, &SetTime, true);  
  timerAlarmWrite(msTimer, 1000, true); // trigger once per milli second  
  timerAlarmEnable(msTimer);
  
  interrupts();             // enable all interrupts
}

// retrieve a single value from the ADC - slow with low noise
double getADC_V(ADS1115_MUX channel, ADS1115_RANGE range) {
  adc.setVoltageRange_mV(range);
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while ( adc.isBusy() );
  return adc.getResult_V();
}


#define beta 0.3 // correspond to 1/n of the width of the sliding exp. window
double getFilteredVoltage( double volti, double volts ) {
  if (volts < -10.) return volti;
  return volts - beta*(volts - volti); 
}

int nAvr=0;
double eff;

void loop() {

  if( isSave ) {
    persistentBatCapacity(false); // write current values to EEPROM
    isSave = false;  
  }

  if( isADC) {
    deltaTime = msTime - msLast;
    msLast = msTime;
    filtered12V = getFilteredVoltage( getADC_V(adc12V, RangeV12), filtered12V ); 
    filteredSensor = getFilteredVoltage( getADC_V(adcSensor, RangeSensor), filteredSensor ); 
    VBat = get12V(filtered12V);
    AmpsBat = getCurrent(filteredSensor);
    sprintf(txt,"Time diff [ms] %d, VBat [V] %10.2f, Vsens [mV] %10.3f, Current [A] %10.3f - ",deltaTime,VBat,filteredSensor*1000, AmpsBat);       
    Serial.println(txt);

    AmpsBat = getCurrent(filteredSensor);
    AmpsBat = (long)(AmpsBat/currentBin)*currentBin - BatIoff;
    eff = sign(AmpsBat)<0 ? 1.0 : chargeEff;	// take efficiency of charging (I>0) into account
    BatCap += AmpsBat*deltaTime/1000/3600.0*eff; // calculate the change of the battery capacity
    if ( BatCap <= 0.0) BatCap = 0.0;

    ++nAvr;
    avrVolts += VBat;
    avrAmps += AmpsBat;
    avrAh += BatCap;
        
    isADC = false;  
  }

  if( isAVR && nAvr > 0) {
    histVolts[histPos] = avrVolts/nAvr;
    histAmps[histPos] = avrAmps/nAvr;
    histAh[histPos++] = avrAh/nAvr;
    if (histPos >= nhistory) histPos = 0;
    sprintf(txt,"Volts %10.2f, Amps %10.2f, Ah %10.1f - ",avrVolts/nAvr, avrAmps/nAvr, avrAh/nAvr);       
    Serial.println(txt);
    avrVolts = avrAmps = avrAh = 0.0;
    nAvr = 0;
    calcStats();
    isAVR = false;
  }



  if ( isBLE && deviceConnected && isValidClient) {
    int16_t txValue[10]={0.0};

//    pService->stop();

    sprintf(txt,"%10.1f,%10.1f - ",filtered12V, filteredSensor);       
    Serial.print(txt);
    
    float PowerBat = VBat*AmpsBat; // power
    sprintf(txt,"%10.4f,%10.4f",filtered12V, filteredSensor);
    Serial.println(txt);
    memset(txValue, 0, sizeof(txValue));
    txValue[0] = 0; // mark data as current values
    txValue[1] = scale16(VBat,100); // transmit with 2 digits precision
    txValue[2] = scale16(AmpsBat,100);  // transmit with 2 digits precision
    txValue[3] = scale16(PowerBat,10); // transmit with 1 digit precision
    txValue[4] = scale16(BatCapNominal,10); 
    txValue[5] = scale16(BatCap,10);
    float bPerc = BatCap/BatCapNominal*100.0;
    if (bPerc < 0.0) bPerc = 0.0;
    if (bPerc > 100.0) bPerc = 100.0;
    txValue[6] = scale16(bPerc,10); 
    txValue[7] = scale16(BatIoff*1000,1); 
    
//    txValue[7] = scale16(filtered12V,1);
//    txValue[8] = scale16(filteredSensor,1);

    pCharacteristic_tx->setValue((uint8_t*)txValue,20); // To send 10 uint16 values
    pCharacteristic_tx->notify(); // Send the values to the app!
    memset(txValue, 0, sizeof(txValue));

    txValue[0] = 1; // mark data as statistics
    txValue[1] = scale16(statAh[0],10); // transmit with 1 digits precision
    txValue[2] = scale16(statAh[1],10); // transmit with 1 digits precision
    txValue[3] = scale16(statAh[2],10); // transmit with 1 digits precision
    txValue[4] = scale16(statAh[3],10); // transmit with 1 digits precision
    txValue[5] = scale16(statWh[0],1); // transmit with 0 digits precision
    txValue[6] = scale16(statWh[1],1); // transmit with 0 digits precision
    txValue[7] = scale16(statWh[2],1); // transmit with 0 digits precision
    txValue[8] = scale16(statWh[3],1); // transmit with 0 digits precision

    pCharacteristic_tx->setValue((uint8_t*)txValue,20); // To send 10 uint16 values
    pCharacteristic_tx->notify(); // Send the values to the app!

//    pCharacteristic_tx->setValue("Hello!"); // Sending a test message
//    pCharacteristic_tx->setValue(txt);
//    if(isSetSignonOK) {
//      pCharacteristic_tx->setValue("siOK"); // acknowledge reception 
//      pCharacteristic_tx->notify(); 
//      isSetSignonOK=false;
//    }
  }
  isBLE = false;

  if (deviceConnected && countSignon > waitSignon && !isValidClient && !isNoSignon) { // check if signon message has been received in time
    pServer->removePeerDevice(pServer->getConnId(),true);
    deviceConnected = false;
    countSignon = 0;
  }
  delay(doDelay);
}

const int n1h  =  1*60*60*1000/histBin; // # vals to integrate for 1h 
const int n6h  =  6*60*60*1000/histBin; // # vals to integrate for 6h 
const int n12h = 12*60*60*1000/histBin; // # vals to integrate for 12h 
const int n24h = 24*60*60*1000/histBin; // # vals to integrate for 24h 

void calcStats() {
// loop backwards through the history arrays (ring buffer)
  int i;
  int n=0;
  int idx = histPos-1;  // start index
  for(i=0; i<4;++i) statAh[i]=statWh[i]=0.0;  // clear statistics
  for (i=0; i<n24h; ++i) {
    if(idx < 0) idx = nhistory -1;  // switch to the end of the array
    if(histAh[idx] < -999) break;    // value is not set yet -> break out of for loop 
    if(idx == histPos) break;       // this should never happen!
    if(n < n24h) {
      statAh[3] += histAmps[idx]*histBin/1000./3600.;  // calculate Ah 
      statWh[3] += histAmps[idx]*histVolts[idx]*histBin/1000.0/3600.; // calculate Wh
//      statAh[3] += histAh[idx];  // add up Ah - incorrect
//      statWh[3] += histAh[idx]*histVolts[idx]; // calculate Wh  - incorrect
  sprintf(txt,"n %d, idx %d, %10.4f ",n,idx,statAh[3]);       
  Serial.println(txt);
    }
    if(n < n1h) { statAh[0] = statAh[3]; statWh[0] = statWh[3]; }
    if(n < n6h) { statAh[1] = statAh[3]; statWh[1] = statWh[3]; }
    if(n < n12h){ statAh[2] = statAh[3]; statWh[2] = statWh[3]; }
    ++n;
    --idx;
  }
  if(n==0) {
    for(i=0; i<4;++i) statAh[i]=statWh[i]=-10000.0;  // no data available    
  }
  sprintf(txt,"Ah  %10.2f 1h, %10.2f 6h, %10.2f 12h ",statAh[0],statAh[1],statAh[2]);       
  Serial.println(txt);
  sprintf(txt,"Wh  %10.2f 1h, %10.2f 6h, %10.2f 12h ",statWh[0],statWh[1],statWh[2]);       
  Serial.println(txt);
}

void sendHistory() {
  Serial.println("Send History");
  int16_t txValue[10]={0.0};
  int i=0;
  int n=0;
  int idx = histPos-1;  // start index
  while (i++ < nhistory && idx >= 0 && idx != histPos && histAh[idx] > -999 ) {
    sprintf(txt,"i = %d, idx = %d, histAh = %d",i, idx, histAh[idx]);
    Serial.println(txt);
    --idx;       
    ++n;  // get number of entries in history array
  }
  if (n==0) return;  // nothing to send
  sprintf(txt,"n = %d",n);
  Serial.println(txt);

  // loop backwards through the history arrays (ring buffer)
  txValue[0] = 3; // mark data as history package 
  txValue[1] = (int16_t)n*2;  // total number of values (voltage and Ah)
  txValue[2] = (int16_t)(histBin/1000*n); // total time of data in secs
  txValue[3] = (int16_t)(histBin/1000); // time width per bin in secs
  txValue[4] = (int16_t)((n*2)/5+1); // number of 20 byte packages 
  pCharacteristic_tx->setValue((uint8_t*)txValue,20); // To send 10 uint16 values
  pCharacteristic_tx->notify(); // Send the value to the app
  int iv=0;
  idx = histPos-1;  // start index
  for (i=0; i<n; ++i) {
    if(idx < 0) idx = nhistory -1;  // switch to the end of the array
    if(idx == histPos) break;       // this should never happen!
    txValue[iv++] = scale16(histAh[idx],10); // transmit with 1 digits precision
    txValue[iv++] = scale16(histVolts[idx],100); // transmit with 2 digits precision
    if(iv == 10) {
      pCharacteristic_tx->setValue((uint8_t*)txValue,20); // To send 10 uint16 values
      pCharacteristic_tx->notify(); // Send the values to the app!
      memset(txValue, 0, sizeof(txValue));
      iv=0;
    }
    --idx;
  }
  if(iv > 0) {
    pCharacteristic_tx->setValue((uint8_t*)txValue,20); // To send 10 uint16 values  
    pCharacteristic_tx->notify(); // Send the values to the app!
  }
}

int16_t scale16(float val,float fac) {
#define vmax16   32767.0
#define vmin16  -32768.0
  val = (val*fac)+sign(val)*0.5;  // scale and round 
  if ( val < vmin16) val = vmin16;
  if ( val > vmax16) val = vmax16;
  return (int16_t)val;
}

void clearStats() {
  for (int i=0; i<nhistory;++i) {
    histAmps[i] = -1000.0;
    histAh[i] = -1000.0;
    histVolts[i] = -1000.0;    
  }
}
