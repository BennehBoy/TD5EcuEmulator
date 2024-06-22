/*
  Software serial multple serial test
 
 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.
 
 The circuit: 
 * RX is digital pin 2 (connect to TX of other device)
 * TX is digital pin 3 (connect to RX of other device)
 
 * A4 is SDA
 * A5 is SDL
 
 created back in the mists of time
 modified 9 Apr 2012
 by Tom Igoe
 based on Mikal Hart's example
 further modified by Ben Anderson March 2017
 
 This example code is in the public domain.
 
 */
#define _DEBUG_
#define _LCD_
 
#include <SoftwareSerialEx.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h> 

#define ISORequestByteDelay 5
#define ISORequestDelay 55
#define ISOKeepAliveDelay 4000

#define ledPin     9
#define K_IN      A3
#define K_OUT     A2

#define I2C_ADDR    0x27  // Define I2C Address for controller
#define BACKLIGHT_PIN  3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7


SoftwareSerialEx obdSerial(K_IN, K_OUT); // RX, TX
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

int incomingByte = -1;   // for incoming serial data
static long initTime;
static long interbyteTime;
static unsigned long lastReceivedPidTime = 0;

static boolean initStep;

byte dataResponse[32];
byte dataIndex;
static byte seed[2];
static byte key[2];

String line1 = "                    ";
String line2 = "                    ";
String line3 = "                    ";

byte dataStreamInit[] = {0x81, 0x13, 0xF7, 0x81, 0x0C};
byte dataStreamStartDiagSession[] = {0x02, 0x10, 0xA0, 0xB2};
byte dataStreamSeedRequest[] = {0x02, 0x27, 0x01, 0x2A};
byte dataStreamKeyResponse[] = {0x04, 0x27, 0x02, 0xF8, 0x8B, 0xB0};
byte dataStreamStartFuelling[] = {0x02, 0x21, 0x20, 0x43};
byte dataStreamFuelling1[] = {0x02, 0x21, 0x09, 0x2C}; 			// RPM
byte dataStreamFuelling2[] = {0x02, 0x21, 0x1A, 0x3D}; 			// TEMP
byte dataStreamFuelling3[] = {0x02, 0x21, 0x1C, 0x3F}; 			// MAP
byte dataStreamFuelling4[] = {0x02, 0x21, 0x10, 0x33}; 			// BATT VOLTAGE
byte dataStreamFuelling5[] = {0x02, 0x21, 0x23, 0x46}; 			// AAP - AMBIENT PRESSURE
byte dataStreamFuelling6[] = {0x02, 0x21, 0x0D, 0x30}; 			// SPEED
byte dataKeepAlive[] = { 0x02, 0x3E, 0x01, 0x41 };				// KEEP ALIVE

// RESULTS OF FOLLLWING TO BE SNIFFED
byte dataStreamFuelling7[] = { 0x02, 0x21, 0x1B, 0x3E };		// THROTTLE_POS
byte dataStreamFuelling8[] = { 0x03, 0x30, 0xC0, 0xF0, 0xE3 };	// IO_CONTROL
byte dataStreamFuelling9[] = { 0x02, 0x21, 0x40, 0x63 };		// INJ_BALANCE
byte dataStreamFuelling10[] = { 0x02, 0x21, 0x21, 0x44 };		// RPM_ERROR
byte dataStreamFuelling11[] = { 0x02, 0x21, 0x37, 0x5A };		// EGR_MOD
byte dataStreamFuelling12[] = { 0x02, 0x21, 0x38, 0x5B };		// ILT_MOD
byte dataStreamFuelling14[] = { 0x02, 0x21, 0x1D, 0x40 };   // Fuelling
//byte dataStreamFuelling13[] = { 0x02, 0x21, 0x38, 0x00 };		// TWG_MOD   0x38 no good: TODO find good one...
byte faultCodes[] = { 0x02, 0x21, 0x3B, 0x5E };					// FAULT_CODES
byte clearFaultCodes[] = { 0x14, 0x31, 0xDD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22 };              // CLEAR_FAULTS

void remote_log_byte(byte b)
{
    // say what you got:
    if (b < 16) 
    {
        Serial.print('0'); 
    }        
    Serial.print(b, HEX); 
    Serial.print(' '); 
}

void iso_write_byte(byte b)
{
  obdSerial.write(b);
  delay(ISORequestByteDelay);  // ISO requires 5-20 ms delay between bytes.
}

byte iso_checksum(byte *data, byte len)
{
  byte crc=0;
  for(byte i=0; i<len; i++)
    crc=crc+data[i];
  return crc;
}

boolean iso_compare_data(byte *rcv, byte *cpm, byte len)
{
  byte matched = 0;
  
  for(byte i=0; i<len; i++)
  {
    if(rcv[i] == cpm[i] )
    {
      matched++;
    }
  }
  
  if (matched == len)
    return true;
  else
    return false;
}

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(57600);

  // Initialize LCD
  lcd.begin(20, 4);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home (); // go home

  // init pinouts
  pinMode(ledPin, OUTPUT);

#ifdef _DEBUG_
  Serial.println("Startup...");
#endif
#ifdef _LCD_
  lcd.setCursor(0, 0); lcd.print("Startup...          ");
  lcd.setCursor(0, 1); lcd.print("                    ");
  lcd.setCursor(0, 2); lcd.print("TD5 ISO14230-1      ");
  lcd.setCursor(0, 3); lcd.print("ECU Emulator        ");
  delay(500);
#endif

  // set the data rate for the SoftwareSerial port
  obdSerial.begin(10400);
  
  long initTime = millis();
  interbyteTime = 20;
  dataIndex = 0;
  initStep = true;
}

void scrollLcd(String text) {
	line1=line2;
	line2=line3;
	line3=text;

	lcd.setCursor(0,0);
	lcd.print(line1);
	
	lcd.setCursor(0,1);
	lcd.print(line2);

	lcd.setCursor(0,2);
	lcd.print(line3);
}

 
void loop() 
{
   long currentTime = millis();


  /////////////////////////////////////////////////////////
  //                   Read OBD K-Line                   //
  /////////////////////////////////////////////////////////
  if (obdSerial.available() > 0) 
  {
    // read the incoming byte:
    incomingByte = obdSerial.read();
  
    if( (initStep == false) ||
        ((incomingByte != 0x00) && (incomingByte != 0xFF)) )
    {
#ifdef _DEBUG_
      if (incomingByte < 16) 
      {
          Serial.print('0'); 
      }        
      Serial.print(incomingByte, HEX);
      Serial.print(' ');
#else
      delay(1);
#endif      

      digitalWrite(ledPin, HIGH);
      
      if(dataIndex < 32)
        dataResponse[dataIndex] = incomingByte;
      
      dataIndex++;
    }
    
    initTime = currentTime + interbyteTime;    
  }
  else
  {
    digitalWrite(ledPin, LOW);     
  }


  /////////////////////////////////////////////////////////
  //                   Catch end frame                   //
  /////////////////////////////////////////////////////////
  if ((currentTime >= initTime) && (incomingByte != -1))
  {
#ifdef _DEBUG_
    Serial.println();
#endif
    incomingByte = -1;
    initTime = currentTime + interbyteTime;
  }
  

  /////////////////////////////////////////////////////////
  //                        Init                         //
  /////////////////////////////////////////////////////////
  //if (iso_compare_data(dataResponse, dataStreamInit, 5))      
  if((initStep == true) && (dataResponse[dataIndex - 1] == 0x0C))
  {
    // Send the message
    byte dataStream[] = {0x03, 0xC1, 0x57, 0x8F, 0xAA};
    for (byte i = 0; i < 5; i++)
    {
      iso_write_byte(dataStream[i]);
    }
    
#ifdef _DEBUG_    
    Serial.print("03 C1 57 8F AA");
#endif  
#ifdef _LCD_
    scrollLcd("Init Request        ");
#endif  
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    initStep = false;
    dataIndex = 0;
  }


  /////////////////////////////////////////////////////////
  //                 Diagnostic session                  //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamStartDiagSession, 4))
  {
    // Send the message
    byte dataStream[] = {0x01, 0x50, 0x51};
    for (byte i = 0; i < 3; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
    Serial.print("01 50 51");
#endif    
#ifdef _LCD_
    scrollLcd("Start diag. request ");
#endif        
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }
    
    lastReceivedPidTime = millis();
    dataIndex = 0;
  }


  /////////////////////////////////////////////////////////
  //                      Seed request                   //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamSeedRequest, 4))  // 0x02, 0x27, 0x01, 0x2A
  {
    // Send the message
    byte dataStream[] = {0x04, 0x67, 0x01, 0x45, 0xDA, 0x8B};
    for (byte i = 0; i < 6; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
    Serial.print("04 67 01 45 DA 8B");
#endif    
#ifdef _LCD_
    scrollLcd("Seed request        ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }


  /////////////////////////////////////////////////////////
  //                       Key                           //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamKeyResponse, 6)) // 0x04, 0x27, 0x02, 0xF8, 0x8B, 0xB0
  {
    // Send the message
    byte dataStream[] = {0x02, 0x67, 0x02, 0x6B};
    for (byte i = 0; i < 4; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
    Serial.print("02 67 02 6B");
#endif    
#ifdef _LCD_
    scrollLcd("Key request         ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }
  
  /////////////////////////////////////////////////////////
  //                 Start fuelling                      //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamStartFuelling, 4))
  {
    // Send the message
    byte dataStream[] = {0x06, 0x61, 0x20, 0x9D, 0xBB, 0x0C, 0x84, 0x6F};
    for (byte i = 0; i < 8; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
    Serial.print("06 61 20 9D BB 0C 84 6F");
#endif    
#ifdef _LCD_
    scrollLcd("Start fuelling......");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }  

  /////////////////////////////////////////////////////////
  //                       Fuelling 1  RPM               //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamFuelling1, 4))
  {
    // Send the message
    byte dataStream[] = {0x04, 0x61, 0x09, 0x02, 0xED, 0x5D};
	
    for (byte i = 0; i < 6; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
      for(int i=0; i<6; i++)
        remote_log_byte(dataStream[i]);
#endif    
#ifdef _LCD_
    scrollLcd("F1 - RPM request    ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }  
  
  /////////////////////////////////////////////////////////
  //                       Fuelling 2     TEMPS          //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamFuelling2, 4))
  {
    // Send the message
    byte dataStream[] = {0x12, 0x61, 0x1A, 0x0B, 0xCD, 0x09, 0xA9, 0x0B, 0xB9, 0x0A, 0x0A, 0x0B, 0xB9, 0x0A, 0x05, 0x0B, 0xC9, 0x09, 0xC1, 0x60};
	
    for (byte i = 0; i < 20; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
      for(int i=0; i<20; i++)
        remote_log_byte(dataStream[i]);
#endif    
#ifdef _LCD_
    scrollLcd("F2 - Temps request  ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }    

  /////////////////////////////////////////////////////////
  //                       Fuelling 3 MAP                //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamFuelling3, 4))
  {
    // Send the message
    byte dataStream[] = {0x0A, 0x61, 0x1C, 0x27, 0x48, 0x27, 0x1C, 0x02, 0x61, 0x07, 0xE9, 0x8C};

    for (byte i = 0; i < 12; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
      for(int i=0; i<12; i++)
        remote_log_byte(dataStream[i]);
#endif    
#ifdef _LCD_
    scrollLcd("F3 - MAP request    ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }  

  /////////////////////////////////////////////////////////
  //                       Fuelling 4 BATT VOLTAGE       //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamFuelling4, 4))
  {
    // Send the message
    byte dataStream[] = {0x06, 0x61, 0x10, 0x36, 0x69, 0x36, 0x99, 0xE5};

    for (byte i = 0; i < 8; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
      for(int i=0; i<8; i++)
        remote_log_byte(dataStream[i]);
#endif    
#ifdef _LCD_
    scrollLcd("F4 - Bat V request  ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }  
  
  /////////////////////////////////////////////////////////
  //                       Fuelling 5   AAP                 //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamFuelling5, 4))
  {
    // Send the message
    byte dataStream[] = {0x06, 0x61, 0x23, 0x27, 0x70, 0x27, 0x45, 0x8D};

    for (byte i = 0; i < 8; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
      for(int i=0; i<8; i++)
        remote_log_byte(dataStream[i]);
#endif    
#ifdef _LCD_
    scrollLcd("F5 - AAP request    ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }  
  
  /////////////////////////////////////////////////////////
  //                       Fuelling 6    SPEED             //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamFuelling6, 4))
  {
    // Send the message
    byte dataStream[] = {0x03, 0x61, 0x0D, 0x27, 0x98};   // 39 MPH

    for (byte i = 0; i < 5; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
      for(int i=0; i<5; i++)
        remote_log_byte(dataStream[i]);
#endif    
#ifdef _LCD_
    scrollLcd("F6 - Speed request  ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }  
  
    /////////////////////////////////////////////////////////
  //                       Fuelling 14    Fuelling       //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataStreamFuelling14, 4))
  {
    // DriverDemand      = bytes [ 3]&[ 4]  (referenced from 0)
    // MafAirMass        = bytes [ 5]&[ 6]
    // MapAirMass        = bytes [ 7]&[ 8]
    // InjectionQuantity = bytes [ 9]&[10]
    // AfRatio           = bytes [11]&[12]
    // TorqueLimit       = bytes [13]&[14]
    // SmokeLimit        = bytes [15]&[16]
    // IdleDemand        = bytes [17]&[18]
    // Send the message
    //byte dataStream[] = {0x14, 0x61, 0x1D, 0xFE, 0x0C, 0x75, 0x30, 0x11, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xA0, 0x00, 0x00, 0x15, 0x7C, 0x00, 0x28, 0xF8};   // 0xF8 last byte is CRC checksum
    byte dataStream[] = {0x14, 0x61, 0x1D, 0xFE, 0x0C, 0x75, 0x30, 0x11, 0x3E, 0x50, 0x50, 0x00, 0x00, 0x0F, 0xA0, 0x00, 0x00, 0x15, 0x7C, 0x00, 0x28, 0x98};   // injection q set to 205
    for (byte i = 0; i < 22; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
      for(int i=0; i<22; i++)
        remote_log_byte(dataStream[i]);
#endif    
#ifdef _LCD_
    scrollLcd("F14 - Fuelling request  ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }  
  
  /////////////////////////////////////////////////////////
  //                        Keep Alive                   //
  /////////////////////////////////////////////////////////
  if (iso_compare_data(dataResponse, dataKeepAlive, 4))
  {
    // Send the message
    byte dataStream[] = {0x01, 0x7E, 0x7F};

    for (byte i = 0; i < 3; i++)
    {
      iso_write_byte(dataStream[i]);
    }
#ifdef _DEBUG_        
      for(int i=0; i<3; i++)
        remote_log_byte(dataStream[i]);
#endif    
#ifdef _LCD_
    scrollLcd("Keep Alive request  ");
#endif    
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }

    lastReceivedPidTime = millis();
    dataIndex = 0;
  }    


  /////////////////////////////////////////////////////////
  //                No comm: back to init                //
  /////////////////////////////////////////////////////////
  if(((millis() - lastReceivedPidTime) > 5000) && (initStep == false))
  {
    for (byte i = 0; i < dataIndex; i++)
    {
      dataResponse[i] = 0x00;
    }
   
    initStep = true;
    dataIndex = 0;
    
#ifdef _DEBUG_
  Serial.println("No communication: back to init");
#endif
#ifdef _LCD_
  scrollLcd("No comm: init.      ");
#endif
  }
}  
