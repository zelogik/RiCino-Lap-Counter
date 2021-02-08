// **********************************************************************************
//
// Receiver Code
// See https://github.com/cdemetriadis/RiCino-Lap-Counter for credit
// Ricino v3.. as almost every bit of code have changed.
//
// **********************************************************************************

#define RED_PIN 4
#define GREEN_PIN 6
#define RECV_PIN 10
#define LIGHT_PIN 11
#define BUZZER_PIN 5
#define SERIAL_PIN 7
#define CONNECTED_PIN 8
#define CONNECTED_PIN_BIS 9

#define CAR_NUMBER 32 // Number of filtered/buffered cars
#define CAR_IRCODE_START 0xAB // Put a ramdom Number between 0x00 and 0xFF - CAR_NUMBER
#define DEBOUNCE_SECOND 3 // could be millis, but add 1 Byte/car

#include <IRremote.h>
#include <NewTone.h>
#include <SoftwareSerial.h>
SoftwareSerial ESPSerial(2, 3); // RX | TX

Stream *selectedSerial = &Serial;

typedef struct{
  public:
      uint16_t irCode;

      bool getState(uint8_t debounceSecond){
          uint8_t tmpSecond = millis() / 1000;
          _isReady = (tmpSecond - _lastSecondView >= debounceSecond) ? true : false;
          _lastSecondView = tmpSecond;
          return _isReady;
      }

  private:
      uint8_t _lastSecondView = millis() / 1000; // seconds
      bool _isReady = true;
} IrCodeBuffer;

class Buzzer
{
private:
  bool _enabled = true;
  uint16_t _buzzerDelay[8] = {0};
  uint16_t _buzzerFreq[8] = {0};
  bool _tonePending  = false;
  uint8_t _pin;
  uint8_t _currentLoop = 0;
  uint8_t _currentLoopLength = 0;
  uint32_t _lastMillis = 0;

public:
  Buzzer(uint8_t pin){
      this->_pin = pin;
  }
//  ~Buzzer();

  void init(bool state){
      _enabled = state;
  }

  bool getTonePending(){
      return _tonePending;
  }

  void setTone(uint16_t frequency[], uint16_t duration[]){   
      this->_currentLoopLength = 8;
      for (int i = 0 ; i < _currentLoopLength ; i++)
      {
          this->_buzzerDelay[i] = *(duration + i);
          this->_buzzerFreq[i] = *(frequency + i);
      }
      this->_tonePending = true;
//      this->_currentLoop = 0;
  }

  void loop(){
      if (_tonePending && _enabled)
      {
          if ((millis() - _lastMillis) >= _buzzerDelay[_currentLoop])
          {
              NewTone(_pin, _buzzerFreq[_currentLoop]);
              _lastMillis = millis();
              _currentLoop++;
          }
      }

      if (_currentLoop >= _currentLoopLength)
      {
          _currentLoop = 0;
          _tonePending = false;
          noNewTone(_pin);
          digitalWrite(_pin, HIGH);
      }
  }
};

class Led // lKeep RED and GREEN led active
{
private:
    uint8_t _ledPin;
    uint32_t OnTime = 1000;     // milliseconds of on-time
    uint32_t OffTime = 1000;    // milliseconds of off-time
    bool ledState = LOW;                 // ledState used to set the LED
    uint32_t previousMillis;   // will store last time LED was updated
 
    void setOutput(bool state_, uint32_t currentMillis_){
        ledState = state_;
        previousMillis = currentMillis_;
        digitalWrite(_ledPin, state_);
    }

public:
    Led(uint8_t _ledPin)
    {
        this->_ledPin = _ledPin;
        pinMode(_ledPin, OUTPUT);
        previousMillis = 0; 
    }

    void set(uint32_t on, uint32_t off){
        OnTime = on;
        OffTime = off;
    }

    void loop(){
        uint32_t currentMillis = millis();
            
        if((ledState == HIGH) && (currentMillis - previousMillis >= OnTime))
        {
            setOutput(LOW, currentMillis);
        }
        else if ((ledState == LOW) && (currentMillis - previousMillis >= OffTime))
        {
            setOutput(HIGH, currentMillis);
        }
    }
};


IrCodeBuffer irCode[CAR_NUMBER]; // 5bytes x each "filtered/memorized" car 

Buzzer buzzer = Buzzer(BUZZER_PIN);

Led redLed = Led(RED_PIN);
Led greenLed = Led(GREEN_PIN);

void setup() {
    delay(1000); //maybe more... so millis() can take some value
    Serial.begin(9600);
//    ESPSerial.begin(9600);
    pinMode(RED_PIN,OUTPUT);
    pinMode(GREEN_PIN,OUTPUT);
    pinMode(LIGHT_PIN,OUTPUT);
    pinMode(SERIAL_PIN, INPUT);
    
    for (int i = 0 ; i < CAR_NUMBER ; i++)
    {
       irCode[i].irCode = CAR_IRCODE_START + i;
    }
    IrReceiver.begin(RECV_PIN, true);
}

void loop() {
    char message[3];

    buzzer.loop();
    redLed.loop();
    greenLed.loop();

    ricinoLoop();

    serialLoop();
}

void serialLoop(){
  const uint32_t debounceDelay = 100;
  static uint32_t lastDebounceTime = 0;

  static bool serialSelect = false; // false = USB, true = softwareSerial
  static bool serialSelectLast = serialSelect;
  
  serialSelect = digitalRead(SERIAL_PIN) ? true : false;

  if (serialSelect != serialSelectLast){
      serialSelectLast = serialSelect;
      if (serialSelect){
//          Serial.end();
          selectedSerial=&ESPSerial;
          ESPSerial.begin(9600);
      }else{
          ESPSerial.end();
          selectedSerial=&Serial;
//          Serial.begin(9600);
          selectedSerial->println("Serial");
      }
  }
}

void ricinoLoop(){
    static uint32_t runningStart = 0;
    static uint32_t connectLastMillis = 0;
    static uint8_t count = 0;

    static uint32_t standByHeartbeat = 0;
    const uint32_t standByHeartbeatDelay = 1000;

    enum RaceState {
        off,
        standby,
        race
    };
    static RaceState raceState = off;

    char message[3] = {0};
    uint8_t idx = 0;
    while (selectedSerial->available() > 0) {
       char tmp = selectedSerial->read();
       if ( tmp != '\r' || tmp != '\n'){
           message[idx] = tmp;
           idx++;
           delay(2);
       }else{
           selectedSerial->flush();
           break;
       }
    }
    
    // Check message[] integrity/checksum
    if (message[0] == '%' && message[2] == '&')
    {
        char byte_received = message[1];

//        uint16_t melody_start[] = {100, 0, 100, 0, 100, 0, 500, 650};
//        uint16_t freq_start[] = { 250, 750, 250, 750, 250, 750, 250, 250};
//        uint16_t melody_pass[] = {262, 0, 0, 0, 0, 0, 0, 0 };
//        uint16_t freq_pass[] = { 250, 0, 0, 0, 0, 0, 0, 0};
//        uint16_t melody_stop[] = { 254, 196, 24, 220, 196, 0, 247, 24 };
//        uint16_t freq_stop[] = { 250, 125, 125, 250, 250, 250, 250, 250};
        uint16_t melody_start[] = {100, 0, 0, 0, 0, 0, 0, 0};
        uint16_t freq_start[] = { 250, 0, 0, 0, 0, 0, 0, 0};
        uint16_t melody_pass[] = {500, 0, 0, 0, 0, 0, 0, 0 };
        uint16_t freq_pass[] = { 250, 0, 0, 0, 0, 0, 0, 0};
        uint16_t melody_stop[] = { 254, 0, 0, 0, 0, 0, 0, 0 };
        uint16_t freq_stop[] = { 250, 0, 0, 0, 0, 0, 0, 0};

        switch (byte_received) {
        case 'I': // init timer
            if (raceState = standby)
            {
                raceState = race;
                // lastHeartbeat = 0;
                greenLed.set(700,300);
                redLed.set(0,1000);
                buzzer.setTone(melody_start, freq_start);
                runningStart = millis();
            }
            break;

        case 'F': // End connection
            if (raceState == race)
            {
                raceState = standby;
                greenLed.set(0,1000);
                redLed.set(900,100);
                buzzer.setTone(melody_stop, freq_stop);
            }
            break;

        case 'C': // connection
            buzzer.setTone(melody_pass, freq_pass);
            greenLed.set(35,35);
            redLed.set(50,50);
            while (count <= 20)
            { // Message: Confirmation "%A&"
                if (millis() - connectLastMillis >= 50)
                {
                    selectedSerial->print("%A&");
                    selectedSerial->println();
                    connectLastMillis = millis();
                    count++;
                }
                greenLed.loop();
                redLed.loop();
                buzzer.loop();
            }
            count = 0;
            raceState = standby;
            greenLed.set(0,1000);
            redLed.set(900,100);
            break;

        default:
            break;
        }
    }

    if (raceState == race)
    {
        uint32_t tmpMillis = (millis() - runningStart);
        raceLoop(tmpMillis);
    }
    else if (raceState == standby)
    {
        if ( millis() - standByHeartbeat > standByHeartbeatDelay)
        {
            standByHeartbeat = millis();
            selectedSerial->print("%A&");
            selectedSerial->println();
        }
    }
}

uint16_t isAnIrCodeLoop(){
  if (IrReceiver.decode())
  {
      if (IrReceiver.decodedIRData.decodedRawData == NEC)
      {
          uint16_t tmp16 = IrReceiver.decodedIRData.decodedRawData;
          // uint32_t tmp32 = IrReceiver.decodedIRData.decodedRawData;
          IrReceiver.resume();
          return tmp16;
      }
  }
  return 0;
}

void raceLoop(uint32_t currentTime){
  static uint32_t lastHeartbeat;
  const uint32_t delayHeartbeat = 5 * 1000; // send message every 5sec

  const uint8_t debugId[4] = {0x13, 0x23, 0x35, 0x34};
  static uint32_t debugIdTimer[4];
  static uint32_t debugIdDelay[4] = {3000, 3200, 3205, 4000};

  uint16_t tmpIrCode = isAnIrCodeLoop();
  
  if (currentTime < 1){ // 0ms is enough to detect start ?
      lastHeartbeat = millis() - delayHeartbeat; // tricky hack, buggy if race start before the first 5sec boot
      debugIdTimer[0] = millis();
      debugIdTimer[1] = millis();
      debugIdTimer[2] = millis();
      debugIdTimer[3] = millis();
  }

  if (millis() - lastHeartbeat >= delayHeartbeat)
  {
      selectedSerial->print("%T");
      selectedSerial->print(currentTime,HEX);
      selectedSerial->print("&");
      selectedSerial->println();
      // Serial.print("\r\n");
      lastHeartbeat += delayHeartbeat; // sort of modulo
  }

  for (uint8_t i = 0; i < 4; i++){
      if (millis() - debugIdTimer[i] > debugIdDelay[i]){
          debugIdDelay[i] = random(4000,8000);
          selectedSerial->print("%L");
          selectedSerial->print(debugId[i]);
          selectedSerial->print(",");
          selectedSerial->print(currentTime, HEX);
          selectedSerial->print("&");
          selectedSerial->println();
          debugIdTimer[i] = millis();
      }
  }

  if (tmpIrCode > 0)
  {
      for (int i = 0 ; i < CAR_NUMBER ; i++)
      {
          if (irCode[i].irCode == tmpIrCode && irCode[i].getState(DEBOUNCE_SECOND))
          {
              selectedSerial->print("%L");
              selectedSerial->print(irCode[i].irCode, HEX);
              selectedSerial->print(",");
              selectedSerial->print(currentTime, HEX);
              selectedSerial->print("&");
              selectedSerial->println();
          }
      }
  }
}

//// **********************************************************************************
//        switch (results.value) {
//            case 0x287EAF36: tx=1; break; // R8 LMS 2015
//            case 0xB7CF8A27: tx=2; break; // McLaren P1 GTR
//            case 0xD097C1CC: tx=3; break; // Toyota lemans
//            case 0xDDB60357: tx=4; break; // Nissan Fairlady Official car
//            case 0x3d834070: tx=5; break; // Toyota Vitz RS 
//            case 0xC86EE88D: tx=6; break; // LaFerrari
//            case 0x584DC384: tx=7; break; // Porsche GTS
//            case 0x656C050F: tx=8; break; // Fiat 500 Abarth
//            case 0x8DA34F8A: tx=9; break; // Honda Civic Type R
//            case 0x1CF42A7B: tx=10; break; // Subaru Impreza STI 2010
//            case 0xCD3118BA: tx=11; break; // Citroen DS3 
//            case 0x5C81F3AB: tx=12; break; // Honda HSV-010 
//            case 0x754A2B50: tx=13; break; // 
//            case 0x82686CDB: tx=14; break; // 
//            case 0xE235A9F4: tx=15; break; // 
//            case 0x6D215211: tx=16; break; //
//            case 0xFD002D08: tx=17; break; // 
//            case 0xA1E6E93:  tx=18; break; // 
//            case 0x74E73AF0: tx=19; break; //
//            case 0xFFD2E30D: tx=20; break; // 
//            case 0x7646E32E: tx=21; break; // 
//            default:  
//              break;
//        }
//    }
