//Include Require Libraries
#include <ShiftRegister74HC595.h>
#include <CmdMessenger.h>
#include <ErriezRotaryFullStep.h>

//Initialize Rotary Encoder Functions
void rotaryInterruptBig();
void rotaryInterruptSmall();

CmdMessenger messenger(Serial);

//Define the channels that SPAD will use
enum
{
  kRequest = 0, // Request from SPAD.neXt
  kCommand = 1, // Command to SPAD.neXt
  kEvent = 2, // Events from SPAD.neXt
  kDebug = 3,
  kSimCommand = 4, // Send Event to Simulation
  kNAV1A = 10,
  kNAV1S = 11
};

//Show board is not ready and has not been addressed by SPAD yet
bool isReady = false;

//Initialize Shift Registers
ShiftRegister74HC595<2> NAV1SSR(22, 24, 26);
ShiftRegister74HC595<2> NAV1ASR(28, 30, 32);

//Initialize Rotary Encoder Pins
#define ROTARY_PIN1B   21
#define ROTARY_PIN2B   20
#define ROTARY_PIN1S   3
#define ROTARY_PIN2S   2

//Initialize Rotary Encoder String
RotaryFullStep rotaryB(ROTARY_PIN1B, ROTARY_PIN2B, true, 75);
RotaryFullStep rotaryS(ROTARY_PIN1S, ROTARY_PIN2S, true, 75);

//Define Digit place arrays
byte DigitNAV1S[5];
byte DigitNAV1A[5];

int IntsNAV1S[5];
int IntsNAV1A[5];

int wholeNAV1S;
int wholeNAV1A;

//Define bit assignments to nubmer slots
byte Numbers[10] = {B11000000, B11111001, B10100100, B10110000, B10011001, B10010010, B10000010, B11111000, B10000000, B10010000};
byte NumbersD[10] = {B01000000, B01111001, B00100100, B00110000, B00011001, B00010010, B00000010, B01111000, B00000000, B00010000};

void setup() {
  // 115200 is typically the maximum speed for serial over USB
  Serial.begin(115200);
  //Attack Interrupts for Rotary Encoder Functions
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN1B), rotaryInterruptBig, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN2B), rotaryInterruptBig, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN1S), rotaryInterruptSmall, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_PIN2S), rotaryInterruptSmall, CHANGE);
  //Make display say "rEAdY" after Arduino has booted
  DigitNAV1S[0] = B10101111;
  DigitNAV1S[1] = B10000110;
  DigitNAV1S[2] = B00001000;
  DigitNAV1S[3] = B10100001;
  DigitNAV1S[4] = B10010001;
  DigitNAV1A[0] = B10101111;
  DigitNAV1A[1] = B10000110;
  DigitNAV1A[2] = B00001000;
  DigitNAV1A[3] = B10100001;
  DigitNAV1A[4] = B10010001;

  //Start function to run config and assign callbacks
  attachCommandCallbacks();
}

void loop() {
  //Receive any serial data from SPAD
  messenger.feedinSerialData();

  //Write Byte Data to Shift Registers
  uint8_t pinValuesANAV1S[] = {DigitNAV1S[4], B00010000 };
  NAV1SSR.setAll(pinValuesANAV1S);
  uint8_t pinValuesANAV1A[] = {DigitNAV1A[4], B00010000 };
  NAV1ASR.setAll(pinValuesANAV1A);
  uint8_t pinValuesBNAV1S[] = {DigitNAV1S[3], B00001000 };
  NAV1SSR.setAll(pinValuesBNAV1S);
  uint8_t pinValuesBNAV1A[] = {DigitNAV1A[3], B00001000 };
  NAV1ASR.setAll(pinValuesBNAV1A);
  uint8_t pinValuesCNAV1S[] = {DigitNAV1S[2], B00000100 };
  NAV1SSR.setAll(pinValuesCNAV1S);
  uint8_t pinValuesCNAV1A[] = {DigitNAV1A[2], B00000100 };
  NAV1ASR.setAll(pinValuesCNAV1A);
  uint8_t pinValuesDNAV1S[] = {DigitNAV1S[1], B00000010 };
  NAV1SSR.setAll(pinValuesDNAV1S);
  uint8_t pinValuesDNAV1A[] = {DigitNAV1A[1], B00000010 };
  NAV1ASR.setAll(pinValuesDNAV1A);
  uint8_t pinValuesENAV1S[] = {DigitNAV1S[0], B00000001 };
  NAV1SSR.setAll(pinValuesENAV1S);
  uint8_t pinValuesENAV1A[] = {DigitNAV1A[0], B00000001 };
  NAV1ASR.setAll(pinValuesENAV1A);
}

void onUnknownCommand()
{
  //If the board sees a command that it does not know, return "UNKNOWN COMMAND"
  messenger.sendCmd(kDebug, "UNKNOWN COMMAND");
}

void attachCommandCallbacks() {
  // Attach callbacks
  messenger.attach(kRequest  , onIdentifyRequest);
  messenger.attach(kNAV1S , NAV1SUpdate);
  messenger.attach(kNAV1A , NAV1AUpdate);
  messenger.attach(onUnknownCommand);
}

void onIdentifyRequest()
{
  //Read requests from SPAD for INIT
  char *szRequest = messenger.readStringArg();

  //When INIT is requested, send proper formatted data back to SPAD
  if (strcmp(szRequest, "INIT") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("SPAD");
    messenger.sendCmdArg(F("{dd717c5d-eb5c-4e5f-9512-a8b4a77ebe54}"));
    messenger.sendCmdArg("Quarks Nav Com Panel");
    messenger.sendCmdEnd();
    return;
  }
  //Setup Ping request return
  if (strcmp(szRequest, "PING") == 0) {
    messenger.sendCmdStart(kRequest);
    messenger.sendCmdArg("PONG");
    messenger.sendCmdArg(messenger.readInt32Arg());
    messenger.sendCmdEnd();
    return;
  }
  //Subscribe to data requests from SPAD
  if (strcmp(szRequest, "CONFIG") == 0) {

    // Request NAV1 Standby Frequency Updates
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kNAV1S);
    messenger.sendCmdArg("SIMCONNECT:NAV STANDBY FREQUENCY:1");
    messenger.sendCmdEnd();

    // Request NAV1 Active Frequency Updates
    messenger.sendCmdStart(kCommand);
    messenger.sendCmdArg("SUBSCRIBE");
    messenger.sendCmdArg(kNAV1A);
    messenger.sendCmdArg("SIMCONNECT:NAV Active FREQUENCY:1");
    messenger.sendCmdEnd();

    // tell SPAD.neXT we are done with config
    messenger.sendCmd(kRequest, "CONFIG");
    messenger.sendCmd(kDebug, "NAV1 ACTIVE FREQ SUBSCRIBED");
    messenger.sendCmd(kDebug, "NAV1 STANDBY FREQ SUBSCRIBED");
    isReady = true;
    return;
  }
}

void NAV1SUpdate() {
  //Define float NAV1A in this scope and read serial data from SPAD
  float NAV1S = messenger.readFloatArg();
  //Send data back to SPAD through debug console
  messenger.sendCmd(kDebug, NAV1S);
  //Multiply NAV1A by 100 to make number a whole number and save as an int
  wholeNAV1S = NAV1S * 100;
  //Extrapolate number and assign each digit in number to slot in array
  IntsNAV1S[4] = (wholeNAV1S / 1U) % 10;
  IntsNAV1S[3] = (wholeNAV1S / 10U) % 10;
  IntsNAV1S[2] = (wholeNAV1S / 100U) % 10;
  IntsNAV1S[1] = (wholeNAV1S / 1000U) % 10;
  IntsNAV1S[0] = (wholeNAV1S / 10000U) % 10;
  //Check which numbers the first two digits are and assign the proper bit code
  for (int x = 0; x < 2; x++) {
    for (int i = 0; i < 10; i++) {
      if (IntsNAV1S[x] == i) {
        DigitNAV1S[x] = Numbers[i];
      }
    }
  }
  //Check third number and assign the proper bit code with deciaml point
  for (int i = 0; i < 10; i++) {
    if (IntsNAV1S[2] == i) {
      DigitNAV1S[2] = NumbersD[i];
    }
  }
  //Check which number each digit is and assign the proper bit code
  for (int x = 3; x < 5; x++) {
    for (int i = 0; i < 10; i++) {
      if (IntsNAV1S[x] == i) {
        DigitNAV1S[x] = Numbers[i];
      }
    }
  }
}

void NAV1AUpdate() {
  //Define float NAV1A in this scope and read serial data from SPAD
  float NAV1A = messenger.readFloatArg();
  //Send data back to SPAD through debug console
  messenger.sendCmd(kDebug, NAV1A);
  //Multiply NAV1A by 100 to make number a whole number and save as an int
  wholeNAV1A = NAV1A * 100;
  //Extrapolate number and assign each digit in number to slot in array
  IntsNAV1A[4] = (wholeNAV1A / 1U) % 10;
  IntsNAV1A[3] = (wholeNAV1A / 10U) % 10;
  IntsNAV1A[2] = (wholeNAV1A / 100U) % 10;
  IntsNAV1A[1] = (wholeNAV1A / 1000U) % 10;
  IntsNAV1A[0] = (wholeNAV1A / 10000U) % 10;
  //Check which number each digit is and assign the proper bit code
  for (int x = 0; x < 2; x++) {
    for (int i = 0; i < 10; i++) {
      if (IntsNAV1A[x] == i) {
        DigitNAV1A[x] = Numbers[i];
      }
    }
  }
  //Check third number and assign the proper bit code with deciaml point
  for (int i = 0; i < 10; i++) {
    if (IntsNAV1A[2] == i) {
      DigitNAV1A[2] = NumbersD[i];
    }
  }
  //Check which number each digit is and assign the proper bit code
  for (int x = 3; x < 5; x++) {
    for (int i = 0; i < 10; i++) {
      if (IntsNAV1A[x] == i) {
        DigitNAV1A[x] = Numbers[i];
      }
    }
  }
}
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
ICACHE_RAM_ATTR
#endif
void rotaryInterruptBig()
{
  int rotaryState;

  // Read rotary state
  rotaryState = rotaryB.read();
  //If rotary state is less than zero (turned to the right) send "MSFS:AS1000_PFD_NAV_LARGE_INC" to SPAD
  if (rotaryState < 0) {
    messenger.sendCmd(kSimCommand, "MSFS:AS1000_PFD_NAV_LARGE_INC");
    messenger.sendCmd(kDebug, "INCREASE NAV1 WHOLE");
  }
  //If rotary state is greater than zero (turned to the left) send "MSFS:AS1000_PFD_NAV_LARGE_DEC" to SPAD
  if (rotaryState > 0) {
    messenger.sendCmd(kSimCommand, "MSFS:AS1000_PFD_NAV_LARGE_DEC");
    messenger.sendCmd(kDebug, "DECREASE NAV1 WHOLE");
  }
}
void rotaryInterruptSmall()
{
  int rotaryState;

  // Read rotary state
  rotaryState = rotaryS.read();

  //If rotary state is less than zero (turned to the right) send "MSFS:AS1000_PFD_NAV_SMALL_INC" to SPAD
  if (rotaryState < 0) {
    messenger.sendCmd(kSimCommand, "MSFS:AS1000_PFD_NAV_SMALL_INC");
    messenger.sendCmd(kDebug, "INCREASE NAV1 DECIMAL");
  }
  //If rotary state is greater than zero (turned to the left) send "MSFS:AS1000_PFD_NAV_SMALL_DEC" to SPAD
  if (rotaryState > 0) {
    messenger.sendCmd(kSimCommand, "MSFS:AS1000_PFD_NAV_SMALL_DEC");
    messenger.sendCmd(kDebug, "DECREASE NAV1 DECIMAL");
  }
}
