/***************************************************************************
*	ArduMIMA
*	This circuit controls a 1st Generation Honda Insight's
*	hybrid system manually.
*
*	The circuit:
*	* Inputs:
*	* Outputs:
*
*	Created: 24 SEP 2012
*	By: Nick Roth
*	
*	
*
*	www.twitter.com/rothnic
*
****************************************************************************/

#include <SoftwareSerial.h>
#include <PWM.h>

/***************************************************************************
*                            Variables/Constants
****************************************************************************/
//Constants
const int ERROR_WINDOW = 50;                      //Defines the error window for a button press
const int BUTTONDEBOUNCE = 3;                     //Defines the number of button samples required before reported
const int DEBUG = 0;                              //Defines whether to output some PWM debugging information
const int32_t maMode1Freq = 20000;                //Defines the MaMode1 Frequency setting
const int32_t cmdPwrFreq = 2000;                  //Defines the CmdPwr Frequency setting
const long BUTTONDELAY = 20;                      //Defines the time to wait before resampling the button press

//Pins
const int cmdPwrPin = 3;                          //2kHz PWM Output Pin
const int bluetoothTx = 5;                        //Bluetooth Transmit Pin
const int bluetoothRx = 6;                        //Bluetooth Receive Pin
const int assistLED = 7;                          //Assist LED Pin (RED)
const int regenLED = 8;                           //Regen LED Pin (Green)
const int maMode1Pin = 10;                        //20kHz PWM Output Pin
const int maMode2Pin = 12;                        //MaMode2 Output Pin (High/Low)
const int enableControlPin = 13;                  //Controls MAX4619 Output, High enables custom PWM signals to pass through
const int buttonPin = 7;                          //Analog Pin 7, Button input. 

//Initial Digital Pin States
int lastEnableControlState = LOW;
int enableControlState = LOW;
int maMode2State = LOW;
int assistLEDState = LOW;
int regenLEDState = LOW;

//Initialize Variables
int buttonValue = 0;                              //Button Number
int buttonVal = 0;                                //Analog Value Read
long buttonLastChecked = 0;
long lastRead = 0;
int buttonCandidate = 0;
int buttonCandidateCount = 0;
int lastButtonCandidate = 0;
int maMode1Duty = 127;
int cmdPwrDuty = 127;
int lastMaMode2State = 0;
int lastMaMode1Duty = 0;
int lastCmdPwrDuty = 0;

//Initialize Software Serial Pins
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);



/***************************************************************************
*                                    Setup
*
* Ran first, before moving onto the Main Loop. Used to initialize pins,
*   serial outputs, and pwm timers.
*
****************************************************************************/
void setup()
{
  //Initialize Serial Connections
  Serial.begin(115200);    //To PC
  bluetooth.begin(9600);   //Bluetooth
  
  //Setup pins
  pinMode(assistLED, OUTPUT);
  pinMode(regenLED, OUTPUT);
  pinMode(enableControlPin, OUTPUT);
  pinMode(cmdPwrPin, OUTPUT);
  pinMode(maMode1Pin, OUTPUT);
  pinMode(maMode2Pin, OUTPUT);
  
  //Initialize IMA Control Outputs
  initializeIMA();
  
  //Setup Button Pin
  digitalWrite((14+buttonPin), HIGH);
  
  //Initialize ouputs
  digitalWrite(maMode2Pin, maMode2State);
  digitalWrite(enableControlPin, enableControlState);
  
  if (DEBUG)
    debug();
}



/***************************************************************************
*                                  Main Loop
*
* Checks inputs, updates PWM signals, then sends serial data.
*
****************************************************************************/
void loop()
{
  
  //Check Buttons
  buttonValue = buttonPushed();
  if (buttonValue > 0) {
    Serial.println(buttonValue);
  }
  
  //Update PWM states
  updatePWM();
  
  //Update LED states
  updateLEDs();
   
  //Write data to output pins (BT, USB or LCD)
  outputData();
}



/***************************************************************************
*                             Update PWM Function
*
* Stores previous PWM states, checks inputs, adjusts PWM states, applies
*   new PWM states
*
****************************************************************************/
void updatePWM()
{
  //Store last state and duty values
  lastMaMode2State = maMode2State;
  lastMaMode1Duty = maMode1Duty;
  lastCmdPwrDuty = cmdPwrDuty;
  lastEnableControlState = enableControlState;
  
  //Brake, Clutch, and Throttle values
  
  //Update CmdPwr Duty Cycle based on button press
  switch (buttonValue) {
    //Center
    case 1:
      cmdPwrDuty = 127;
      break;
      
    //Up
    case 2:
      cmdPwrDuty++;
      if (cmdPwrDuty > 240) {
        cmdPwrDuty = 240;
      }
      break;
      
    //Right
    case 3:
      cmdPwrDuty = cmdPwrDuty + 10;
      if (cmdPwrDuty > 240) {
        cmdPwrDuty = 240;
      }
      break;
      
    //Down
    case 4:
      cmdPwrDuty--;
      if (cmdPwrDuty <= 5) {
        cmdPwrDuty = 5;
      }
      break;
      
    //Left
    case 5:
      cmdPwrDuty = cmdPwrDuty - 10;
      if (cmdPwrDuty <= 5) {
        cmdPwrDuty = 5;
      }
      break;
  }
  
  //Update MaMode1, MaMode2, and enableControl based on CmdPwr  
  if (cmdPwrDuty > 127) {
    maMode1Duty = 64;
    maMode2State = LOW;
    enableControlState = HIGH;
  }
  else if (cmdPwrDuty < 127) {
    maMode1Duty = 89;
    maMode2State = HIGH;
    enableControlState=HIGH;
  }
  else if (cmdPwrDuty == 127) {
    maMode1Duty = 127;
    maMode2State = HIGH;
    enableControlState = LOW;
  }
  
  //Write states if they have changed
  if (maMode1Duty != lastMaMode1Duty) {  
    pwmWrite(maMode1Pin, maMode1Duty);
  }
  if (cmdPwrDuty != lastCmdPwrDuty) {  
    pwmWrite(cmdPwrPin, cmdPwrDuty);
  }
  if (maMode2State != lastMaMode2State) {
    digitalWrite(maMode2Pin, maMode2State);
  }
  if (enableControlState != lastEnableControlState) {
    digitalWrite(enableControlPin, enableControlState);
  }
  
}



/***************************************************************************
*                             Initialize PWM
*
* Uses PWM library to initialize only Timer 1 and 2, Timer 0 used for time
*   related functions. Each PWM signal adjusted to 50% duty cycle.
*
****************************************************************************/
void initializeIMA()
{
  //Initialize all timers except for 0, to save time keeping functions
  InitTimersSafe();

  //sets the frequency for the specified pin
  bool successMaMode1 = SetPinFrequency(maMode1Pin, maMode1Freq);
  if (successMaMode1) {
    pwmWrite(maMode1Pin, 127);
  }
  
  bool successCmdPwr = SetPinFrequency(cmdPwrPin, cmdPwrFreq);
  if (successCmdPwr) {
    pwmWrite(cmdPwrPin, 127);
  }
}
  


/***************************************************************************
*                     Button Interpreter and Hysteresis
*
* Converts analog voltage level to button number. Returns a button value
*   only after a sequence of three samples of the same button press.
*
* 1=Center, 2=Up, 3=Right, 4=Down, 5=Left, 0=No Press
****************************************************************************/
int buttonPushed()
{
  //Exit if BUTTONDELAY time value hasn't passed yet
  if((millis() - lastRead) < BUTTONDELAY) {
    return 0;
  }
  else {
    lastRead = millis();
  }
  
  //Read analog value
  buttonVal = analogRead(buttonPin);
  lastButtonCandidate = buttonCandidate;
 
  //Convert analog value to button number
  if (buttonVal >= (835 - ERROR_WINDOW) && buttonVal <= (835 + ERROR_WINDOW)) {
    buttonCandidate = 1;
  } 
  else if(buttonVal >= (658 - ERROR_WINDOW) && buttonVal <= (658 + ERROR_WINDOW)) {
    buttonCandidate = 2;
  }
  else if (buttonVal >= (475 - ERROR_WINDOW) && buttonVal <= (475 + ERROR_WINDOW)) {
    buttonCandidate = 3;
  }
  else if (buttonVal >= (263 - ERROR_WINDOW) && buttonVal <= (263 + ERROR_WINDOW)) {
    buttonCandidate = 4;
  } 
  else if (buttonVal <= (0 + ERROR_WINDOW)) {
    buttonCandidate = 5;
  }
  else {
    buttonCandidate = 0;
  }
  
  // Hysterisis Logic 
  //Ignore if no button is pressed
  if (buttonCandidate == 0) {
    buttonCandidateCount = 0;
    return 0;
  }
  //First time seeing press, increase counter
  else if (buttonCandidateCount == 0) {
    buttonCandidateCount = buttonCandidateCount + 1;
  }
  
  //Seen this button recently, increase counter
  if (buttonCandidate == lastButtonCandidate) {
    buttonCandidateCount = buttonCandidateCount + 1;
    //Seen this button enough times in a row, return button value
    if (buttonCandidateCount == BUTTONDEBOUNCE) {
      return buttonCandidate;
    }
    else {
      //Need more returns on counter, return
      return 0;
    }
  }
  else { 
    //No match on last button value, reset counter
    buttonCandidateCount = 0;
    return 0;
  }
}



/***************************************************************************
*                            Output Data
*
* Writes important system data to Arduino outputs
****************************************************************************/
void outputData()
{
  //Read from bluetooth and write to usb serial
  if(bluetooth.available()) {
    char toSend = (char)bluetooth.read();
    Serial.print(toSend);
  }

  //Read from usb serial to bluetooth
  if(Serial.available()) {
    char toSend = (char)Serial.read();
    bluetooth.print(toSend);
  }
}



/***************************************************************************
*                            Update LEDs
****************************************************************************/
void updateLEDs()
{
  //Check pin states so we can avoid overwriting the same state
  assistLEDState = digitalRead(assistLED);
  regenLEDState = digitalRead(regenLED);
  
  if ((cmdPwrDuty == 127) && ((assistLEDState != LOW) || (regenLEDState != LOW)))
  {
    digitalWrite(assistLED, LOW);
    digitalWrite(regenLED, LOW);
  }
  else if ((cmdPwrDuty < 127) && ((assistLEDState != LOW) || (regenLEDState != HIGH)))
  {
    digitalWrite(regenLED, HIGH);
    digitalWrite(assistLED, LOW);
  }
  else if ((cmdPwrDuty > 127) && ((assistLEDState != HIGH) || (regenLEDState != LOW)))
  {
    digitalWrite(regenLED, LOW);
    digitalWrite(assistLED, HIGH);
  }
  
}


/***************************************************************************
*                            Debug Information
****************************************************************************/
void debug()
{
    //Timer 1 Debug
    Serial.println("Success: MaMode1 at 20kHz");
    Serial.print("Timer1 has frequency of: ");
    Serial.println(Timer1_GetFrequency());
    Serial.print("Timer1 has possible duties: ");
    Serial.println(Timer1_GetTop()+1);
    Serial.print("MaMode1 has resolution of: ");
    Serial.println(GetPinResolution(maMode1Pin));  
    
    //Timer 2 Debug
    Serial.println("Success: CmdPwr at 2kHz");
    Serial.print("Timer2 has frequency of: ");
    Serial.println(Timer2_GetFrequency());
    Serial.print("Timer2 has possible duties: ");
    Serial.println(Timer2_GetTop()+1);
    Serial.print("CmdPwr has resolution of: ");
    Serial.println(GetPinResolution(cmdPwrPin));
}

