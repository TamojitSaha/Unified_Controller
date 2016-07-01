/*
  Firmware Source Code for Unified Controller v1
  Features
    Cabinet Fan RPM Counter for arduino without using Interrupt Service Routine (ISR)
    Real Time temperature display (LM35 Sensor)
    Temperature relative close loop control for Fans (PWM Control)
    RGB Strip Control wsing Single Potentiometer
    Autometic Color Spectrum display
    Single Key anytime mode switch

  Authors:
    Sandeepan Sengupta(Fan Control)
    Tamojit Saha(RGB Control)
  
  Firmware Revision v0.1 (Pre-Alpha 1)
  Publication Date: 28th June, 2016
  West Bengal, India
*/

//Comment out the followings to disable fan features
#define FAN_1_cfg
#define FAN_2_cfg
#define FAN_3_cfg

//Comment out the following to disable debug
#define display_debug
#define targetRPM_debug

//Comment out the followings to disable propagation delay in code
#define display_delay
#define button_delay
#define pulseIn_delay

//Assigning tachometer/fan monitor pins
int FAN_1 = 8;
int FAN_2 = 16; //Digital pin D16 mapped on Analog pin A2
int FAN_3 = 17; //Digital pin D17 mapped on Analog pin A3

//Assigning PWM Pins
int PWM_1 = 3;
int PWM_2 = 5;
int PWM_3 = 6;

//Assigning RGB Pins
int pinRED    = 9;
int pinGREEN  = 10;
int pinBLUE   = 11;

//POT configuration
int pot_pin             = 1;
int pot_referance_HIGH  = 6;
int pot_referance_LOW   = 7;
int pot_MAX             = 767;
int pot_min             = 0;

//Button Configuration
int   button            = 2;    //Interrupt(INT0) Capable, Unused Feature
bool  buttonTrigger     = LOW;  //Initializing button Trigger

//Preset RGBmode to manual
int RGBmode = 1;

//Indicator LED
int LED = 7;

//Attatch LM35 temperature Sensor to Analog pin
int tmp_PIN = 0;

//Temperature related configuration
double  tmp_MAX              = 102.4; //Maximum expected temperature (equivalant to 50°C)
double  tmp_min              = 51.2;  //Minimum expected temperature (equivalant to 25°C)
double  tmp_tolerance        = 0;     //Temperature Tolerance in percentage (eg. 12 for 12% tolerance)
int     initial_duty_cycle   = 128;   //Start FAN at 50% Duty Cycle
int     final_duty_cycle     = 255;   //Maximum at 100% Duty Cycle

int     pulse_count_1        = 2;     //Pulse per revolution value for sample fan
int     pulse_count_2        = 2;     //Pulse per revolution value for sample fan
int     pulse_count_3        = 2;     //Pulse per revolution value for sample fan

double  FAN_1_RPM;
double  FAN_1_RPM_MAX        = 1200;  // Manufacturer specified upper limit of RPM, may differ for the test sample
double  FAN_1_RPM_min        = 300;   // Manufacturer specified lower limit of RPM, may differ for the test sample
double  FAN_1_RPM_Tolerance  = 10;    // User or Manufacturer assigned tolerance range in percentage (eg. 12 for 12% tolerance)

double  FAN_2_RPM;
double  FAN_2_RPM_MAX        = 1200;  // Manufacturer specified upper limit of RPM, may differ for the test sample
double  FAN_2_RPM_min        = 300;   // Manufacturer specified lower limit of RPM, may differ for the test sample
double  FAN_2_RPM_Tolerance  = 10;    // User or Manufacturer assigned tolerance range in percentage (eg. 12 for 12% tolerance)

double  FAN_3_RPM;
double  FAN_3_RPM_MAX        = 1200;  // Manufacturer specified upper limit of RPM, may differ for the test sample
double  FAN_3_RPM_min        = 300;   // Manufacturer specified lower limit of RPM, may differ for the test sample
double  FAN_3_RPM_Tolerance  = 10;    // User or Manufacturer assigned tolerance range in percentage (eg. 12 for 12% tolerance)

void setup()
{
Serial.begin(9600);
 {
  while(!Serial);
 }

//Declearing button as input
pinMode(button, INPUT);

//Declearing LED as output
pinMode(LED, OUTPUT);

//Declearing tachometer inputs
pinMode(FAN_1, INPUT);
pinMode(FAN_2, INPUT);
pinMode(FAN_3, INPUT);

//Declearing PWM outputs
pinMode(PWM_1, OUTPUT);
pinMode(PWM_2, OUTPUT);
pinMode(PWM_3, OUTPUT);

//Internal pull-ups
digitalWrite(FAN_1,HIGH);
digitalWrite(FAN_2,HIGH);
digitalWrite(FAN_3,HIGH);
} 

void loop()
{
  int PWM_value = Temperature_to_PWM(tmp_PIN, tmp_MAX, tmp_min, tmp_tolerance);
  
  #ifdef display_debug
  Serial.print("\nPWM Code\t\t: ");
  Serial.print(PWM_value);
  Serial.print("\nDuty Cycle\t\t: ");
  Serial.print(PWM_value*100/255);
  Serial.print(" percent");
  
  #ifdef targetRPM_debug
  Serial.print("\nTarget RPM\t\t: ");
  Serial.print(FAN_1_RPM_MAX*PWM_value/255);
  #endif
  
  Serial.println("\n\n");
  #endif

  #ifdef FAN_1_cfg
  Serial.println("Results for FAN 1");
  FAN_1_RPM = RPM_Counter(FAN_1, pulse_count_1, FAN_1_RPM_MAX, FAN_1_RPM_min, FAN_1_RPM_Tolerance);
  #endif

  #ifdef FAN_2_cfg
  Serial.println("Results for FAN 2");
  FAN_2_RPM = RPM_Counter(FAN_2, pulse_count_2, FAN_2_RPM_MAX, FAN_2_RPM_min, FAN_2_RPM_Tolerance);
  #endif

  #ifdef FAN_3_cfg
  Serial.println("Results for FAN 3");
  FAN_3_RPM = RPM_Counter(FAN_3, pulse_count_3, FAN_3_RPM_MAX, FAN_3_RPM_min, FAN_3_RPM_Tolerance);
  #endif

  #ifdef display_delay
  delay(1000);
  #endif

  monitorButton(button, buttonTrigger);
  if(RGBmode == 1)
  {
    //Manual RGB Color Selection Mode
    int potValue = potRead(pot_pin, pot_referance_HIGH, pot_referance_LOW, pot_MAX, pot_min);
    {
      if(pot_MAX > pot_min)
      {
        if(potValue >= pot_min && potValue < pot_MAX/3) //0~33.333% of range
        {
          potValue = map(potValue, pot_min, pot_MAX/3, 0 ,255);
        
          //RED to GREEN Transition
          analogWrite(pinRED, 255 - potValue);
          analogWrite(pinGREEN, potValue);
          analogWrite(pinBLUE, 0);
        }
        else
        if(potValue >= pot_MAX/3 && potValue <= pot_MAX*2/3) //33.333%~66.666% of range
        {
          potValue = map(potValue, pot_MAX/3, pot_MAX*2/3, 0 ,255);
        
          //GREEN to BLUE Transition
          analogWrite(pinRED, 0);
          analogWrite(pinGREEN, 255 - potValue);
          analogWrite(pinBLUE, potValue);
        }
        else
        if(potValue > pot_MAX*2/3 && potValue <= pot_MAX) //66.666~100% of range
        {
          potValue = map(potValue, pot_MAX*2/3, pot_MAX, 0 ,255);
        
          //BLUE to RED Transition
          analogWrite(pinRED, potValue);
          analogWrite(pinGREEN, 0);
          analogWrite(pinBLUE, 255 - potValue);
        }
      }
      else
      if(pot_min > pot_MAX)
      {
        //Value swap for minimum and maximum
        pot_MAX = pot_min + pot_MAX - (pot_min = pot_MAX);
      
        if(potValue >= pot_min && potValue < pot_MAX/3) //0~33.333% of range
        {
          //RED to GREEN Transition
          potValue = map(potValue, pot_min, pot_MAX/3, 0 ,255);
          analogWrite(pinRED, 255 - potValue);
          analogWrite(pinGREEN, potValue);
          analogWrite(pinBLUE, 0);
        }
        else
        if(potValue >= pot_MAX/3 && potValue <= pot_MAX*2/3) //33.333%~66.666% of range
        {
          //GREEN to BLUE Transition
          potValue = map(potValue, pot_MAX/3, pot_MAX*2/3, 0 ,255);
          analogWrite(pinRED, 0);
          analogWrite(pinGREEN, 255 - potValue);
          analogWrite(pinBLUE, potValue);
        }
        else
        if(potValue > pot_MAX*2/3 && potValue <= pot_MAX) //66.666~100% of range
        {
          //BLUE to RED Transition
          potValue = map(potValue, pot_MAX*2/3, pot_MAX, 0 ,255);
          analogWrite(pinRED, potValue);
          analogWrite(pinGREEN, 0);
          analogWrite(pinBLUE, 255 - potValue);
        }
      }
    }
  }
  else
  if(RGBmode = 0)
  {
    //Automatic RGB Color Selection Mode
    
    monitorButton(button, buttonTrigger);
    
    unsigned int rgbColour[3];
    // Start off with red.
    rgbColour[0] = 255;
    rgbColour[1] = 0;
    rgbColour[2] = 0; 
    
    // Choose the colours to increment and decrement.
    for (int decColour = 0; decColour < 3; decColour += 1) 
    {
      int incColour = decColour == 2 ? 0 : decColour + 1;
      // cross-fade the two colours.
      for(int i = 0 ; i < 255 ; i += 1) 
      { 
        rgbColour[decColour] -= 1;
        rgbColour[incColour] += 1;          
        setRGB(pinRED, pinGREEN, pinBLUE, rgbColour[0], rgbColour[1], rgbColour[2]); 
      }
    }
  }
}

//Description of monitorButton() function
/*
  buttonPin = Digital pin to attatch button
  trigger   = Trigger state
  return    = void
*/
void monitorButton(int buttonPin, bool trigger)
{
  if(trigger == HIGH)
  {
    if(digitalRead(buttonPin)== HIGH) 
    {
      trigger = LOW; 
      #ifdef button_delay
      delay(100);
      #endif
      int blink = 3;
      indication(LED, blink); 
    }
  }
  else 
  {
    if(digitalRead(buttonPin) == HIGH)
    {
      trigger = LOW; 
      #ifdef button_delay
      delay(100);
      #endif
      if(digitalRead (!(buttonPin )== HIGH))
      {
        trigger = HIGH;
        int blink = 3;
        indication(LED, blink);
      }
    }
  }
}

//Description of monitorButton() function
/*
  redPin    = PWM pin to attatch red pole of RGB
  greenPin  = PWM pin to attatch green pole of RGB
  bluePin   = PWM pin to attatch blue pole of RGB
  red       = PWM value for red
  green     = PWM value for green
  blue      = PWM value for blue
  return    = void
*/
void setRGB(int redPin, int grnPin, int bluPin, unsigned int red, unsigned int green, unsigned int blue ) 
{     
  analogWrite(redPin, red);
  analogWrite(grnPin, green);
  analogWrite(bluPin, blue);
}

//Description of monitorButton() function
/*
  indicator     = Digital pin to attatch indicator (LED, Buzzer etc)
  throwCounter  = Counter
  return        = void
*/
void indication(int indicator, int throwCounter)
{
  for(int i=0; i<throwCounter ; i++)
  {
    digitalWrite(indicator, HIGH);
    delay(100);
    digitalWrite(indicator, LOW);      
    delay(150);
  }
}

//Description of potRead() function
/*
  potPin        = Analog pin to attatch pot
  pot_HIGH_pin  = Analog pin to monitor HIGH level
  pot_LOW_pin   = Analog pin to monitor LOW level
  upperBound    = Remapped MAX value
  lowerBound    = Remapped min value
  return        = Integer
*/
int potRead(int potPin, int pot_HIGH_pin, int pot_LOW_pin, int upperBound, int lowerBound)
{
  int analog_HIGH  = analogRead(pot_HIGH_pin);
  int analog_LOW   = analogRead(pot_LOW_pin);
  int potPin_value = analogRead(potPin);
  potPin_value     = map(potPin_value, analog_HIGH, analog_LOW, upperBound, lowerBound);
  return(potPin_value);
}

//Description of validate() Function
/*
  Value     = Value to validate
  MAXIMUM   = Expected maximum value
  minimum   = Expected minimum value
  Tolerance = Tolerance range in percentage for the value (eg. 12 for 12% tolerance)
  return    = boolean (true or false)
*/
boolean validate(double Value, double MAXIMUM, double minimum, double Tolerance)
{
  if (Value<minimum*(1-0.01*Tolerance)||Value>MAXIMUM*(1+0.01*Tolerance)) //Tolerance Calculation. Value cannot be lower than lower limit below tolerance or cannot be higher than uppor limit above tolerance
  {
    return false;
  }
  else
  {
    return true;
  }
}

//Description of RPM_Counter() function.
/*
  tacho_pin     = digital pin number to attatch Tachometer (RPM source)
  pulse         = Number of pulse produced by Tachometer for one full rotation (360 degree).
                  (Usually 2 Pulses per revolution.)
  RPM_MAX       = Upper Limit of expected RPM value
  RPM_min       = Lower Limit of expected RPM value
  RPM_Tolerance = Tolerance Range (Percentage Value)
  RPM_debug     = Set HIGH/LOW to eanble/disable serial monitor output for debugging
  return        = double-precision floating-point
*/
double RPM_Counter(int tacho_pin, int pulse, double RPM_MAX, double RPM_min, double RPM_Tolerance)
{
  unsigned long Rise_Time   = 0;
  unsigned long Fall_Time   = 0;

  //Function call : pulseIn();
  Rise_Time = pulseIn(tacho_pin, HIGH);
  #ifdef pulseIn_delay
  delay(50);
  #endif
 
  Fall_Time = pulseIn(tacho_pin, LOW);
  #ifdef pulseIn_delay
  delay(50);
  #endif

  //Calculations for RPM
  unsigned long Total_Time  = Rise_Time + Fall_Time;
  double        frequency   = 1000000/(pulse*Total_Time);
  double        RPM_value   = frequency*60;

  boolean RPM_validation = validate(RPM_value, RPM_MAX, RPM_min, RPM_Tolerance); //Validation for RPM value
 
  if (RPM_validation == true)
  {
    #ifdef display_debug
    Serial.print("\nHIGH Pulse Duration\t: ");
    Serial.print(Rise_Time); 
    Serial.print(" microseconds");
   
    Serial.print("\nLOW pulse duration\t: ");
    Serial.print(Fall_Time); 
    Serial.print(" microseconds");
   
    Serial.print("\nTotal Time\t\t: ");
    Serial.print(Total_Time);
    Serial.print(" microseconds");
   
    Serial.print("\nFrequency\t\t: ");
    Serial.print(frequency);
    Serial.print(" Hertz");
   
    Serial.print("\nRPM\t\t\t: ");
    Serial.print(RPM_value);
    Serial.println("\n\n");
    #endif
   
    return RPM_value;
  }
  else
  if (RPM_validation == false)
  {
    Serial.println("System Error\t\t: RPM out of range\n\n");
  }
}

//Description of Temperature_to_PWM() Function.
/*
  PIN_tmp       = Analog pin to attatch LM35 temperature sensor
  MAX_tmp       = Expected upperlimit of temperature value
  min_tmp       = Expected lower limit of temperature value
  Tolerance_tmp = Tolerance in percentage for temperature value (eg. 12 for 12% tolerance)
  debug_tmp     = Binary value to enable or disable debug feature
  return        = Integer
*/
int Temperature_to_PWM(int PIN_tmp, double MAX_tmp, double min_tmp, double Tolerance_tmp)
{
  int     RAW_tmp       = analogRead(PIN_tmp);                                //Analog Reading
  boolean validate_tmp  = validate(RAW_tmp, MAX_tmp, min_tmp, Tolerance_tmp); //Temperature raw value validation

  //Temperature value calculation in Celsius and Fahrenheit scale
  float tmp_value = (RAW_tmp/1024.0)*5000;
  float celsius = tmp_value/10;
  float fahrenheit = (celsius*9)/5 + 32;

  #ifdef display_debug 
  Serial.print("\nRAW\t\t\t: ");
  Serial.print(RAW_tmp);
  Serial.print(" millivolts");

  Serial.print("\nTemperature\t\t: ");
  Serial.print(celsius);
  Serial.print(" degree C");

  Serial.print("\nTemperature\t\t: ");
  Serial.print(fahrenheit);
  Serial.print(" degree F");
  #endif

  #ifdef display_delay
  delay(1000);
  #endif

  if (validate_tmp == false)
  {
    if (RAW_tmp > tmp_MAX)
    {
      int PWM = 255; //Return full PWM duty cycle if temperature is higher than expected maximum
      return(PWM);
    }
    else
    if (RAW_tmp < tmp_min)
    {
      int PWM = 0;  //Return zero PWM duty cycle if temperature is lower than expected minimum
      return(PWM);
    }
  }
  else
  if (validate_tmp == true)
  {
    int PWM = map(RAW_tmp, min_tmp, MAX_tmp, initial_duty_cycle, final_duty_cycle);
    return(PWM);
  }
}
