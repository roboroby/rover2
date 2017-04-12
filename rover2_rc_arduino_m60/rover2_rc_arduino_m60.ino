/*
 * Author: Roby
 * deepsouthrobotics.com
 * April 10, 2017
 * 
 * Code running Rover 2 with Radio/Arduino/M60
 * 
 * Note that M60 is what we're calling the 60A High Power MOS Dual 
 * Channel H-bridge DC Motor Driver Module like you can find here:
 * http://www.ebay.com/sch/i.html?_nkw=60A+High+Power+MOS+Dual+Channel+H-bridge+DC+Motor+Driver&_sacat=0
 * 
 * 
 */

const int arduinoPinCorrespondingToRcChannel1 = 6; //Steering
const int arduinoPinCorrespondingToRcChannel3 = 7; //Speed

const long pulseInDelayMicros = 250000; //Max number of microseconds to wait for pulse reading to be completed
const float steerSensitivity = 0.75; //Higher number makes more aggressive turning
const int deadZone = 30; //Between 0 and 255 -- any steering/throttle below this is ignored

/*
 * Adjust channel1 and channel3 min/mid/max
 * per your radio
 */
const int channel1Min = 1040;
const int channel1Mid = 1437;
const int channel1Max = 1840;

const int channel3Min = 1040;
const int channel3Mid = 1430;
const int channel3Max = 1850;

// M60 to Arduino
// right motor
const int pb = 10; //pb should be 9 or 10 if you plan to call setPwmFrequency(pb, 8) for noise reduction below
const int b1 = 8;
const int b2 = 13;

// M60 to Arduino
// left motor 
const int pa = 9; //pa should be 9 or 10 if you plan to call setPwmFrequency(pa, 8) for noise reduction below
const int a1 = 12; 
const int a2 = 11;

int channel1 = 0; // Steering channel pwm value
int channel3 = 0; // Speed channel pwm value

int leftMotorSpeed = 0; //pwm speed calculated for left motor
int rightMotorSpeed = 0; //pwm speed calculated for right motor

void setup() 
{
  //Debugging:
  //Serial.begin(9600);
  
  pinMode(arduinoPinCorrespondingToRcChannel1, INPUT); // Set our input pins as such
  pinMode(arduinoPinCorrespondingToRcChannel3, INPUT);

  //Set M60 control pins as OUTPUT
  pinMode(pa, OUTPUT);
  pinMode(pb, OUTPUT);
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);

  //Dividing pwm frequency via setPwmFrequency(...)
  //makes the motor noies less obnoxious
  //Check out the function's docs @
  //http://playground.arduino.cc/Code/PwmFrequency
  //Note that we use 9 and 10 for A and B pwm signal -- per
  //the documentation linked above, this would mess
  //up the servo library -- we aren't using Servo
  //library so we should be fine
  setPwmFrequency(pa, 8);
  setPwmFrequency(pb, 8);
}

/*
 * This is where we do the mixing -- converting steer and speed input
 * into left and right motor speed
 */
void calculateLeftAndRightMotorSpeed(int speedChannelScaled, int steerChannelScaled) 
{
  leftMotorSpeed = speedChannelScaled + (steerSensitivity * steerChannelScaled);
  rightMotorSpeed = speedChannelScaled - (steerSensitivity * steerChannelScaled);

  // neither motor can go faster than maximum speed
  leftMotorSpeed = constrain(leftMotorSpeed, -254, 254); //reduce to +-254 for M60
  rightMotorSpeed = constrain(rightMotorSpeed, -254, 254); //reduce to +-254 for M60

  //Debugging:

  /*
  //Serial.print("Left motor speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print("\t\t");
  //Serial.print("  Right motor speed: ");
  Serial.print(rightMotorSpeed);
  Serial.println();
  */
}

void setLeftMotorForward() 
{
  digitalWrite(b1, LOW);
  digitalWrite(b2, HIGH); 
}

void setLeftMotorBackward() 
{
  digitalWrite(b1, HIGH);
  digitalWrite(b2, LOW); 
}

void setRightMotorForward() 
{
  digitalWrite(a1, LOW);
  digitalWrite(a2, HIGH);  
}

void setRightMotorBackward() 
{
  digitalWrite(a1, HIGH);
  digitalWrite(a2, LOW);  
}

void stop() 
{
  analogWrite(pa, 0);
  analogWrite(pb, 0);
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);  
  digitalWrite(b1, LOW);
  digitalWrite(b2, LOW);  
}

void loop() 
{
  // Read the pulse width of each channel
  channel1 = pulseIn(arduinoPinCorrespondingToRcChannel1, HIGH, pulseInDelayMicros); 
  channel3 = pulseIn(arduinoPinCorrespondingToRcChannel3, HIGH, pulseInDelayMicros);

  //Debugging

  /*
  Serial.print("Channel1: ");
  Serial.print(channel1);
  Serial.print("\t\tChannel3: ");
  Serial.println(channel3);
  */
  
  //Handle case where radio is turned off (or out of range) -- channel1 and channel3 are zero
  if(channel1 == 0 && channel3 == 0) 
  {
    //stop motors
    Serial.println("No signal from radio channel 1 or radio channel 3");
    stop();
  }
  else
  {
    //Now we've got channel1 and channel3 radio pwm, convert to -255 to 255
    if(channel1 < channel1Mid) 
    {
      channel1 = map(channel1, channel1Mid, channel1Min, 0, -255);
    }
    else 
    {
      channel1 = map(channel1, channel1Mid, channel1Max, 0, 255);
    }
  
    if(channel3 < channel3Mid) 
    {
      channel3 = map(channel3, channel3Mid, channel3Min, 0, -255);
    }
    else 
    {
      channel3 = map(channel3, channel3Mid, channel3Max, 0, 255);
    }

    //Now calculate left and right motor speed from channel1 
    //and channel3 scaled values
    calculateLeftAndRightMotorSpeed(channel3, channel1);

    //If the calculated speed for both motors is in the dead zone
    //then stop rover
    if((abs(leftMotorSpeed) <= deadZone) && (abs(rightMotorSpeed) <= deadZone))
    {
      stop();
      Serial.println("In dead zone");
    }
    else //Run logic to set left and right motor speed via the L298N controller
    {
      if(leftMotorSpeed >= 0) 
      {
        setLeftMotorForward();
      }
      else 
      {
        setLeftMotorBackward();
      }
      
      analogWrite(pb, abs(leftMotorSpeed));
      
      if(rightMotorSpeed >= 0) 
      {
        setRightMotorForward();
      }
      else 
      {
        setRightMotorBackward();
      }
      analogWrite(pa, abs(rightMotorSpeed));
    }

  }

}

/**
 * Code from: http://playground.arduino.cc/Code/PwmFrequency
 * 
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

