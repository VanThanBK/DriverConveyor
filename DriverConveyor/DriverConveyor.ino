/*
 Name:		DriverConveyor.ino
 Created:	5/18/2019 9:47:48 AM
 Author:	Than
*/

#include <EEPROM.h>
#include "MultiThread.h"

#define DESIRED_SPEED	60//mm/s

#define COMMAND_PORT		Serial

#define PWM1				PA2
#define PWM2				PA1

#define LED					PB12

#define PWM					PB1
#define DIR					PB0

#define CHANEL_A			PB15
#define CHANEL_B			PB14

#define RESOLUTION      668.0
#define DIAMETER		63.5		//mm

#define MAX_VELOCITY		140 //mm/s

#define VELOCITY_SAMPLE_TIME_MS		30

#define CONTROL true
#define AUTO false

float Kp = 0.28;
float Ki = 0;
float Kd = 0;

float P = 0;
float I = 0;
float D = 0;

int CountedPulse = 0;
float DesiredSpeed = 0;
float CurrentSpeed = 0;
float PWMValue = 0;

int32_t Position;

float Error = 0;
float LastError = 0;
float LastLastError = 0;
float dError = 0;

MultiThread BlinkScheduler;
MultiThread VelocityPIDScheduler;

bool LedState = true;
bool Mode = CONTROL;

String inputString;
bool stringComplete;

void setup()
{
	InitIO();
}

//-------------------- MAIN --------------------

void loop()
{
	Check();
	SerialExecute();
	Blink();
	AdjustSpeed(DesiredSpeed);
}

//-----------------------------------------------

void InitIO()
{
	COMMAND_PORT.begin(9600);

	pinMode(PWM1, OUTPUT);
	pinMode(PWM2, OUTPUT);
	pinMode(LED, OUTPUT);

	pinMode(PWM, INPUT);
	pinMode(DIR, INPUT);

	pinMode(CHANEL_A, INPUT);
	pinMode(CHANEL_B, INPUT);

	digitalWrite(LED, 1);

	attachInterrupt(CHANEL_A, CountPulseA_x2, FALLING);
	attachInterrupt(CHANEL_B, CountPulseB_x2, FALLING);

	stringComplete = false;
	inputString = "";
}

void Check()
{
	if (digitalRead(PWM) == LOW)
	{
		Mode = CONTROL;
	}
	else
	{
		Mode = AUTO;
	}
}

void CountPulseA_x2()
{
	if (digitalRead(CHANEL_B) == LOW)
	{
		CountedPulse++;
		Position++;
	}
	else
	{
		CountedPulse--;
		Position--;
	}

	if (DesiredSpeed == 0)
	{
		CountedPulse = 0;
	}
	//COMMAND_PORT.println(CountedPulse);
}

void CountPulseB_x2()
{
	if (digitalRead(CHANEL_A) == HIGH)
	{
		CountedPulse++;
		Position++;
	}
	else
	{
		CountedPulse--;
		Position--;
	}

	if (DesiredSpeed == 0)
	{
		CountedPulse = 0;
	}
}

void Blink()
{
	if (BlinkScheduler.isSchedule(MAX_VELOCITY - abs(DesiredSpeed)))
	{
		LedState = !LedState;

		if (DesiredSpeed == 0)
		{
			digitalWrite(LED, 1);
		}
		else
		{
			digitalWrite(LED, LedState);
		}
	}
}

void AdjustSpeed(float speed)
{
	RUN_EVERY(VelocityPIDScheduler, VELOCITY_SAMPLE_TIME_MS)

	if (speed == 0)
	{
		PWMValue = 0;
		SetDesiredSpeed(PWMValue);
		return;
	}

	float desiredVel = abs(speed);

	float currenVel = abs(((float)CountedPulse * PI * DIAMETER) / RESOLUTION);
	currenVel = currenVel * 1000.0 / (float)VELOCITY_SAMPLE_TIME_MS;
	CountedPulse = 0;

	Error = desiredVel - currenVel;
	dError = Error - 2 * LastError + LastLastError;
	LastLastError = LastError;
	LastError = Error;

	P = (Error)* Kp;

	I += (Error * Ki) * ((float)VELOCITY_SAMPLE_TIME_MS / 1000);

	I = constrain(I, -100, 100);

	D = (dError * Kd) / ((float)VELOCITY_SAMPLE_TIME_MS / 1000);

	PWMValue = PWMValue + P + I + D;

	PWMValue = constrain(PWMValue, 0, 180);

	SetDesiredSpeed(PWMValue * (speed / abs(speed)));

	if (Mode == CONTROL)
	{
		COMMAND_PORT.println(currenVel);
	}

}

void SetDesiredSpeed(int pwmValue)
{
	if (pwmValue < 0)
	{  
		analogWrite(PWM1, abs(pwmValue));
		analogWrite(PWM2, 0);
	}
	else
	{
		analogWrite(PWM1, 0);
		analogWrite(PWM2, abs(pwmValue));
	}
}

void SerialExecute()
{
	while (COMMAND_PORT.available())
	{
		char inChar = (char)COMMAND_PORT.read();

		if (inChar == '\n')
		{
			stringComplete = true;
			break;
		}

		inputString += inChar;
	}

	if (!stringComplete)
		return;

	String messageBuffer = inputString.substring(0, 7);

	if (Mode == CONTROL)
	{
		if (messageBuffer == "ValueKp")
		{
			Kp = inputString.substring(8).toFloat();
			//COMMAND_PORT.println(Kp, DEC);
		}
		else if (messageBuffer == "ValueKi")
		{
			Ki = inputString.substring(8).toFloat();
			//COMMAND_PORT.println(Ki, DEC);
		}
		else if (messageBuffer == "ValueKd")
		{
			Kd = inputString.substring(8).toFloat();
			//COMMAND_PORT.println(Kd, DEC);
		}
		else if (messageBuffer == "Velocit")
		{
			DesiredSpeed = inputString.substring(8).toFloat();
			//COMMAND_PORT.println(DesiredSpeed, DEC);
		}
	}
	else
	{
		if (messageBuffer == "Positio")
		{
			COMMAND_PORT.println(GetPosition());
		}
		else if (messageBuffer == "Velocit")
		{
			DesiredSpeed = inputString.substring(8).toFloat();
			COMMAND_PORT.println("Ok");
		}
		else if (messageBuffer == "Confirm")
		{
			COMMAND_PORT.println("YesConveyor");
		}
		else
		{
			String messageBuffer1 = inputString.substring(0, 4);
			if (messageBuffer1 == "M701")
			{
				COMMAND_PORT.println(GetPosition());
			}
		}
	}

	inputString = "";
	stringComplete = false;
}

float GetPosition()
{
	float position = abs(((float)Position * PI * DIAMETER) / RESOLUTION);
	Position = 0;
	return position;
}

int32_t GetPulse()
{
	float position = Position;
	Position = 0;
	return position;
}