#include <Servo.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

//#define ENABLE_SHIFT_BUTTONS

#define BLUETOOTH_RX_PIN 11
#define BLUETOOTH_TX_PIN 10
#define BLE_SERIAL_SPEED 57600

/* How many shift register chips are daisy-chained.
*/
#define NUMBER_OF_SHIFT_CHIPS 1

/* Width of data (how many ext lines).
*/
#define DATA_WIDTH 8

/* Width of pulse to trigger the shift register to read and latch.
*/
#define PULSE_WIDTH_USEC 5

/* Optional delay between shift register reads.
*/
#define POLL_DELAY_MSEC 1

#if defined(ENABLE_SHIFT_BUTTONS)

unsigned long lastPinsReadTime;

#define SHIFT_PLOAD_PIN 15
#define SHIFT_CLOCK_EN_PIN 17
#define SHIFT_DATA_PIN 16
#define SHIFT_CLOCK_PIN 14

bool pinValuesArr[DATA_WIDTH];
bool oldPinValuesArr[DATA_WIDTH];

#endif

String robotName = "Q1 mini"; // Robot name

const int enableCalibration = true; // Enable calibration button

#define NUMBER_OF_SERVOS 8
const int numberOfACE = 9; // Number of action code elements
int servoCal[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo calibration data
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
#define SERVO_PRG_PERIOD 20
Servo servo[NUMBER_OF_SERVOS]; // Servo object

//const enum RemoteRobotCommand {
//  Zero = 0,
//  Standby = 1,
//  Forward = 2,
//  Backward = 3,
//  Move_left = 4,
//  Move_right = 5,
//  Turn_left = 6,
//  Turn_right = 7,
//  Lie = 8,
//  Say_hi = 10,
//  Fighting = 11,
//  Push_up = 12,
//  Sleep = 13,
//  Dancing1 = 14,
//  Dancing2 = 15,
//  Dancing3 = 16
//};

bool _foundCalibrationSymbol = false;
bool _foundStartingSymbol = false;
int _lastReadCommand;
String _readCommandData = "";

SoftwareSerial _bluetoothSerial(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);

// Action code
// --------------------------------------------------------------------------------

// Servo zero position
// -------------------------- P02, P03, P05, P15, P07, P08, P11, P16
const int servoAct00[] = { 135,  45, 135,  45,  45, 135,  45, 135 };

// Zero
int servoPrg00step = 1;
const int servoPrg00[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{  135,  45, 135,  45,  45, 135,  45, 135,  400  }, // zero position
};

// Standby
int servoPrg01step = 2;
const int servoPrg01[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   90,  90,  90,  90,  90,  90,  90,  90,  200  }, // servo center point
	{   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
};

// Forward
int servoPrg02step = 11;
const int servoPrg02[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
	{   90,  90,  90, 110, 110,  90,  45,  90,  100  }, // leg1,4 up; leg4 fw
	{   70,  90,  90, 110, 110,  90,  45,  70,  100  }, // leg1,4 dn
	{   70,  90,  90,  90,  90,  90,  45,  70,  100  }, // leg2,3 up
	{   70,  45, 135,  90,  90,  90,  90,  70,  100  }, // leg1,4 bk; leg2 fw
	{   70,  45, 135, 110, 110,  90,  90,  70,  100  }, // leg2,3 dn
	{   90,  90, 135, 110, 110,  90,  90,  90,  100  }, // leg1,4 up; leg1 fw
	{   90,  90,  90, 110, 110, 135,  90,  90,  100  }, // leg2,3 bk
	{   70,  90,  90, 110, 110, 135,  90,  70,  100  }, // leg1,4 dn
	{   70,  90,  90, 110,  90, 135,  90,  70,  100  }, // leg3 up
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg3 fw dn
};

// Backward
int servoPrg03step = 11;
const int servoPrg03[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
	{   90,  45,  90, 110, 110,  90,  90,  90,  100  }, // leg4,1 up; leg1 fw
	{   70,  45,  90, 110, 110,  90,  90,  70,  100  }, // leg4,1 dn
	{   70,  45,  90,  90,  90,  90,  90,  70,  100  }, // leg3,2 up
	{   70,  90,  90,  90,  90, 135,  45,  70,  100  }, // leg4,1 bk; leg3 fw
	{   70,  90,  90, 110, 110, 135,  45,  70,  100  }, // leg3,2 dn
	{   90,  90,  90, 110, 110, 135,  90,  90,  100  }, // leg4,1 up; leg4 fw
	{   90,  90, 135, 110, 110,  90,  90,  90,  100  }, // leg3,1 bk
	{   70,  90, 135, 110, 110,  90,  90,  70,  100  }, // leg4,1 dn
	{   70,  90, 135,  90, 110,  90,  90,  70,  100  }, // leg2 up
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg2 fw dn
};

// Move Left
int servoPrg04step = 11;
const int servoPrg04[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
	{   70,  90,  45,  90,  90,  90,  90,  70,  100  }, // leg3,2 up; leg2 fw
	{   70,  90,  45, 110, 110,  90,  90,  70,  100  }, // leg3,2 dn
	{   90,  90,  45, 110, 110,  90,  90,  90,  100  }, // leg1,4 up
	{   90, 135,  90, 110, 110,  45,  90,  90,  100  }, // leg3,2 bk; leg1 fw
	{   70, 135,  90, 110, 110,  45,  90,  70,  100  }, // leg1,4 dn
	{   70, 135,  90,  90,  90,  90,  90,  70,  100  }, // leg3,2 up; leg3 fw
	{   70,  90,  90,  90,  90,  90, 135,  70,  100  }, // leg1,4 bk
	{   70,  90,  90, 110, 110,  90, 135,  70,  100  }, // leg3,2 dn
	{   70,  90,  90, 110, 110,  90, 135,  90,  100  }, // leg4 up
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg4 fw dn
};

// Move Right
int servoPrg05step = 11;
const int servoPrg05[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
	{   70,  90,  90,  90,  90,  45,  90,  70,  100  }, // leg2,3 up; leg3 fw
	{   70,  90,  90, 110, 110,  45,  90,  70,  100  }, // leg2,3 dn
	{   90,  90,  90, 110, 110,  45,  90,  90,  100  }, // leg4,1 up
	{   90,  90,  45, 110, 110,  90, 135,  90,  100  }, // leg2,3 bk; leg4 fw
	{   70,  90,  45, 110, 110,  90, 135,  70,  100  }, // leg4,1 dn
	{   70,  90,  90,  90,  90,  90, 135,  70,  100  }, // leg2,3 up; leg2 fw
	{   70, 135,  90,  90,  90,  90,  90,  70,  100  }, // leg4,1 bk
	{   70, 135,  90, 110, 110,  90,  90,  70,  100  }, // leg2,3 dn
	{   90, 135,  90, 110, 110,  90,  90,  70,  100  }, // leg1 up
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1 fw dn
};

// Turn left
int servoPrg06step = 8;
const int servoPrg06[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
	{   90,  90,  90, 110, 110,  90,  90,  90,  100  }, // leg1,4 up
	{   90, 135,  90, 110, 110,  90, 135,  90,  100  }, // leg1,4 turn
	{   70, 135,  90, 110, 110,  90, 135,  70,  100  }, // leg1,4 dn
	{   70, 135,  90,  90,  90,  90, 135,  70,  100  }, // leg2,3 up
	{   70, 135, 135,  90,  90, 135, 135,  70,  100  }, // leg2,3 turn
	{   70, 135, 135, 110, 110, 135, 135,  70,  100  }, // leg2,3 dn
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1,2,3,4 turn
};

// Turn right
int servoPrg07step = 8;
const int servoPrg07[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
	{   70,  90,  90,  90,  90,  90,  90,  70,  100  }, // leg2,3 up
	{   70,  90,  45,  90,  90,  45,  90,  70,  100  }, // leg2,3 turn
	{   70,  90,  45, 110, 110,  45,  90,  70,  100  }, // leg2,3 dn
	{   90,  90,  45, 110, 110,  45,  90,  90,  100  }, // leg1,4 up
	{   90,  45,  45, 110, 110,  45,  45,  90,  100  }, // leg1,4 turn
	{   70,  45,  45, 110, 110,  45,  45,  70,  100  }, // leg1,4 dn
	{   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // leg1,2,3,4 turn
};

// Lie
int servoPrg08step = 1;
const int servoPrg08[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{  110,  90,  90,  70,  70,  90,  90, 110,  500  }, // leg1,4 up
};

// Say Hi
int servoPrg09step = 4;
const int servoPrg09[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
	{   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
	{  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
	{   70,  90,  90, 110, 110,  90,  90,  70,  200  }, // standby
};

// Fighting
int servoPrg10step = 11;
const int servoPrg10[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 2 down
	{  120,  70,  70, 110,  60,  70,  70,  70,  200  }, // body turn left
	{  120, 110, 110, 110,  60, 110, 110,  70,  200  }, // body turn right
	{  120,  70,  70, 110,  60,  70,  70,  70,  200  }, // body turn left
	{  120, 110, 110, 110,  60, 110, 110,  70,  200  }, // body turn right
	{   70,  90,  90,  70, 110,  90,  90, 110,  200  }, // leg1, 2 up ; leg3, 4 down
	{   70,  70,  70,  70, 110,  70,  70, 110,  200  }, // body turn left
	{   70, 110, 110,  70, 110, 110, 110, 110,  200  }, // body turn right
	{   70,  70,  70,  70, 110,  70,  70, 110,  200  }, // body turn left
	{   70, 110, 110,  70, 110, 110, 110, 110,  200  }, // body turn right
	{   70,  90,  90,  70, 110,  90,  90, 110,  200  }  // leg1, 2 up ; leg3, 4 down
};

// Push up
int servoPrg11step = 11;
const int servoPrg11[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // start
	{  100,  90,  90,  80,  80,  90,  90, 100,  400  }, // down
	{   70,  90,  90, 110, 110,  90,  90,  70,  500  }, // up
	{  100,  90,  90,  80,  80,  90,  90, 100,  600  }, // down
	{   70,  90,  90, 110, 110,  90,  90,  70,  700  }, // up
	{  100,  90,  90,  80,  80,  90,  90, 100, 1300  }, // down
	{   70,  90,  90, 110, 110,  90,  90,  70, 1800  }, // up
	{  135,  90,  90,  45,  45,  90,  90, 135,  200  }, // fast down
	{   70,  90,  90,  45,  60,  90,  90, 135,  500  }, // leg1 up
	{   70,  90,  90,  45, 110,  90,  90, 135,  500  }, // leg2 up
	{   70,  90,  90, 110, 110,  90,  90,  70,  500  }  // leg3, leg4 up
};

// Sleep
int servoPrg12step = 2;
const int servoPrg12[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   30,  90,  90, 150, 150,  90,  90,  30,  200  }, // leg1,4 dn
	{   30,  45, 135, 150, 150, 135,  45,  30,  200  }, // protect myself
};

// Dancing 1
int servoPrg13step = 10;
const int servoPrg13[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1,2,3,4 up
	{   50,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1 dn
	{   90,  90,  90, 130,  90,  90,  90,  90,  300  }, // leg1 up; leg2 dn
	{   90,  90,  90,  90,  90,  90,  90,  50,  300  }, // leg2 up; leg4 dn
	{   90,  90,  90,  90, 130,  90,  90,  90,  300  }, // leg4 up; leg3 dn
	{   50,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg3 up; leg1 dn
	{   90,  90,  90, 130,  90,  90,  90,  90,  300  }, // leg1 up; leg2 dn
	{   90,  90,  90,  90,  90,  90,  90,  50,  300  }, // leg2 up; leg4 dn
	{   90,  90,  90,  90, 130,  90,  90,  90,  300  }, // leg4 up; leg3 dn
	{   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg3 up
};

// Dancing 2
int servoPrg14step = 9;
const int servoPrg14[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  45, 135, 110, 110, 135,  45,  70,  300  }, // leg1,2,3,4 two sides
	{  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg1,2 up
	{   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
	{  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
	{   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
	{  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
	{   70,  45, 135, 110,  65, 135,  45, 115,  300  }, // leg1,2 dn; leg3,4 up
	{  115,  45, 135,  65, 110, 135,  45,  70,  300  }, // leg3,4 dn; leg1,2 up
	{   75,  45, 135, 105, 110, 135,  45,  70,  300  }, // leg1,2 dn
};

// Dancing 3
int servoPrg15step = 10;
const int servoPrg15[][numberOfACE] = {
	// P02, P03, P05, P15, P07, P08, P11, P16,  ms
	{   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3,4 bk
	{  110,  45,  45,  60,  70, 135, 135,  70,  300  }, // leg1,2,3 up
	{   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3 dn
	{  110,  45,  45, 110,  70, 135, 135, 120,  300  }, // leg1,3,4 up
	{   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,3,4 dn
	{  110,  45,  45,  60,  70, 135, 135,  70,  300  }, // leg1,2,3 up
	{   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3 dn
	{  110,  45,  45, 110,  70, 135, 135, 120,  300  }, // leg1,3,4 up
	{   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,3,4 dn
	{   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // standby
};

// --------------------------------------------------------------------------------


void setup()
{
	//eepromClear();

	Serial.begin(9600);

#if defined(ENABLE_SHIFT_BUTTONS)
	/* Initialize our digital pins...
	*/
	pinMode(SHIFT_PLOAD_PIN, OUTPUT);
	pinMode(SHIFT_CLOCK_EN_PIN, OUTPUT);
	pinMode(SHIFT_CLOCK_PIN, OUTPUT);
	pinMode(SHIFT_DATA_PIN, INPUT);

	digitalWrite(SHIFT_CLOCK_PIN, LOW);
	digitalWrite(SHIFT_PLOAD_PIN, HIGH);

	/* Read in and display the pin states at startup.
	*/
	lastPinsReadTime = millis();
	read_shift_regs(pinValuesArr);
	display_pin_values();
	copyToArray(pinValuesArr, oldPinValuesArr, DATA_WIDTH);
#endif

	_bluetoothSerial.begin(BLE_SERIAL_SPEED);
	getServoCal(); // Get servoCal from EEPROM
  printArray(servoCal, NUMBER_OF_SERVOS);

	// Servo Pin Set
	servo[0].attach(2);
	servo[1].attach(3);
	servo[2].attach(4);
	servo[3].attach(5);
	servo[4].attach(6);
	servo[5].attach(7);
	servo[6].attach(8);
	servo[7].attach(9);

	runServoPrg(servoPrg00, servoPrg00step); // zero position
}

void loop()
{
	processBluetoothInput();

#if defined(ENABLE_SHIFT_BUTTONS)
	if (millis() - lastPinsReadTime > POLL_DELAY_MSEC)
	{
		read_shift_regs(pinValuesArr);

		if (compareArrays(pinValuesArr, oldPinValuesArr, DATA_WIDTH) == false)
		{
			Serial.print("*Pin value change detected*\r\n");
			printArray(pinValuesArr, DATA_WIDTH);
			display_pin_values();
			copyToArray(pinValuesArr, oldPinValuesArr, DATA_WIDTH);
			activateCommandByIndex(getFirstActiveButtonIndex(pinValuesArr, DATA_WIDTH));
		}
		lastPinsReadTime = millis();
	}
#endif
	//Serial.println("-");
}

// Robot functions functions
// --------------------------------------------------------------------------------

#if defined(ENABLE_SHIFT_BUTTONS)

void activateCommandByIndex(int buttonIndex)
{
	Serial.println("Executing index: " + (String)buttonIndex);
	switch (buttonIndex)
	{
	case 0:
	{
		//Backward
		runServoPrg(servoPrg03, servoPrg03step);
		break;
	}
	case 1:
	{
		//Turn left
		runServoPrg(servoPrg06, servoPrg06step);
		break;
	}
	case 2:
	{
		//Turn right
		runServoPrg(servoPrg07, servoPrg07step);
		break;
	}
	case 3:
	{
		//Forward
		runServoPrg(servoPrg02, servoPrg02step);
		break;
	}
	case 4:
	{
		//Zero
		runServoPrg(servoPrg00, servoPrg00step);
		break;
	}
	case 5:
	{
		//Push up
		runServoPrg(servoPrg11, servoPrg11step);
		break;
	}
	case 6:
	{
		//Dancing 1
		runServoPrg(servoPrg13, servoPrg13step);
		break;
	}
	case 7:
	{
		//Lie
		runServoPrg(servoPrg08, servoPrg08step);
		break;
	}
	default:
		break;
	}
}

int getFirstActiveButtonIndex(bool pinValuesData[], int arrLength)
{
	for (int counter = 0; counter < arrLength; counter++)
	{
		if (pinValuesData[counter] == false)
			return counter;
	}
	return -1;
}

#endif

void processBluetoothInput()
{
	while (_bluetoothSerial.available())
	{
		char currentInput = _bluetoothSerial.read();

		if (_foundCalibrationSymbol == false && currentInput == '[')
		{
			_foundCalibrationSymbol = true;
		}
		else if (_foundCalibrationSymbol == true && currentInput == ']')
		{
			parseAndSetCalibration(_readCommandData);
			_foundCalibrationSymbol = false;
			_readCommandData = "";
		}
		else if (_foundCalibrationSymbol == true)
		{
			_readCommandData += currentInput;
		}
		else if (_foundStartingSymbol == false && currentInput == '<')
		{
			_foundStartingSymbol = true;
		}
		else if (_foundStartingSymbol == true && currentInput == '>')
		{
			_lastReadCommand = _readCommandData.toInt();			
			_foundStartingSymbol = false;
			_readCommandData = "";
			executeCommand(_lastReadCommand);
		}
		else if (_foundStartingSymbol == true)
		{
			_readCommandData += currentInput;
		}
	}
}

void parseAndSetCalibration(String calibrationData)
{
  //Serial.println(calibrationData);
  char calDataArr[calibrationData.length() + 1];
  calibrationData.toCharArray(calDataArr, calibrationData.length() + 1);
	int calibrationDataLocal[2] = {0,0};
	int tempCalibrationIndex = 0;
	String tempNumber = "";
	for (int counter = 0; counter < calibrationData.length(); counter++)
	{
		if (calDataArr[counter] == ':')
		{
      //Serial.println(":");
			calibrationDataLocal[tempCalibrationIndex] = tempNumber.toInt();
			tempNumber = "";
			tempCalibrationIndex++;
		}
		else
		{
      //Serial.println(tempNumber);
			tempNumber += calDataArr[counter];
		}
   //Serial.println("ARR: " + (String)calDataArr[counter]);
	}
  calibrationDataLocal[tempCalibrationIndex] = tempNumber.toInt();
  //Serial.println("CAL " + (String)calibrationDataLocal[0] + " " + (String)calibrationDataLocal[1]);
  
	servoCal[calibrationDataLocal[0]] = calibrationDataLocal[1];
	putServoCal();
	runServoPrg(servoPrg01, servoPrg01step); //standby	
}

void executeCommand(int command)
{
  Serial.println(command);
	switch (command)
	{
	case 0: //Backward:
	{
		runServoPrg(servoPrg03, servoPrg03step);
		break;
	}
	case 1: //Turn_left:
	{
		runServoPrg(servoPrg06, servoPrg06step);
		break;
	}
	case 2: //Turn_right:
	{
		runServoPrg(servoPrg07, servoPrg07step);
		break;
	}
	case 3: //Forward:
	{
		runServoPrg(servoPrg02, servoPrg02step);
		break;
	}
	case 4: //Zero:
	{
		runServoPrg(servoPrg00, servoPrg00step);
		break;
	}
//	case 5: //Push_up:
//	{
//		runServoPrg(servoPrg11, servoPrg11step);
//		break;
//	}
//	case 6: //Dancing1:
//	{
//		runServoPrg(servoPrg13, servoPrg13step);
//		break;
//	}
	case 7: //Lie:
	{
		runServoPrg(servoPrg08, servoPrg08step);
		break;
	}
//	case 8: //Dancing2:
//	{
//		runServoPrg(servoPrg14, servoPrg14step);
//		break;
//	}
//	case 9: //Dancing3:
//	{
//		runServoPrg(servoPrg15, servoPrg15step);
//		break;
//	}
	case 10: //Sleep:
	{
		runServoPrg(servoPrg12, servoPrg12step);
		break;
	}
//	case 11: //Fighting:
//	{
//		runServoPrg(servoPrg10, servoPrg10step);
//		break;
//	}
	case 12: //Say_hi:
	{
		runServoPrg(servoPrg09, servoPrg09step);
		break;
	}
	case 13: //Move_left:
	{
		runServoPrg(servoPrg04, servoPrg04step);
		break;
	}
	case 14: //Move_right:
	{
		runServoPrg(servoPrg05, servoPrg05step);
		break;
	}
	case 15: //Standby:
	{
		runServoPrg(servoPrg01, servoPrg01step);
		break;
	}
	default:
		break;
	}
}

// EEPROM Clear (For debug only)
void eepromClear()
{
	for (int i = 0; i < EEPROM.length(); i++) {
		EEPROM.write(i, 0);
	}
	//randomSound();
}

// Get servoCal from EEPROM
void getServoCal()
{
	int eeAddress = 0;
	for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
		EEPROM.get(eeAddress, servoCal[i]);
		eeAddress += sizeof(servoCal[i]);
	}
}

// Put servoCal to EEPROM
void putServoCal()
{
	int eeAddress = 0;
	for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
		EEPROM.put(eeAddress, servoCal[i]);
		eeAddress += sizeof(servoCal[i]);
	}
}

// Clear Servo calibration data
void clearCal()
{
	for (int i = 0; i < NUMBER_OF_SERVOS; i++) {
		servoCal[i] = 0;
	}
	putServoCal(); // Put servoCal to EEPROM
	runServoPrg(servoPrg00, servoPrg00step); // zero position
}

// Calibration
void calibration(int i, int change)
{
	servoCal[i] = servoCal[i] + change;
	servo[i].write(servoAct00[i] + servoCal[i]);
	putServoCal(); // Put servoCal to EEPROM
	delay(400);
}

void runServoPrg(const int servoPrg[][numberOfACE], int step)
{
	for (int i = 0; i < step; i++) { // Loop for step

		int totalTime = servoPrg[i][numberOfACE - 1]; // Total time of this step
		Serial.println("Total time: " + (String)totalTime);

		// Get servo start position
		for (int s = 0; s < NUMBER_OF_SERVOS; s++) {
			servoPos[s] = servo[s].read() - servoCal[s];
		}

		for (int j = 0; j < totalTime / SERVO_PRG_PERIOD; j++) { // Loop for time section
			for (int k = 0; k < NUMBER_OF_SERVOS; k++) { // Loop for servo
				servo[k].write((map(j, 0, totalTime / SERVO_PRG_PERIOD, servoPos[k], servoPrg[i][k])) + servoCal[k]);
			}
			delay(SERVO_PRG_PERIOD);
		}
		Serial.println("Run servo loop: " + (String)i);
	}
	Serial.println("Run servo loop end");
}

#if defined(ENABLE_SHIFT_BUTTONS)

// Buttons functions
// --------------------------------------------------------------------------------

void read_shift_regs(bool* output)
{
	long bitVal;
	bool newPinValues[DATA_WIDTH];

	/* Trigger a parallel Load to latch the state of the data lines,
	*/
	digitalWrite(SHIFT_CLOCK_EN_PIN, HIGH);
	digitalWrite(SHIFT_PLOAD_PIN, LOW);
	delayMicroseconds(PULSE_WIDTH_USEC);
	digitalWrite(SHIFT_PLOAD_PIN, HIGH);
	digitalWrite(SHIFT_CLOCK_EN_PIN, LOW);

	/* Loop to read each bit value from the serial out line
	 * of the SN74HC165N.
	*/
	for (int i = 0; i < DATA_WIDTH; i++)
	{
		bitVal = digitalRead(SHIFT_DATA_PIN);

		newPinValues[i] = (bool)bitVal;

		digitalWrite(SHIFT_CLOCK_PIN, HIGH);
		delayMicroseconds(PULSE_WIDTH_USEC);
		digitalWrite(SHIFT_CLOCK_PIN, LOW);
	}
	copyToArray(newPinValues, output, DATA_WIDTH);
}

/* Dump the list of zones along with their current status.
*/
void display_pin_values()
{
	Serial.print("Pin States:\r\n");

	for (int i = 0; i < DATA_WIDTH; i++)
	{
		Serial.print("  Pin-");
		Serial.print(i);
		Serial.print(": ");

		if (pinValuesArr[i])
			Serial.print("HIGH");
		else
			Serial.print("LOW");

		Serial.print("\r\n");
	}

	Serial.print("\r\n");
}

#endif

bool compareArrays(bool first[], bool second[], int arrLength)
{
	for (int counter = 0; counter < arrLength; counter++)
	{
		if (first[counter] != second[counter])
		{
			return false;
		}
	}
	return true;
}

void copyToArray(bool* copyFrom, bool* copyTo, int arrLength)
{
	for (int counter = 0; counter < arrLength; counter++)
	{
		copyTo[counter] = copyFrom[counter];
	}
}

void printArray(bool input[], int arrLength)
{
	for (int counter = 0; counter < arrLength; counter++)
	{
		Serial.print(input[counter]);
	}
	Serial.println();
}

void printArray(int input[], int arrLength)
{
  for (int counter = 0; counter < arrLength; counter++)
  {
    Serial.print(input[counter]);
    Serial.print(" ");
  }
  Serial.println();
}




