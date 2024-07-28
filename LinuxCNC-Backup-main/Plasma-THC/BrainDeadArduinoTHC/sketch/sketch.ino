#include <trigger.h>
#include <EasyNextionLibrary.h>

#include <digitalWriteFast.h>

/* Trin modded
*  
*  V1 -  replace sevseg with 16x2 i2C, renamed and changed inputs 
*        and outputs pin names to match older THC standalone,
*        added stepper outputs etc to use as standalone.
*        **to do -- add on/off so passthrough can work
*  
*  V2 -  Added thcEngageSwitch into setup(), now if the switch is closed
*        the THC is on and the settings are locked. So if starting the 
*        THC with the switch on will immediately start and lock the settings
*
*  V3 -  removed standalone option and changed heaps. ie variable naming,
*        removed standalone option (sends raise and lower torch to Controller)
*        [currently linuxCNC]
*
*  V4 -  Modified by J. Carter:  Switched display to Nextion NX4832K035_011
*/  

/*Pin allocation
- A0 = arcVoltageInput // voltage input from voltage divider in arcVoltageInput machine.
- A2 = //
- A3 = setPointInput //pot to adjust voltage setpoint.
- A4 = SDA LCD
- A5 = SCL LCD

D2 = //
D4 = //
D5 = raiseTorchPin //Directly connected to breakout board Pin 12(PlasmaUp).
D6 = lowerTorchLedPin //HIGH if volts are high lower torch (red led pin)
D7 = //
D8 = raiseTorchLedPin //HIGH if volts are low raise torch(yellow led pin)
D9 = lowerTorchPin //Directly connected to breakout board Pin 13 (PlasmaDown).
D10 = thcOnLedPin // independant led pin
D11 = thcOnSwitch //thc On, locks voltage setting.
D12 = dataDisplyInput //show last 200 recording of arcV


*  A braindead plasma torch THC. Goes along with the board schematic in this repo.
The gist of how this is works is, on startup, you read the potentiometer and
pick VOLT & SPEED% set points. Then TURN THC ON and it reads the analog pin for the plasma voltage, does
comparisons to that setpoint, and triggers the optocouplers accordingly, if the arcOk signal exists.

There's a little bit of smoothing that goes into to the ADC reads to improve
the signal's stability a bit, but we can't do too much, since otherwise, the
LinuxCNC can't respond fast enough when the torch height really is changing.
I do a few tricks in here to make sure the loop() function runs quickly. Stuff
like only reading the setpoint on startup (so if you want to adjust it, hit the
reset button), using bitwise shifts for division, etc. Last I checked, this was
able to do about 6000 samples per second, whereas the maximum you could get
from simply looping analogRead() is supposed to be 9000 samples per second
(with 10 bit precision).

Two more noteworthy quirks:
Firstly, while we let the signals for the plasma change rather rapidly, we only
update the display several times a second and give it a much, much longer
average. This just makes it easier on the eyes.

Secondly, this is going to be giving the "UP" signal whenever the plasma
is not cutting (since the voltage will be 0), so it is important to use
LinuxCNC's HAL configs to only respect the THC's signals once the torch is
cutting and is not piercing or cornering. The "thcud" component should help
with that, as well as additional YouTube videos and files coming soon.
(c) The Swolesoft Development Group, 2019
License: http://250bpm.com/blog:82
*/

// the variables to be using be the code below
EasyNex THCNex(Serial);  // Create an object of EasyNex class with the name < TCHNex >
// Set as parameter the Serial you are going to use
// Default baudrate 9600



// Naming inputs and outputs pins.

namespace
{
constexpr int arcVoltageInput = A0;
constexpr int setPointInput = A3;
constexpr int raiseTorchPin = 5;
constexpr int lowerTorchLedPin = 6;
constexpr int raiseTorchLedPin = 8;
constexpr int lowerTorchPin = 9;
constexpr int thcOnLedPin = 10;
constexpr int thcOnSwitch = 11;
}

bool thcOn = false;

// Setting the scale for the converting analogRead values to volts.
// 4.891 AREF voltage * 50 built-in voltage divider / 1023 resolution = ***** ADC counts per volt
// As far as I can tell, the arithmetic below *does* get optimized out by the compiler.

// note: my voltage divider has a 100k and a 2k ohm resistor.... 
// hence 102k/2k = 51 (ratio, not 50 as manufacturer states) 
// note below *****
// calibrated at half scale manually against calibrated DMM
// cutting happens closer to this than full scale 
#define SCALE (4.8125*50/1023)
// #define SCALE (3.1125*50/1023)

// bandwidth/Threshold in ADC counts for when we say the torch voltage is too high or low 
// Multiply by SCALE for the threshold in volts.
// FYI: Some degree of buffer is needed to prevent awful see-sawing.
#define THRESH 5

// Voltage adjustment range for the knob.
#define MINSET 80
#define MAXSET 160

//required for sample averaging...
#define BUFSIZE 256  // Would technically let us do running averages up to BUFSIZE samples. In testing, shorter averages seemed better.
#define SAMP 16  // Use this many samples in the average; must be a power of 2 and no larger than BUFSIZE.
#define DISP 256 // The number of samples to use in calculating a slower average for the display. Must also be a power of 2.

unsigned int shift = 0;

unsigned int values[BUFSIZE] = {0}; // buffer for ADC reads
unsigned long total = 0; // for keeping a rolling total of the buffer
unsigned long disp = 0;  // for separately tracking ADC reads for the display
unsigned long target = 0; // voltage target, in ADC counts

// for tracking when to move torch up and down
int diff = 0;
int mean = 0;
int mode = -1;

// generic temp vars
unsigned long tmp = 0;
float ftmp = 0;
float ftmp2 = 0;

// generic looping vars
int i = 0;
int j = 0;

void setup() 
{

	//THC inputs and outputs
	pinMode(raiseTorchPin, OUTPUT);
	pinMode(lowerTorchPin, OUTPUT);

	pinMode(thcOnSwitch, INPUT_PULLUP);
	pinMode(thcOnLedPin, OUTPUT);

	pinMode(raiseTorchLedPin, OUTPUT);
	pinMode(lowerTorchLedPin, OUTPUT);


	//analog inputs
	pinMode(setPointInput, INPUT);
	pinMode(arcVoltageInput, INPUT);


	// Begin the object with a baud rate of 9600
	THCNex.begin(115200);  // If no parameter was given in the begin(), the default baud rate of 9600 will be used
	while (!Serial) {
		;  // wait for serial port to connect. Needed for native USB port only
	}
	// Wait for Nextion Screen to bootup
	delay(2500);

	// Set the reference voltage to the external linear regulator
	// Do a few throwaway reads so the ADC stabilizes, as recommended by the docs.
	analogReference(EXTERNAL);
	analogRead(arcVoltageInput); 
	analogRead(arcVoltageInput); 
	analogRead(arcVoltageInput); 
	analogRead(arcVoltageInput); 
	analogRead(arcVoltageInput);


	// We need to calculate how big the shift must be, for a given sample size.
	// Since we are using bitshifting instead of division, I'm using a != here,
	// so your shit will be totally broke if you don't set SAMP to a power of 2.
	while((1 << shift) != SAMP)
		shift++;
}

//**********************************************************
//**********************************************************
void loop() 
{
// ....thcOnSwitch NOW LOCKS IN THE SETTINGS INSTEAD OF TIMER BELOW.....
// Now enter the period where you can set the voltage via the potentiomenter.
// OLD ---- Default 5s period, plus an extension 2s as long as you keep adjusting it.
// By fixing this after boot, we save cycles from needing to do two ADC reads per loop(),
// avoid any nonsense from potentiometer drift, and don't need to think about the
// capacitance of the ADC muxer.
	if (!thcOn)
	{
		i=0;
		while (digitalReadFast(thcOnSwitch)) 
		{
			tmp = analogRead(setPointInput);

			// Keep a rotating total, buffer, and average.  Since this value only moves
			// a small amount due to noise in the AREF voltage and the physical
			// potentiometer itself, 10 samples is fine.
			total = total + tmp - values[i];
			values[i] = tmp;
			target = total / 10;

			// Calculate the setpoint, based on min/max, and chop it to one decimal point.
			ftmp2 = MINSET + ((MAXSET-MINSET) * (target/1023.0));
			ftmp2 = ((int) (ftmp2*10))/10.0;
			ftmp = ftmp2;
			  
			uint32_t tempSP = ftmp * 1000;
			THCNex.writeNum("TV.val", 000.000);
			THCNex.writeNum("SP.val", tempSP);

			i = (i + 1) % 10;

			if (!digitalReadFast(thcOnSwitch)){
				thcOn=true;    
			}
		}

		// Convert the voltage target back into an int, for ADC comparison, with the scale the plasma pin uses.
		target = ftmp / SCALE;

		// Before carrying on, we now reset some of those variables.
		for (i = 0; i < BUFSIZE; i++)
			values[i] = 0;

		total = 0;
		i = 0;
		j = 1; // Keeps display from triggering until we've done BUFSIZE samples.
	}

	tmp = analogRead(arcVoltageInput);
	disp += tmp; // non-rolling tally for the lower sample rate display

	// Rolling window for a smaller sample
	total = total + tmp - values[i];
	values[i] = tmp;

	// This mean truncates downwards. Oh well. At least it's fast.
	mean = total >> shift;
	diff = mean - target;

	if (digitalReadFast(thcOnSwitch))
	{
		thcOn=false;
		digitalWriteFast(thcOnLedPin,LOW);
	}
	else
	{
		thcOn=true;
		digitalWriteFast(thcOnLedPin,HIGH);
	}

	THCNex.NextionListen();


	if (thcOn)
	{
		//Starts THC when thcOnSwitch signal is true
		// If the mean is very low, then the plasma is turned off - it's just ADC
		// noise you're seeing and it and should be ignored.
		// This effectively checks if it's less than 2^4, ie. 16 counts, or ~3V 
		// with my scale factor.
		// mode 0 = mean is low
		if (!(mean>>4)) 
		{
			mode = 0;
			digitalWriteFast(raiseTorchLedPin, LOW);
			digitalWriteFast(raiseTorchPin, LOW);

			digitalWriteFast(lowerTorchLedPin, LOW);
			digitalWriteFast(lowerTorchPin, LOW);
		}
		// Otherwise, set pins as per reading.
		// Set 0's first to turn off one direction before turning on reverse.
		// We should never have both the UP and DOWN pins set to 1 - that would be nonsense.
		// Checking for current 'mode' setting before flipping saves a few cycles.
		// mode 2 = torch voltage is high so send lower signal
		else if (diff > THRESH) 
		{
			if (mode != 2) 
			{
				mode = 2;
				digitalWriteFast(raiseTorchLedPin, LOW);
				digitalWriteFast(raiseTorchPin, LOW);  

				digitalWriteFast(lowerTorchLedPin, HIGH);
				digitalWriteFast(lowerTorchPin, HIGH);
			}
		}
		// mode 1 = torch voltage is low so send raise signal
		else if (diff < -THRESH) 
		{
			if (mode != 1) 
			{
				mode = 1;

				digitalWriteFast(raiseTorchLedPin, HIGH);
				digitalWriteFast(raiseTorchPin, HIGH);  

				digitalWriteFast(lowerTorchLedPin, LOW);
				digitalWriteFast(lowerTorchPin, LOW);
			}
		}
		else 
			{
				mode = 0;
				digitalWriteFast(raiseTorchLedPin, LOW);
				digitalWriteFast(raiseTorchPin, LOW);

				digitalWriteFast(lowerTorchLedPin, LOW);
				digitalWriteFast(lowerTorchPin, LOW);
			}
	}
	//if thcOn is false all LEDs 
	else 
	{
		digitalWriteFast(raiseTorchLedPin, LOW);
		digitalWriteFast(raiseTorchPin, LOW);

		digitalWriteFast(lowerTorchLedPin, LOW);
		digitalWriteFast(lowerTorchPin, LOW);
		mode = 0;
	}

	// Every DISP reads, update what's displayed on the screen with a slower average.
	// This would be roughly 5 or 6 times per second at our current speeds.
	if (!j) 
	{
		//lcd.setCursor(0, 1);
		//lcd.print("Arc ");
		//lcd.print((float) ((disp / DISP) * SCALE));
		//lcd.setCursor(10, 1);
		//lcd.print(" Volts");
		uint32_t tempTV = ((disp / DISP) * SCALE) * 1000;
		uint32_t tempSP = ftmp * 1000;
		THCNex.writeNum("TV.val", tempTV);
		THCNex.writeNum("SP.val", tempSP);


		disp = 0;
	}

	// Code below resets i and j to zero once SAMP and DISP are about to be reached 
	// Faster than modular arithmetic, by far. Doing that drops us down to ~3kS/sec.
	i = (i + 1) & (SAMP - 1);
	j = (j + 1) & (DISP - 1);

	//unsigned int cycles = TCNT1; //for speed testing
	//  Serial.print("Cycles: ");
	//  Serial.println(cycles - 1);
	//  Serial.print("Microseconds: ");
	//  Serial.println((float)(cycles - 1) / 16);

}
