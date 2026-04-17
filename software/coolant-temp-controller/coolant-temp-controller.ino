#define SSD1306_NO_SPLASH // disable default splash screen

#include "EEPROM_FLASH.h" // RP2040 does not support EEPROM natively, nor SAMD EEPROM emulation
#include <ArduinoJson.h>  // JSON over Serial for telemetry
#include "AnalogLadderButton.h"
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Lato_Thin_30.h"
#include "Lato_Thin_12.h"
#include <math.h>

#define OLED_RESET -1 // [ ALIAS ] analog pin to reset the OLED display\

// #define DEBUG

// Sensor Input Pins

// DEBUG analog pin to read simulated temperature (use potentiometer on this pin)
#ifdef DEBUG
	#define AC_STATE_PIN A1 // [ ALIAS ] analog pin to read simulated temperature. borrow button pin b/c XIAO has limited analog pins
#else
	#define AC_STATE_PIN D6 // [ ALIAS ] digtial pin to read A/C fan state
#endif

#define TEMP_SENSE_PIN D8 // [ INT ] digital pin to read Temperature data

// User Input Pins
#define NUMBER_OF_INPUT_BUTTONS 3 // [ INT ] number of buttons in the resistor ladder
#define ENTER_KEY 1 // [ INT ] button index for enter button on the ezAnalogKeypad
#define UP_KEY 2 // [ INT ] button index for up button on the ezAnalogKeypad
#define DOWN_KEY 3 // [ INT ] button index for down button on the ezAnalogKeypad
#define BUTTONS_INPUT_PIN A2 // [ INT ]  digital read pin for all button inputs ADC resistor laddder

#define ACC_SIGNAL D7 // [ INT ]  digital read pin for determining vehicle accessory power state
#define POWER_HOLD_PIN D0 // [ INT ]  digital output pin for holding relay to maintain power when ACC signal is lost
#define LOW_FAN_SPEED_OVERRIDE_PIN D1 // [ INT ]  digital input pin for LOW speed override
#define HIGH_FAN_SPEED_OVERRIDE_PIN D3 // [ INT ]  digital input pin for HIGH speed override

// Controller Output Pins
#define LOW_FAN_SPEED_PIN D9 // [ INT ]  digital out pin for LOW speed fan trigger
#define HIGH_FAN_SPEED_PIN D10 // [ INT ]  digital out pin for HIGH speed fan trigger

// Display, UI Characteristics
#define DISPLAY_WIDTH 128 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 64 // [ PIXELS ] number of available vertical pixels
#define SCROLLING_GRAPH_HEIGHT 30 // [ PIXELS ] vertical size of the historical graph
#define SCROLLING_GRAPH_SAMPLE_SIZE 27 // [ INT ] number of readings displayed in the historical graph
#define TEMP_LABEL "'C" // temperature unit label
#define LOW_LABEL "LOW"
#define HIGH_LABEL "HIGH"
#define NUM_READINGS 3 // [ INT ] number of readings that contribute to a rolling average of RAW VOLTAGE
#define FAN_ANIMATION_INTERVAL 8 // [ MILLISECONDS ] before updating fan animation
#define GRAPH_ANIMATION_INTERVAL 500 // [ MILLISECONDS ] to animate historical graph and record another datum
#define NO_TEMPERATURE_ALERT_INTERVAL 400 // [ MILLISECONDS ] between alert flashes
#define LEAVE_EDIT_MODE_TIME 5000 // [ MILLISECONDS ] before disabling editMode due to inactivity
#define PRIMARY_DISPLAY_SHOW_TIME 30000 // [ MILLISECONDS ] before wiping off primary display
#define EASE_COEFFICIENT 0.15 // [ DECIMAL ] : between 0-1 strength with which to dampen title scrolling, 0: strong damping 1: no damping
#define LAST_LINE_INDENT 10 // [ PIXELS ] default padding for next line printed, used by slowType()

// User Input Button Behaviour
#define MAX_SHORT_PRESS_TIME 500 // [ MILLISECONDS ] : before a short press is no longer 'short'
#define MIN_LONG_PRESS_TIME 1000 // [ MILLISECONDS ] : before a press is considered 'long'

// Voltage Measurements and Display Characteristics
#define SENSOR_MAX_TEMPERATURE 125.0 // [ DEGREES ] : ( Celcius ) : maximum temperature reported from sensor
#define SENSOR_MIN_TEMPERATURE -55.0 // [ DEGREES ] : ( Celcius ) : minimum temperature reported from sensor
#define MIN_DISPLAY_TEMPERATURE 60.0  // [ DEGREES ] : ( Celcius ) : minimum temperature displayed ( 140F )
#define MAX_DISPLAY_TEMPERATURE 120.0 // [ DEGREES ] : ( Celcius ) : maximum temperature displayed ( 250F )
#define OVERHEAT_TEMPERATURE 112 // [ DEGREES ] : ( Celcius ) : maximum safe operating temperature ( 239F )
#define OPERATING_TEMPERATURE 90.5 // [ DEGREES ] : ( Celcius ) : normal operating temperature, also the thermostat trigger temp ( 195F )

// Sensor Configuration and Timing
#define TEMPERATURE_SENSOR_RESOLUTION 9 // [ BITS ] : resolution of temperature sensor, higher is slower but more accurate, 9-12 bits available for this sensor
#define TEMPERATURE_SENSOR_READ_INTERVAL 750 // [ MILLISECONDS ] : interval at which to read the temperature sensor
// #define TEMP_CONVERSION_MS 1000 // [ MILLISECONDS ] : time it takes for the temperature sensor to perform a conversion
#define NUMBER_OF_BAD_READINGS_THRESHOLD 10 // [ INT ] : number of consecutive bad readings before we alert the user to no temperature data and set isTemperatureReadingValid

Adafruit_SSD1306 displayPrimary( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );
Adafruit_SSD1306 displayRemote( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );

OneWire oneWire( TEMP_SENSE_PIN );
DallasTemperature temperatureSensor( &oneWire );

// analogRead at 10 bit resolution reads on button ladder yield raw values:
// no-pres: 1023, enter: 679, down: 509, up: 4

// improved resolution by using 12 bit reads and adjusting resistor values to yield more distinct readings:
// no-press: 4000, enter: 2716, down: 2040, up: 18

int thresholds[ NUMBER_OF_INPUT_BUTTONS ] = { 18, 2040, 2716 }; // raw values at which the buttons are triggered, in ascending order, not including the no-press state ( 4000 ) and should be calibrated to a specific resistor ladder and analog read resolution
AnalogLadder ladder( BUTTONS_INPUT_PIN, thresholds, NUMBER_OF_INPUT_BUTTONS ); // analog pin, array of thresholds, number of thresholds

AnalogLadderButton upButton( ladder, 1 ); // analog pin, button ID, low threshold, high threshold
AnalogLadderButton downButton( ladder, 2 );
AnalogLadderButton enterButton( ladder, 3 );

// forward declarations
void drawProgressFilledCircle(
	Adafruit_SSD1306 &display,
	int16_t x, int16_t y,
	int16_t radius,
	float progress,
	bool invert,
	uint16_t color
);

void drawDonutSegmentsAA(
	Adafruit_SSD1306 &display,
	int cx, int cy,
	int innerRadius,
	int thickness,
	int maxSegments,
	int activeSegments,
	float startDeg,
	float spanDeg,
	float gapDeg
);

enum VehicleState {
	STATE_RUN,       // ACC ON, normal operation
	STATE_COOLDOWN,  // ACC just turned OFF, active cooling
	STATE_SLEEP,     // cooling complete, system waiting
	STATE_OFF        // fully shut down, no cooling allowed
};

VehicleState vehicleState = STATE_OFF;

bool isVehicleOn = false;
bool previouIsVehicleOn = false;

byte coolDownExtensions = 0;
byte vehiclePowerCycles = 0;

unsigned long stateStartMillis = 0;
unsigned long cooldownEndMillis = 0;
uint32_t cooldownStartMillis;

const unsigned long COOLDOWN_TIME_MS = 5000;
const unsigned long EXTEND_TIME_MS   = 2000;

const byte MAX_EXTENSIONS = 4;

const float OPERATING_TEMP = 90.5;
const float OVERHEAT_TEMP  = 112;

byte remainingExtensions;
byte maxExtensionsThisCycle = 0;

// States

// UI States
bool redraw = true;
bool advanceGraph = false;
bool noTemperatureDataAlertUI = false;
bool isPressing = false;
bool isLongPressDetected = false;
bool isEditingLowSpeedTrigger = false;
bool isEditingHighSpeedTrigger = false;
bool fanUIShouldSpin = false;
bool showPrimayDisplay = true;
bool wipeOnce = true;

// Controller States
bool lowSpeedFanShouldRun = false;
bool highSpeedFanShouldRun = false;
bool isBufferCooling = false;
bool shouldBufferCoolHigh = false;
bool shouldBufferCoolLow = false;
bool isHighOverride = false;
bool isLowOverride = false;
bool externalRequestToRunLowSpeed = true;
bool isTemperatureReadingValid = false;
bool isOverHeating = false;
bool shouldKeepControllerAlive = false;
bool accRising = false;
bool accFalling = false;

// Sensor States
bool shouldReadTemperature = false;

// Stateful variables
unsigned long previousMillisGraphAnimation = 0;
unsigned long previousMillisNoTempAlert = 0;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
unsigned long editModeTime = 0;
unsigned long showPrimayDisplayTime = 0;
byte lastLine = 0; // [ PIXELS ] used to track where the last line of text was printed for slowType() and provide consistent spacing for new lines

unsigned long previousMillisTempRead = 0;
unsigned long tempRequestDelayMillis = 0;
byte numberOfBadReadings = 0; // [ INT ] count of consecutive bad readings from the sensor, used to determine when to alert user of no temperature data

StaticJsonDocument<100> jsonOut;

byte buttonPressed = 0;
byte buttonReleased = 0;

float currentTemperatureReading = 0; // [ DEGREES ] : ( Celcius ) ( instantaneous ) between MIN_DISPLAY_TEMPERATURE and MAX_DISPLAY_TEMPERATURE
float currentDisplayReading = 0; // [ DEGREES ] : ( Celcius ) : ( smoothed ) between MIN_DISPLAY_TEMPERATURE and MAX_DISPLAY_TEMPERATURE shown on screen
float targetDisplayReading = 0; // [ DEGREES ] : ( Celcius ) : ( smoothed ) between MIN_DISPLAY_TEMPERATURE and MAX_DISPLAY_TEMPERATURE to which currentDisplayReading is approaching
float currentFanRotationAngle = 0; // [ DEGREES ] : target fan rotation angle
float currentFanSpeed = 0; // [ DEGREES ] : current fan speed

byte storedLowSpeedTriggerTemperature;
byte storedHighSpeedTriggerTemperature;
byte lowSpeedTriggerTemperature;
byte highSpeedTriggerTemperature;
byte optimalTemperature;

// Continuosly-Scrolling Vertical Bar Graph Characteristics
char scrollingGraphArray[ SCROLLING_GRAPH_SAMPLE_SIZE ]; // float the bar graph resolution

// Raw data collection from sensor used for averaging signal
float readings[ NUM_READINGS ] ;
char readIndex = 0;
float rawTotal = 0;

const unsigned char warningIcon [] PROGMEM = { // 30x30
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x0c, 0xc0, 0x00,
	0x00, 0x1c, 0xe0, 0x00, 0x00, 0x18, 0x60, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x33, 0x30, 0x00,
	0x00, 0x63, 0x18, 0x00, 0x00, 0xe3, 0x1c, 0x00, 0x00, 0xc3, 0x0c, 0x00, 0x01, 0x83, 0x06, 0x00,
	0x01, 0x83, 0x06, 0x00, 0x03, 0x03, 0x03, 0x00, 0x03, 0x03, 0x03, 0x00, 0x06, 0x00, 0x01, 0x80,
	0x06, 0x00, 0x01, 0x80, 0x0c, 0x03, 0x00, 0xc0, 0x1c, 0x00, 0x00, 0xe0, 0x18, 0x00, 0x00, 0x60,
	0x38, 0x00, 0x00, 0x70, 0x3f, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void setup(){

	analogReadResolution( 12 );

	displayPrimary.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	wipeDisplay( displayPrimary );

	displayRemote.begin(SSD1306_SWITCHCAPVCC, 0x3D);
	wipeDisplay( displayRemote );

	displayPrimary.setTextColor( WHITE );
	displayPrimary.setTextSize( 1 );

	displayRemote.setTextColor( WHITE );
	displayRemote.setTextSize( 1 );

	slowType( F("CONTROLLER STARTING >"), 20, true );

	// Configure Temperature Sensor
	slowType( F("BRINGING ONLINE >"), 20, true );
	temperatureSensor.begin();
	temperatureSensor.setResolution( TEMPERATURE_SENSOR_RESOLUTION );
	temperatureSensor.setWaitForConversion( false );
	temperatureSensor.requestTemperatures();
	slowPrintSuccessOrFail( temperatureSensor.getDeviceCount() > 0);

	tempRequestDelayMillis = TEMPERATURE_SENSOR_READ_INTERVAL / ( 1 << ( 12 - TEMPERATURE_SENSOR_RESOLUTION )); // calculate delay based on sensor resolution, per datasheet timing
	previousMillisTempRead= millis();

	slowType( F("CONFIG INPUTS >"), 20, true );

	ladder.begin();

	enterButton.begin();
	upButton.begin();
	downButton.begin();

	Wire.setClock(1000000L);

	// Onboard
	pinMode(BUTTONS_INPUT_PIN, INPUT);

	// Vehicle Signals
	pinMode( AC_STATE_PIN, INPUT );
	pinMode( ACC_SIGNAL, INPUT );
	pinMode( TEMP_SENSE_PIN, INPUT );
	pinMode( LOW_FAN_SPEED_OVERRIDE_PIN, INPUT );
	pinMode( HIGH_FAN_SPEED_OVERRIDE_PIN, INPUT );

	// Outputs
	pinMode( LOW_FAN_SPEED_PIN, OUTPUT );
	pinMode( HIGH_FAN_SPEED_PIN, OUTPUT );
	pinMode( POWER_HOLD_PIN, OUTPUT );

	initializeVehicleState();

	Serial.begin( 115200 );

	EEPROM_FLASH::begin();

	storedLowSpeedTriggerTemperature = EEPROM_FLASH::read( 0 );
	storedHighSpeedTriggerTemperature = EEPROM_FLASH::read( 1 );

	lowSpeedTriggerTemperature =  ( storedLowSpeedTriggerTemperature != 255 ) ? storedLowSpeedTriggerTemperature : 93;// [ DEGREES ] : ( Celcius ) : when to kick on low speed relay
	highSpeedTriggerTemperature = ( storedHighSpeedTriggerTemperature != 255 ) ? storedHighSpeedTriggerTemperature : 105; // || 105;// [ DEGREES ] : ( Celcius ) : when to kick on high speed relay
	optimalTemperature = constrain( lowSpeedTriggerTemperature - 2, MIN_DISPLAY_TEMPERATURE, highSpeedTriggerTemperature ); // [ DEGREES ] : ( Celcius ) : the target to which fans should cool before turning off

	slowPrintSuccessOrFail( lowSpeedTriggerTemperature && highSpeedTriggerTemperature && optimalTemperature );

	currentDisplayReading = 0;
	delay( 1500 );
	wipeDisplay( displayRemote );
	wipeDisplay( displayPrimary );
}

void runSplashScreen( Adafruit_SSD1306 &display ){

	rollingTitle(F("TEMP CONTROL"), display );
	wipeDisplay( display );

	delay( 500 );
}

void loop(){

	isVehicleOn = !digitalRead(ACC_SIGNAL);

	accRising = (!previouIsVehicleOn && isVehicleOn);
	accFalling = (previouIsVehicleOn && !isVehicleOn);

	previouIsVehicleOn = isVehicleOn;

	calculateCoolantTemperature();

	// Update States
	updateVehicleState();
	updatePowerHoldState();
	updateFanRelayState();

	// todo: check if this is safe to remove
	bool lowSpeedFanHeatSoak = ( vehicleState == STATE_COOLDOWN ) && //|| vehicleState == STATE_EXTEND) &&
		(currentTemperatureReading >= OPERATING_TEMPERATURE);



	digitalWrite( LOW_FAN_SPEED_PIN, lowSpeedFanShouldRun || lowSpeedFanHeatSoak );
	digitalWrite( HIGH_FAN_SPEED_PIN, highSpeedFanShouldRun );
	digitalWrite( POWER_HOLD_PIN, shouldKeepControllerAlive );

	listenToButtonPushes();
	updateDisplayReading();
	advanceUpdateTicks();

	render( displayRemote ); // always render to remote display

	if( showPrimayDisplay || isOverHeating ){ // draw to both displays during critical state too

		memcpy( displayPrimary.getBuffer(), displayRemote.getBuffer(), 1024 );
		displayPrimary.display();

	}else if( wipeOnce  ){

		wipeDisplay( displayPrimary );
		wipeOnce = false;
	}
}

void render( Adafruit_SSD1306 &display ){

	displayNumericScrollView( display );
	displayAnimatedFan( display );
	display.display();

	if( redraw ){

		redraw = !redraw;
	}
}

void advanceUpdateTicks(){

	unsigned long currentMillis = millis();
	advanceGraph = false;
	shouldReadTemperature = false;

	bool graphAnimationTick = ( previousMillisGraphAnimation == 0 || currentMillis - previousMillisGraphAnimation > GRAPH_ANIMATION_INTERVAL );

	if( graphAnimationTick ){

		previousMillisGraphAnimation = currentMillis;
		advanceGraph = true;

		int scaledLevel = round((float) SCROLLING_GRAPH_HEIGHT *
			((( currentDisplayReading - MIN_DISPLAY_TEMPERATURE ) / ( MAX_DISPLAY_TEMPERATURE - MIN_DISPLAY_TEMPERATURE ))));

			if( scaledLevel <= 0 ){

			scaledLevel = 0;
		}

		scrollingGraphArray[ 0 ] = scaledLevel;
	}

	if( fanUIShouldSpin ){

		float rate = ( highSpeedFanShouldRun ) ? 12.0 : // should be divisible by 360
			( lowSpeedFanShouldRun ) ? 6.0 : 0.0;  // should be divisible by 360

		currentFanSpeed = currentFanSpeed + ( rate - currentFanSpeed ) * 0.06;
		currentFanRotationAngle += currentFanSpeed;

		if( currentFanSpeed <  0.05 ){

			fanUIShouldSpin = false;
		}
	}

	if( previousMillisNoTempAlert == 0 || currentMillis - previousMillisNoTempAlert > NO_TEMPERATURE_ALERT_INTERVAL ){

		previousMillisNoTempAlert = currentMillis;
		noTemperatureDataAlertUI = !noTemperatureDataAlertUI;
	}

	if( previousMillisTempRead == 0 || currentMillis - previousMillisTempRead > tempRequestDelayMillis ){

		previousMillisTempRead = currentMillis;
		shouldReadTemperature = true;
	}
}

void writeToSerial(){

	jsonOut[ "currentTemperatureReading" ] = currentTemperatureReading;
	jsonOut[ "lowSpeedFanShouldRun" ] = lowSpeedFanShouldRun;
	jsonOut[ "highSpeedFanShouldRun" ] = highSpeedFanShouldRun;
	jsonOut[ "isBufferCooling" ] = isBufferCooling;
	jsonOut[ "isHighOverride" ] = isHighOverride;
	jsonOut[ "isLowOverride" ] = isLowOverride;
	jsonOut[ "externalRequestToRunLowSpeed" ] = externalRequestToRunLowSpeed;
	jsonOut[ "isTemperatureReadingValid" ] = isTemperatureReadingValid;
	jsonOut[ "lowSpeedTriggerTemperature" ] = lowSpeedTriggerTemperature;
	jsonOut[ "highSpeedTriggerTemperature" ] = highSpeedTriggerTemperature;
	jsonOut[ "isOverHeating" ] = isOverHeating;

	serializeJson( jsonOut, Serial );
	Serial.println(); // new line to separate json entries
}

void updateDisplayReading(){

	currentDisplayReading = lerp( currentDisplayReading, targetDisplayReading,  EASE_COEFFICIENT );

	bool targetCloseEnoughToCurrent = ( targetDisplayReading - currentDisplayReading < 0.05 ) || ( currentTemperatureReading > targetDisplayReading );

	if( targetCloseEnoughToCurrent ){

		targetDisplayReading = currentTemperatureReading;
	}
}

void updateFanRelayState(){

	// Flags for Controller and UI States

	// UI
	// fanUIShouldSpin -- Fan Animation

	// Controller
	// highSpeedFanShouldRun -- Run HIGH speed fan relay
	// lowSpeedFanShouldRun	 -- Run LOW speed fan relay
	// shouldBufferCoolLow -- Run LOW speed fan relay until OPTIMAL temperature is reached
	// shouldBufferCoolHigh -- Run HIGH speed fan relay until OPTIMAL temperature is reached


	//NOTE: Inputs are pulled up to 3.3V, so invert logic for digitalRead()
	#ifdef DEBUG
		// DEBUG disble digital reads to allow simulated temperature (analog read from same pin)
		externalRequestToRunLowSpeed = false;
	#else
		// whether we should run the fans on external user or ECU request
		externalRequestToRunLowSpeed = !digitalRead( AC_STATE_PIN );
	#endif

	isOverHeating = isTemperatureReadingValid && currentTemperatureReading >= OVERHEAT_TEMPERATURE;
	wipeOnce = isOverHeating ? true : wipeOnce;

	// handle manual overrides first
	highSpeedFanShouldRun = isHighOverride = !digitalRead( HIGH_FAN_SPEED_OVERRIDE_PIN );
	lowSpeedFanShouldRun = isLowOverride = !digitalRead( LOW_FAN_SPEED_OVERRIDE_PIN );

	// Manual Override, set relevant flags, and bail early
	if( ( isLowOverride || isHighOverride ) && vehicleState != STATE_COOLDOWN ){

		fanUIShouldSpin = true;
		return;
	}

	// Buffered cooling should trigger when either a LOW or HIGH fan event is triggered
	// This should continue in the highest event reached until coolant returns to the optimal temperature.
	if(( shouldBufferCoolHigh || shouldBufferCoolLow ) && currentTemperatureReading <= optimalTemperature ){

		shouldBufferCoolHigh = shouldBufferCoolLow = false;
	}

	// cooldown state is buffering to a target lower than the threshold
	isBufferCooling =
		( shouldBufferCoolHigh && currentTemperatureReading < highSpeedTriggerTemperature ) ||
		( shouldBufferCoolLow && currentTemperatureReading < lowSpeedTriggerTemperature );

	// RUN HIGH SPEED
	// - specific fan speed threshold reached
	// - HIGH speed is already triggerd and should cool below the LOW trigger before turning off
	if( currentTemperatureReading >= highSpeedTriggerTemperature || shouldBufferCoolHigh  ){

		// fan activation is mutually exclusive. We also care bout the vehicle state here
		// during cooldown, only the lowspeed fan should be allowed to spin
		highSpeedFanShouldRun = !( lowSpeedFanShouldRun = vehicleState == STATE_COOLDOWN ? true : false );
		shouldBufferCoolHigh = true; // Controller flag for buffered cooling at HIGH speed to OPTIMAL temperature
		fanUIShouldSpin = true; // UI flag for Fan Animation

	// RUN LOW SPEED
	// - specific fan speed threshold reached
	// - external source requested and system is warm enough to supplement cooling
	// - temp sensor is not reporting, but ECU is requesting supplemental cooling ie. A/C trigger
	// - LOW speed is already triggerd and should cool below the LOW trigger before turning off
	}else if(
		( currentTemperatureReading >= lowSpeedTriggerTemperature ) ||
		( externalRequestToRunLowSpeed && currentTemperatureReading > optimalTemperature ) ||
		( externalRequestToRunLowSpeed && !isTemperatureReadingValid ) ||
		shouldBufferCoolLow
	){

		highSpeedFanShouldRun = !( lowSpeedFanShouldRun = true );
		shouldBufferCoolLow = true;
		fanUIShouldSpin = true;
	}
}

byte computeMaxExtensions( float temperature ){

	if( temperature  < OPERATING_TEMPERATURE ) return 0;

	// linear ramp between operating and overheat
	float t = ( temperature  - OPERATING_TEMPERATURE ) / ( OVERHEAT_TEMPERATURE - OPERATING_TEMPERATURE );

	t = clamp01(t);

	// scale to your max allowed
	return (byte)roundf( t * MAX_EXTENSIONS );
}


// For Heat Soak to occur, the following conditions must be met:
// - Vehicle has gone through at least one power cycle (ACC key on, regardless of running condition)
// - Vehicle is currently powered off (ACC off)
// - Coolant Temperature is at or above OPERATING temperature

// To prevent Heat Soak, when a plausible condition exists, the following
// are monitored to substantiate a short or extended cooling session:
// - Coolant Temperature (should presumable grow after shutdown, but serves as primary factor for aggressive or moderate cooling)

// Cooling Algorithm:

// Temperature below OPERATING_TEMPERATURE or never reached OPERATING_TEMPERATURE:
// - NO cooling, monitor for temperature increase
// - todo: maybe If temperature exceeds OPERATING_TEMPERATURE, trigger MODERATE cooling with no extensions.

// Temperature at or above OPERATING_TEMPERATURE:
// - MODERATE cooling where LOW speed Fan is triggered for a fixed duration
// - If temperature remains at or above OPERATING_TEMPERATURE after fixed duration,
// and total cooldown extensions have not exceeded threshold,
// EXTEND cooling duration by triggering LOW speed fan for the maximum configured extensions

// Temperature at or above lowSpeedTriggerTemperature:
// - AGGRESSIVE cooling run for the MAXIMUM_COOLDOWN_RUN_TIME
// absolute run time never exceed fixed duration (say 10 minutes)

void updateVehicleState(){

	static bool prevACC = false;
	static unsigned long accHighSince = 0;

	unsigned long now = millis();

	isVehicleOn = !digitalRead( ACC_SIGNAL );

	bool accRising = ( !prevACC && isVehicleOn );
	bool accFalling = ( prevACC && !isVehicleOn );
	prevACC = isVehicleOn;

	// HARD ENTRY: ACC OFF while RUN → enter COOLDOWN
	if( !isVehicleOn && vehicleState == STATE_RUN ){

		vehicleState = STATE_COOLDOWN;
		stateStartMillis = now;

		// initialize cycle
		cooldownStartMillis = now;
		cooldownEndMillis = now + COOLDOWN_TIME_MS;

		// snapshot extensions at start of cooldown session
		maxExtensionsThisCycle = computeMaxExtensions(currentTemperatureReading);
		remainingExtensions = maxExtensionsThisCycle;
	}

	switch ( vehicleState ){

		case STATE_RUN:

			if( accFalling ){

				vehicleState = STATE_COOLDOWN;
				stateStartMillis = now;

				cooldownStartMillis = now;
				cooldownEndMillis   = now + COOLDOWN_TIME_MS;
			}
			break;

		case STATE_COOLDOWN:

			// ACC returned → abort cooldown
			if (isVehicleOn)
			{
				accHighSince = now;
				vehicleState = STATE_RUN;
				break;
			}


			// Cooling cycle complete
			if( now >= cooldownEndMillis ){

				if( remainingExtensions > 0 ){

					remainingExtensions--;

					// restart a fresh cycle (pie goes FULL again)
					cooldownStartMillis = now;
					cooldownEndMillis   = now + COOLDOWN_TIME_MS;
				}else{

					vehicleState = STATE_SLEEP;
				}
			}

			break;

		case STATE_SLEEP:

			if( isVehicleOn ){

				if( accHighSince == 0 ) accHighSince = now;

				if( now - accHighSince > 300 ){

					vehicleState = STATE_RUN;
				}
			}else{

				accHighSince = 0;
			}
			break;

		case STATE_OFF:

			if( isVehicleOn ){
				vehicleState = STATE_RUN;

				cooldownStartMillis = 0;
				cooldownEndMillis   = 0;

				coolDownExtensions = 0;
				remainingExtensions = 0;
			}
			break;
	}
}

void updatePowerHoldState(){

	shouldKeepControllerAlive =( vehicleState == STATE_RUN) || ( vehicleState == STATE_COOLDOWN);
}

void initializeVehicleState(){

	unsigned long now = millis();

	vehiclePowerCycles = 0;
	coolDownExtensions = 0;

	stateStartMillis = now;
	cooldownEndMillis = 0;

	// read raw ACC once at boot
	isVehicleOn = !digitalRead( ACC_SIGNAL );
	previouIsVehicleOn = isVehicleOn;

	if( isVehicleOn ){

		vehicleState = STATE_RUN;
		vehiclePowerCycles = 1;
	}else{

		// Start in SLEEP instead of OFF (recommended for stability)
		vehicleState = STATE_SLEEP;
	}
}

void listenToButtonPushes(){

	bool isButtonPressed = false;
	bool isButtonReleased = false;

	ladder.update();

	enterButton.update();
	upButton.update();
	downButton.update();

	bool enterButtonIsPressed = enterButton.isPressed();
	bool upButtonIsPressed = upButton.isPressed();
	bool downButtonIsPressed = downButton.isPressed();

	bool enterButtonIsReleased = enterButton.isReleased();
	bool upButtonIsReleased = upButton.isReleased();
	bool downButtonIsReleased = downButton.isReleased();

	if(( isEditingLowSpeedTrigger || isEditingHighSpeedTrigger ) && ( millis() - editModeTime ) > LEAVE_EDIT_MODE_TIME ) {

		isEditingLowSpeedTrigger = false;
		isEditingHighSpeedTrigger = false;
	}

	if( showPrimayDisplay && ( millis() - showPrimayDisplayTime ) > PRIMARY_DISPLAY_SHOW_TIME ) {

		showPrimayDisplay = false;
		wipeOnce = true;
	}

	if( enterButtonIsPressed ){

		buttonPressed = 1;
		isButtonPressed = true;
	}else if( upButtonIsPressed ){

		buttonPressed = 2;
		isButtonPressed = true;
	} else if( downButtonIsPressed ) {
		buttonPressed = 3;
		isButtonPressed = true;
	}

	if( isButtonPressed > 0 ){

		pressedTime = millis();
		isPressing = true;
		isLongPressDetected = false;
	}

	if( enterButtonIsReleased ) {

		buttonReleased = 1;
		isButtonReleased = true;
	}else if( upButtonIsReleased ) {

		buttonReleased = 2;
		isButtonReleased = true;
	}else if( downButtonIsReleased ) {

		buttonReleased = 3;
		isButtonReleased = true;
	}

	if( isButtonReleased ){

		isPressing = false;
		releasedTime = millis();

		if(( releasedTime - pressedTime ) < MAX_SHORT_PRESS_TIME ){

			changeProgramState( buttonPressed, "short" );
		}

		buttonReleased = 0;
		buttonPressed = 0;
	}

	if( isPressing && !isLongPressDetected ) {

		if( ( millis() - pressedTime ) > MIN_LONG_PRESS_TIME ) {

			changeProgramState( buttonPressed, "long" );
			isLongPressDetected = true;
		}
	}
}

void changeProgramState( int buttonIndex, String type ){

	bool isEditMode = ( isEditingLowSpeedTrigger || isEditingHighSpeedTrigger );

	showPrimayDisplayTime = millis();
	showPrimayDisplay = true;

	// Enter Button is long pressed ( enter edit mode )
	if( type == "long" && buttonIndex == 1 && !isEditMode ){

		isEditingLowSpeedTrigger = true;
		editModeTime = millis();
		return;
	}

	// Long press Enter Button to exit edit mode
	if( type == "long" && isEditMode ){

		isEditingLowSpeedTrigger = false;
		isEditingHighSpeedTrigger = false;
		return;
	}

	// EDIT MODE

	if( isEditMode && type == "short"  ){

		editModeTime = millis(); // Keep edit mode alive on any short press during edit mode


		if( buttonIndex == 1 ){ // enter button short press, switch targets

			isEditingLowSpeedTrigger = !isEditingLowSpeedTrigger;
			isEditingHighSpeedTrigger = !isEditingHighSpeedTrigger;
			return;
		}

		int increment = buttonIndex == 2 ? 1 : buttonIndex == 3 ? -1 : 0;

		if(isEditingLowSpeedTrigger ){

			lowSpeedTriggerTemperature += increment;
			constrainValueAndSave( lowSpeedTriggerTemperature, 0, optimalTemperature, highSpeedTriggerTemperature );
		}else{

			highSpeedTriggerTemperature += increment;
			constrainValueAndSave( highSpeedTriggerTemperature, 1, lowSpeedTriggerTemperature, SENSOR_MAX_TEMPERATURE );
		}
	}
}

void constrainValueAndSave( byte &value, byte address, float min, float max ){

	byte valueToSave = constrain( value, min, max );

	value = valueToSave; // keep display value constrained

	updateMemory( address, valueToSave );
}

void updateMemory( byte address, byte value ){

	EEPROM_FLASH::update( address, value );
}

void calculateCoolantTemperature(){

	// debouounce temperature reading to reduce errors and misreads. the sensor can handle about a 750 MS update interval
	if( !shouldReadTemperature ){ return; }


	#ifdef DEBUG
		// DEBUG : simulate temp reading
		float reading = map( analogRead( AC_STATE_PIN ), 0, 4095, 0, MAX_DISPLAY_TEMPERATURE ); // read from a potentiometer
	#else
		// Get latest Temperature Reading from the sensor
		float reading = temperatureSensor.getTempCByIndex(0);

		// Async request for next temperature reading to allow sensor to perform conversion without blocking code execution, per datasheet timing recommendations based on resolution
		temperatureSensor.requestTemperatures();

	#endif

	// Be tolerant of read errors. If more than a set number of readings is recorded, set isTemperatureReadingValid until we get a valid reading.
	if( (int)reading == DEVICE_DISCONNECTED_C ){

		numberOfBadReadings++;

		if( numberOfBadReadings >= NUMBER_OF_BAD_READINGS_THRESHOLD && isTemperatureReadingValid ){

			noTemperatureDataAlertUI = true;
			isTemperatureReadingValid = false;
		}
	}else{

		numberOfBadReadings = 0;
		isTemperatureReadingValid = true;
		noTemperatureDataAlertUI = false;

		rawTotal -= readings[ readIndex ];
		readings[ readIndex ] = reading; // actual as-measured
		// readings[ readIndex ] = 100.0; // DEBUG -> fixed temp value
		// readings[ readIndex ] = ( random()%3 == 0 ) ? MIN_DISPLAY_TEMPERATURE : MAX_DISPLAY_TEMPERATURE; // DEBUG -> random raw boost value

		rawTotal += readings[ readIndex ];
		readIndex = ( readIndex + 1 ) % NUM_READINGS; // wrap if at end of total samples

		if( readIndex >= NUM_READINGS ){

			readIndex = 0;
		}
	}

	float rawValue = rawTotal / NUM_READINGS;
	currentTemperatureReading = constrain( rawValue, SENSOR_MIN_TEMPERATURE, MAX_DISPLAY_TEMPERATURE );
	// writeToSerial(); todo turn on
	delay(1);
}

void rollingTitle( String label, Adafruit_SSD1306 &display ){

	byte destinationY = DISPLAY_HEIGHT/2 + 10;
	float splashScreenTextPosition = 0;
	byte stringLength = label.length();
	byte splashRectWidth = (8 * stringLength) + 14;
	byte isCycle = 0; // 3 complete

	display.setFont(&Lato_Thin_12);
	display.setTextColor(BLACK);
	display.setTextSize(1);

	while( !isEqual( splashScreenTextPosition, destinationY )){

		display.fillRoundRect(( DISPLAY_WIDTH - splashRectWidth )/2, DISPLAY_HEIGHT - splashScreenTextPosition, splashRectWidth,25, 3, BLACK); // clear previous num
		splashScreenTextPosition = lerp( splashScreenTextPosition, destinationY,  EASE_COEFFICIENT );
		display.fillRoundRect(( DISPLAY_WIDTH - splashRectWidth )/2 , DISPLAY_HEIGHT - splashScreenTextPosition, splashRectWidth,25, 3, WHITE); // new rect

		display.setCursor( (0.5 * stringLength) + (DISPLAY_WIDTH - splashRectWidth ) /2, DISPLAY_HEIGHT - splashScreenTextPosition + 17);
		display.println(label);

		display.display();

		if( isCycle == 0 && isEqual( splashScreenTextPosition, destinationY )){

			destinationY = DISPLAY_HEIGHT + 35;
			isCycle++;
		}

		delay(16);
	}

	splashScreenTextPosition = 0;
}

void displayNumericScrollView( Adafruit_SSD1306 &display ){

	// Rolling Bar Chart
	drawGraph( 73, 0, 50, display );

	// Large Numeric Display
	drawNumeric( 0, 21, 0, TEMP_LABEL, display );
}

void drawNumeric( byte xOffset, byte yOffset, byte decimal, String label, Adafruit_SSD1306 &display ){

	display.fillRect (xOffset, 0, 70, yOffset + 5, BLACK); // clear previous num
	display.setFont(&Lato_Thin_30);
	display.setTextColor( WHITE );

	if( !isTemperatureReadingValid ){

		display.drawBitmap( xOffset + 16, yOffset - 25, warningIcon, 30, 30, WHITE );
	}else{

		display.setCursor( xOffset - 20	, yOffset );

		char buffer[4];
		display.print( dtostrf(currentDisplayReading, 4, 0, buffer));
	}

	// Temp Unit Label
	display.setTextSize(1);
	display.setFont( &Lato_Thin_12 );
	display.setCursor( xOffset + DISPLAY_WIDTH/2 - 8, yOffset );
	display.print( label );

	display.setFont();
	display.setCursor( 0, yOffset + 4 );

	display.fillRect (xOffset, yOffset + 3, DISPLAY_WIDTH, 10, BLACK); // clear previous num
	display.fillRect (xOffset, yOffset + 5, ( DISPLAY_WIDTH ) + 8, 40, BLACK); // clear state area

	// clear the HIGH speed indicator circle
	display.fillCircle( DISPLAY_WIDTH / 2 + 19, yOffset + 20, 7, ( highSpeedFanShouldRun ));

	// paint high and low letters, regardless of state, since they're always black, and a run state introduces a white-filled circle
	// note: this is done before the cooldown animation, which also uses the LOW speed circle area,
	// to allow for the cooldown outer ring to not collide with this element
	display.setFont( &Lato_Thin_12 );
	display.setCursor( DISPLAY_WIDTH / 2 + 15,  yOffset + 24 );
	display.setTextColor( BLACK );
	display.print("H");

	drawCooldownAnimation( display, yOffset + 15 );

	// clear the LOW speed indicator circle
	// note: this is done after the cooldown display, which collides with the LOW speed circle area,
	// to allow for the cooldown outer ring to not collide with this element
	display.fillCircle( DISPLAY_WIDTH / 2 + 19, yOffset + 35, 7, ( lowSpeedFanShouldRun ));

	display.setCursor( DISPLAY_WIDTH / 2 + 16,  yOffset + 39 );
	display.setTextColor( BLACK );
	display.print("L");

	// fans are in manual override
	if(
		!isTemperatureReadingValid &&
		!( lowSpeedFanShouldRun || highSpeedFanShouldRun )
	){

		display.setTextColor( !noTemperatureDataAlertUI );
		display.setCursor( 0, yOffset + 22 );
		display.print( F(" NO TEMPERATURE DATA") );
		display.setTextColor( WHITE );

		return; // prevent UI draw collisions
	}else if( isEditingLowSpeedTrigger || isEditingHighSpeedTrigger ){

		display.print( F("SET TRIGGERS") );

		// Temp Unit Label
		display.setFont( &Lato_Thin_12 );
		display.setCursor( 0, yOffset + 26 );
		display.print( " " );
		display.print( LOW_LABEL );
		display.print( "  : " );
		display.print( lowSpeedTriggerTemperature );

		// Temp Unit Label
		display.setCursor( 0, yOffset + 40 );
		display.print( " " );
		display.print( HIGH_LABEL );
		display.print( " : " );
		display.print( highSpeedTriggerTemperature );
	}else if( currentTemperatureReading < OVERHEAT_TEMPERATURE && ( externalRequestToRunLowSpeed || isBufferCooling )){

		display.setFont( &Lato_Thin_12 );
		display.setTextColor( BLACK );

		if( externalRequestToRunLowSpeed ){

			display.setCursor( 15, yOffset + 20 );
			display.fillRoundRect(10, yOffset + 10, 60, 12, 2, WHITE );
			display.print( F("A/C ON") );
		}

		if( isBufferCooling ){
			display.fillRoundRect(10, yOffset + 30, 60, 12, 2, WHITE );
			display.setCursor( 15, yOffset + 40 );
			display.print( F("BUFFER") );
		}
	}else if( vehicleState == STATE_COOLDOWN ){

		display.setFont( &Lato_Thin_12 );
		display.setCursor( 3, yOffset + 28 );
		display.setTextColor( BLACK );

		display.fillRoundRect( 2, yOffset + 15, 72, 18, 2, WHITE );
		display.print( F("HEATSOAK") );

		// communicate the specific strategy for best preventing heat soak
		display.setFont();
		display.setCursor( 2, yOffset + 36 );
		display.setTextColor( WHITE );
		display.print( F("MODE:") );
		display.print( ( (int)remainingExtensions > 0 ) ? F(" EXTEND") : lowSpeedFanShouldRun ? F(" NORMAL") : F("MONITOR") );
	}else{

		display.setFont( &Lato_Thin_12 );
		display.setCursor( 7, yOffset + 28 );
		display.setTextColor( WHITE );

		if( isHighOverride || isLowOverride ){

			display.drawRoundRect( 2, yOffset + 15, 72, 18, 2, WHITE );
			display.print( F("OVERRIDE") );
		// under temp
		}else if( currentTemperatureReading < OPERATING_TEMPERATURE ){

			display.drawRoundRect( 2, yOffset + 15, 68, 18, 2, WHITE );
			display.print( F("HEATING") );

		// system at operating temperature
		}else if( currentTemperatureReading < lowSpeedTriggerTemperature ){

			display.print( F("OPTIMAL") );

		}else if( isOverHeating ){

			display.fillRoundRect( 2, yOffset + 15, 68, 18, 2, WHITE );
			display.setTextColor( BLACK );
			display.setCursor( 5, yOffset + 28 );
			display.print( F("WARNING!") );

		// actively cooling
		}else if( lowSpeedFanShouldRun || highSpeedFanShouldRun ){

			display.drawRoundRect( 2, yOffset + 15, 68, 18, 2, WHITE );
			display.print( F("COOLING") );
		}
	}

	if( isEditingLowSpeedTrigger ){
		display.drawRoundRect(0, yOffset + 15, 40, 14, 2, WHITE );
	}

	if( isEditingHighSpeedTrigger ){
		display.drawRoundRect(0, yOffset + 29, 40, 14, 2, WHITE );
	}
}

void drawCooldownAnimation( Adafruit_SSD1306 &display, byte yOffset ){

if( vehicleState != STATE_COOLDOWN ) { return; }

	float progress = getCooldownProgress();

	// Outer ring to represent extensions remaining vs total available extensions for cooldown
	drawDonutSegmentsAA(
		display,
		DISPLAY_WIDTH / 2 + 20, yOffset + 18,
		9, // radius
		3, // thickness
		MAX_EXTENSIONS,
		remainingExtensions,
		-60, // start angle (top center)
		28, // span of each segment (degrees)
		12 // gap between segments (degrees)
	);

	// Inner filled circle, divided into slices, to represent progress of a current cooldown cycle
	drawProgressFilledCircle(
		display,
		DISPLAY_WIDTH / 2 + 20, yOffset + 18,
		7, // radius
		progress,
		false, // invert
		SSD1306_WHITE
	);
}

void displayAnimatedFan( Adafruit_SSD1306 &display ){

	if( !fanUIShouldSpin  ){

		return;
	}

	byte numberOfFanBlades = 5;
	float angle = 360.0 / (float) numberOfFanBlades;

	byte radius= 14;

	// Define exact pixel-center (sub-pixel to remove rounding drift)
	const float centerX = (DISPLAY_WIDTH  - (radius * 2.0f) + radius ) - 3.0f;
	const float centerY = (DISPLAY_HEIGHT / 2.0f) + radius + 1.0f;

	display.fillCircle(roundf( centerX ), roundf( centerY ), radius + 2, BLACK);
	display.drawCircle(roundf( centerX ), roundf( centerY ), radius + 2, WHITE);
	display.fillCircle(roundf( centerX ), roundf( centerY ), 2, WHITE);

	for( int i = 0; i < numberOfFanBlades; i++ ){

		drawFanBlade( radius, centerX, centerY, (float) i * angle, display );
	}
}

void drawGraph( byte xOffset, byte yOffset, byte scrollingGraphWidth, Adafruit_SSD1306 &display ){

	byte bottomOfGraph = yOffset + SCROLLING_GRAPH_HEIGHT;
	byte centerPoint = bottomOfGraph / 2;

	// skips a line for each tick
	for (byte step = 0; step < SCROLLING_GRAPH_SAMPLE_SIZE; step++ ){

		byte position = scrollingGraphArray[ step ];

		byte start = ( position >= centerPoint ) ? ( bottomOfGraph - position ) : centerPoint;
		byte length = ( position >= centerPoint ) ? centerPoint - start : centerPoint - position;

		display.writeFastVLine( scrollingGraphWidth + xOffset - (step * 2 ), yOffset, SCROLLING_GRAPH_HEIGHT, BLACK );

		if( position > 0 ){ // only draw values that should be in visible range

			display.writeFastVLine( scrollingGraphWidth + xOffset - (step * 2 ), start, length, WHITE ); // Line Datum
		}
	}
	if( advanceGraph ){
		// advanced historical values
		for( byte step2 = SCROLLING_GRAPH_SAMPLE_SIZE; step2 >= 2; step2-- ){

			scrollingGraphArray[ step2 - 1 ] = scrollingGraphArray[ step2 - 2 ];
		}
	}

	if( redraw ){
		// center point of graph
		display.writeFastHLine(scrollingGraphWidth + xOffset + 3, centerPoint - 1, 2, WHITE);
	}
}

void wipeDisplay( Adafruit_SSD1306 &display ){

	display.clearDisplay(); // remove library banner
	display.fillScreen( BLACK ); // I see a red door...
	display.display(); // because fillScreen is misleading
}

bool isEqual(float x, float y){
	return abs(x - y) <= 1e-2 * abs(x);
}

float lerp(float a, float b, float x){
	return a + x * (b - a);
}

void drawFanBlade(float radius, float cx, float cy, float angleOffsetDegrees, Adafruit_SSD1306 &display ) {

	const float hubClearance = 5.0f; // distance from center to blade base
	const float bladeLength = radius - hubClearance - 1; // blade span (pixels)
	const float theta = ( currentFanRotationAngle + angleOffsetDegrees  ) * ( M_PI / 180.0f );

	// tip is further out from center: base + bladeLength
	const float tipDistance = hubClearance + bladeLength;
	const float tipX = cx + tipDistance * cos( theta );
	const float tipY = cy + tipDistance * sin( theta );

	// pass bladeLength so triangle length matches actual blade geometry
	drawRotatedTriangle(1, tipX, tipY, theta, bladeLength, display );
}

void drawRotatedTriangle(int sign, float tipX, float tipY, float theta, float bladeLength, Adafruit_SSD1306 &display ) {

	// Triangle defined relative to the tip. Back distance equals bladeLength.
	const float tX  = -bladeLength; // back toward hub (float to avoid rounding drift)
	const float tY  = 0.0f;
	const float t1X = 0.0f;
	const float t1Y = 3.0f * sign;
	const float t2X = 0.0f;
	const float t2Y = -3.0f * sign;

	// Rotate points around the tip
	const float rtX  =  tX * cos( theta ) - tY * sin( theta );
	const float rtY  =  tX * sin( theta ) + tY * cos( theta );
	const float rt1X = t1X * cos( theta ) - t1Y * sin( theta );
	const float rt1Y = t1X * sin( theta ) + t1Y * cos( theta );
	const float rt2X = t2X * cos( theta ) - t2Y * sin( theta );
	const float rt2Y = t2X * sin( theta ) + t2Y * cos( theta );

	// Round only at draw time for consistency with circle center rounding
	display.drawTriangle(

		roundf( tipX + rtX ),
		roundf( tipY + rtY ),

		roundf( tipX + rt1X ),
		roundf( tipY + rt1Y ),

		roundf( tipX + rt2X ),
		roundf( tipY + rt2Y ),
		WHITE
	);
}

void slowType( String text, int delayTime, bool newLine ){

	if( newLine ){
		displayPrimary.setCursor(0, lastLine );
		displayRemote.setCursor(0, lastLine );
		lastLine += LAST_LINE_INDENT;
	}

	for( byte i = 0; i < text.length(); i++ ){

		displayPrimary.print( text[ i ] );
		displayPrimary.display();

		displayRemote.print( text[ i ] );
		displayRemote.display();
		delay( delayTime );
	}
}

void slowPrintSuccessOrFail( bool condition ){

	condition ? slowType( F("OK"), 50, false ) : slowType( F("FAIL"), 50, false );
}


float getCooldownProgress(){

	uint32_t now = millis();

	if( cooldownEndMillis <= cooldownStartMillis ) return 0.0f;

	uint32_t total = cooldownEndMillis - cooldownStartMillis;

	// BEFORE cooldown : full
	if( now <= cooldownStartMillis ) return 1.0f;

	// AFTER cooldown : empty
	if( now >= cooldownEndMillis ) return 0.0f;

	// DURING cooldown : linear 1 → 0
	uint32_t remaining = cooldownEndMillis - now;

	float progress = (float)remaining / (float)total;

	return clamp01( progres s);
}


void drawProgressFilledCircle(
	Adafruit_SSD1306 &display,
	int16_t cx, int16_t cy,
	int16_t radius,
	float progress,
	bool invert,
	uint16_t color
) {
	progress = clamp01( progress );
	if( invert ) progress = 1.0f - progress;
	if( progress <= 0.0f ) return;

	float drawAngle = progress * TWO_PI;
	int r2 = radius * radius;

	for( int y = -radius; y <= radius; y++ ){

		for( int x = -radius; x <= radius; x++ ){


			float dx = x + 0.5f;
			float dy = y + 0.5f;

			if( dx * dx + dy * dy > r2 + radius * 0.5f ) continue;

			float angle = atan2f( dx, -dy );
			if( angle < 0 ) angle += TWO_PI;

			if( angle <= drawAngle ){

				display.drawPixel( cx + x, cy + y, color );
			}
		}
	}
}

void drawDonutSegmentsAA(
	Adafruit_SSD1306 &display,
	int cx, int cy,
	int innerRadius,
	int thickness,
	int maxSegments,
	int activeSegments,
	float startDeg,
	float spanDeg,
	float gapDeg
) {
	int outerRadius = innerRadius + thickness;

	display.fillCircle( cx, cy, outerRadius, BLACK );

	float startRad = radians( startDeg );
	float spanRad  = radians( spanDe g);
	float gapRad   = radians( gapDeg );

	int minX = cx - outerRadius;
	int maxX = cx + outerRadius;
	int minY = cy - outerRadius;
	int maxY = cy + outerRadius;

	for (int i = 0; i < maxSegments; i++) {

		bool active = (i < activeSegments);
		uint16_t color = active ? WHITE : BLACK;

		// each segment has fixed width
		float a0 = startRad + i * (spanRad + gapRad);
		float a1 = a0 + spanRad;

		// normalize
		while( a0 < 0 ) a0 += TWO_PI;
		while( a1 < 0 ) a1 += TWO_PI;
		while( a0 >= TWO_PI ) a0 -= TWO_PI;
		while( a1 >= TWO_PI ) a1 -= TWO_PI;

		for( int y = minY; y <= maxY; y++ ){

			for( int x = minX; x <= maxX; x++ ){

				float dx = x - cx;
				float dy = y - cy;

				float r2 = dx*dx + dy*dy;

				if( r2 < innerRadius * innerRadius ) continue;
				if( r2 > outerRadius * outerRadius ) continue;

				float angle = atan2f( dx, -dy );

				if( angle < 0 ) angle += TWO_PI;

				bool inAngle;

				if( a0 <= a1 ){

					inAngle = ( angle >= a0 && angle <= a1 );
				} else {

					inAngle = ( angle >= a0 || angle <= a1 );
				}

				if( inAngle ){

					display.drawPixel( x, y, color );
				}
			}
		}
	}
}

inline float normAngle( float a ){

	while( a < 0 ) a += TWO_PI;
	while( a >= TWO_PI ) a -= TWO_PI;
	return a;
}

static inline float clamp01(float v) {

	if( v < 0.0f ) return 0.0f;
	if( v > 1.0f ) return 1.0f;
	return v;
}
