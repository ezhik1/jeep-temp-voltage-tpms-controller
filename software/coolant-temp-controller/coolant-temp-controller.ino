#include "EEPROM_FLASH.h" // RP2040 does not support EEPROM natively, nor SAMD EEPROM emulation

#include <ArduinoJson.h>
#include <ezButton.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Lato_Thin_30.h"
#include "Lato_Thin_12.h"

#define OLED_RESET -1 // [ ALIAS ] analog pin to reset the OLED display\

// #define DEBUG

// Sensor Input Pins

// DEBUG analog pin to read simulated temperature (use potentiometer on this pin)
#ifdef DEBUG
	#define AC_STATE_PIN A3 // [ ALIAS ] analog pin to read simulated temperature. borrow A/C state pin b/c XIAO  has limited analog pins
#else
	#define AC_STATE_PIN D3 // [ ALIAS ] digtial pin to read A/C fan state
#endif

#define TEMP_SENSE_PIN D8 // [ INT ] digital pin to read Temperature data

// User Input Pins
#define ENTER_BUTTON_PIN D0 // [ INT ]  digital read pin for button inputs
#define DOWN_BUTTON_PIN D1 // [ INT ]  digital read pin for button inputs
#define UP_BUTTON_PIN D2 // [ INT ]  digital read pin for button inputs
#define LOW_FAN_SPEED_OVERRIDE_PIN D6 // [ INT ]  digital input pin for LOW speed override
#define HIGH_FAN_SPEED_OVERRIDE_PIN D7 // [ INT ]  digital imput pin for HIGH speed override

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
#define EASE_COEFFICIENT 0.15 // [ DECIMAL ] : between 0-1 strength with which to dampen title scrolling, 0: strong damping 1: no damping

Adafruit_SSD1306 displayPrimary( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );
Adafruit_SSD1306 displayRemote( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );

OneWire oneWire( TEMP_SENSE_PIN );
DallasTemperature temperatureSensor( &oneWire );
ezButton enterButton( ENTER_BUTTON_PIN );
ezButton upButton( UP_BUTTON_PIN );
ezButton downButton( DOWN_BUTTON_PIN );

// States

// UI States
bool redraw = true;
bool advanceGraph = false;
bool alert = false;
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

// Stateful variables
unsigned long previousMillisGraphAnimation = 0;
unsigned long previousMillisNoTempAlert = 0;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
unsigned long editModeTime = 0;
unsigned long showPrimayDisplayTime = 0;

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

	#ifdef DEBUG
		// DEBUG set analog read resolution to 12 bits (0-4095)
		analogReadResolution(12);
	#endif

	temperatureSensor.begin();
	enterButton.setDebounceTime( 10 );
	upButton.setDebounceTime( 10 );
	downButton.setDebounceTime( 10 );

	displayPrimary.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	wipeDisplay( displayPrimary );

	displayRemote.begin(SSD1306_SWITCHCAPVCC, 0x3D);
	wipeDisplay( displayRemote );

	pinMode(AC_STATE_PIN, INPUT);
	pinMode(TEMP_SENSE_PIN, INPUT);

	pinMode(LOW_FAN_SPEED_PIN, OUTPUT);
	pinMode(HIGH_FAN_SPEED_PIN, OUTPUT);

	Serial.begin( 115200 );

	EEPROM_FLASH::begin();

	storedLowSpeedTriggerTemperature = EEPROM_FLASH::read( 0 );
	storedHighSpeedTriggerTemperature = EEPROM_FLASH::read( 1 );

	lowSpeedTriggerTemperature =  ( storedLowSpeedTriggerTemperature != 255 ) ? storedLowSpeedTriggerTemperature : 93;// [ DEGREES ] : ( Celcius ) : when to kick on low speed relay
	highSpeedTriggerTemperature = ( storedHighSpeedTriggerTemperature != 255 ) ? storedHighSpeedTriggerTemperature : 105; // || 105;// [ DEGREES ] : ( Celcius ) : when to kick on high speed relay
	optimalTemperature = constrain( lowSpeedTriggerTemperature - 2, MIN_DISPLAY_TEMPERATURE, highSpeedTriggerTemperature ); // [ DEGREES ] : ( Celcius ) : the target to which fans should cool before turning off

	runSplashScreen( displayPrimary );
	runSplashScreen( displayRemote );

	currentDisplayReading = 0;
}

void runSplashScreen( Adafruit_SSD1306 &display ){

	rollingTitle(F("TEMP CONTROL"), display );
	wipeDisplay( display );

	delay( 500 );
}

void loop(){

	calculateCoolantTemperature();
	determineFanRelayState();
	digitalWrite( LOW_FAN_SPEED_PIN, lowSpeedFanShouldRun );
	digitalWrite( HIGH_FAN_SPEED_PIN, highSpeedFanShouldRun );

	listenToButtonPushes();
	updateDisplayReading();
	advanceAnimationTicks();

	StaticJsonDocument<100> jsonOut;

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


	if( showPrimayDisplay || isOverHeating ){

		render( displayPrimary );
	}else if( wipeOnce  ){

		wipeDisplay( displayPrimary );
		wipeOnce = false;
	}

	render( displayRemote );
}

void render( Adafruit_SSD1306 &display ){

	displayNumericScrollView( display );
	displayAnimatedFan( display );
	display.display();

	if( redraw ){

		redraw = !redraw;
	}
}

void advanceAnimationTicks(){

	unsigned long currentMillis = millis();
	advanceGraph = false;

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
		alert = !alert;
	}
}

void updateDisplayReading(){

	currentDisplayReading = lerp( currentDisplayReading, targetDisplayReading,  EASE_COEFFICIENT );

	bool targetCloseEnoughToCurrent = ( targetDisplayReading - currentDisplayReading < 0.05 ) || ( currentTemperatureReading > targetDisplayReading );

	if( targetCloseEnoughToCurrent ){

		targetDisplayReading = currentTemperatureReading;
	}
}

void determineFanRelayState(){

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

	isTemperatureReadingValid = currentTemperatureReading > SENSOR_MIN_TEMPERATURE;
	isOverHeating = currentTemperatureReading >= OVERHEAT_TEMPERATURE;
	wipeOnce = isOverHeating ? true : wipeOnce;

	// handle manual overrides first
	highSpeedFanShouldRun = isHighOverride = !digitalRead( HIGH_FAN_SPEED_OVERRIDE_PIN );
	lowSpeedFanShouldRun = isLowOverride = !digitalRead( LOW_FAN_SPEED_OVERRIDE_PIN );

	// Manual Override, set relevant flags, and bail early
	if( isLowOverride || isHighOverride ){

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

		highSpeedFanShouldRun = !( lowSpeedFanShouldRun = false );
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

void listenToButtonPushes(){

	bool isButtonPressed = false;
	bool isButtonReleased = false;

	enterButton.loop();
	upButton.loop();
	downButton.loop();

	if(( isEditingLowSpeedTrigger || isEditingHighSpeedTrigger ) && ( millis() - editModeTime ) > LEAVE_EDIT_MODE_TIME ) {

		isEditingLowSpeedTrigger = false;
		isEditingHighSpeedTrigger = false;
	}

	if( showPrimayDisplay && ( millis() - showPrimayDisplayTime ) > PRIMARY_DISPLAY_SHOW_TIME ) {

		showPrimayDisplay = false;
		wipeOnce = true;
	}

	if( enterButton.isPressed() ){

		buttonPressed = 1;
		isButtonPressed = true;
	}else if( upButton.isPressed() ){

		buttonPressed = 2;
		isButtonPressed = true;
	} else if( downButton.isPressed() ){

		buttonPressed = 3;
		isButtonPressed = true;
	}

	if( isButtonPressed > 0 ){

		pressedTime = millis();
		isPressing = true;
		isLongPressDetected = false;
	}

	if( enterButton.isReleased() ) {

		buttonReleased = 1;
		isButtonReleased = true;
	}else if( upButton.isReleased() ) {

		buttonReleased = 2;
		isButtonReleased = true;
	}else if( downButton.isReleased() ) {

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

	// Async request for temperature
	temperatureSensor.setWaitForConversion( false );
	temperatureSensor.requestTemperatures();
	temperatureSensor.setWaitForConversion( true );

	#ifdef DEBUG
		// DEBUG : simulate temp reading
		float reading = map( analogRead( AC_STATE_PIN ), 0, 4095, 0, MAX_DISPLAY_TEMPERATURE ); // read from a potentiometer
	#else
		// actual temp reading
		float reading = temperatureSensor.getTempCByIndex(0);
	#endif


	if( reading < SENSOR_MIN_TEMPERATURE ){

		currentTemperatureReading = reading;
		return;
	}

	rawTotal -= readings[ readIndex ];
	readings[ readIndex ] = reading; // actual as-measured
	// readings[ readIndex ] = 100.0; // DEBUG -> fixed temp value
	// readings[ readIndex ] = ( random()%3 == 0 ) ? MIN_DISPLAY_TEMPERATURE : MAX_DISPLAY_TEMPERATURE; // DEBUG -> random raw boost value

	rawTotal += readings[ readIndex ];
	readIndex = ( readIndex + 1 ) % NUM_READINGS; // wrap if at end of total samples

	if( readIndex >= NUM_READINGS ){

		readIndex = 0;
	}

	float rawValue = rawTotal / NUM_READINGS;
	currentTemperatureReading = constrain( rawValue, SENSOR_MIN_TEMPERATURE, MAX_DISPLAY_TEMPERATURE );

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

	// Large Numeric Display
	drawNumeric( 0, 21, 0, TEMP_LABEL, display );

	// Rolling Bar Chart
	drawGraph( 73, 0, 50, display );
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

	if(
		!isTemperatureReadingValid &&
		!( lowSpeedFanShouldRun || highSpeedFanShouldRun )
	){

		display.setTextColor( !alert );
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
	}else{

		// interpret cooling state
		display.setFont( &Lato_Thin_12 );
		display.setCursor( 7, yOffset + 28 );
		display.setTextColor( WHITE );

		// fans are in manual override
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

	display.fillCircle( DISPLAY_WIDTH / 2 + 19, yOffset + 20, 7, ( highSpeedFanShouldRun ));
	display.fillCircle( DISPLAY_WIDTH / 2 + 19, yOffset + 35, 7, ( lowSpeedFanShouldRun ));

	// paint high and low letters, regardless of state, since they're always black, and a run state introduces a white-filled circle
	display.setCursor( DISPLAY_WIDTH / 2 + 15,  yOffset + 24 );
	display.setTextColor( BLACK );
	display.print("H");

	display.setCursor( DISPLAY_WIDTH / 2 + 16,  yOffset + 39 );
	display.setTextColor( BLACK );
	display.print("L");
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
