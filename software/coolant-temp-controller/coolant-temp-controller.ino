// Cooland Temperature Controller
// - OLED display
// - Historical reporting
// - Temperature Control:
// -- Fan OFF Temp:
// -- Fan ON:
// ---- LOW SPEED : Temp from Probe [ or ] signal from A/C state
// ---- HIGH SPEED : Temp from Probe

#define BUFFER_LENGTH  8
#include <Wire.h>
#include <EEPROM.h>
#include <ezButton.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Lato_Thin_30.h"
#include "Lato_Thin_12.h"

#define OLED_RESET -1 // [ ALIAS ] analog pin to reset the OLED display

// Sensor Input Pins
#define AC_STATE_PIN A2 // [ ALIAS ] analog pin to read Air Conditioning or car Fan State
#define TEMP_SENSE_PIN 9 // [ INT ] pin to read Temperature data

// User Input Pins
#define ENTER_BUTTON_PIN 2 // [ INT ]  digital read pin for button inputs
#define DOWN_BUTTON_PIN 3 // [ INT ]  digital read pin for button inputs
#define UP_BUTTON_PIN 4 // [ INT ]  digital read pin for button inputs
#define LOW_FAN_SPEED_OVERRIDE_PIN 10 // [ INT ]  digital out pin for HIGH speed fan trigger
#define HIGH_FAN_SPEED_OVERRIDE_PIN 11 // [ INT ]  digital out pin for HIGH speed fan trigger

// Controller Output Pins
#define LOW_FAN_SPEED_PIN 5 // [ INT ]  digital out pin for LOW speed fan trigger
#define HIGH_FAN_SPEED_PIN 6 // [ INT ]  digital out pin for HIGH speed fan trigger

// Display, UI Characteristics
#define DISPLAY_WIDTH 128 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 64 // [ PIXELS ] number of available vertical pixels
#define SCROLLING_GRAPH_HEIGHT 30 // [ PIXELS ] vertical size of the historical graph
#define SCROLLING_GRAPH_SAMPLE_SIZE 25 // [ INT ] number of readings displayed in the historical graph
#define TEMP_LABEL "'C" // temperature unit label
#define LOW_LABEL "LOW"
#define HIGH_LABEL "HIGH"
#define NUM_READINGS 3 // [ INT ] number of readings that contribute to a rolling average of RAW VOLTAGE
#define FAN_ANIMATION_INTERVAL 8 // [ MILLISECONDS ] before updating fan animation
#define GRAPH_ANIMATION_INTERVAL 5000 // [ MILLISECONDS ] to animate historical graph and record another datum
#define LEAVE_EDIT_MODE_TIME 5000 // [ MILLISECONDS ] before disabling editMode due to inactivity

// User Input Button Behaviour
#define MAX_SHORT_PRESS_TIME 500 // [ MILLISECONDS ] : before a short press is no longer 'short'
#define MIN_LONG_PRESS_TIME 1000 // [ MILLISECONDS ] : before a press is considered 'long'

// Voltage Measurements and Display Characteristics
#define SENSOR_MAX_TEMPERATURE 125.0 // [ DEGREES ] : ( Celcius ) : maximum temperature reported from sensor
#define SENSOR_MIN_TEMPERATURE -55.0 // [ DEGREES ] : ( Celcius ) : minimum temperature reported from sensor
#define MIN_TEMPERATURE 60.0  // [ DEGREES ] : ( Celcius ) : minimum temperature displayed ( 140F )
#define MAX_TEMPERATURE 121.11 // [ DEGREES ] : ( Celcius ) : maximum temperature displayed ( 250F )
#define OVERHEAT_TEMPERATURE 112 // [ DEGREES ] : ( Celcius ) : maximum safe operating temperature ( 239F )
#define OPERATING_TEMPERATURE 90.5 // [ DEGREES ] : ( Celcius ) : normal operating temperature, also the thermostat trigger temp ( 195F )
#define EASE_COEFFICIENT 0.15 // [ DECIMAL ] : between 0-1 strength with which to dampen title scrolling, 0: strong damping 1: no damping

Adafruit_SSD1306 display( DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET );
OneWire oneWire( TEMP_SENSE_PIN );
DallasTemperature temperatureSensor( &oneWire );
ezButton enterButton( ENTER_BUTTON_PIN );
ezButton upButton( UP_BUTTON_PIN );
ezButton downButton( DOWN_BUTTON_PIN );

// States
bool redraw = true;
bool advanceGraph = false;
bool isPressing = false;
bool isLongPressDetected = false;
bool isEditingLowSpeedTrigger = false;
bool isEditingHighSpeedTrigger = false;
bool lowSpeedFanShouldRun = false;
bool highSpeedFanShouldRun = false;
bool isBufferCooling = false;
bool shouldBufferCoolHigh = false;
bool shouldBufferCoolLow = false;
bool isCoolingBufferLow = false;
bool isHighOverride = false;
bool isLowOverride = false;
bool externalRequestToRunLowSpeed = true;

// Stateful variables
unsigned long previousMillisGraphAnimation = 0;
unsigned long previousMillisFanAnimation = 0;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
unsigned long editModeTime = 0;
byte buttonPressed = 0;
byte buttonReleased = 0;
float currentTemperatureReading = 0; // [ DEGREES ] : ( Celcius ) ( instantaneous ) between MIN_TEMPERATURE and MAX_TEMPERATURE
float currentDisplayReading = 0; // [ DEGREES ] : ( Celcius ) : ( smoothed ) between MIN_TEMPERATURE and MAX_TEMPERATURE shown on screen
float targetDisplayReading = 0; // [ DEGREES ] : ( Celcius ) : ( smoothed ) between MIN_TEMPERATURE and MAX_TEMPERATURE to which currentDisplayReading is approaching
float angle = 0;  // [ DEGREES ] : current animated fan angle from 0 - 360

byte storedLowSpeedTriggerTemperature = EEPROM.read( 0 );
byte storedHighSpeedTriggerTemperature = EEPROM.read( 1 );

byte lowSpeedTriggerTemperature =  ( storedLowSpeedTriggerTemperature != 255 ) ? storedLowSpeedTriggerTemperature : 93;// [ DEGREES ] : ( Celcius ) : when to kick on low speed relay
byte highSpeedTriggerTemperature = ( storedHighSpeedTriggerTemperature != 255 ) ? storedHighSpeedTriggerTemperature : 105; // || 105;// [ DEGREES ] : ( Celcius ) : when to kick on high speed relay

byte optimalTemperature = lowSpeedTriggerTemperature - 2; // [ DEGREES ] : ( Celcius ) : the target to which fans should cool before turning off

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

	temperatureSensor.begin();
	enterButton.setDebounceTime( 10 );
	upButton.setDebounceTime( 10 );
	downButton.setDebounceTime( 10 );

	pinMode(AC_STATE_PIN, INPUT);
	pinMode(TEMP_SENSE_PIN, INPUT);

	pinMode(LOW_FAN_SPEED_PIN, OUTPUT);
	pinMode(HIGH_FAN_SPEED_PIN, OUTPUT);

	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	wipeDisplay();
	runSplashScreen();
}

void runSplashScreen(){

	rollingTitle(F("TEMP CONTROL"));
	wipeDisplay();
	currentDisplayReading = 0;
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

	render();
}

void render(){

	displayNumericScrollView();
	displayAnimatedFan();
	display.display();

	if( redraw ){
		redraw = !redraw;
	}
}

void advanceAnimationTicks(){

	unsigned long currentMillis = millis();
	advanceGraph = false;

	bool graphAnimationTick = ( previousMillisGraphAnimation == 0 || currentMillis - previousMillisGraphAnimation > GRAPH_ANIMATION_INTERVAL );
	bool fanAnimationTick = ( previousMillisFanAnimation == 0 || currentMillis - previousMillisFanAnimation > FAN_ANIMATION_INTERVAL );

	if( graphAnimationTick ){

		previousMillisGraphAnimation = currentMillis;
		advanceGraph = true;

		int scaledLevel = round(SCROLLING_GRAPH_HEIGHT * (((currentDisplayReading - MIN_TEMPERATURE ) / (MAX_TEMPERATURE- MIN_TEMPERATURE ))));
		if( scaledLevel <= 0 ){
			scaledLevel = 0;
		}

		scrollingGraphArray[ 0 ] = scaledLevel;
	}

	if( fanAnimationTick && ( lowSpeedFanShouldRun || highSpeedFanShouldRun || isLowOverride || isHighOverride )){

		previousMillisFanAnimation = currentMillis;

		float rate = ( highSpeedFanShouldRun || isHighOverride ) ? 18.0 : // should be divisible by 360
			( lowSpeedFanShouldRun || isLowOverride ) ? 8.0 : 0.0;  // should be divisible by 360

		angle = ( angle + rate == 360 ) ? 0 : angle + rate;
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

	// temporarily assume no fan should run
	highSpeedFanShouldRun = lowSpeedFanShouldRun = false;

	// whether we should run the fans on external user or ECU request
	externalRequestToRunLowSpeed = digitalRead( AC_STATE_PIN );
	isLowOverride = digitalRead( LOW_FAN_SPEED_OVERRIDE_PIN );
	isHighOverride = digitalRead( HIGH_FAN_SPEED_OVERRIDE_PIN );

	// Buffered cooling should trigger when either a LOW or HIGH fan event is triggered
	// This should continue in the highest event reached until coolant returns to the optimal temperature.
	// shouldBufferCoolHigh = ( shouldBufferCoolHigh && ( currentTemperatureReading <= optimalTemperature )) ? false : shouldBufferCoolHigh;

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

		if( !isLowOverride ){

			highSpeedFanShouldRun = !( lowSpeedFanShouldRun = false );
			shouldBufferCoolHigh = true;
		}

	// RUN LOW SPEED
	// - specific fan speed threshold reached
	// - external source requested and system is warm enough to supplement cooling
	// - LOW speed is already triggerd and should cool below the LOW trigger before turning off
	}else if(
		( currentTemperatureReading >= lowSpeedTriggerTemperature ) ||
		( externalRequestToRunLowSpeed && currentTemperatureReading > optimalTemperature ) ||
		shouldBufferCoolLow
	){

		if( !isHighOverride ){

			highSpeedFanShouldRun = !( lowSpeedFanShouldRun = true );
			shouldBufferCoolLow = true;
		}
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

	// Enter Button is long pressed ( enter edit mode )
	if( type == "long" && buttonIndex == 1 && !isEditingLowSpeedTrigger ){
		isEditingLowSpeedTrigger = true;
		editModeTime = millis();
	}

	// Long press Enter Button to exit edit mode
	if( type == "long" && buttonIndex == 1 && isEditMode ){
		isEditingLowSpeedTrigger = false;
		isEditingLowSpeedTrigger = false;
		return;
	}

	// EDIT MODE

	// Keep edit mode alive
	if( isEditMode && type == "short"  ){
		editModeTime = millis();
	}

	// EDIT : LOW TRIGGER

	// UP Button is short pressed
	if( isEditingLowSpeedTrigger && type == "short" && buttonIndex == 2 ){

		lowSpeedTriggerTemperature+= 1;
		updateMemory( 0, lowSpeedTriggerTemperature );
	// DOWN Button is short pressed
	}else if( isEditingLowSpeedTrigger && type == "short" && buttonIndex == 3 ){

		lowSpeedTriggerTemperature-= 1;
		updateMemory( 0, lowSpeedTriggerTemperature );
	// ENTER Button is short pressed
	}else if( isEditingLowSpeedTrigger && type == "short" && buttonIndex == 1 ){
		isEditingLowSpeedTrigger = false;
		isEditingHighSpeedTrigger = true;
		return;
	}

	// EDIT : HIGH TRIGGER

	if( isEditingHighSpeedTrigger && type == "short" && buttonIndex == 2 ){

		highSpeedTriggerTemperature += 1;
		updateMemory( 1, highSpeedTriggerTemperature );
	// DOWN Button is short pressed
	}else if( isEditingHighSpeedTrigger && type == "short" && buttonIndex == 3 ){

		highSpeedTriggerTemperature -= 1;
		updateMemory( 1, highSpeedTriggerTemperature );
	}else if( isEditingHighSpeedTrigger && type == "short" && buttonIndex == 1 ){

		isEditingHighSpeedTrigger = false;
		isEditingLowSpeedTrigger = true;
	}
}

void updateMemory( byte address, byte value ){

	EEPROM.update( address, value );
}

void calculateCoolantTemperature(){

	// Async request for temperature
	temperatureSensor.setWaitForConversion( false );
	temperatureSensor.requestTemperatures();
	temperatureSensor.setWaitForConversion( true );

	float reading = temperatureSensor.getTempCByIndex(0);
	// float reading = map( analogRead( AC_STATE_PIN ), 0, 280, 0, 110 ); // DEBUG : simulate temp reading

	if( reading < SENSOR_MIN_TEMPERATURE ){

		currentTemperatureReading = reading;
		return;
	}

	rawTotal -= readings[ readIndex ];
	readings[ readIndex ] = reading; // actual as-measured
	// readings[ readIndex ] = 100.0; // DEBUG -> fixed temp value
	// readings[ readIndex ] = ( random()%3 == 0 ) ? MIN_TEMPERATURE : MAX_TEMPERATURE; // DEBUG -> random raw boost value

	rawTotal += readings[ readIndex ];
	readIndex = ( readIndex + 1 ) % NUM_READINGS; // wrap if at end of total samples

	if( readIndex >= NUM_READINGS ){

		readIndex = 0;
	}

	float rawValue = rawTotal / NUM_READINGS;
	currentTemperatureReading = constrain( rawValue, SENSOR_MIN_TEMPERATURE, MAX_TEMPERATURE );

	delay(1);
}

void rollingTitle( String label ){

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

void displayNumericScrollView(){

	// Large Numeric Display
	drawNumeric( 0, 21, 0, TEMP_LABEL );

	// Rolling Bar Chart
	drawGraph( 73, 0, 49 );
}

void drawNumeric( byte xOffset, byte yOffset, byte decimal, String label  ){

	display.fillRect (xOffset, 0, 70, yOffset + 5, BLACK); // clear previous num
	display.setFont(&Lato_Thin_30);
	display.setTextColor( WHITE );

	if( currentTemperatureReading < SENSOR_MIN_TEMPERATURE ){

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

	if( currentTemperatureReading < SENSOR_MIN_TEMPERATURE ){

		display.setTextColor( !advanceGraph );
		display.setCursor( 0, yOffset + 22 );
		display.print( F(" NO TEMPERATURE DATA") );
		display.setTextColor( WHITE );
		return; // may be safe to remove
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

		// under temp
		if( currentTemperatureReading < OPERATING_TEMPERATURE ){

			display.drawRoundRect( 2, yOffset + 15, 68, 18, 2, WHITE );
			display.print( F("HEATING") );

		// system at operating temperature
		}else if( currentTemperatureReading < lowSpeedTriggerTemperature ){

			display.print( F("OPTIMAL") );

		}else if( currentTemperatureReading > OVERHEAT_TEMPERATURE ){

			display.fillRoundRect( 2, yOffset + 15, 68, 18, 2, WHITE );
			display.setTextColor( BLACK );
			display.setCursor( 5, yOffset + 28 );
			display.print( F("WARNING!") );

		// actively cooling
		}else if( lowSpeedFanShouldRun || highSpeedFanShouldRun || isLowOverride || isHighOverride ){

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

	display.fillCircle( DISPLAY_WIDTH / 2 + 20, yOffset + 20, 7, ( highSpeedFanShouldRun || isHighOverride ));
	display.fillCircle( DISPLAY_WIDTH / 2 + 20, yOffset + 35, 7, ( lowSpeedFanShouldRun || isLowOverride ));

	// paint high and low letters, regardless of state, since they're always black, and a run state introduces a white-filled circle
	display.setCursor( DISPLAY_WIDTH / 2 + 16,  yOffset + 24 );
	display.setTextColor( BLACK );
	display.print("H");

	display.setCursor( DISPLAY_WIDTH / 2 + 17,  yOffset + 39 );
	display.setTextColor( BLACK );
	display.print("L");
}

void displayAnimatedFan(){

	byte numberOfFanBlades = 5;
	byte angle = 360 / numberOfFanBlades;

	int radius= 13;
	int xCenter= radius;
	int yCenter= radius;

	byte xOffset = DISPLAY_WIDTH - ( radius * 2 ) - 4;
	byte yOffset = DISPLAY_HEIGHT / 2 + 3;

	if( lowSpeedFanShouldRun || highSpeedFanShouldRun || isLowOverride || isHighOverride ){

		display.fillCircle(xCenter + xOffset, yCenter + yOffset,radius + 2, BLACK); //  clear out old fan and everything inside it
		display.drawCircle(xCenter + xOffset, yCenter + yOffset,radius + 2, WHITE); // fan perimeter
		display.fillCircle(xCenter + xOffset, yCenter + yOffset, 2, WHITE); // center point

		for( int i = 0; i < numberOfFanBlades; i++ ){
			drawFanBlade( radius, xOffset, yOffset, i * angle );
		}
	}else{

		display.fillCircle( xCenter + xOffset, yCenter + yOffset, radius + 2, BLACK );
	}
}

void drawGraph( byte xOffset, byte yOffset, byte scrollingGraphWidth ){

	byte bottomOfGraph = yOffset + SCROLLING_GRAPH_HEIGHT;
	byte centerPoint = bottomOfGraph / 2;

	// skips a line for each tick
	for (byte step = 0; step < SCROLLING_GRAPH_SAMPLE_SIZE; step++ ){

		byte position = scrollingGraphArray[ step ];

		byte start= ( position > centerPoint ) ? ( bottomOfGraph - position ) : centerPoint;
		byte length = ( position > centerPoint ) ? centerPoint - start : centerPoint - position;

		if( (position > centerPoint || position < centerPoint ) && position > 0 ){

			display.writeFastVLine( scrollingGraphWidth + xOffset - (step * 2 ), start, length, WHITE ); // Line Datum
		}

		// clear previous line datum overflow below centerPoint
		if( position < centerPoint ){
			display.writeFastVLine( scrollingGraphWidth + xOffset - (step * 2 ), yOffset, centerPoint, BLACK ); // clear potential prev num off the top
			display.writeFastVLine( scrollingGraphWidth + xOffset - (step * 2 ), start + length, position, BLACK );
		}else{
			// clear previous line datum above centerPoint
			display.writeFastVLine( scrollingGraphWidth + xOffset - (step * 2 ), yOffset, SCROLLING_GRAPH_HEIGHT - position, BLACK );
			display.writeFastVLine( scrollingGraphWidth + xOffset - (step * 2 ), centerPoint, centerPoint, BLACK ); // clear potential prev num below the centerPoint
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
		display.writeFastHLine(scrollingGraphWidth + xOffset + 4, centerPoint, 2, WHITE);
	}
}

void wipeDisplay(){
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

void drawFanBlade( int radius, int xOffset, int yOffset, int offset ){

	int needleLength = radius - 5;
	float theta = ( angle + offset ) * ( M_PI / 180 );
	float x = -needleLength/2;
	float y = 0;
	float x1 = needleLength/2;
	float y1 = 0;
	float cx = (x + x1) / 2;
	float cy = (y + y1) / 2;
	float rotXEnd = (  (x1 - cx) * cos( theta ) + (y1 - cy) * sin( theta ) ) + cx;
	float rotYEnd = ( -(x1 - cx) * sin( theta ) + (y1 - cy) * cos( theta ) ) + cy;

	drawRotatedTriangle( 1, (rotXEnd + xOffset + radius), (rotYEnd + yOffset + radius), theta);
}

void drawRotatedTriangle( int sign, float xOffset, float yOffset, float theta ){

	int tX = 1 * sign;
	int tY = 0;

	int t1X = 8;
	int t1Y = 3 * sign;

	int t2X = 8;
	int t2Y = -3 * sign;

	float rtX = (  tX * cos( theta ) + tY * sin( theta ) );
	float rtY = ( -tX * sin( theta ) + tY * cos( theta ) );

	float rt1X = (  t1X * cos( theta ) + t1Y * sin( theta ) );
	float rt1Y = ( -t1X * sin( theta ) + t1Y * cos( theta ) );

	float rt2X = (  t2X * cos( theta ) + t2Y * sin( theta ) );
	float rt2Y = ( -t2X * sin( theta ) + t2Y * cos( theta ) );

	display.drawTriangle(
		xOffset + rtX,
		yOffset + rtY,

		xOffset + rt1X,
		yOffset + rt1Y,

		xOffset + rt2X,
		yOffset + rt2Y,
	WHITE);
}
