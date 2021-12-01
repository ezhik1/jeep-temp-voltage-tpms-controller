// Tire Pressure Monitoring System
// - OLED display
// - LOW, HIGH pressure warnings ( configurable, persistent )
// - Tire ID assignment ( configurable, persistent )
// #include <Wire.h>
#include "SoftwareI2C.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FlashStorage_SAMD.h>
#include <AnalogMultiButton.h>

#include "configs.h"
#include "CC1101.h"
#include "common.h"
#include "pmv-c210-decode.h"

#include "Lato_Thin_12.h"
#include "Lato_Thin_16.h"

#define INPUT_BUTTON_PIN 3 // [ INT ]  digital read pin for button inputs
#define HIGH_PRESSURE_PIN 6 // [ OUTPUT ] to fan relay HIGH
#define LOW_PRESSURE_PIN 7 // [ OUTPUT ] to fan relay LOW

#define OLED_RESET -1 // [ ALIAS ] analog pin to reset the OLED display
#define DISPLAY_WIDTH 64 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 128 // [ PIXELS ] number of available vertical pixels
#define EASE_COEFFICIENT 0.2 // [ DECIMAL ] : between 0-1 strength with which to dampen title scrolling, 0: strong damping 1: no damping
#define LOW_HIGH_ALERT_INTERVAL 500 // [ MILLISECONDS ] : between flashes of Pressure warning

#define MAX_SHORT_PRESS_TIME 500 // [ MILLISECONDS ] : before a short press is no longer 'short'
#define MIN_LONG_PRESS_TIME 1000 // [ MILLISECONDS ] : before a press is considered 'long'

#define LEAVE_EDIT_MODE_TIME 20000 // [ MILLISECONDS ] before disabling editMode due to inactivity
#define NUMBER_OF_EDIT_MODE_SCREENS 2 // UIs for editing stored values: pressure set, tire id
#define LAST_LINE_PADDING 12 // [ PIXELS ] default padding for next line printed, used by slowType()

const int BUTTONS_TOTAL = 3;
const int BUTTONS_VALUES[ BUTTONS_TOTAL ] = { 0, 320, 460 }; // [ INT ] : ADC values from voltage divider that separates the action buttons
const int BUTTON_ENTER = 0;
const int BUTTON_UP = 1;
const int BUTTON_DOWN = 2;

// States
bool alert = false;
bool redraw = false;
bool displayUpdate = true;
bool isPressing = false;
bool isLongPressDetected = false;
bool isEditMode = false;

unsigned long previousMillisAlertFlash = 0;
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;
unsigned long editModeTime = 0;
byte buttonPressed = 0;
byte buttonReleased = 0;

byte editModeScreen = 0;
char *editModeScreens[ NUMBER_OF_EDIT_MODE_SCREENS ] = { "pressureLimits", "tireID" };
byte currentTireBeingEditted = 0;
int currentHexBeingEditted = 0;
byte currentThreshold = 0;
int rfChipRegistrationHasFailed;
byte lastLine = 0;

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

Adafruit_SSD1306 display( DISPLAY_HEIGHT, DISPLAY_WIDTH, &Wire, OLED_RESET, 800000L, 800000L );
AnalogMultiButton buttons( INPUT_BUTTON_PIN, BUTTONS_TOTAL, BUTTONS_VALUES );
SoftwareI2C softwarei2c;
#include "display.h"

void setup(){

	#ifdef SHOWDEBUGINFO
	SerialUSB.begin(115200);
	while( !SerialUSB ) {
		; // wait for serial port to connect
	}
	#endif

	// Wire.begin();
	// Wire.setWireTimeout(1000, true);
	// Wire.setClock( 400000 );
	softwarei2c.begin(4,5);

	if( !display.begin( SSD1306_SWITCHCAPVCC, 0x3C )){
		#ifdef SHOWDEBUGINFO
		Serial.println(F("SSD1306 initialization: FAILED"));
		#endif
	}else{
		#ifdef SHOWDEBUGINFO
		Serial.println(F("SSD1306 initialization: SUCCEEDED"));
		#endif
	}

	wipeDisplay();
	display.setRotation( 1 );
	display.setCursor( 0, 0 );
	display.setTextColor( WHITE );
	display.setFont( );
	slowType( "STARTING >", 50, true );

	//SPI CC1101 chip select set up
	pinMode( CC1101_CS, OUTPUT );
	digitalWrite( CC1101_CS, HIGH );
	pinMode( LOW_PRESSURE_PIN, OUTPUT );
	pinMode( HIGH_PRESSURE_PIN, OUTPUT );

	pinMode( LED_RX, OUTPUT );
	pinMode( RXPin, INPUT );
	pinMode( CDPin, INPUT );

	SPI.begin();
	delay( 2000 );

	#ifdef SHOWDEBUGINFO
	Serial.println(F(""));
	Serial.println(F(""));
	Serial.println(F("########################################################################"));
	Serial.println(F(""));
	Serial.println(F("STARTING..."));
	Serial.println(PROC_TYPE);
	#endif

	initializeRFChip();
	wipeDisplay();
	runSplashScreen();

	while( rfChipRegistrationHasFailed ){

		display.drawBitmap( DISPLAY_WIDTH / 2 - 15 , DISPLAY_HEIGHT / 2 - 15, warningIcon, 30, 30, WHITE );
		display.setCursor( 7, 40 );
		display.setFont( &Lato_Thin_12 );
		display.setTextColor( alert);
		display.setTextSize( 1 );
		display.print( "RF LINK" );
		display.setCursor( 10, 100 );
		display.print("FAILED");
		display.display();
		advanceAnimationTick();
	}
}

void loop() {

	if( millis() - lastCalibrationTime > CAL_PERIOD_MS ){

		setIdleState();  // configuration is set to auto-cal when goimg from Idle to RX
		lastCalibrationTime = millis();
		setRxState();
	}

	InitDataBuffer();

	// wait for carrier status to go low
	while( GetCarrierStatus() == true ){

	};

	// wait for carrier status to go high  looking for rising edge
	while( GetCarrierStatus() == false ){

	};

	// Data Stream
	if( GetCarrierStatus() == true ){

		receiveMessage();
	}

	listenToButtonPushes();
	advanceAnimationTick();
	setLEDNotifications();
	checkTimeoutsAndClearOldTPMS();
	render();
}

void render(){

	if( redraw ){

		redraw = !redraw;
		wipeDisplay();
	}

	if( isEditMode ){

		if( editModeScreens[ editModeScreen ] == "pressureLimits" ){

			drawEditMode();
		}else if( editModeScreens[ editModeScreen ] == "tireID" ){

			drawTireIDEditMode();
		}
	}else if( hasTPMSChanged || isPressureOutOfLimit || displayUpdate ){

		drawTPMSInfo();
		hasTPMSChanged = false;
		displayUpdate = false;
	}
}

void setLEDNotifications(){

	bool shouldAlarmLow = false;
	bool shouldAlarmHigh = false;

	for( int i = 0; i < TIRE_COUNT; i++ ){
		if( TPMS[ i ].LowPressure && !shouldAlarmLow ){

			shouldAlarmLow = true;
		}

		if( TPMS[ i ].HighPressure && !shouldAlarmHigh ){

			shouldAlarmHigh = true;
		}
	}

	digitalWrite( LOW_PRESSURE_PIN, shouldAlarmLow ? alert : false );
	digitalWrite( HIGH_PRESSURE_PIN, shouldAlarmHigh ? alert : false );
}

void listenToButtonPushes(){

	buttons.update();

	if(( isEditMode ) && ( millis() - editModeTime ) > LEAVE_EDIT_MODE_TIME ){

		saveTireIDPositions();
		exitEditMode();
	}

	bool enterButtonIsPressed = buttons.isPressed( BUTTON_ENTER );
	bool upButtonIsPressed = buttons.isPressed( BUTTON_UP );
	bool downButtonIsPressed = buttons.isPressed( BUTTON_DOWN );

	bool enterButtonIsReleased = buttons.onRelease( BUTTON_ENTER );
	bool upButtonIsReleased = buttons.onRelease( BUTTON_UP );
	bool downButtonIsReleased = buttons.onRelease( BUTTON_DOWN );

	byte buttonPressed = enterButtonIsPressed ? 1 : upButtonIsPressed ? 2 : downButtonIsPressed ? 3 : 0;
	bool isButtonPressed = ( buttonPressed != 0 ) ? true : false;

	byte buttonReleased = enterButtonIsReleased ? 1 : upButtonIsReleased ? 2 : downButtonIsReleased ? 3 : 0;
	bool isButtonReleased = ( buttonReleased != 0 ) ? true : false;

	if( !isPressing && isButtonPressed ){

		pressedTime = millis();
		isPressing = true;
		isLongPressDetected = false;
	}

	if( isButtonReleased ){

		isPressing = false;
		releasedTime = millis();

		if(( releasedTime - pressedTime ) < MAX_SHORT_PRESS_TIME ){

			changeProgramState( buttonReleased, "short" );
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

	// Enter Button is long pressed ( enter edit mode )
	if( type == "long" && buttonIndex == 1 && !isEditMode ){

		isEditMode = true;
		redraw = true;
		editModeTime = millis();
		return;
	}

	// Long press Enter Button to run next exit edit mode screen
	if( type == "long" && buttonIndex == 1 && isEditMode ){

		if( editModeScreen == NUMBER_OF_EDIT_MODE_SCREENS - 1 ){

			saveTireIDPositions();
			exitEditMode();
			return;
		}

		if( editModeScreens[ editModeScreen ] == "tireID" ){

			currentTireBeingEditted = 0;
		}

		editModeScreen++;
		wipeDisplay();
	}

	// EDIT MODE

	// Keep edit mode alive
	if( isEditMode && type == "short"  ){

		editModeTime = millis();
	}

	// EDIT : LOW TRIGGER

	// UP/DOWN Button is short pressed
	if( isEditMode && type == "short" ){

		int direction = 0;

		switch( buttonIndex ){

			case 2:
				direction = -1;
				break;
			case 3:
				direction = 1;
				break;
			case 1:
				direction = 0;
				break;
		}

		if( editModeScreens[ editModeScreen ] == "pressureLimits" ){

			updatePressureSetting( direction );
		 }else if( editModeScreens[ editModeScreen ] == "tireID" ){

			moveTireID( -direction ); // invert directon for moving up/down the lists
		}
	}
}

void exitEditMode(){

	isEditMode = false;
	currentTireBeingEditted = 0;
	currentHexBeingEditted = 0;
	editModeScreen = 0;
	redraw = true;
	displayUpdate = true;
}

void saveTireIDPositions(){

	int updatedCount = 0;
	char buffer[10];

	for( int i = 0; i < TIRE_COUNT; i++ ){
		int current = sensorIDMap[ i ];
		int old = storedIDMap[ i ];

		if( current != old ){
			updatedCount++;
		}
		updateMemory( 10 + i, sensorIDMap[ i ] );

		storedIDMap[ i ] = current;
	}

	if( updatedCount > 0 ){

		wipeDisplay();
		lastLine = 20;
		slowType( "TIRE IDS", 100, true );
		slowType( "UPDATED", 100, true );
		sprintf( buffer, "IDS: %d", updatedCount );
		slowType( buffer , 100, true );
		delay( 2000 );
	}
}

void moveTireID( int direction ){

	if( direction == 0 ){

		currentHexBeingEditted = currentHexBeingEditted == ( TIRE_COUNT - 1 ) ? 0 : currentHexBeingEditted + 1;
		return; // no swap
	}

	int nextID = currentHexBeingEditted + direction; // ID we want to swap
	bool nextSwapIsValid = ( nextID >= 0 && nextID < TIRE_COUNT );
	int tempValue;
	int nextValue;

	if( nextSwapIsValid ){

		tempValue = sensorIDMap[ nextID ];
		sensorIDMap[ nextID ] = sensorIDMap[ currentHexBeingEditted ]; // next ID to the selected ID
		sensorIDMap[ currentHexBeingEditted ] = tempValue; //selected ID to the cached nextID
	}

	currentHexBeingEditted += direction;

	// end of ID list
	if( currentHexBeingEditted == TIRE_COUNT ){

		tempValue = sensorIDMap[ 0 ]; // remember the first value before swapping the travelling ID
		sensorIDMap[ 0 ] = sensorIDMap[ TIRE_COUNT - 1 ]; // carry ID back to the top

		for( int i = 1; i < TIRE_COUNT; i++ ){
			nextValue = sensorIDMap[ i ];
			sensorIDMap[ i ] = tempValue;
			tempValue = nextValue;
		}

		currentHexBeingEditted = 0;
	// start of ID list
	}else if( currentHexBeingEditted == -1 ){

		tempValue = sensorIDMap[ TIRE_COUNT - 1 ]; // remember the last value before swapping the travelling ID
		sensorIDMap[ TIRE_COUNT - 1 ] = sensorIDMap[ 0 ]; // carry ID to bottom

		for( int i = TIRE_COUNT - 2; i >= 0 ; i-- ){
			nextValue = sensorIDMap[ i ];
			sensorIDMap[ i ] = tempValue;
			tempValue = nextValue;

		}
		currentHexBeingEditted = TIRE_COUNT - 1;
	}
}

void updatePressureSetting( int direction ){

	// move next
	if( direction == 0 ){

		// last threshold set
		if( currentThreshold == 1 ){

			currentTireBeingEditted = ( currentTireBeingEditted == TIRE_COUNT - 1 ) ? 0 : currentTireBeingEditted + 1;
			currentThreshold = 0;
		}else{

			currentThreshold++;
		}
	}else if( direction == -1 ||  direction == 1 ){

		// HIGH threshold
		if( currentThreshold == 0 ){

			TPMS[ currentTireBeingEditted ].TPMS_HighPressureLimit += direction;
			PressureHighLimits[ currentTireBeingEditted ] = TPMS[ currentTireBeingEditted ].TPMS_HighPressureLimit;
			updateMemory( currentTireBeingEditted, TPMS[ currentTireBeingEditted ].TPMS_HighPressureLimit );

		// LOW threshold
		}else{

			TPMS[ currentTireBeingEditted ].TPMS_LowPressureLimit += direction;
			PressureLowLimits[ currentTireBeingEditted ] = TPMS[ currentTireBeingEditted ].TPMS_LowPressureLimit;
			updateMemory( currentTireBeingEditted + 5, TPMS[ currentTireBeingEditted ].TPMS_LowPressureLimit );
		}
	}
}

void updateMemory( byte address, byte value ){

	EEPROM.update( address, value );
	EEPROM.commit();
}

void advanceAnimationTick(){

	unsigned long currentMillis = millis();

	if( previousMillisAlertFlash == 0 || currentMillis - previousMillisAlertFlash > LOW_HIGH_ALERT_INTERVAL ){

		previousMillisAlertFlash = currentMillis;
		alert = !alert;
	}
}

void initializeRFChip(){

	byte response;
	bool resetFailed = false;
	int LEDState = LOW;

	#ifdef SHOWDEBUGINFO
	Serial.print(F("Resetting CC1101 "));
	#endif

	slowType( "FQ: 433MHZ", 50, true );
	slowType("ATTEMPTS:", 50, true );

	byte retrycount = 0;

	while( retrycount < 5 ){
		display.fillRect( 54, lastLine - LAST_LINE_PADDING, 10,10,BLACK );
		display.setCursor( 54, lastLine - LAST_LINE_PADDING );
		display.print( retrycount + 1 );
		display.display();
		#ifdef SHOWDEBUGINFO
		Serial.print(F("."));
		#endif
		CC1101_reset();
		if( readConfigReg( 0 ) == 0x29 ){
			break;
		}
		retrycount++;
		delay( 300 );
	}

	#ifdef SHOWDEBUGINFO
	Serial.println("");
	#endif

	if( readConfigReg( 0 ) == 0x29 ){
		slowType( "RESET: OK", 50, true );
		#ifdef SHOWDEBUGINFO
		Serial.println(F("CC1101 reset successful"));
		#endif
	}else{
		slowType( "RESET:FAIL", 50, true );
		resetFailed = true;
		#ifdef SHOWDEBUGINFO
		Serial.println(F("CC1101 reset failed. Try rebooting"));
		#endif
	}

	ConfigureCC1101();
	#ifdef SHOWDEBUGINFO
	Serial.print(F("CC1101 configured for 433MHz on PMV-C210 TPMS sensor"));
	#endif
	setIdleState();
	digitalWrite( LED_RX, LED_OFF );

	response = readStatusReg( CC1101_PARTNUM );
	#ifdef SHOWDEBUGINFO
	Serial.print(F("Part Number: "));
	Serial.println(response, HEX);
	#endif

	response = readStatusReg( CC1101_VERSION );
	#ifdef SHOWDEBUGINFO
	Serial.print(F("Version: "));
	Serial.println(response, HEX);
	#endif

	rfChipRegistrationHasFailed = VerifyCC1101Config();

	if( rfChipRegistrationHasFailed > 0 ){

		slowType( "LINK : FAILED", 50, true );
		#ifdef SHOWDEBUGINFO
			Serial.print("Config verification fail #");
			Serial.println(rfChipRegistrationHasFailed);
		#endif
	}else{

		slowType( "LINK : OK", 50, true );
		#ifdef SHOWDEBUGINFO
			Serial.println("Config verification OK");
		#endif
	}

	if( resetFailed || rfChipRegistrationHasFailed > 0 ){
		lastLine += LAST_LINE_PADDING + 5;
		slowType( "PUSH ENTER", 50, true );
		slowType( "TO REBOOT", 50, true );
		display.setFont(&Lato_Thin_12);

		while( true ){

			display.setCursor( 10, lastLine + LAST_LINE_PADDING );
			display.fillRoundRect( 8, lastLine, 46, 16, 3, alert );
			display.setTextColor( !alert );
			display.print( "ENTER" );

			buttons.update();

			if( buttons.isPressed( BUTTON_ENTER ) ){

				wipeDisplay();
				lastLine = 20;
				display.setFont();
				display.setTextColor( WHITE );
				slowType( "RESETTING", 100, true );
				slowType( "...", 100, true );
				delay( 2000 );
				NVIC_SystemReset();
				return;
			}
			advanceAnimationTick();
			display.display();
		}
	}

	digitalWrite( LED_RX, LED_ON );
	LEDState = HIGH;

	pinMode( DEBUGPIN, OUTPUT );
	digitalWrite( DEBUGPIN, LOW );

	InitTPMS();

	digitalWrite( LED_RX, LED_OFF );

	lastCalibrationTime = millis();
	setRxState();
	delay( 2000 );
}

void runSplashScreen(){

	rollingTitle( F(" TPMS") );
	wipeDisplay();
	delay( 500 );
}

void wipeDisplay(){

	display.clearDisplay(); // remove library banner
	display.fillScreen(BLACK); // I see a red door...
	display.display(); // because fillScreen is misleading
}

void rollingTitle( String label ){

	byte destinationY = DISPLAY_HEIGHT / 2 + 10;
	float splashScreenTextPosition = 0;
	byte stringLength = label.length();
	byte splashRectWidth = ( 8 * stringLength ) + 14;
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
		delay( 16 );
	}

	splashScreenTextPosition = 0;
}

void slowType( String text, int delayTime, bool newLine ){

	if( newLine ){

		display.setCursor(0, lastLine );
		lastLine += LAST_LINE_PADDING;
	}

	for( int i = 0; i < text.length(); i++ ){

		display.print( text[ i ] );
		display.display();
		delay( delayTime );
	}
}

bool isEqual( float x, float y ){
	return abs( x - y ) <= 1e-2 * abs( x );
}

float lerp( float a, float b, float x ){
	return a + x * ( b - a );
}
