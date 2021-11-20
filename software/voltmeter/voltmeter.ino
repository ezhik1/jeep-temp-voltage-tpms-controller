// VoltMeter
// - OLED display
// - Historical reporting

#define BUFFER_LENGTH  8
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Lato_Thin_50.h"
#include "Lato_Thin_12.h"

#define OLED_RESET A4 // [ ALIAS ] analog pin to reset the OLED display
#define VOLTAGE_RAW_PIN A3 // [ ALIAS ] analog pin to read Manifold Presure sensor raw data
#define DISPLAY_WIDTH 128 // [ PIXELS ] number of available horizontal pixels
#define DISPLAY_HEIGHT 64 // [ PIXELS ] number of available vertical pixels
#define SCROLLING_GRAPH_HEIGHT 20 // [ PIXELS ] vertical size of the historical graph
#define NUM_READINGS 5 // [ INT ] number of readings that contribute to a rolling average of RAW VOLTAGE
#define SCROLLING_GRAPH_SAMPLE_SIZE 64
#define VOLTAGE_LABEL "V"
#define LOW_LABEL "LOW"

// Voltage Measurements and Display Characteristics
#define REFERENCE_VOLTAGE 1.1 // [ Volts ] : FLOAT : internal VCC+ used for reference to compute Measured Voltage on A3
#define MAX_VOLTAGE_GRAPHED 18 // [ Volts ] : INT : maximum voltage displayed  * nominal voltage 14
#define MIN_VOLTAGE_GRAPHED 10  // [ Volts ] : INT : minimum voltage displayed
#define LOW_VOLTAGE_ALERT_VOLTAGE 11.0 // [ Volts ] : FLOAT : at which point the display should warn of low voltage
#define MIN_VOLTAGE_MEASURED 0.0 // [ VOLTS ] : FLOAT : minimum voltage measured
#define MAX_VOLTAGE_MEASURED 18.0 // [ VOLTS ] : FLOAT : maximum voltage measured

#define EASE_COEFFICIENT 0.2 // [ DECIMAL ] : between 0-1 strength with which to dampen title scrolling, 0: strong damping 1: no damping
#define GRAPH_ANIMATION_INTERVAL 500 // [ MILLISECONDS ] : between historical graph updates
#define LOW_VOLTAGE_ALERT_INTERVAL 400 // [ MILLISECONDS ] : between flashes of Low Voltage

float R1 = 150000.00; // [ OHMS ] : 100K
float R2 = 10000.00; // [ OHMS ] : 10K

Adafruit_SSD1306 display(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire, OLED_RESET);

// States
bool redraw = true;
bool alert = false;
bool advanceGraph = false;

// Stateful variables
unsigned long previousMillisSensor = 0;
unsigned long previousMillisLowVoltageAlert = 0;
float currentVoltageReading = 0; // [ Volts ] : ( instantaneous ) between MIN_VOLTAGE_GRAPHED and MAX_VOLTAGE_GRAPHED
float currentDisplayReading = 0; // [ Volts ] : ( smoothed ) between MIN_VOLTAGE_GRAPHED and MAX_VOLTAGE_GRAPHED
float targetDisplayReading = 0; // [ PSI ] : ( smoothed ) between MIN_VOLTAGE_GRAPHED and MAX_VOLTAGE_GRAPHED

// Continuosly-Scrolling Vertical Bar Graph Characteristics
char scrollingGraphArray[SCROLLING_GRAPH_SAMPLE_SIZE]; // float the bar graph resolution

int readings[ NUM_READINGS ] ;
char readIndex = 0;
unsigned int rawTotal = 0;

const unsigned char batteryIcon [] PROGMEM = { // 30x30
	0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x3c, 0x00, 0x3f, 0x80, 0xfe, 0x00, 0xff, 0xff, 0xff, 0x80,
	0x80, 0x00, 0x00, 0x80, 0x8c, 0x00, 0x00, 0x80, 0x9e, 0x00, 0x3c, 0x80, 0x8c, 0x00, 0x00, 0x80,
	0xcc, 0x00, 0x01, 0x80, 0xff, 0xff, 0xff, 0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x80,
	0x80, 0x18, 0x00, 0x80, 0x80, 0x18, 0x00, 0x80, 0x80, 0x38, 0x00, 0x80, 0x80, 0x38, 0x00, 0x80,
	0x80, 0x3e, 0x00, 0x80, 0x80, 0x0e, 0x00, 0x80, 0x80, 0x0c, 0x00, 0x80, 0x80, 0x08, 0x00, 0x80,
	0x80, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0x00,
	0x00, 0x00, 0x00, 0x00
};

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

	analogReference(INTERNAL);
	pinMode(VOLTAGE_RAW_PIN, INPUT);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	wipeDisplay();
	runSplashScreen();
}

void runSplashScreen(){

	rollingTitle(F("CABIN VOLTAGE"));
	wipeDisplay();
	currentDisplayReading = 0;
	delay( 500 );
}

void loop(){

	calculateVoltage();
	updateDisplayReading();
	advanceAnimationTicks();

	render();
}

void render(){

	displayNumericScrollView( );
	display.display();

	if( redraw ){
		redraw = !redraw;
	}
}

void advanceAnimationTicks(){

	unsigned long currentMillis = millis();
	advanceGraph = false;

	if( previousMillisSensor == 0 || currentMillis - previousMillisSensor > GRAPH_ANIMATION_INTERVAL ){

		previousMillisSensor = currentMillis;
		advanceGraph = true;

		int scaledLevel = round(SCROLLING_GRAPH_HEIGHT * (( currentDisplayReading - MIN_VOLTAGE_GRAPHED ) / (MAX_VOLTAGE_GRAPHED - MIN_VOLTAGE_GRAPHED )));

		if( scaledLevel <= 0 ){
			scaledLevel = 0;
		}
		scrollingGraphArray[ 0 ] = scaledLevel;
	}

	if( previousMillisLowVoltageAlert == 0 || currentMillis - previousMillisLowVoltageAlert > LOW_VOLTAGE_ALERT_INTERVAL ){

		previousMillisLowVoltageAlert = currentMillis;
		alert = !alert;
	}
}

void updateDisplayReading(){

	currentDisplayReading = lerp( currentDisplayReading, targetDisplayReading,  EASE_COEFFICIENT );

	bool targetCloseEnoughToCurrent = ( targetDisplayReading - currentDisplayReading < 0.05 ) || (currentVoltageReading > targetDisplayReading );

	if( targetCloseEnoughToCurrent ){

		targetDisplayReading = constrain( currentVoltageReading, MIN_VOLTAGE_MEASURED, MAX_VOLTAGE_MEASURED );
	}
}

void calculateVoltage(){

	int rawVoltageReading = analogRead( VOLTAGE_RAW_PIN );

	// bail early if nearly-zero volts read on sense pin
	if( rawVoltageReading < 50 ){

		currentVoltageReading = -1;
		return;
	}

	rawTotal -= readings[ readIndex ];
	readings[ readIndex ] = rawVoltageReading; // [ 0 - 5v ] -> [ 102 - 1023 ]
	// readings[ readIndex ] = 1024; // DEBUG -> fixed raw analog read value
	// readings[ readIndex ] = ( random()%2 == 0 ) ? 600.0 : 300.0; // DEBUG -> random raw analog read value

	rawTotal += readings[ readIndex ];
	readIndex = ( readIndex + 1 ) % NUM_READINGS; // wrap if at end of total samples

	if (readIndex >= NUM_READINGS) {
		readIndex = 0;
	}

	float voltageOut = (( rawTotal / NUM_READINGS ) * REFERENCE_VOLTAGE ) / 1024.0;
	currentVoltageReading = voltageOut / ( R2 / ( R1+R2 ));

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

	byte scrollingGraphWidth = 120;
	byte scrollGraphXOffset = 0;
	byte scrollGraphYOffset = 43;

	// Large Numeric Display
	drawNumeric( 0, 20, 1, VOLTAGE_LABEL );

	// Rolling Bar Chart
	drawGraph( scrollGraphXOffset, scrollGraphYOffset, scrollingGraphWidth);
}

void drawNumeric( byte xOffset, byte yOffset, byte decimal, String label ){

	display.fillRect( xOffset,0, DISPLAY_WIDTH, yOffset + 23, BLACK ); // clear previous num

	display.setCursor( xOffset, yOffset + 23 );
	display.setFont(&Lato_Thin_50);


	if( currentVoltageReading < MIN_VOLTAGE_MEASURED ){
		// Alert Text
		display.setTextColor( alert );
		display.setCursor( xOffset + 20, yOffset + 18 );
		display.setTextSize(1);
		display.setFont( &Lato_Thin_12 );
		display.print("NO VOLTAGE");

		// Alert Icon
		display.drawBitmap(( DISPLAY_WIDTH / 2 ) - 15, yOffset - 20, warningIcon, 30, 30, WHITE );

	}else{

		if( currentVoltageReading < LOW_VOLTAGE_ALERT_VOLTAGE ){
			display.setTextColor( alert );
			display.fillRoundRect( xOffset + 1 ,4, 100, yOffset + 19, 3, !alert );
		}else{
			display.setTextColor( WHITE );
		}
		// Numeric Voltage
		display.print(Format( currentDisplayReading,3, decimal ));
		display.setCursor( xOffset + DISPLAY_WIDTH/2 + 47, yOffset + 20 );

		// Unit
		display.setTextColor( WHITE );
		display.setTextSize(1);
		display.setFont( &Lato_Thin_12 );
		display.print( label );

		// Icon
		display.drawBitmap( xOffset + 103, yOffset - 20, batteryIcon, 25, 25, WHITE );
	}
}

void drawGraph( byte xOffset, byte yOffset, byte scrollingGraphWidth ){

	byte bottomOfGraph = SCROLLING_GRAPH_HEIGHT;
	byte centerPoint = SCROLLING_GRAPH_HEIGHT / 2;

	// skips a line for each tick
	for (byte step = 0; step < SCROLLING_GRAPH_SAMPLE_SIZE; step++ ){

		byte position = scrollingGraphArray[step];

		byte start= ( position > centerPoint ) ? ( yOffset + bottomOfGraph - position ) : yOffset + centerPoint;
		byte length = ( position > centerPoint ) ? yOffset + centerPoint - start : centerPoint - position;

		if( (position > centerPoint || position < centerPoint ) && position > 0 ){

			display.writeFastVLine(scrollingGraphWidth + xOffset - (step * 2 ), start, length, WHITE); // Line Datum
		}

		// clear previous line datum overflow below centerPoint
		if( position < centerPoint ){
			display.writeFastVLine(scrollingGraphWidth + xOffset - (step * 2 ), yOffset, centerPoint, BLACK); // clear potential prev num off the top
			display.writeFastVLine(scrollingGraphWidth + xOffset - (step * 2 ), start + length, position, BLACK);
		}else{
			// clear previous line datum above centerPoint
			display.writeFastVLine(scrollingGraphWidth + xOffset - (step * 2 ), yOffset, SCROLLING_GRAPH_HEIGHT - position, BLACK);
			display.writeFastVLine(scrollingGraphWidth + xOffset - (step * 2 ), yOffset + centerPoint, yOffset + centerPoint, BLACK); // clear potential prev num below the centerPoint
		}
	}
	if( advanceGraph ){
		// advanced historical values
		for (byte step2 = SCROLLING_GRAPH_SAMPLE_SIZE; step2 >= 2; step2--){
			scrollingGraphArray[step2 - 1] = scrollingGraphArray[step2 - 2];
		}
	}

	if( redraw ){
		display.writeFastVLine(scrollingGraphWidth + xOffset + 5, yOffset, SCROLLING_GRAPH_HEIGHT, WHITE); // scope of graph
		display.writeFastHLine(scrollingGraphWidth + xOffset + 3, yOffset + centerPoint, 4, WHITE); // center point horizontal tick
	}
}

void wipeDisplay(){
	display.clearDisplay(); // remove library banner
	display.fillScreen(BLACK); // I see a red door...
	display.display(); // because fillScreen is misleading
}

String Format(float val, byte dec, byte dig ) {

	byte addpad = 0;
	byte sbuf[8];
	String fdata = (dtostrf(val, dec, dig, sbuf));
	byte slen = fdata.length();
	for ( addpad = 1; addpad <= dec + dig - slen; addpad++) {
		fdata = " " + fdata;
	}
	return (fdata);
}

bool isEqual(float x, float y){
	return abs(x - y) <= 1e-2 * abs(x);
}

float lerp(float a, float b, float x){
	return a + x * (b - a);
}
