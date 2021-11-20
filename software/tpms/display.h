void drawTPMSInfo(){

	byte i;
	int x = 0;
	int y = 0;
	char s[ 6 ];

	display.setFont(&Lato_Thin_12);
	display.setTextColor(WHITE);
	display.setTextSize(1);

	// Tire Layout
	for( i = 0; i < TIRE_COUNT; i++ ){

		switch( i ){
			case 0:
				x = 0;
				y = 18;
				break;
			case 1:
				x = DISPLAY_WIDTH / 2 + 1;
				y = 18;
				break;
			case 2:
				x = 0;
				y = 62;
				break;
			case 3:
				x = DISPLAY_WIDTH / 2 + 1;
				y = 62;
				break;
			case 4:
				x = DISPLAY_WIDTH / 2 - 15;
				y = 106;
				break;
		}

		display.fillRoundRect( x, y - 18, 31, 40, 4, BLACK ); // clear previous num

		// valid TPMS signal to show
		if( TPMS[i].TPMS_ID != 0  ){

			bool shouldAlarm = ( TPMS[ i ].LowPressure || TPMS[ i ].HighPressure );

			if( shouldAlarm ) {

				display.setTextColor( !alert );
				display.fillRoundRect( x, y - 18, 31, 40, 4, alert ); // clear Previous num
			}else{

				display.setTextColor( WHITE );
			}

			// Tire Orientation
			if( i < 4 ){
				display.setCursor( x + 4, y - 10);
			}else{
				display.setCursor( x - 6, y - 10);
			}

			// Signal Health
			int lastUpdated = signalHealth(millis() - TPMS[ i ].lastupdated );

			for( byte j = 0; j < lastUpdated; j++ ){

				display.drawCircle( x + 5 + (j * 5) , y + 14, 1, shouldAlarm ? !alert : WHITE );
			}

			// Pressure
			if( TPMS[ i ].TPMS_Pressure < 10 ){
				display.setCursor( x + 5, y - 3 );
			}else{
				display.setCursor( x + 6, y - 3 );
			}

			display.setFont(&Lato_Thin_16);
			dtostrf( TPMS[ i ].TPMS_Pressure, 2, 0, s );
			display.print(s);
			display.setFont(&Lato_Thin_12);

			// Temperature
			if( TPMS[ i ].TPMS_Temperature < 10 ){
				display.setCursor( x + 3, y + 10 );
			}else{
				display.setCursor( x + 2, y + 10 );
			}
			dtostrf( TPMS[ i ].TPMS_Temperature, 2, 0, s );
			display.print(s);
			display.write(0xf7);
			display.print("'C");

		// NO TPMS data to show
		}else{
			display.drawRoundRect( x, y - 18, 31, 40, 4, WHITE ); // tire outline
			display.setTextColor( WHITE );
			display.setCursor( x + 5, y );
			display.print("NO");
			display.setCursor( x + 5, y + 10 );
			display.print("SIG");
		}
	}

	display.display();
}

void drawEditMode(){

	byte x = 0;
	byte y = 0;

	display.setFont();
	display.setTextColor( WHITE );
	display.setTextSize( 1 );

	display.setCursor( x, y );
	display.print( "POS  H : L" );

	display.setFont( &Lato_Thin_12 );
	y += 20;

	char *tireLabels[] = {"FL", "FR","RL","RR","SP"};

	for( int i = 0; i < TIRE_COUNT; i++ ){

		bool highColor = WHITE;
		bool lowColor = WHITE;

		display.drawRoundRect( x, y + (i * 20 ) - 4, DISPLAY_WIDTH, 20, 3, BLACK ); // clear previous TIRE EDIT
		display.fillRoundRect(  x + 25, y + (i * 20 ) - 2, 15, 15, 3, BLACK ); // clear previous HIGH Threshold
		display.fillRoundRect(  x + 47, y + (i * 20 ) - 2, 15, 15, 3, BLACK ); // clear previous LOW Threshold

		// Tire Type
		display.setCursor( x + 2, y + (i * 20 + 10 ));
		display.print( tireLabels[ i ] );

		// Current Tire being set
		if( currentTireBeingEditted == i ){
			// Rect around tire
			display.drawRoundRect( x, y + (i * 20 ) - 4, DISPLAY_WIDTH, 20, 3, WHITE );

			// Filled Rect around threshold
			if( currentThreshold == 0 ){
				display.fillRoundRect(  x + 25, y + (i * 20 ) - 2, 15, 15, 3, WHITE );
				highColor = BLACK;
			}else{
				display.fillRoundRect(  x + 47, y + (i * 20 ) - 2, 15, 15, 3, WHITE );
				lowColor = BLACK;
			}
		}

		// High Setting
		display.setTextColor( highColor );
		display.setCursor( x + 25, y + (i * 20 + 10 ));
		display.print( round( TPMS[ i ].TPMS_HighPressureLimit ));

		// Low Setting
		display.setTextColor( lowColor );
		display.setCursor( x + 47, y + (i * 20 + 10 ));
		display.print( round(TPMS[ i ].TPMS_LowPressureLimit ));
		display.setTextColor( WHITE );
	}

	display.display();
}

void drawTireIDEditMode(){

	byte x = 0;
	byte y = 0;

	display.setFont();
	display.setTextColor( WHITE );
	display.setTextSize( 1 );

	display.setCursor( x, y );
	display.print( "POS  HEX" );

	y += 20;

	char *tireLabels[] = { "FL", "FR","RL","RR","SP"};

	for( int i = 0; i < TIRE_COUNT; i++ ){

		bool highColor = WHITE;

		display.fillRoundRect(  x + 18, y + (i * 20 ) - 2, 45, 15, 3, BLACK ); // clear previous HEX value

		// Tire Type
		display.setFont(&Lato_Thin_12);
		display.setCursor( x, y + (i * 20 + 10 ));
		display.print( tireLabels[ i ] );

		// Current Tire being set
		if( currentHexBeingEditted == i ){

			// Filled Rect around threshold
			display.fillRoundRect(  x + 18, y + ( i * 20 ) - 2, 45, 15, 3, WHITE );
			highColor = BLACK;

		}

		// HEX Value for Tire
		display.setFont();
		display.setTextColor( highColor );
		display.setCursor( x + 20, y + (i * 20 + 2 ));
		display.print( round( sensorIDs[ sensorIDMap[ i ]]), HEX);
		display.setTextColor( WHITE );
	}

	display.display();
}
