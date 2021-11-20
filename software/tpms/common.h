byte signalHealth( unsigned long timeSinceLastUpdate ){

	int howCloseToTimeOut;
	howCloseToTimeOut = (int)(( timeSinceLastUpdate )/( TPMS_TIMEOUT / 5));

	switch( howCloseToTimeOut ){
		case 0:
			return(5);
			break;
		case 1:
			return(4);
			break;
		case 2:
			return(3);
			break;
		case 3:
			return(2);
			break;
		case 4:
			return(1);
			break;
		default:
			return(0);
		break;
	}
}

void EdgeInterrupt(){

	unsigned long ts = micros();
	unsigned long BitWidth;

	if( timingsIndex == MAXTIMINGS ){
		return;
	}

	if( waitingForFirstEdge ){
		if( digitalRead( RXPin ) == LOW ){
			firstEdgeIsHighToLow = true;
		}else{
			firstEdgeIsHighToLow = false;
		}
	}

	waitingForFirstEdge = false;
	BitWidth = ts - lastEdgeTimeMicros;

	if( BitWidth > 0xFFFF ){
		BitWidth = 0xFFFF;
	}

	lastEdgeTimeMicros = ts;
	Timings[ timingsIndex++ ] = ( unsigned int ) BitWidth;
}

bool IsTooShort( unsigned int Width ){
	if( Width < SHORTTIMING_MIN ){
		return ( true );
	}else{
		return ( false );
	}
}

bool IsTooLong( unsigned int Width ){
	// if( Width > LONGTIMING_MAX ){
	// 	return ( true );
  	// }else{
	// 	return ( false );
	// }

	return ( Width > LONGTIMING_MAX );
}

bool IsValidSync( unsigned int Width ){
	// if(( Width >= SYNCTIMING_MIN ) && ( Width <= SYNCTIMING_MAX )){
	// 	return ( true );
	// }else{
	// 	return ( false );
	// }

	return (( Width >= SYNCTIMING_MIN ) && ( Width <= SYNCTIMING_MAX ));
}

bool IsValidShort( unsigned int Width ){
	// if(( Width >= SHORTTIMING_MIN ) && ( Width <= SHORTTIMING_MAX )){
	// 	return ( true );
	// }else{
	// 	return ( false );
	// }

	return (( Width >= SHORTTIMING_MIN ) && ( Width <= SHORTTIMING_MAX ));
}


bool IsValidLong( unsigned int Width ){
//   if ((Width >= LONGTIMING_MIN) && (Width <= LONGTIMING_MAX))
//   {
// 	return (true);
//   }
//   else
//   {
// 	return (false);
//   }
	return (( Width >= LONGTIMING_MIN ) && ( Width <= LONGTIMING_MAX ));
}

bool IsEndMarker(unsigned int Width){
//   if ((Width >= ENDTIMING_MIN) && (Width <= ENDTIMING_MAX))
//   {
// 	 return(true);
//   }
//   else
//   {
// 	return(false);
//   }
	return (( Width >= ENDTIMING_MIN ) && ( Width <= ENDTIMING_MAX ));
}


int ValidateBit(){

	unsigned int BitWidth = Timings[checkIndex];

	if( IsValidLong( BitWidth )){

		return (BITISLONG);
	}

	if( IsValidShort( BitWidth )){

		return ( BITISSHORT );
	}

	if( IsValidSync( BitWidth )){

		return ( BITISSYNC );
	}

	return ( -1 );
}

int ValidateBit( int Index ){

	unsigned int BitWidth = Timings[Index];

	if( IsValidLong( BitWidth ) ){

		return ( BITISLONG );
	}

	if( IsValidShort( BitWidth )){

		return ( BITISSHORT );
	}

	if( IsValidSync( BitWidth )){

		return ( BITISSYNC );
	}

	return ( BITISUNDEFINED );
}

byte Compute_CRC8( int bcount, byte Poly, byte crc_init ){

	byte crc = crc_init;

	for( int c = 0; c < bcount; c++ ){

		byte b = receivedBytes[ c ];
		/* XOR-in next input byte */
		byte data = ( byte )( b ^ crc );
		/* get current CRC value = remainder */
		if( Poly == 0x07 ){
			crc = ( byte )( pgm_read_byte( &CRC8_Poly_07_crctable2[ data ] ));
		}else{

			if( Poly == 0x13 ){
				crc = ( byte )( pgm_read_byte( &CRC8_Poly_13_crctable2[	data ] ));
			}
		}
	}

	return crc;
}

byte Compute_CRC_XOR(int Start, int bcount, byte crc_init)
{
  byte crc = crc_init;
  int c;

  for (c = Start; c < bcount; c++)
  {
	crc = crc ^ receivedBytes[c];
  }

  return(crc);
}

byte Compute_CRC_SUM(int Start, int bcount, byte crc_init)
{
  byte crc = crc_init;
  int c;

  for (c = Start; c < bcount; c++)
  {
	crc = crc + receivedBytes[c];
  }

  return(crc);
}

int GetRSSI_dbm()
{
  byte RSSI_Read;
  byte RSSI_Offset = 74;
  int ret;

  RSSI_Read = readStatusReg(CC1101_RSSI);
  if (RSSI_Read >= 128)
  {
	ret = (int)((int)(RSSI_Read - 256) /  2) - RSSI_Offset;
  }
  else
  {
	ret = (RSSI_Read / 2) - RSSI_Offset;
  }
  return(ret);
}

void ClearRXBuffer()
{
  int i;

  for (i = 0; i < sizeof(receivedBytes); i++)
  {
	receivedBytes[i] = 0;
  }
}

int ManchesterDecode(int StartIndex)
{
   int i;
   bool bit1, bit2;
   byte b = 0;
   byte n = 0;

   receivedBytesCount = 0;
   for (i = StartIndex; i< bitCount-1;i+=2)
   {
	  bit1 = incomingBits[i];
	  bit2 = incomingBits[i+1];

	  if (bit1 == bit2)
	  {
		 return receivedBytesCount;
	  }

	b = b << 1;
	b = b + (bit2 == true? 1:0);
	n++;
	if (n == 8)
	{
	  receivedBytes[receivedBytesCount] = b;
	  receivedBytesCount++;
	  n = 0;
	  b = 0;
	}

   }

   return receivedBytesCount;

}

int DifferentialManchesterDecode(int StartIndex)
{
   int i;
   bool bit1, bit2, bit3;
   byte b = 0;
   byte n = 0;

   receivedBytesCount = 0;
   for (i = StartIndex; i< bitCount-1;i+=2)
   {
	  bit1 = incomingBits[i];
	  bit2 = incomingBits[i+1];
	  bit3 = incomingBits[i+2];

	  if (bit1 != bit2)
	  {
		if (bit2 != bit3)
		{
		  b = b << 1;
		  b = b + 0;
		  n++;
		  if (n == 8)
		  {
			receivedBytes[receivedBytesCount] = b;
			receivedBytesCount++;
			n = 0;
			b = 0;
		  }
		}
		else
		{
		  bit2 = bit1;
		  i+=1;
		  break;
		}
	  }
	  else
	  {
		bit2 = 1 - bit1;
		break;
	  }
   }


   for (; i< bitCount-1;i+=2)
   {
	  bit1 = incomingBits[i];

	  if (bit1 == bit2)
		 return receivedBytesCount;
	  bit2 = incomingBits[i+1];

	  b = b << 1;
	  b = b + (bit1 == bit2? 1:0);
	  n++;
	  if (n == 8)
	  {
		receivedBytes[receivedBytesCount] = b;
		receivedBytesCount++;
		n = 0;
		b = 0;
	  }


   }

   return receivedBytesCount;

}

void InvertBitBuffer()
{
   int i;

   for (i = 0;i < bitCount;i++)
   {
	  incomingBits[i] = !incomingBits[i];
   }

}

static inline uint8_t bit_at(const uint8_t *bytes, unsigned bit)

{
	return (uint8_t)(bytes[bit >> 3] >> (7 - (bit & 7)) & 1);
}

void InitDataBuffer()
{
  bitIndex = 0;
  bitCount = 0;
  validBlock = false;
  // WaitingTrailingZeroEdge = false;
  waitingForFirstEdge  = true;
  checkIndex = 0;
  timingsIndex = 0;
  syncFound = false;
  //digitalWrite(DEBUGPIN, LOW);

}


void ClearTPMSData(int i)
{
  if (i > TIRE_COUNT)
	return;

  TPMS[i].TPMS_ID = 0;
  TPMS[i].lastupdated = 0;
  TPMS[i].TPMS_LowPressureLimit = PressureLowLimits[i];
  TPMS[i].TPMS_HighPressureLimit = PressureHighLimits[i];
  TPMS[i].LowPressure = false;
  TPMS[i].HighPressure = false;
  TPMS[i].AudibleAlarmActive = false;
}

void PulseDebugPin(int width_us)
{
  digitalWrite(DEBUGPIN, HIGH);
  delayMicroseconds(width_us);
  digitalWrite(DEBUGPIN, LOW);
}

void UpdateFreqOffset()
{
	freqOffsetAcc = freqOffsetAcc + readStatusReg(CC1101_FREQEST);
	writeReg(CC1101_FSCTRL0, freqOffsetAcc);

}



int getStandardizedIndexOfSensor( unsigned long ID ){
  for ( int i = 0; i  < TIRE_COUNT; i++ ){
	if (sensorIDs[ i ] == ID ){
	  return ( i );
	}
  }
  return (-1);
}

void setPreferredTireByIndex(unsigned long ID, char* sptr)
{

  switch ( getStandardizedIndexOfSensor( ID ) ){
	case 0:
	   strcpy(sptr, "FL");
	   break;
	case 1:
	   strcpy(sptr, "FR");
	   break;
	case 2:
	   strcpy(sptr, "RL");
	   break;
	case 3:
	   strcpy(sptr, "RR");
	   break;
	case 4:
	   strcpy(sptr, "SP");
	   break;
	default:
	   sptr[0] = '\0';
	   break;

  }
}

void PrintTimings(byte StartPoint, unsigned int Count)
{
  unsigned int i;
  char c[10];
  for (i = 0; i < Count; i++)
  {
	if ((StartPoint == 0) && (i == startDataIndex))
		#ifdef SHOWDEBUGINFO
		 Serial.println();
		 #endif
	sprintf(c, "%3d,",Timings[StartPoint + i]);
	#ifdef SHOWDEBUGINFO
	  Serial.print(c);
	#endif

  }
  #ifdef SHOWDEBUGINFO
  Serial.println(F(""));
  #endif
}

void PrintData(int StartPos, unsigned int Count)
{
  #ifdef SHOWDEBUGINFO
  unsigned int i, c;
  byte hexdata;
  for (i = StartPos, c = 1; c <= Count; i++, c++)
  {

	Serial.print(incomingBits[i]);

	hexdata = (hexdata << 1) + incomingBits[i];
	if (c % 8 == 0)
	{
	  Serial.print(F(" ["));
	  Serial.print(hexdata, HEX);
	  Serial.print(F("] "));
	  hexdata = 0;
	}
  }
  Serial.println(F(""));
  #endif
}

void PrintData(unsigned int Count)
{
   PrintData(0,Count);
}



void PrintBytes(unsigned int Count)
{
  byte i;
  #ifdef SHOWDEBUGINFO
  for (i = 0; i < Count; i++)
  {
	Serial.print(F(" ["));
	Serial.print(receivedBytes[i],HEX);
	Serial.print(F("] "));
  }
  Serial.println(F(""));
  #endif

}
void InitTPMS()
{
  int i;

  for (i = 0; i < TIRE_COUNT; i++)
  {
	ClearTPMSData(i);
  }
  #ifdef USE_LCDDISPLAY
	//  UpdateDisplay();
  #endif

}

void UpdateTPMSData(int index, unsigned long ID, unsigned int status, float Temperature, float Pressure)
{

  if (index >= TIRE_COUNT)
	return;

  TPMS[index].TPMS_ID = ID;
  TPMS[index].TPMS_Status = status;
  TPMS[index].lastupdated = millis();
  TPMS[index].TPMS_Temperature = Temperature;

  #ifdef USE_BAR
	TPMS[index].TPMS_Pressure = Pressure/PSI2BAR;
  #else
	 TPMS[index].TPMS_Pressure = Pressure - pressureOffsets[ index ];
  #endif

  #ifdef ENABLE_PRESSURE_ALARMS
	  if (round(TPMS[index].TPMS_Pressure * 10)/10.0 < TPMS[index].TPMS_LowPressureLimit)
	  {


		TPMS[index].LowPressure = true;
		#ifdef SHOWDEBUGINFO
		   Serial.print("Low Pressure warning.");
		   Serial.print("  Limit: ");
		   Serial.print(TPMS[index].TPMS_LowPressureLimit);
		   Serial.print("  Measured: ");
		   Serial.println(TPMS[index].TPMS_Pressure);
		#endif
	  }
	  else
	  {
		TPMS[index].LowPressure = false;
	  }

	  if (round(TPMS[index].TPMS_Pressure * 10)/10.0 > TPMS[index].TPMS_HighPressureLimit)
	  {

		TPMS[index].HighPressure = true;
	   #ifdef SHOWDEBUGINFO
		   Serial.print("High Pressure warning.");
		   Serial.print("  Limit: ");
		   Serial.print(TPMS[index].TPMS_HighPressureLimit);
		   Serial.print("  Measured: ");
		   Serial.println(TPMS[index].TPMS_Pressure);
		#endif
	  }
	  else
	  {
		TPMS[index].HighPressure = false;
	  }

  #endif

  bitSet(TPMSChangeBits,index);

  TPMS[index].RSSIdBm = valueRSSI;
}


void OutOfLimitsPressureCheck(){

	isPressureOutOfLimit = false;

	for( int i = 0; i < TIRE_COUNT; i++ ){
		if(( TPMS[ i ].LowPressure == true) || ( TPMS[ i ].HighPressure == true )){
			isPressureOutOfLimit = true;
		}
	}
}

void checkTimeoutsAndClearOldTPMS(){



	for( byte i = 0; i < TIRE_COUNT; i++ ){
		if(( TPMS[ i ].TPMS_ID != 0 ) && ( millis() - TPMS[ i ].lastupdated > TPMS_TIMEOUT )){
	  		ClearTPMSData( i );
	  		OutOfLimitsPressureCheck();
		}
 	}
}

void MatchIDandUpdate(unsigned long incomingID ,unsigned int status, float realtemp,float realpressure)
{

  bool IDFound = false;
  int standardizedIndex;
  int i;

  //update the array of tyres data
  for (i = 0; i < TIRE_COUNT; i++)
  { //find a matching ID if it already exists
	if (incomingID == TPMS[ i ].TPMS_ID)
	{
	  UpdateTPMSData(i, incomingID, status, realtemp, realpressure);
	  IDFound = true;
	  break;
	}

  }

  //no matching IDs in the array, so see if there is an empty slot to add it into, otherwise, ignore it.
  if (IDFound == false)
  {

	standardizedIndex = getStandardizedIndexOfSensor( incomingID );
	if( standardizedIndex == -1 ){ //not found a specified index, so use the next available one..
	  #ifndef SPECIFIC_IDS_ONLY
		for (i = 0; i < TIRE_COUNT; i++)
		{
		  if (TPMS[i].TPMS_ID == 0)
		  {
			UpdateTPMSData(i, incomingID, status, realtemp, realpressure);
			break;
		  }
		}
	  #endif
	}else{ //found a match in the known ID list...
	  int i;
	  for( i = 0; i < TIRE_COUNT; i++ ){
		if( sensorIDMap[ i ] == standardizedIndex ){
		  UpdateTPMSData( i , incomingID, status, realtemp, realpressure);
		}
	  }
	}

  }
  OutOfLimitsPressureCheck();
}


int DecodeBitArray( byte ShiftRightbitCount)
{
  //convert 1s and 0s array to byte array
  int i;
  int n = 0;
  byte b = 0;

  n = ShiftRightbitCount;  //pad with this number of 0s to the left
  receivedBytesCount = 0;

  for (i = 0; i < bitCount; i++)
  {
	b = b << 1;
	b = b + incomingBits[i];
	n++;
	if (n == 8)
	{
	  receivedBytes[receivedBytesCount] = b;
	  receivedBytesCount++;
	  n = 0;
	  b = 0;
	}

  }
  //Serial.println("");
  return (receivedBytesCount);
}

int receiveMessage()
{

  //Check bytes in FIFO
  int FIFOcount;
  int resp;
  int lRSSI = 0;
  bool StatusUpdated = true;
  byte crcResult;
  int ByteCount = 0;
  bool ValidMessage = false;
  unsigned long t1;

#ifdef USE_TEST_TIMINGS
  //test set up....

  cdWidth = CDWIDTH_MIN + ((CDWIDTH_MAX - CDWIDTH_MIN)/2);

  //copy timings to timings array as if they've come from the interrupt

  for (timingsIndex=0;timingsIndex<TestTimings_len;timingsIndex++)
  {
	 Timings[timingsIndex] = TestTimings[timingsIndex];
  }

  firstEdgeIsHighToLow = !FirstTimingIsLow;

#else



  //set up timing of edges using interrupts...
  lastEdgeTimeMicros = micros();
  cdWidth = lastEdgeTimeMicros;

  attachInterrupt(digitalPinToInterrupt(RXPin), EdgeInterrupt, CHANGE);
  #ifdef ARDUINO_SEEED_XIAO_M0
	NVIC_SetPriority(EIC_IRQn, 2);  //!!!!! this is necessary for the Seeeduino Xiao due external interupts having a higher priority than the micros() timer rollover (by default).
									 // This can cause the micros() value to appear to go 'backwards' in time in the external interrupt handler and end up giving an incorrect 65536 bit width result.
  #endif
  valueRSSI = -1000;


  while (GetCarrierStatus() == true)
  {

	 //get the maximum RSSI value seen during data receive window
	 lRSSI = GetRSSI_dbm();
	 if (lRSSI > valueRSSI)
	 {
	   valueRSSI = lRSSI;
	 }
	 noInterrupts();
	 t1 = micros();
	 interrupts();
	 if (t1 - cdWidth > CDWIDTH_MAX)
	 {

		#ifdef SHOWDEBUGINFO
		Serial.print(t1 - cdWidth);
		Serial.println("   Exiting CS loop");
		#endif
		break;
	 }
  }


  //digitalWrite(DEBUGPIN,LOW);

  delayMicroseconds(1000);  //there is a delay on the serial data stream so ensure we allow a bit of extra time after CD finishes to ensure all the data is captured
  detachInterrupt(digitalPinToInterrupt(RXPin));
  EdgeInterrupt();  //force a final edge change just to be sure
  cdWidth = micros() - cdWidth;


#endif


  if ((cdWidth >= CDWIDTH_MIN) && (cdWidth <= CDWIDTH_MAX) && (timingsIndex > EXPECTEDbitCount ))
  {
	PulseDebugPin(100);
	#ifdef SHOWDEBUGINFO
	   Serial.println("******************************************************************");
	   Serial.println(F("Checking...."));
	#endif
	digitalWrite(LED_RX,LED_ON);
	checkIndex = 0;
	ValidMessage = validateTimings();

	#ifdef SHOWDEBUGINFO
	   Serial.println(F("Timings...."));
	   Serial.print(F("cdWidth="));
	   Serial.println(cdWidth);
	   Serial.print(F("timingsIndex="));
	   Serial.println(timingsIndex);
	   Serial.print(F("Checking complete. bitCount: "));
	   Serial.print(bitCount);
	   Serial.print(F("  startDataIndex: "));
	   Serial.println(startDataIndex);
	   Serial.print(F(" RSSI(dBm):"));
	   Serial.println(valueRSSI);
	   #ifdef ALWAYSSHOWTIMINGS
		  PrintTimings(0,timingsIndex+1);
		  PrintData(bitCount);
		  PrintBytes(EXPECTEDBYTECOUNT);
	   #else
	   if (ValidMessage)
	   {
		  PrintTimings(0,timingsIndex+1);
		  PrintData(bitCount);
		  PrintBytes(EXPECTEDBYTECOUNT);
	   }
	   #endif

	#endif

	digitalWrite(LED_RX,LED_OFF);
	return (bitCount);
  }
  else
  {
	return (0);
  }
}
