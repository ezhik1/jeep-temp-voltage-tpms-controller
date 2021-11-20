void decodeTPMS()
{
  int i;
  unsigned int id = 0;
  unsigned int status, pressure1, pressure2, temp;
  float realpressure;
  float realtemp;
  bool IDFound = false;
  char preferredTireLabel[3];

  for (i = 0; i < 4; i++)
  {
    id = id << 8;
    id = id + receivedBytes[i];

  }

  id = id & 0xFFFFFFF;

  setPreferredTireByIndex( id, preferredTireLabel );

  // id = (unsigned)receivedBytes[0] << 24 | receivedBytes[1] << 16 | receivedBytes[2] << 8 | receivedBytes[3];

  status = (receivedBytes[4] & 0x80) | (receivedBytes[6] & 0x7f); // status bit and 0 filler

  pressure1 = (receivedBytes[4] & 0x7f) << 1 | receivedBytes[5] >> 7;

  temp = (receivedBytes[5] & 0x7f) << 1 | receivedBytes[6] >> 7;

  pressure2 = receivedBytes[7] ^ 0xff;



  if (pressure1 != pressure2)
  {
    #ifdef SHOWDEBUGINFO
    Serial.println(F("Pressure check mis-match"));

    Serial.print(pressure1);
    Serial.print(" : ");
    Serial.print(pressure2);
    Serial.print("\n");
    #endif
    // return;
  }

  realpressure = pressure1 * 0.25 - 7.35;
  if (realpressure < 0)
     realpressure = 0.0;
  realtemp = temp - 40.0;
  #ifdef DISPLAY_TEMP_AS_FAHRENHEIT
    realtemp = ((realtemp * 9.0)/5.0) + 32.0;
  #endif


  #ifdef SHOWDEBUGINFO
    Serial.print(F("Pos: "));
    Serial.print( preferredTireLabel );
    Serial.print(F("   ID: "));
    Serial.print(id, HEX);
    Serial.print(F("   Status: 0x"));
    Serial.print(status,HEX);
    Serial.print(F("   Temperature: "));
    Serial.print(realtemp);
    Serial.print(F("   Tyre Pressure: "));
    Serial.print(realpressure);
    Serial.print(F(" (psi)  "));
    Serial.print(realpressure/PSI2BAR);
    Serial.print(F(" (bar)"));
    Serial.println(F(""));
  #endif

  MatchIDandUpdate(id,status, realtemp, realpressure);

  #ifdef SHOWDEBUGINFO
     Serial.println(F(""));
  #endif
}

bool validateTimings()
{
  unsigned int BitWidth;
  unsigned int BitWidthNext;
  unsigned int BitWidthNextPlus1;
  unsigned int BitWidthPrevious;
  unsigned int diff = timingsIndex - checkIndex;
  //unsigned long tmp;
  bool WaitingTrailingZeroEdge = false;
  int ret;
  int ByteCount = 0;
  byte crcResult = 0;

  startDataIndex = 0;

  if (diff < EXPECTEDbitCount)
  { //not enough in the buffer to consider a valid message
    #ifdef SHOWDEBUGINFO
      Serial.print(F("Insufficient data in buffer ("));
      Serial.print(diff);
      Serial.println(")");
    #endif
    return(false);
  }

  syncFound = false;

  while ((diff > 0) && (bitCount < EXPECTEDbitCount))
  { //something in buffer to process...
    diff = timingsIndex - checkIndex;

    BitWidth = Timings[checkIndex];

    if (syncFound == false)
    {
      if (IsValidSync(BitWidth))
      {
        syncFound = true;
        bitIndex = 0;
        bitCount = 0;
        WaitingTrailingZeroEdge = false;
        startDataIndex = checkIndex + 1;
      }
    }
    else
    {
      ret = ValidateBit();
      switch (ret)
      {
        case -1:
          syncFound = false;
          break;

        case 0:
          if (WaitingTrailingZeroEdge)
          {
            //BitTimings[bitIndex] = BitWidth;
            incomingBits[bitIndex++] = 0;
            bitCount++;
            WaitingTrailingZeroEdge = false;
          }
          else
          {
            WaitingTrailingZeroEdge = true;
          }
          break;

        case 1:
          if (WaitingTrailingZeroEdge)
          { //shouldn't get a long pulse when waiting for the second short pulse (i.e. expecting bit = 0)
            //try to resync from here?
            bitIndex = 0;
            bitCount = 0;
            WaitingTrailingZeroEdge = false;
            checkIndex--;  //recheck this entry
            startDataIndex = checkIndex + 1;
            syncFound = false;
          }
          else
          {
            //BitTimings[bitIndex] = BitWidth;
            incomingBits[bitIndex++] = 1;
            bitCount++;
          }
          break;

        case 2:
          if ((syncFound == true) && (bitCount > 50))
          {
             return(false);
          }
          else
          {
            syncFound = true;
            bitIndex = 0;
            bitCount = 0;
            WaitingTrailingZeroEdge = false;
            startDataIndex = checkIndex + 1;
            break;
          }
      }
    }
    checkIndex++;
  }


  if (bitCount >= EXPECTEDbitCount)
  {

      ByteCount = DecodeBitArray(0);

      crcResult = Compute_CRC8(8,0x7, 0x80);

      if (crcResult != receivedBytes[8])
      {
        #ifdef SHOWDEBUGINFO
          Serial.print(F("CRC calc: "));
          Serial.println(crcResult, HEX);
          Serial.print(F("  CRC rcvd: "));
          Serial.println(receivedBytes[8], HEX);
          Serial.println(F("CRC Check failed"));
        #endif
        #ifdef IGNORECHECKSUMERRORS
          decodeTPMS();
          hasTPMSChanged = false;  //don't update display for csum error
        #endif
      }
      else
      {
        //decode the message...
        #ifdef SHOWDEBUGINFO
           Serial.println(F("CRC Check OK"));
        #endif
        decodeTPMS();
        hasTPMSChanged = true;  //indicates the display needs to be updated.

      }

    return(true);
  }
  else
  {
    return(false);
  }
}
