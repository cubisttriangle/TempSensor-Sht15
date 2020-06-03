/*
 * A program to read data off of SHT15 sensor.
 * 
 * Important:  To  keep  self  heating  below  0.1°C, 
 * SHT1x should not be active for more than 10% of 
 * the time – e.g. maximum one measurement per second 
 * at 12bit accuracy shall be made.
 */

// Required time to wait after boot and reset.
#define STARTUP_DELAY_MS 11

/* Value of DATA_PIN is valid on rising edge of CLOCK_PIN
 * and must remain stable while CLOCK_PIN is high (DATA_SYNC).
 *
 * Value of DATA_PIN may be changed after falling edge of
 * CLOCK_PIN (DATA_TRANSITION).
 */
#define PIN_DATA  21
#define PIN_CLOCK 20

#define STATE_DATA_SYNC       HIGH
#define STATE_DATA_TRANSITION LOW

// Commands lead with 000 - the non-configurable SHT1x address.
#define CMD_MEASURE_TEMPERATURE       B00000011
#define CMD_MEASURE_RELATIVE_HUMIDITY B00000101
#define CMD_READ_STATUS_REG           B00000111
#define CMD_WRITE_STATUS_REG          B00000110
#define CMD_SOFT_RESET                B00011110


// The setup routine runs once when you press reset:
void setup()
{
  Serial.begin( 9600 );
  pinMode( PIN_CLOCK, OUTPUT );
  clockTransition();
  delay( STARTUP_DELAY_MS );
  writeStatusRegister( B00000000 );
}

// The loop routine runs over and over again forever:
void loop()
{
  getTemperature();
  getRelativeHumidity();
  delay( 1000 );
}

void writeStatusRegister( byte statusRegister )
{
  sendCmd( CMD_WRITE_STATUS_REG );
  writeByte( statusRegister );
}

void getStatusRegister()
{
  sendCmd( CMD_READ_STATUS_REG );
  if ( dataReady( 100, 10 ) )
  {
    byte statReg = readByte();
    sendAck();
    byte crc = readByte();
    sendAck();
    Serial.println( statReg, BIN );
  }
}

void getTemperature()
{
  short tempReadOut = 0;
  byte crc = 0;
  if ( getDataAndCrc( CMD_MEASURE_TEMPERATURE, &tempReadOut, &crc ) )
  {
    float temp = -40.2 + 0.018 * tempReadOut;
    Serial.print( "Temperature (F): " );
    Serial.println( temp );
  }
}

void getRelativeHumidity()
{
  short humidityReadOut = 0;
  byte crc = 0;
  if ( getDataAndCrc( CMD_MEASURE_RELATIVE_HUMIDITY, &humidityReadOut, &crc ) )
  {
    float humidity = -2.0468 + 0.0367 * humidityReadOut + -0.0000015955 * ( humidityReadOut * humidityReadOut );
    Serial.print( "Relative humidity (%): " );
    Serial.println( humidity );
  }
}

bool getDataAndCrc( byte cmd, short* data, byte* crc )
{
  sendCmd( cmd );
  if ( dataReady( 500, 10 ) )
  {
    getData( data, crc );
    return true;
  }
  else
  {
    Serial.println( "Sensor not ready!" );
    return false;
  }
}

bool sendCmd( char cmd )
{
  sendTransmissionStart();
  return writeByte( cmd );
}

void sendTransmissionStart()
{  
  // Set data pin to output, so we can control it.
  dataOut();

  // While clock is high, transition data low.
  dataHigh();
  clockSync();
  dataLow();

  // Pulse clock pin.
  clockTransition();
  clockSync();

  // Raise data pin, while clock is still high.
  dataHigh();

  // Finish by lowering clock.
  clockTransition();
}

bool writeByte( byte b )
{
  dataOut(); 
  shiftOut( PIN_DATA, PIN_CLOCK, MSBFIRST, b );
  return checkAck();
}

bool checkAck()
{
  dataIn();
  clockSync();
  bool gotAck = digitalRead( PIN_DATA ) == LOW;
  clockTransition();
  return gotAck;
}

bool dataReady( short timeoutMs, short timeoutIntervalMs )
{
  // Set data pin to INPUT so it floats.
  dataIn();

  while ( timeoutMs > 0 )
  {
    if ( LOW == digitalRead( PIN_DATA ) )
    {
      return true;
    }

    delay( timeoutIntervalMs );

    timeoutMs -= timeoutIntervalMs;
  }

  Serial.println( "Timed out waiting for measurement." );

  return false;
}

void getData( short* data, byte* crc )
{
  *data = ( readByte() << 8 ) | readByte();
  *crc = readByte();
}

byte readByte()
{
  dataIn();
  return shiftIn( PIN_DATA, PIN_CLOCK, MSBFIRST );
}

void sendAck()
{
  dataOut();
  dataLow();
  clockSync();
  clockTransition();
}

// If connection gets lost or bugs out.
void resetConnection()
{
  clockTransition();

  dataOut();
  dataHigh();

  int numClockToggles = 9;
  while ( numClockToggles-- )
  {
    clockSync();
    clockTransition();
  }
}

void clockTransition()
{
  digitalWrite( PIN_CLOCK, STATE_DATA_TRANSITION );
}

void clockSync()
{
  digitalWrite( PIN_CLOCK, STATE_DATA_SYNC );
}

void dataOut()
{
  pinMode( PIN_DATA, OUTPUT );
}

void dataIn()
{
  pinMode( PIN_DATA, INPUT );
}

void dataHigh()
{
  digitalWrite( PIN_DATA, HIGH );
}

void dataLow()
{
  digitalWrite( PIN_DATA, LOW );
}



