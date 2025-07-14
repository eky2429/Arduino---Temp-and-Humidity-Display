#include <dht.h>
#include <LiquidCrystal.h>

#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

//Varaibles for Humidity ensior
static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
dht DHT; 

//Varaibles for lcd
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Variables for LEDs
bool isCelcius = true;
const int FAHR_LED_PIN = 4;
const int CELS_LED_PIN = 3;

//Varaibles for button
const int buttonPin = 5;  // the number of the pushbutton pin
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print("BEGIN");
  pinMode(DHT_SENSOR_PIN, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(FAHR_LED_PIN, OUTPUT);
  pinMode(CELS_LED_PIN, OUTPUT);
}

/*
 * Poll for a measurement, keeping the state machine alive.  Returns
 * true if a measurement is available.
 */
static bool measure_environment( float *temperature, float *humidity ){
  static unsigned long measurement_timestamp = millis( );

  /* Measure once every four seconds. */
  if( millis( ) - measurement_timestamp > 3000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}

float celcius_to_fahrenheit(float c) {
  return (c * (9.0 / 5.0)) + 32.0;
}

void outputInfo(float temp_c, float hum){
  lcd.setCursor(0, 0);

  lcd.print( "T = " );

  if (isCelcius) {
    lcd.print( temp_c, 1 );
    lcd.print( " deg. C" );
  } else {
    float temp_f = celcius_to_fahrenheit(temp_c);
    lcd.print( temp_f, 1 );
    lcd.print( " deg. F" );
  }
  lcd.setCursor(0, 1);


  lcd.print("H = ");
  lcd.print( hum, 1 );
  lcd.print( "%" );
}

void debugInfo(float temp_c, float hum){
  Serial.print( "T = " );

  if (isCelcius) {
    Serial.print( temp_c, 1 );
    Serial.print( " deg. C, " );
  } else {
    float temp_f = celcius_to_fahrenheit(temp_c);
    Serial.print( temp_f, 1 );
    Serial.print( " deg. F, " );
  }


  Serial.print("H = ");
  Serial.print( hum, 1 );
  Serial.println( "%" );
}

void loop() {
  float temperature;
  float humidity;

  /* Measure temperature and humidity.  If the functions returns
     true, then a measurement is available. */
  if( measure_environment( &temperature, &humidity ) == true )
  {
    outputInfo(temperature, humidity);
    debugInfo(temperature, humidity);
  }

  //Sets LED
  if (isCelcius) {
    digitalWrite(FAHR_LED_PIN, LOW);
    digitalWrite(CELS_LED_PIN, HIGH);
  } else {
    digitalWrite(FAHR_LED_PIN, HIGH);
    digitalWrite(CELS_LED_PIN, LOW);
  }

  //debounce
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        isCelcius = !isCelcius;
        outputInfo(temperature, humidity);
        debugInfo(temperature, humidity);
      }
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}
