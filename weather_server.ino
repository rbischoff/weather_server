

#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SPI.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>

#include "Adafruit_MCP9808.h"



//*********************************************************************************************
// ***********                        UNIVERSAL SETTINGS                          *************
//*********************************************************************************************

#define SERIAL_BAUD   9600

//*********************************************************************************************
// ***********               NETWORK SETTINGS FOR RFM RADIO MODULE                *************
//*********************************************************************************************

char ssid[] = "";     //  your network SSID (name)
char pass[] = "";  // your network password
int status = WL_IDLE_STATUS;

//*********************************************************************************************
// ***********                     WEATHER SENSOR SETTINGS                        *************
//*********************************************************************************************

#define SEALEVELPRESSURE_HPA (1013.25)
#define WIND_DIR_AVG_SIZE 120

//*********************************************************************************************


//*********************************************************************************************
// ***********                       HARDWARE PIN SETTINGS                        *************
//*********************************************************************************************

// DIGITAL PINS
const byte WSPEED = 5;
const byte RAIN = 6;

// ANALOG PINS
const byte WDIR = A0;
const byte REFERENCE_3V3 = A3;


//*********************************************************************************************
// ***********                          GLOBAL VARIABLES                          *************
//*********************************************************************************************

long lastSecond; //The millis counter to see when a second rolls by
unsigned int minutesSinceLastReset; //Used to reset variables after 24 hours, incase we lose track of time
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data
long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
byte windspdavg[120];//Keep an average of wind speeds over 2 minutes
int winddiravg[WIND_DIR_AVG_SIZE]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of largest gust in the last 10 minutes
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//For use in weather machine
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0.0; // [mph instantaneous wind speed]
float windgustmph = 0.0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0.0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0.0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float rainin = 0.0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile float dailyrainin = 0.0; // [rain inches so far today in local time]

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;

//*********************************************************************************************


//*********************************************************************************************
// ***********                          INTERUPT ROUTINES                         *************
//*********************************************************************************************

void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}

//*********************************************************************************************


//*********************************************************************************************
// ***********                          HELPER ROUTINES                           *************
//*********************************************************************************************

void printWiFiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  Serial.print(bssid[5], HEX);
  Serial.print(":");
  Serial.print(bssid[4], HEX);
  Serial.print(":");
  Serial.print(bssid[3], HEX);
  Serial.print(":");
  Serial.print(bssid[2], HEX);
  Serial.print(":");
  Serial.print(bssid[1], HEX);
  Serial.print(":");
  Serial.println(bssid[0], HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();

}

//*********************************************************************************************


//*********************************************************************************************
// ***********                          SENSOR ROUTINES                           *************
//*********************************************************************************************
//Initialize chips.
Adafruit_BME280 bme;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void displayLuxSensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void configureLuxSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  //tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  //tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

}

void dispErr(int errn){
  switch (errn){
    case 1:
      Serial.println("No TSL2561 detected.");
      break;
    case 2:
      Serial.println("No MCP9808 detected");
      break;
    case 3:
      Serial.println("No TSL2561 detected.");
      Serial.println("No MCP9808 detected.");
      break;
    case 4:
      Serial.println("No BME280 detected.");
      break;
    case 5:
      Serial.println("No TSL2561 detected.");
      Serial.println("No BME280 detected.");
      break;
    case 6:
      Serial.println("Couldn't find MCP9808!");
      Serial.println("No BME280 detected.");
      break;
    case 7:
      Serial.println("No TSL2561 detected.");
      Serial.println("No MCP9808 detected");
      Serial.println("No BME280 detected.");
      break;
    default:
      Serial.println("Good to go!");
      break;
  }
}

//*********************************************************************************************


//*********************************************************************************************
// ***********                             SETUP ROUTINE                          *************
//*********************************************************************************************
//WiFiServer server(7001);
WiFiUDP Udp;
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(SERIAL_BAUD);

  //For Adafruit Wifi Feather
  WiFi.setPins(8,7,4,2);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  
  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWiFiData();

  int err;
  bool error = true;
  while(error){
    err = 0;
    error = false;
    if (!bme.begin()) {
      error = true;
      err += 1;
    }
    if (!tempsensor.begin()) {
      error = true;
      err += 2;
    }
    if (!tsl.begin()) {
      error = true;
      err += 4;
    }
    dispErr(err);

    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

    pinMode(WDIR, INPUT);
    pinMode(REFERENCE_3V3, INPUT);

    midnightReset(); //Reset rain totals

    seconds = 0;
    lastSecond = millis();

    //attach external interrupt pins to IRQ functions
    attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
    attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);

    // turn on interrupts
    interrupts();

    /* Display some basic information on the lux sensor */
    displayLuxSensorDetails();
  
    /* Setup the Lux sensor gain and integration time */
    configureLuxSensor();
  }
  Serial.println("Error free");
  //server.begin();
  Udp.begin(7001);
}

void loop() {
  
  double c = tempsensor.readTempC();
  double humidity = bme.readHumidity();
  double pressure = bme.readPressure() / 100.0F;
  double altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  double temp = c * 9.0 / 5.0 + 32;
  //Keep track of which minute it is
  if((lastSecond - millis()) >= 1000){
    lastSecond = millis();
    //Take a speed and direction reading every second for 2 minute average
    if(++seconds_2m > 119) seconds_2m = 0;

    //Calc the wind speed and direction every second for 120 second to get 2 minute average
    windspeedmph = get_wind_speed();
    winddir = get_wind_direction();
    windspdavg[seconds_2m] = (int)windspeedmph;
    winddiravg[seconds_2m] = winddir;
    //if(seconds_2m % 10 == 0) displayArrays();

    //Check to see if this is a gust for the minute
    if(windspeedmph > windgust_10m[minutes_10m])
    {
      windgust_10m[minutes_10m] = windspeedmph;
      windgustdirection_10m[minutes_10m] = winddir;
    }

    //Check to see if this is a gust for the day
    //Resets at midnight each night
    if(windspeedmph > windgustmph)
    {
      windgustmph = windspeedmph;
      windgustdir = winddir;
    }


    //If we roll over 60 seconds then update the arrays for rain and windgust
    if(++seconds > 59){
      seconds = 0;

      if(++minutes > 59) minutes = 0;
      if(++minutes_10m > 9) minutes_10m = 0;

      rainHour[minutes] = 0; //Zero out this minute's rainfall amount
      windgust_10m[minutes_10m] = 0; //Zero out this minute's gust

      minutesSinceLastReset++; //It's been another minute since last night's midnight reset
    }
    
  }


  //If we go for more than 24 hours without a midnight reset then force a reset
  //24 hours * 60 mins/hr = 1,440 minutes + 10 extra minutes. We hope that Imp is doing it.
  if(minutesSinceLastReset > (1440 + 10)){
    midnightReset(); //Reset a bunch of variables like rain and daily total rain
    //Serial.print("Emergency midnight reset");
  }

  sensors_event_t event;
  tsl.getEvent(&event);

  char json[512];
  char header[50];
  snprintf(json, 512,
  "{\"Temperature\": %0.2f, \"Humidity\": %0.2f, \"Pressure\": %0.2f, \"Altitude\": %0.2f, \"Lumens\": %0.2f,"
  " \"Wind Speed\": %0.2f, \"Wind Gust\": %0.2f, \"Direction\": %d, \"Gust Direction\": %d,"
  " \"Wind Speed Avg 2m\": %0.2f, \"Wind Dir Avg 2m\": %d,  \"Wind Gust 10m\": %0.2f, \"Wind Gust Dir 10m\": %d," 
  " \"Rain\": %0.2f, \"Daily Rain\": %0.2f, \"RSSI\": %ld }", 
  temp, humidity, pressure, altitude, event.light, windspeedmph, windgustmph, winddir, windgustdir, windspdmph_avg2m,\
  winddir_avg2m, windgustmph_10m, windgustdir_10m, rainin, dailyrainin, WiFi.RSSI());
  snprintf(header, 50, "Payload size: %d\r\n", strlen(json));
  
//  WiFiClient client = server.available();
//  if(client){
//    while(client.connected()){
//      if(client.available()){
//        char c = client.read();
//        if (c == '\n'){
//          client.print(header);
//          client.print(json);
//        }
//        client.flush(); 
//      }
//    }
//    client.stop();
//  }

  Udp.beginPacket("192.168.0.255", 7001);
  Udp.write(json);
  Udp.endPacket();
  delay(250);
}

//Prints the various arrays for debugging
void displayArrays()
{
  //Windgusts in this hour
  Serial.println();
  Serial.print(minutes);
  Serial.print(":");
  Serial.println(seconds);

  Serial.print("Windgust last 10 minutes:");
  for(int i = 0 ; i < 10 ; i++)
  {
    if(i % 10 == 0) Serial.println();
    Serial.print(" ");
    Serial.print(windgust_10m[i]);
  }

  //Wind speed avg for past 2 minutes
  /*Serial.println();
   Serial.print("Wind 2 min avg:");
   for(int i = 0 ; i < 120 ; i++)
   {
   if(i % 30 == 0) Serial.println();
   Serial.print(" ");
   Serial.print(windspdavg[i]);
   }*/

  //Rain for last hour
  Serial.println();
  Serial.print("Rain hour:");
  for(int i = 0 ; i < 60 ; i++)
  {
    if(i % 30 == 0) Serial.println();
    Serial.print(" ");
    Serial.print(rainHour[i]);
  }

}

//When the imp tells us it's midnight, reset the total amount of rain and gusts
void midnightReset()
{
  dailyrainin = 0; //Reset daily amount of rain

  windgustmph = 0; //Zero out the windgust for the day
  windgustdir = 0; //Zero out the gust direction for the day

  minutes = 0; //Reset minute tracker
  seconds = 0;
  lastSecond = millis(); //Reset variable used to track minutes

  minutesSinceLastReset = 0; //Zero out the backup midnight reset variable
}

//Calculates each of the variables that wunderground is expecting
void calcWeather()
{
  //current winddir, current windspeed, windgustmph, and windgustdir are calculated every 100ms throughout the day

  //Calc windspdmph_avg2m
  float temp = 0;
  for(int i = 0 ; i < 120 ; i++)
    temp += windspdavg[i];
  temp /= 120.0;
  windspdmph_avg2m = temp;

  //Calc winddir_avg2m, Wind Direction
  //You can't just take the average. Google "mean of circular quantities" for more info
  //We will use the Mitsuta method because it doesn't require trig functions
  //And because it sounds cool.
  //Based on: http://abelian.org/vlf/bearings.html
  //Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
  long sum = winddiravg[0];
  int D = winddiravg[0];
  for(int i = 1 ; i < WIND_DIR_AVG_SIZE ; i++)
  {
    int delta = winddiravg[i] - D;

    if(delta < -180)
      D += delta + 360;
    else if(delta > 180)
      D += delta - 360;
    else
      D += delta;

    sum += D;
  }
  winddir_avg2m = sum / WIND_DIR_AVG_SIZE;
  if(winddir_avg2m >= 360) winddir_avg2m -= 360;
  if(winddir_avg2m < 0) winddir_avg2m += 360;


  //Calc windgustmph_10m
  //Calc windgustdir_10m
  //Find the largest windgust in the last 10 minutes
  windgustmph_10m = 0;
  windgustdir_10m = 0;
  //Step through the 10 minutes
  for(int i = 0; i < 10 ; i++)
  {
    if(windgust_10m[i] > windgustmph_10m)
    {
      windgustmph_10m = windgust_10m[i];
      windgustdir_10m = windgustdirection_10m[i];
    }
  }

  //Total rainfall for the day is calculated within the interrupt
  //Calculate amount of rainfall for the last 60 minutes
  rainin = 0;
  for(int i = 0 ; i < 60 ; i++)
    rainin += rainHour[i];
}


//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  return(windSpeed);
}

int get_wind_direction()
// read the wind direction sensor, return heading in degrees
{
  unsigned int adc;

  adc = averageAnalogRead(WDIR); // get the current reading from the sensor
  
  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  if (adc < 95) return (113);
  if (adc < 110) return (68);
  if (adc < 140) return (90);
  if (adc < 200) return (158);
  if (adc < 250) return (135);
  if (adc < 300) return (203);
  if (adc < 355) return (180);
  if (adc < 490) return (23);
  if (adc < 540) return (45);
  if (adc < 670) return (248);
  if (adc < 700) return (225);
  if (adc < 775) return (338);
  if (adc < 835) return (0);
  if (adc < 880) return (293);
  if (adc < 930) return (315);
  if (adc < 985) return (270);
  return (-1); // error, disconnected?
}

//Reports the weather string to the Imp
void reportWeather()
{
  calcWeather(); //Go calc all the various sensors

  Serial.print("$,winddir=");
  Serial.print(winddir);
  Serial.print(",windspeedmph=");
  Serial.print(windspeedmph, 1);
  Serial.print(",windgustmph=");
  Serial.print(windgustmph, 1);
  Serial.print(",windgustdir=");
  Serial.print(windgustdir);
  Serial.print(",windspdmph_avg2m=");
  Serial.print(windspdmph_avg2m, 1);
  Serial.print(",winddir_avg2m=");
  Serial.print(winddir_avg2m);
  Serial.print(",windgustmph_10m=");
  Serial.print(windgustmph_10m, 1);
  Serial.print(",windgustdir_10m=");
  Serial.print(windgustdir_10m);
  Serial.print(",rainin=");
  Serial.print(rainin, 2);
  Serial.print(",dailyrainin=");
  Serial.print(dailyrainin, 2);

  Serial.print(",");
  Serial.println("#,");

  //Test string
  //Serial.println("$,winddir=270,windspeedmph=0.0,windgustmph=0.0,windgustdir=0,windspdmph_avg2m=0.0,winddir_avg2m=12,windgustmph_10m=0.0,windgustdir_10m=0,humidity=998.0,tempf=-1766.2,rainin=0.00,dailyrainin=0.00,-999.00,batt_lvl=16.11,light_lvl=3.32,#,");
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);
}
