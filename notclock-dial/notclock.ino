/* notclock
shows no minutes, just the likely color of the sky on a clear day

This is a dial on the wall that turns to show the approximate sky color on a 24 hr clock.

Sunrise and sunset calculations based off of
http://www.esrl.noaa.gov/gmd/grad/solcalc/solareqns.PDF
  original source:
  Almanac for Computers, 1990
	published by Nautical Almanac Office
	United States Naval Observatory
	Washington, DC 20392
*/

#include "math.h"
#include <stdlib.h>
#include "InternetButton.h"

InternetButton b = InternetButton();

float pi = 3.1415927;

// check for time of sunrise and sunset

double lngHour;
int localOffset = -7; // time off from UTC, aka time zone
int sunrise;
int sunset;
int solarNoon;

double dayOfYear;  // out of 365

int lastCheck=0;  // time of the last check
int checkInterval=1000; // the time between checks in ms
int elapsedDayTime; // the time passed since midnight in ms
int lastTime = 0; // the last time we took millis

double latitude= 37.7749;
double longitude= -122.4194;

int intercept[3];
float m[3];
int red;
int grn;
int blu;

int numStates=16;
int state=0;
int newState=1;
int t;  // current elapsed time
int timing[16];

int colorTiming[16][3]=
{
  {20,20,20},
  {0,0,0},
  {0,0,50},
  {20,0,200},
  {255,100,0},
  {255,100,100},
  {200,100,255},
  {50,255,255},
  {0,255,255},
  {50,255,255},
  {100,50,255},
  {200,0,255},
  {20,20,200},
  {0,0,50},
  {0,0,0},
  {20,20,20}
};

int flankingLedBrightness[16]=
  {10,
  0,
  0,
  0,
  10,
  30,
  60,
  80,
  60,
  30,
  10,
  0,
  0,
  0,
  10};

int extraLedBrightness[16]=
  {0,
  0,
  0,
  0,
  0,
  0,
  5,
  20,
  5,
  0,
  0,
  0,
  0,
  0,
  0};

int lastPublish=0;
int publishInterval=2000;
int lastRed=0;
int lastGrn=0;
int lastBlu=0;

void setup() {
  Serial.begin(9600);

  Particle.function("set",setTime);
  Particle.function("latitude",setLatitude);
  Particle.function("longitude",setLongitude);

  Time.zone(localOffset);  // your local time
  lngHour = longitude / 15;
  getTimes();

  // turn on button
  b.begin();

  RGB.control(true);
  RGB.color(0,0,0);

}

void loop() {

  int xValue = b.readX();
  int yValue = b.readY();
  int zValue = b.readZ();

  // for dial:
  float rawAngle=b.lowestLed();
  float angleTime=24+(rawAngle*12/3.14);
  if (angleTime<18.0) {angleTime=angleTime+6.0;}
  else {angleTime=angleTime-18.0;}


  // for sphere:
  /*float rawAngle=b.lowestLedFlat();
  float angleTime=12+(rawAngle*12/3.14);
  Serial.println(angleTime);*/

  // for sphere opposite side:
  /*float rawAngle=b.lowestLedFlat();
  float angleTime=(rawAngle*12/3.14);
  if (angleTime<0) {angleTime=angleTime+24;}
  Serial.println(angleTime);*/

  // both:
  t = angleTime*60*60*1000/checkInterval;

  // figure out where we should be based on sunset, sunrise, and solar noon

  int x = t-timing[state];

  if (newState==1) {

    // every time there is a new state, make the equation
    for (int i=0; i<3; i++) {
      intercept[i] = colorTiming[state][i];
      m[i] = (float(colorTiming[state+1][i])-float(colorTiming[state][i]))/(float(timing[state+1])-float(timing[state]));
      Serial.println("For position "+String(i)+", the equation is y="+String(m[i])+"x+"+String(intercept[i]));
    }

    newState=0;

  }

  red = m[0]*x+intercept[0];
  grn = m[1]*x+intercept[1];
  blu = m[2]*x+intercept[2];

  if (red!=lastRed || grn!=lastGrn || blu!=lastBlu) {
    // for dial:
    b.ledOn(6,red,grn,blu);
    b.ledOn(5,(int)flankingLedBrightness[state]*red/100,(int)flankingLedBrightness[state]*grn/100,(int)flankingLedBrightness[state]*blu/100);
    b.ledOn(7,(int)flankingLedBrightness[state]*red/100,(int)flankingLedBrightness[state]*grn/100,(int)flankingLedBrightness[state]*blu/100);
    b.ledOn(4,(int)extraLedBrightness[state]*red/100,(int)extraLedBrightness[state]*grn/100,(int)extraLedBrightness[state]*blu/100);
    b.ledOn(8,(int)extraLedBrightness[state]*red/100,(int)extraLedBrightness[state]*grn/100,(int)extraLedBrightness[state]*blu/100);

    // for sphere:
    /*b.allLedsOn(red,grn,blu);*/

    if (millis()-lastPublish>publishInterval) {
      char colorData[128];
      sprintf(colorData, "%d,%d,%d", (int)red, (int)grn, (int)blu);
      Serial.println("Publishing "+String(colorData)+"...");
      Particle.publish("notclock",colorData,60,PRIVATE);
      lastPublish=millis();
    }

    lastRed=red;
    lastGrn=grn;
    lastBlu=blu;

  }

  incrementState();

}

void getTimes() {
  /*WiFi.on();*/
  /*Particle.connect();*/
  if (waitFor(Particle.connected, 10000)) {
    Particle.syncTime();

    dayOfYear = getDayOfYear();

    Serial.println("Day Of Year: "+String(dayOfYear));
    Serial.println("Getting sunrise...");
    double sunriseRaw = getSolarVariable(-1);
    int sunriseHr = sunriseRaw;
    int sunriseMin = 60*(sunriseRaw-sunriseHr);
    sunrise = sunriseMin+sunriseHr*60;
    Serial.println();
    Serial.println("Getting sunset...");
    double sunsetRaw = getSolarVariable(1);
    int sunsetHr = sunsetRaw;
    int sunsetMin = 60*(sunsetRaw-sunsetHr);
    sunset = sunsetMin+sunsetHr*60;

    solarNoon = (sunrise+sunset)/2;

    Serial.println();
    Serial.println("Sunrise: "+ String(sunriseHr)+":"+String(sunriseMin));
    Serial.println("Sunset: "+String(sunsetHr)+":"+String(sunsetMin));
    Serial.println();

    /*Particle.disconnect();*/
    /*WiFi.off();*/

    timing[0] = 0;
    timing[1] = 1000*60*2*60/checkInterval;
    timing[2] = (1000*60*(sunrise-60))/checkInterval;
    timing[3] = (1000*60*(sunrise)-60*1000*15)/checkInterval;
    timing[4] = 1000*60*sunrise/checkInterval;
    timing[5] = (1000*60*(sunrise)+60*1000*15)/checkInterval;
    timing[6] = (1000*60*(sunrise)+60*1000*60)/checkInterval;
    timing[7] = (1000*60*(sunrise+solarNoon)/2)/checkInterval;
    timing[8] = 1000*60*solarNoon/checkInterval;
    timing[9] = (1000*60*(sunset+solarNoon)/2)/checkInterval;
    timing[10] = (1000*60*(sunset)-60*1000*15)/checkInterval;
    timing[11] = 1000*60*sunset/checkInterval;
    timing[12] = (1000*60*(sunset)+60*1000*15)/checkInterval;
    timing[13] = (1000*60*(sunset)+60*1000*60)/checkInterval;
    timing[14] = 1000*60*22*60/checkInterval;
    timing[15] = 1000*60*24*60/checkInterval;

    // check on the timing intervals

    Serial.println("elapsedDayTime: "+String(elapsedDayTime));
    Serial.println("sunrise: "+String(sunrise));
    Serial.println("sunset: "+String(sunset));
    Serial.println("Timing array: ");
    for (int x=0; x<16; x++) {
      Serial.println(timing[x]);
    }

  }

  else { Particle.process(); }

}

double getSolarVariable(int state) {
  // equations from http://williams.best.vwh.net/sunrise_sunset_algorithm.htm

  double T;

  if (state<0) {  // rise
    T = dayOfYear + ((6.0 - lngHour) / 24.0);
  }
  else if (state>0) { // set
    T = dayOfYear + ((18.0 - lngHour) / 24.0);
  }
  Serial.println("dayOfYear: "+String(dayOfYear));
  Serial.println("lngHour: "+String(lngHour));
  Serial.println("T: "+String(T));

  double zenith = 90.833333;
  double meanAnomaly = (0.9856 * T) - 3.289;
  Serial.println("meanAnomaly: "+String(meanAnomaly));
  double trueLongitude = meanAnomaly + (1.916 * sin((pi/180)*meanAnomaly)) + (0.020 * sin((pi/180)* 2 * meanAnomaly)) + 282.634;
  Serial.println("trueLongitude: "+String(trueLongitude));
  int trueLongitudeInt=trueLongitude;
  double trueLongitudeRemainder=trueLongitude-trueLongitudeInt;
  trueLongitude=trueLongitudeInt%360 + trueLongitudeRemainder;
  if (trueLongitude<0) { trueLongitude = trueLongitude + 360; }
  Serial.println("trueLongitude converted: "+String(trueLongitude));
  double rightAscension = (180/pi)*atan(0.91764 * tan((pi/180)*trueLongitude));
  int rightAscensionInt=rightAscension;
  double rightAscensionRemainder=rightAscension-rightAscensionInt;
  rightAscension=rightAscensionInt%360 + rightAscensionRemainder;
  if (rightAscension<0) { rightAscension = rightAscension+360; }
  Serial.println("rightAscension: "+String(rightAscension));
  int leftQuadrant = (floor( trueLongitude/90)) * 90;
  Serial.println("leftQuadrant: "+String(leftQuadrant));
	int RAquadrant = (floor(rightAscension/90)) * 90;
  Serial.println("RAquadrant: "+String(RAquadrant));
  rightAscension = (rightAscension + (leftQuadrant - RAquadrant))/15;
  Serial.println("rightAscension: "+String(rightAscension));
  double sinDec = 0.39782 * sin((pi/180)*trueLongitude);
  Serial.println("sinDec: "+String(sinDec));
  double cosDec = cos(asin(sinDec));
  Serial.println("cosDec: "+String(cosDec));
  double cosH = (cos((pi/180)*zenith) - (sinDec * sin((pi/180)*latitude))) / (cosDec * cos((pi/180)*latitude));
  Serial.println("cosH: "+String(cosH));

  double hourAngle;

  if (state<0) {  // rise
    hourAngle=(360 - (180/pi)*acos(cosH))/15;
  }
  else if (state>0) { // set
    hourAngle = (180/pi)*(acos(cosH))/15;
  }

  Serial.println("hourAngle: "+String(hourAngle));

  double localMeanTime = hourAngle + rightAscension - (0.06571 * T) - 6.622;
  double UTCTime = (localMeanTime - lngHour);
  int UTCTimeInt = UTCTime;
  double UTCTimeRemainder = UTCTime-UTCTimeInt;
  UTCTime = UTCTimeInt%24 + UTCTimeRemainder;
  double localTime = UTCTime + localOffset;
  if (localTime<0) {
    localTime=localTime+24;
  }

  Serial.println("localMeanTime: "+String(localMeanTime));
  Serial.println("UTCTime: "+String(UTCTime));
  Serial.println("localTime: "+String(localTime));

  return localTime;
}

int getDayOfYear() {  // equation from http://williams.best.vwh.net/sunrise_sunset_algorithm.htm

  int month = Time.month();
  int day = Time.day();
  int year = Time.year();

  Serial.println("Month: "+String(month));
  Serial.println("Day: "+String(day));
  Serial.println("Year: "+String(year));

  int N1 = floor(275 * month / 9);
	int N2 = floor((month + 9) / 12);
	int N3 = (1 + floor((year - 4 * floor(year / 4) + 2) / 3));
	int N = N1 - (N2 * N3) + day - 30;

  return N;
}

void incrementState() {

  // find the range and put it there

  for (int q=0; q<numStates; q++) {
    if (t>timing[q] && t<timing[q+1]) {
      if (state!=q) {newState=1;}
      state=q;
    }
  }

  if (state!=numStates-1) {
    if (t>timing[state+1]) {
      state++;
      newState=1;
      Serial.print("State is now "); Serial.print(state); Serial.print(" at "); Serial.print(t); Serial.print(" until "); Serial.println(timing[state+1]);
    }
  }
  else {
    if (t<timing[1]) {
      state=0;
      newState=1;
      Serial.print("State is now "); Serial.print(state); Serial.print(" at "); Serial.print(t); Serial.print(" until "); Serial.println(timing[state+1]);
    }
  }
}

int setTime(String command) {
  // sets the time that you want to reset the device to
  char inputStr[64];
  command.toCharArray(inputStr,64);
  t = atoi(inputStr);
  return t;
}

int setLatitude(String command) {
  char inputStr[64];
  command.toCharArray(inputStr,64);
  latitude=atof(inputStr);
  getSolarVariable(state);

  return latitude;
}

int setLongitude(String command) {
  char inputStr[64];
  command.toCharArray(inputStr,64);
  longitude=atof(inputStr);
  return longitude;
}
