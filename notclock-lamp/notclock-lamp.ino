// These lamps respond to the notclock dial with a delayed response, changing to the color of the notclock every few seconds.

#include "InternetButton.h"

InternetButton b = InternetButton();
int redNext=10;
int grnNext=10;
int bluNext=10;

int red=0;
int grn=0;
int blu=0;

void setup() {
  b.begin();
  b.allLedsOff();
  Serial.begin(9600);
  Particle.subscribe("notclock",notclockHandler,MY_DEVICES);
  RGB.control(true);
  RGB.color(0,0,0);
}

void loop() {

  int changeColor=0;

  if (red!=redNext) {
    if (red<redNext) { red++; }
    else if (red>redNext) { red--; }
    changeColor=1;
  }

  if (grn!=grnNext) {
    if (grn<grnNext) { grn++; }
    else if (grn>grnNext) { grn--; }
    changeColor=1;
  }

  if (blu!=bluNext) {
    if (blu<bluNext) { blu++; }
    else if (blu>bluNext) { blu--; }
    changeColor=1;
  }

  if (changeColor==1) { b.allLedsOn(red,grn,blu); }

}


void notclockHandler(const char *event, const char *data) {
  if (data) {
    char input[64];
    strcpy(input,data);
    char *p;
    p = strtok(input,",");
    redNext = atoi(p);
    p = strtok(NULL,",");
    grnNext = atoi(p);
    p = strtok(NULL,",");
    bluNext = atoi(p);
    p = strtok(NULL,",");
  }
}
