#include <Arduino.h>
#include <ESP32AnalogRead.h>
#include <FastLED.h>
#include "BluetoothSerial.h"



#define NUM_LEDS 27
#define DATA_PIN 13
#define BUTTON1_PIN 1
#define BUTTON2_PIN 3
#define BUTTON3_PIN 21
#define ANALOG_CHANNEL 0

#define FRAMES_PER_SECOND  120
#define BRIGHTNESS         150

#define LED_OFF 0
#define LED_RANDOM 1
#define LED_LOCAL_RANDOM 2
#define LED_INCREMENT 3


#define LED_VOLTAGE 100

#define STATENUM 3



CRGB leds[NUM_LEDS];
ESP32AnalogRead adc;

int ledState=0;
int brightness=BRIGHTNESS;
int oldState;
bool stateChanged = false;
bool voltageDone=true;
String buffer;
BluetoothSerial SerialBT;
CRGB pickedColor;
float gama=0;


// put function declarations here:
int readDivVoltage(int r1,int r2, ESP32AnalogRead adcToUse);
void ledOff();
void ledRandomColor(int interval_ms);
void ledLocaclRandomColor(int interval_ms, int spaceSize);
void setPickedCollor();
void ledIncrement(int interval_ms, int increnent);
void changeState();
void showVoltage();
void btRcv();

void nextPattern();
void off();
void yellow();
void use_pallete();
void lava();
void forest();
void cloud();
void ocean();
void rainbow();
void party();
void heat();
void rainbowWithGlitter() ;
void addGlitter( fract8 chanceOfGlitter);
void confetti();
void sinelon();
void bpm();
void juggle();

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
    #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
   #endif

   #if !defined(CONFIG_BT_SPP_ENABLED)
   #error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip. 
    #endif

void setup() {
  // put your setup code here, to run once:
  adc.attach(analogChannelToDigitalPin(ANALOG_CHANNEL));
	Serial.begin(115200);
  SerialBT.begin("Aether light");
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  for (size_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i]=(0); 
  }
  pinMode(BUTTON1_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), changeState, FALLING);
  pinMode(BUTTON2_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), changeState, FALLING);
  pinMode(BUTTON3_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON3_PIN), changeState, FALLING);
  
  
  /*pinMode(BUTTON2_PIN,INPUT_PULLUP);
  pinMode(2,INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), showBatteryStatus, FALLING);*/


  FastLED.setBrightness(BRIGHTNESS);

}


typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { off, yellow, use_pallete, setPickedCollor, rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm };

typedef void (*PalleteList[])();
PalleteList palleteFills = { cloud, lava, ocean, forest, rainbow, party, heat};

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t currentpallete = 0; 
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void loop() {
  /*if(ledState==LED_OFF){
    ledOff();
    delay(1000);
  }else if(ledState==LED_RANDOM){
    ledRandomColor(100);
  }else if(ledState==LED_LOCAL_RANDOM){
    ledLocaclRandomColor(100,100);
  }else if(ledState==LED_INCREMENT){
    if(stateChanged){
      ledSingleColor(CRGB::Yellow);
      stateChanged=false;
    }
    ledIncrement(100,1000);
  }*/

  /* for (size_t i = 0; i < NUM_LEDS; i++)
  {
    //(rand() % 50)
    //leds[i] = CRGB::White;
    leds[i]=(rand() % 255)*(rand() % 255)*(rand() % 255); 
  }*/
  //FastLED.show();
  //delay(500);
  // put your main code here, to run repeatedly:

  /*if(ledState==LED_VOLTAGE){
    showVoltage();
    ledState=oldState;
    voltageDone=true;
  }*/

 // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();  
  // insert a delay to keep the framerate modest
  FastLED.delay(1000/FRAMES_PER_SECOND); 

  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue=(gHue+1)%256; } // slowly cycle the "base color" through the rainbow
  EVERY_N_MILLISECONDS( 500 ) { btRcv(); }
  //EVERY_N_SECONDS( 10 ) { nextPattern(); } // change patterns periodically


}


#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}


void off() 
{
  // FastLED's built-in rainbow generator
  fill_solid(leds, NUM_LEDS,CRGB::Black);
}

void yellow() 
{
  // FastLED's built-in rainbow generator
  fill_solid(leds, NUM_LEDS,CRGB::OrangeRed);
}

void use_pallete(){
  palleteFills[currentpallete]();
}

void lava(){
  //fill_palette(leds, NUM_LEDS,0,256/NUM_LEDS,LavaColors_p,255,LINEARBLEND);
  fill_palette_circular(leds, NUM_LEDS,gHue,LavaColors_p,255,LINEARBLEND,false);
}

void forest(){
  fill_palette_circular(leds, NUM_LEDS,gHue,ForestColors_p,255,LINEARBLEND,false);
}

void cloud(){
  fill_palette_circular(leds, NUM_LEDS,gHue,CloudColors_p,255,LINEARBLEND,false);
}

void ocean(){
  fill_palette_circular(leds, NUM_LEDS,gHue,OceanColors_p,255,LINEARBLEND,false);
}

void rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void party(){
  fill_palette_circular(leds, NUM_LEDS,gHue,PartyColors_p,255,LINEARBLEND,false);
}

void heat(){
  fill_palette_circular(leds, NUM_LEDS,gHue,HeatColors_p,255,LINEARBLEND,false);
}

void rainbowWithGlitter() 
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}


void setPickedCollor(){
  fill_solid(leds,NUM_LEDS,pickedColor);
}



// put function definitions here:
int readDivVoltage(int r1,int r2, ESP32AnalogRead adcToUse){
float vout=  adcToUse.readVoltage();
return ((r1*vout)/r2)+vout;
}

void ledOff(){
for (size_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i]=(0); 
  }
  FastLED.show();
}







void btRcv(){
  char inbuffer[50];
  if(SerialBT.available())
  {
    buffer=SerialBT.readString();
    if(buffer.charAt(0)=='0')
    {
      gCurrentPatternNumber=0;
    }else if(buffer.charAt(0)=='1')
    {
      gCurrentPatternNumber=1;
    }else if(buffer.charAt(0)=='2')
    {
      for (size_t i = 1; i < buffer.length(); i++)
      {
        inbuffer[i-1]=buffer.charAt(i);
      }
      currentpallete=atoi(inbuffer);
      gCurrentPatternNumber=2;
    }else if(buffer.charAt(0)=='3')
    {
      for (size_t i = 1; i < buffer.length(); i++)
      {
        inbuffer[i-1]=buffer.charAt(i);
      }
      //pickedColor=applyGamma_video(atoi(inbuffer),gama);
      pickedColor=atoi(inbuffer);
      gCurrentPatternNumber=3;
    }else if(buffer.charAt(0)=='4')
    {
      gCurrentPatternNumber=4;
    }else if(buffer.charAt(0)=='5')
    {
      gCurrentPatternNumber=5;
    }else if(buffer.charAt(0)=='6')
    {
      gCurrentPatternNumber=6;
    }else if(buffer.charAt(0)=='B')
    {
      for (size_t i = 1; i < buffer.length(); i++)
      {
        inbuffer[i-1]=buffer.charAt(i);
      }
      brightness=atoi(inbuffer);
      FastLED.setBrightness(brightness);
    }
    
  }
  
}




void showBatteryStatus(){
  if(voltageDone){
    voltageDone=false;
    oldState=ledState;
    ledState=LED_VOLTAGE;
  }
}

void changeState(){
  nextPattern();
  /*if((ledState<STATENUM)){
  ledState++;
  }else{
    ledState=0;
  }
  stateChanged=true;*/

}