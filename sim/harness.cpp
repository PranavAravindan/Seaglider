#include "Arduino.h"
#include "Servo.h"
#include "Adafruit_BMP280.h"
#include <cstring>
SerialMock Serial;
// ---- toy plant ----
static int g_cmd=90; static unsigned long g_t=0;
static double pos=0, depth=0, vel=0;          // pos in ms (0=dived end)
static double TRUE_TRAVEL=15500, TRUE_NEUTRAL=0.40*15500, SPEED_SCALE=0.85, KV=0.20, TAU=2.0;
static double P0=101325; static double noise=30; static int spikeAt=-1;
static double maxDepth=0, minAfter=1e9, maxAfter=-1e9;
static void step(){
  double d=(g_cmd-90)/90.0*SPEED_SCALE;          // ms per ms
  pos=std::min(std::max(pos+d,0.0),TRUE_TRAVEL);
  double b=(pos-TRUE_NEUTRAL)/TRUE_TRAVEL;       // + buoyant
  double vt=-KV*b*2;                             // + = descending
  vel+= (vt-vel)/(TAU*1000.0);
  depth+=vel/1000.0; if(depth<0){depth=0;vel=0;}
  g_t++;
  maxDepth=std::max(maxDepth,depth);
  if(g_t>200000){minAfter=std::min(minAfter,depth);maxAfter=std::max(maxAfter,depth);}
}
unsigned long millis(){ step(); return g_t; }
void delay(unsigned long ms){ for(unsigned long i=0;i<ms;i++) step(); }
int digitalRead(byte){ return pos<=0.0?LOW:HIGH; }
void pinMode(byte,int){}
void Servo::write(int v){ g_cmd=v; }
float Adafruit_BMP280::readPressure(){
  double n=((rand()%2001)-1000)/1000.0*noise;
  if(spikeAt>0 && g_t>(unsigned long)spikeAt && g_t<(unsigned long)spikeAt+300) return P0+9780*(depth+2.5)+n; // 2.5 m glitch for 0.3 s
  return P0+997*9.80665*depth+n; }
float Adafruit_BMP280::readAltitude(float){return 0;}
#include SKETCH
int main(int argc,char**argv){
  Serial.quiet=true;
  if(argc>1 && !strcmp(argv[1],"spike")) spikeAt=60000;  // inject a 0.3 s, 2.5 m pressure glitch
  setup();
  Serial.quiet = (argc<=2);
  for(unsigned long i=0;i<400000;i++){ loop(); }
  printf("final depth %.2f m, max depth %.2f m, depth range over last 200s: %.2f..%.2f m, plunger true %.0f ms\n",depth,maxDepth,minAfter,maxAfter,pos);
}
