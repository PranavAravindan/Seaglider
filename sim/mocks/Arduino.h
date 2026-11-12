#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
using std::isnan; using std::fabs;
typedef unsigned char byte;
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
template<class T> T constrain(T x, T a, T b){ return x<a?a:(x>b?b:x); }
inline float constrain(float x,float a,float b){ return x<a?a:(x>b?b:x);}
unsigned long millis();
void delay(unsigned long ms);
int digitalRead(byte pin);
void pinMode(byte,int);
struct SerialMock{ void begin(long){} void println(const char*s){printf("%s\n",s);} 
 void print(unsigned long v){ if(!quiet) printf("%lu",v);} void print(int v){ if(!quiet)printf("%d",v);} 
 void print(float v,int d=2){ if(!quiet)printf("%.*f",d,v);} void print(char c){ if(!quiet)printf("%c",c);} 
 void println(int v){ if(!quiet)printf("%d\n",v);} bool quiet=false; };
extern SerialMock Serial;
