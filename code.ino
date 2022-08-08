#include <PID_v1.h>
#include "STM32TimerInterrupt.h"
#include "STM32_ISR_Timer.h" 
#define   en     PB4
#define   buck   PB3
#define   in1    PB1
#define   in2    PB0
#define   ivt    PB11
#define   aq     PB10
#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 
LiquidCrystal_I2C lcd(0x27,16,2);
STM32Timer ITimer(TIM1); 
STM32_ISR_Timer ISR_Timer; 
int dem=0;
float vout,vset=13;
float vin;
long tg1;
float i_in;
int ma1[]={
50 ,100 ,151 ,201 ,250 ,300 ,349 ,398 ,446 ,494 ,
542 ,589 ,635 ,681 ,726 ,771 ,814 ,857 ,899 ,940 ,
981 ,1020 ,1058 ,1095 ,1131 ,1166 ,1200 ,1233 ,1264 ,1294 ,
1323 ,1351 ,1377 ,1402 ,1426 ,1448 ,1468 ,1488 ,1505 ,1522 ,
1536 ,1550 ,1561 ,1572 ,1580 ,1587 ,1593 ,1597 ,1599 ,1600 ,
1599 ,1597 ,1593 ,1587 ,1580 ,1572 ,1561 ,1550 ,1536 ,1522 ,
1505 ,1488 ,1468 ,1448 ,1426 ,1402 ,1377 ,1351 ,1323 ,1294 ,
1264 ,1233 ,1200 ,1166 ,1131 ,1095 ,1058 ,1020 ,981 ,940 ,
899 ,857 ,814 ,771 ,726 ,681 ,635 ,589 ,542 ,494 ,
446 ,398 ,349 ,300 ,250 ,201 ,151 ,100 ,50 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0};

int ma2[] = {
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,
50 ,100 ,151 ,201 ,250 ,300 ,349 ,398 ,446 ,494 ,
542 ,589 ,635 ,681 ,726 ,771 ,814 ,857 ,899 ,940 ,
981 ,1020 ,1058 ,1095 ,1131 ,1166 ,1200 ,1233 ,1264 ,1294 ,
1323 ,1351 ,1377 ,1402 ,1426 ,1448 ,1468 ,1488 ,1505 ,1522 ,
1536 ,1550 ,1561 ,1572 ,1580 ,1587 ,1593 ,1597 ,1599 ,1600 ,
1599 ,1597 ,1593 ,1587 ,1580 ,1572 ,1561 ,1550 ,1536 ,1522 ,
1505 ,1488 ,1468 ,1448 ,1426 ,1402 ,1377 ,1351 ,1323 ,1294 ,
1264 ,1233 ,1200 ,1166 ,1131 ,1095 ,1058 ,1020 ,981 ,940 ,
899 ,857 ,814 ,771 ,726 ,681 ,635 ,589 ,542 ,494 ,
446 ,398 ,349 ,300 ,250 ,201 ,151 ,100 ,50 ,0};

//////////////////////
double kp=4,ki=10,kd=0.001;
long pwm1;
int adc_f;
float p,ptrc;
double input=0, output=0, setpoint=0;
PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
int pwmbuck;
void ngattimer()
{ 
    dem++;if(dem>=200)dem=0;
    analogWrite(in1,ma1[dem]);
    analogWrite(in2,ma2[dem]); 
   
} 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for(int i=0;i<100;i++)
  {
    
    ma1[i]=250*sin(100*3.14*(float)i/10000.0);
    ma2[i]=0;
   
    }
    
  for(int i=100;i<200;i++)
  {
    ma2[i]=250*sin(100*3.14*(float)(i-100)/10000.0);
    ma1[i]=0;
    } 
  pinMode(ivt,OUTPUT);
  pinMode(aq,OUTPUT); 
  pinMode(PA0,INPUT);
  pinMode(PA1,INPUT);
  pinMode(PA2,INPUT);
  pinMode(en,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(buck,OUTPUT);
  digitalWrite(en,0);
  delay(1000);
  analogWriteFrequency(100000);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255,255);
  ITimer.attachInterruptInterval(100, ngattimer);  
  for(int i=0;i<1000;i++)
      { 
        adc_f+=analogRead(PA1);
      } 
  adc_f=(adc_f/1000)*3300/1023;
  Serial.println(adc_f);
  lcd.init();
  lcd.backlight(); 
//C:\Users\Admin\AppData\Local\Temp\arduino_build_384351
}

void loop() {
  for(int i=0;i<1000;i++)
  {
  vin+=analogRead(PA0)*3.3/1023*11.5/1.5;
  digitalWrite(en,1);
  vout+=analogRead(PA2)*3.3/1023*11.5/1.5;
  i_in+=analogRead(PA1);
}
  vin=vin/1000;
  vout=vout/1000;
  i_in=i_in/1000*3300/1023;
  i_in=(i_in-2500)/100;
  if(vin>=16)
  {
        digitalWrite(en,1);
        digitalWrite(aq,0);
        input = vout; 
        setpoint=vset;
        //setpoint=20;
        pwm1+=output;
        if(pwm1>180)pwm1=180;
        if(pwm1<0)pwm1=0;
        buck_out(pwm1);
        
        while(!myPID.Compute()){}
        ////tim diem cong suat cuc dai
        if(vout-vset<=2)
        {
          ptrc=p;
          p=vin*i_in;
          if(p>ptrc)vset+=0.05;
          if(p<ptrc)vset-=0.05;
          if(vset>=15)vset=15;
          if(vset<13)vset=13;
          }
          
    }
  if(vin<16)
  {
    digitalWrite(aq,1);
    digitalWrite(en,0);
  }
  if(vout>12&&vout<14)
          {
            digitalWrite(ivt,1);
           
            }
            else digitalWrite(ivt,0);
  if(millis()-tg1>=500)
  {  
    tg1=millis();
    Serial.print(vin);Serial.print("    ");Serial.print(i_in);Serial.print("    ");Serial.println(vout);
    lcd.setCursor(0,0);lcd.print("Vi:");lcd.print(vin);
    lcd.setCursor(8,0);lcd.print("Vo:");lcd.print(vout);
    lcd.setCursor(0,1);lcd.print("Ii:");lcd.print(i_in);
    lcd.setCursor(8,1);lcd.print("P :");lcd.print(p);
  }
  i_in=0;
  vin=0;
  vout=0;
}
void buck_out(int a)
{
  if(a>250)a=255;
  if(a<0)a=0;
  analogWrite(buck,a);
  }
