//Solomon Olshen
//Soycott Razzak

// CPE 301 ACTIVE SONAR FINAL PROJECT

#include "RTClib.h"
#include <LiquidCrystal.h>
#include <dht.h>

// UART REGISTERS
#define TBE 0x20 
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// TIMER 1 REGISTERS
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// TIMER 3 REGISTERS
volatile unsigned char *myTCCR3A = (unsigned char *) 0x90;
volatile unsigned char *myTCCR3B = (unsigned char *) 0x91;
volatile unsigned char *myTCCR3C = (unsigned char *) 0x92;
volatile unsigned char *myTIMSK3 = (unsigned char *) 0x71;
volatile unsigned int  *myTCNT3  = (unsigned  int *) 0x94;
volatile unsigned char *myTIFR3 =  (unsigned char *) 0x38;
volatile unsigned int *myICR3 =  (unsigned int *) 0x96;
volatile unsigned int *myOCR3A =  (unsigned int *) 0x98;

// TIMER 4 REGISTERS
volatile unsigned char *myTCCR4A = (unsigned char *) 0xA0;
volatile unsigned char *myTCCR4B = (unsigned char *) 0xA1;
volatile unsigned char *myTCCR4C = (unsigned char *) 0xA2;
volatile unsigned char *myTIMSK4 = (unsigned char *) 0x72;
volatile unsigned int  *myTCNT4  = (unsigned  int *) 0xA4;
volatile unsigned char *myTIFR4 =  (unsigned char *) 0x39;

// ANALOG TO DIGITAL CONVERTER REGISTERS
volatile unsigned char *myADCSRA = (unsigned char *)0x7A;
volatile unsigned char *myADMUX = (unsigned char *)0x7C;
volatile unsigned char *myDIDR0 = (unsigned char *)0x7E;
unsigned int *portADCDataRegister = (unsigned short *)0x78;

// PORT H REGISTERS
volatile unsigned char* pin_h = (volatile unsigned char*) 0x100;
unsigned char* ddr_h = (unsigned char*) 0x101;
unsigned char* port_h = (unsigned char*) 0x102;

// PORT E REGISTERS
unsigned char* ddr_E = (unsigned char*) 0x2D;
unsigned char* port_E = (unsigned char*) 0x2E;

// PORT F REGISTERS
unsigned char *ddr_f = (unsigned char *)0x30;
unsigned char *port_f = (unsigned char *)0x31;

// PORT A REGISTERS
unsigned char *ddr_A = (unsigned char *)0x21;
unsigned char *port_A = (unsigned char *)0x22;
unsigned char *pin_A = (unsigned char *)0x20;

// PORT B REGISTERS
unsigned char *ddr_B = (unsigned char *)0x24;
unsigned char *port_B = (unsigned char *)0x25;
unsigned char *pin_B = (unsigned char *)0x23;

// Global variables
const int RS = 22, EN = 24, D4 = 26, D5 = 27, D6 = 25, D7 = 23;
volatile unsigned long currentTicks = 0;
volatile unsigned long lastTicks = 0;

volatile unsigned int TimerTime = 0;


volatile unsigned int System_State = 0;

volatile unsigned int Idle_LED = 5; //pin 11
volatile unsigned int Active_LED = 6; //pin 12
volatile unsigned int Event_LED = 7; //pin 13

const int Buzzer = 7; //pin 29

const int DHT11_Power = 3; //pin 6

const int trg1 = 6; //pin 9
const int echo1 = 5; //pin 8

const int servo = 3; //pin 5

const int Start_Btn = 5; //pin 3
const int Stop_Btn = 4; //pin 3

const byte interruptPin1 = 3; //pin 3
const byte interruptPin2 = 2; //pin 2

int Latch = 0; // used to determine which direction idle scan is turning (initially right)
int angle = 3000; // initial idle angle
int Obj_1 = 0; // used to dertermine positive object confidence
int Obj_0 = 0; // used to dertermine positive no object confidence
int rate = 20; // idle servo angle change per loop
int LBS = 1; // "Last Button State"
int ManualToggle = 0; // Logic to determine is manual mode is on or off
int L_OBJ_State = 0;
int OBJ_Pos = 0;
int i = 1;
volatile unsigned int Spd_Sound = 0;

RTC_DS1307 rtc;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

dht DHT;
#define DHT11_PIN 10 //pin 10


void setup(){

    rtc.begin();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    //Temperature and Humidiy module (DHT11) Init.
    *ddr_h |= (1 << DHT11_Power);
    *port_h |= (1 << DHT11_Power);
    int chk = DHT.read11(DHT11_PIN);
    int Air_Temp = DHT.temperature;
    Spd_Sound = (33130 + 0.6*Air_Temp)/2 ;//initializes speed of sound using DHT11 module for high accuracy distance meas.
    *port_h &= ~(1 << DHT11_Power); //Turn power to DHT11 off after getting initial measurment 


    //Start/Stop Button Initialization
    *ddr_E &= ~(1 << Start_Btn); //pin 3 as input, for interupt/start button
    *port_E |= (1 << Start_Btn); // Enables Pull-up resistor for pin 3
    *ddr_E &= ~(1 << Stop_Btn); //pin 2 as input, for interupt/start button
    *port_E |= (1 << Stop_Btn); // Enables Pull-up resistor for pin 2

    //Servo and Ultrasonic Sensor Initialization
    *ddr_E |= (1 << servo);
    *ddr_h |= (1 << trg1);
    *ddr_h &= ~(1 << echo1);
    *port_h |= (1 << echo1);

    //Joystick Initialization
    *ddr_f &= ~(1 << 0); //sets A0 as an input
    *ddr_h &= ~(1 << 4); //set pin 7 as input
    *port_h |= (1 << 4); // Enables Pull-up resistor for pin 7

    //System Idle LED
    *ddr_B |= (1 << Idle_LED); //Sets pin 11 as output
    *port_B |= (1 << Idle_LED); //Writes pin 11 HIGH

    //System Active LED
    *ddr_B |= (1 << Active_LED); //Sets pin 12 as output
    *port_B &= ~(1 << Active_LED); //ensures pin 12 LOW

    //System Event LED
    *ddr_B |= (1 << Event_LED); //Sets pin 13 as output
    *port_B &= ~(1 << Event_LED); //ensures pin 13 LOW

    //Buzzer
    *ddr_A |= (1 << Buzzer); //Sets pin 29 as output
    
    
    *myTCCR1A = 0; // Normal mode
    *myTCCR1B = 0;  
    *myTCCR1C = 0;
    *myTIMSK1 = 0;

    *myTCCR4A = 0; // Normal mode
    *myTCCR4B = 0;  
    *myTCCR4C = 0;
    *myTIMSK4 = 0;

    U0init(9600);
    servo_init();
    adc_init();
    lcd.begin(16, 2);
    lcd.home();
    attachInterrupt(digitalPinToInterrupt(interruptPin1), Start, FALLING);
    attachInterrupt(digitalPinToInterrupt(interruptPin2), Stop, FALLING);


}

void loop(){

    while(!System_State);


    float Distance = 0;

    int BS = (*pin_h & (1 << 4));

    if(*myTIFR4 & 0x01){  
        Timer();                      //If the Timer (Using Timer 4) overflows after being started, the variable accumulates the rough amount of time
        TimerTime = TimerTime + 1; //in seconds that pass each overflow which is 1.049, which should be less computationally cumbersome than using ticks
        *port_A &= ~(1 << Buzzer);
    }

    if(!(BS) && LBS){
        ManualToggle = !ManualToggle;
    }

    LBS = BS;

    Pulse();

    //in cm, and halved to account for distance traveled twice
    Distance = Spd_Sound*currentTicks*0.0000005; // (Relative Speed of Sound)*(Tick Count)*(1/CPU Clock Speed)*(Echo Timer Pre-scaler)
    
    if(Distance < 200){

        *port_B |= (1 << Event_LED);// Write Blue LED HIGH
        Obj_1 = Obj_1 + 1; // Object confidence increase
        Obj_0 = 0; // No Object confidence set zero

        rate = 10; //slows down servo to ensure a stable lock

        if((TimerTime > 10)||i) //if Timer time is roughly more than 15 seconds
        { 
           *port_A |= (1 << Buzzer);
            DateTime now = rtc.now();
            i = 0;

            const char *str1 = "OBJECT DETECTED AT: -> Hour: ";

            while(*str1 != '\0'){
                U0putchar(*str1);
                str1++;
            }

            char bufH[4];
            sprintf(bufH, "%u", now.hour());

            char *h = bufH;          // pointer for traversal
            while (*h != '\0'){     
                U0putchar(*h);
                h++;
            }
            const char *str2 = " | Minute: ";

            while(*str2 != '\0'){
                U0putchar(*str2);
                str2++;
            }

            char bufM[4];
            sprintf(bufM, "%u", now.minute());

            char *m = bufM;          // pointer for traversal
            while (*m != '\0'){     
                U0putchar(*m);
                m++;
            }

            const char *str3 = " | Seconds: ";

            while(*str3 != '\0'){
                U0putchar(*str3);
                str3++;
            }

            char bufS[4];
            sprintf(bufS, "%u", now.second());

            char *s = bufS;          // pointer for traversal
            while (*s != '\0'){     
                U0putchar(*s);
                s++;
            }
            U0putchar('\n');
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Dist.: ");
        lcd.print(Distance);
        lcd.print("cm");

        TimerTime = 0; // These two lines essentially reset the timer every time an object is seen
        Timer();

    }
    else{
        
        *port_B &= ~(1 << Event_LED);
        Obj_0 = Obj_0 + 1;
        rate = 20;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Searching");

        if(!ManualToggle){
            lcd.setCursor(0, 1);
            lcd.print("Mode: Autonomous");
        }
        else{
            lcd.setCursor(0, 1);
            lcd.print("Mode: Manual");
        }

    }
    IdleSearch();

    currentTicks = 0;
    lastTicks = 0;
}

bool PulseDuration()
{     
    while((*pin_h & (1 << echo1)) == 0);
        
        *myTIFR1 |= 0x01; //clear overflow
        *myTCNT1 = 0; //load tick count at 0
        *myTCCR1B &= 0xF8;//stop timer
        *myTCCR1B |= 0x02; //start timer with 8 prescaler
    
    while((*pin_h & (1 << echo1)) && !(*myTIFR1 & 0x01)){}
   
        // wait each interval until overflow OR echo ends
        lastTicks = *myTCNT1;
        *myTCCR1B &= 0xF8;  // stop timer

    if(*pin_h & (1 << echo1)){

        *myTIFR1 |= 0x01;   // clear overflow flag
        currentTicks = currentTicks + 65536;
        if(currentTicks >= 23440){ //stops function if the distance is greater than 201cm
            return false;
        }
        else{
            return true;
        }
    }
    if(!(*pin_h & (1 << echo1))){
        currentTicks = currentTicks + lastTicks;
        *myTIFR1 |= 0x01;   // clear overflow flag
        return false;
    }

}

void Pulse()
{  

    // write trigger LOW
    *port_h &= ~(1 << trg1);
    MicroDelay(32);
    //write trigger HIGH
    *port_h |= (1 << trg1);
    //delay for 10 microseconds
    MicroDelay(160);
    //trigger low
    *port_h &= ~(1 << trg1);

    while(PulseDuration() == true);

}

void MicroDelay(unsigned int ticks)
{
    *myTCCR1B = 0x00;        // stop timer
    *myTCNT1  = 65535 - ticks; // preload value
    *myTIFR1 |= 0x01;        // clear overflow flag
    *myTCCR1B = 0x01;        // start timer with no prescale

    while(!(*myTIFR1 & 0x01));  // wait until overflow

    *myTCCR1B = 0x00;        // stop timer
}
void Timer()
{
    *myTCCR4B = 0x00;        // stop timer
    *myTCNT4  = 0; // preload value
    *myTIFR4 |= 0x01;        // clear overflow flag
    *myTCCR4B = 0x04;        // start timer with prescaler 256

}
void IdleSearch(){

    if(!ManualToggle){ //checks manual toggle state

        if(Obj_1 > 3 && Obj_0 > 1){ // Condition where object confidence threshold is checked
            Latch = !Latch; // servo will switch direction if object confidence was high enough passing an object
            Obj_1 = 0;  //resets object confidence
            L_OBJ_State = 1; // used in other conditions to check if there was an object of confidence
        }
        if(*myOCR3A < 4500 && Latch == 0){
            angle = angle + rate;
            ServoPulse(angle);
        }
        else{
            Latch = 1;
        }

        if(*myOCR3A > 1500 && Latch == 1){
            angle = angle - rate;
            ServoPulse(angle);          
        }
        else{
            Latch = 0;
        }
    
    }

    else{

        int adcVal = adc_read(0);
        rate = (adcVal - 511)*0.1;

        angle = angle + rate;
        ServoPulse(angle);
    }
    
}
unsigned int adc_read(unsigned char adc_channel){
  *myADMUX = (*myADMUX & 0xF0) | (adc_channel & 0x0F);
  *myADCSRA |= 0x40;

  while((*myADCSRA & 0x40) != 0);

  return (*portADCDataRegister & 0x03FF);
}

void ServoPulse(int AngleDelay)
{
    *myOCR3A = AngleDelay * 2;
}

void U0putchar(unsigned char data)
{   
    while (!(*myUCSR0A & TBE));
    *myUDR0 = data;
}

void U0init(unsigned long U0baud)
{
    unsigned long CPUfrequency = 16000000;
    unsigned int baud;
    baud = (CPUfrequency / 16 / U0baud - 1);
    //transmit buffer empty
    *myUCSR0A = 0x20;
    //Transmit and Recieve enable
    *myUCSR0B = 0x18;
    //8 bit character size
    *myUCSR0C = 0x06;
    //Baud rate
    *myUBRR0  = baud;
}
void servo_init()
{
    *myICR3 = 0;
    *myOCR3A = 0;
    *myTCCR3A = 0;
    *myTCCR3B = 0;
    *myTCNT3 = 0;

    // Timer3 Fast PWM, mode 14 (ICR3 as TOP)
    *myTCCR3A |= 0x02; //WGM31
    *myTCCR3B |= 0x18;//WGM32 and WGM33

    // Non-inverting mode on OC3A
    *myTCCR3A |= (1 << 7);//COM3A1

    // Prescaler = 8
    *myTCCR3B |= 0x02;

    // TOP = 20 ms period → 40000 counts
    *myICR3 = 40000;

    // Start centered (1.5 ms)
    *myOCR3A = 3000;
}
void adc_init(){
  
  *myADCSRA |= 0b10000000; //Enables ADC
  *myADCSRA &= 0b11011111; //ADATE = 0, no edge trigger
  *myADCSRA &= 0b11110111; //Disables ADC interupt
  *myADCSRA &= 0b11111000; //Sets ADC Prescalar to a division factor of 2
  
  *myADMUX |= 0b01000000; //AVcc reference
}
void Start(){
    System_State = 1;
    *port_B &= ~(1 << Idle_LED);
    *port_B |= (1 << Active_LED);
}
void Stop(){
    System_State = 0;
    *port_B |= (1 << Idle_LED);
    *port_B &= ~(1 << Active_LED);
    *port_B &= ~(1 << Event_LED);
}