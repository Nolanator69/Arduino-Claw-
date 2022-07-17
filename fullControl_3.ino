//please forgive the shitty tabbing

#include <Servo.h>        // include servo library
#include <NewPing.h>      // include the NewPing library for this program

#define VCC_PIN 13
#define TRIGGER_PIN 12    // sonar trigger pin will be attached to Arduino pin 12
#define ECHO_PIN 11       // sonar echo point will be attached to Arduino pin 11
#define GROUND_PIN 10
#define MAX_DISTANCE 200  // maximum distance set to 200 cm
#define TRUE 1 //boolean expressions for conditional statements
#define FALSE 0
#define MAX_POS 130 //max amound of rotation
#define MIN_POS 0 //starting rotational value
#define INCREMENT 2 //amount claw opens by each sweep loop cycle
#define DECREMENT -2 //amount claw closes by each loop cycle
#define MIN_DISTANCE 4 //minimum distance to trigger opening or closing
#define SECOND 1000 //1000 milliseconds, for delay
#define NUMTICKS 3
#define PINGDELAY 15
#define MOVEDELAY 10

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // initialize NewPing
Servo myServo;

void sweep(int s_start, int s_stop, int s_step);
int get_distance();
void close_arm();
void open_arm();
void control_switch(int flipval);
int ticked_off(int tickerValue, int distance);

void setup() //setup function
{
  Serial. begin(9600);            // set data transmission rate to communicate with computer
  pinMode(ECHO_PIN, INPUT) ;  
  pinMode(TRIGGER_PIN, OUTPUT) ;
  pinMode(GROUND_PIN, OUTPUT);    // tell pin 10 it is going to be an output
  pinMode(VCC_PIN, OUTPUT);       // tell pin 13 it is going to be an output
  digitalWrite(GROUND_PIN,LOW);   // tell pin 10 to output LOW (OV, or ground)
  digitalWrite(VCC_PIN, HIGH) ;   // tell pin 13 to output HIGH (+5V)
  myServo.attach(9);
}

int cond = TRUE; //conditional to open/close arm

void loop()
//runs claw 
{
  if(ticked_off(NUMTICKS, MIN_DISTANCE)){
    control_switch(cond);  
    if(cond == TRUE)
      cond = FALSE;    
    else if(cond == FALSE)
      cond = TRUE;
  }
}


void sweep(int s_start, int s_stop, int s_step)
//PARAM: s_start: starting position
//PARAM: s_stop: end position
//PARAM: s_step: position adjustment value
{
  int pos;
  if(s_stop>s_start){
    for(pos = s_start; pos < s_stop; pos += s_step){
      myServo.write(pos);
      delay(MOVEDELAY);      
      }
    }
  
  else if(s_stop<s_start){
    for(pos = s_start; pos > s_stop; pos += s_step){
      myServo.write(pos);
      delay(MOVEDELAY);      
      }
  }
}


int get_distance()
//RETURN: integer distance value in cm
{
  int distance = sonar.ping_cm();   // read the sonar sensor, using a variable
  Serial.print("Ping: ");                 //print “Ping:" on the computer display
  Serial.print(distance);           //print the value of the variable next
  return distance;
}


void open_arm(){
  sweep(MIN_POS,MAX_POS,INCREMENT);
}


void close_arm(){
  sweep(MAX_POS,MIN_POS,DECREMENT);
}


void control_switch(int flipval)
//PARAM flipval: conditional value to dictate whether to open or close arm
{
  if(flipval==FALSE)
    open_arm();
  else if(flipval==TRUE)
    close_arm();
}


int ticked_off(int tickerValue, int distance)
{
 int ticks = 0; 
 int zeroCount = 0;
 while(ticks<=tickerValue){
  int newDist = get_distance();
  delay(PINGDELAY);
  if (newDist == distance && newDist!=0){
    ticks++;
  }
  else{
    return FALSE; 
  }
 }

return TRUE;
}
