//ultrasonico solo

#define TRIG1 4
#define ECHO1 3
int duration1, duration2;
float distance1, distance2;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG1, OUTPUT);
  
  pinMode(ECHO1, INPUT);
}
void loop() {
digitalWrite(TRIG1, 0);
  
delayMicroseconds(2);
digitalWrite(TRIG1, 1);

delayMicroseconds(10);
digitalWrite(TRIG1, 0);


duration1 = pulseIn(ECHO1,1);

distance1 = duration1*0.34/2;

Serial.println("\t US1: " + String(distance1));
delay(100);
}