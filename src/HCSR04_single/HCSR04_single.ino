//Only one HC-SR04
#define TRIG 9
#define ECHO 8
int duration;
float distance_US;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);

  pinMode(ECHO, INPUT_PULLUP);
}
void loop() {
  digitalWrite(TRIG, 0);

  delayMicroseconds(2);
  digitalWrite(TRIG, 1);

  delayMicroseconds(10);
  digitalWrite(TRIG, 0);


  duration = pulseIn(ECHO, 1);

  distance_US = duration * 0.34 / 2;

  Serial.println("\t US1: " + String(distance_US));
  delay(100);
}