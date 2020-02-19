

int trigPins[] = {2,9};
int echoPins[] = {3,10};
//const int trigPin1 = 2;
//const int echoPin1 = 3;

//const int trigPin2 = 9;
//const int echoPin2 = 10;

long duration;
int newDistCm;
const int MAX_DIST = 300; //cm
int oldVals[] = {MAX_DIST, MAX_DIST};
const int timeout = MAX_DIST * 2 /0.034;
const int TIMEOUT = 12;
unsigned long mytime;
unsigned long responseTime;
//increase to filter more and reduce sensitivity 
const double LOW_PASS_ALPHA = 0.6;
void setup() {
  
  pinMode(trigPins[0], OUTPUT);
  pinMode(echoPins[0], INPUT);
  
  pinMode(trigPins[1], OUTPUT);
  pinMode(echoPins[1], INPUT);
  Serial.begin(9600);

}

double getDist(int pin) {
  digitalWrite(trigPins[pin], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[pin], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[pin], LOW);
  mytime = millis();
  duration = pulseIn(echoPins[pin], HIGH, timeout);
  responseTime = millis() - mytime;
  //Serial.println(responseTime);
  if (responseTime >= TIMEOUT) {
    newDistCm = MAX_DIST;
  }
  else {
    newDistCm = duration*0.034/2;
  }
  oldVals[pin] = LOW_PASS_ALPHA * oldVals[pin] + (1-LOW_PASS_ALPHA)*newDistCm;
  return oldVals[pin];
}

void loop() {
newDistCm= getDist(0);
String dist0 = String(newDistCm);

delayMicroseconds(10000);

newDistCm = getDist(1);
String both = dist0 + " " + String(newDistCm);
Serial.println(both);
delayMicroseconds(10000);
}
