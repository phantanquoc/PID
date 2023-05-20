#define TRIG_PIN 12
#define ECHO_PIN 13
#define TIME_OUT 5000
#define MAX_SPEED 255
#define MIX_SPEED 0
int ena = 5;
int in1 = 6;
int in2 = 7;

double h;
double h_init;
double E,E1,E2,alpha,gamma,beta,sp;
double Kp,Kd,Ki;
double T;

volatile double Output =0;
double LastOutput = 0;
// Ham doc cam bien

float GetDistance()
{
  double duration, distance;
  digitalWrite (TRIG_PIN,LOW);
  delayMicroseconds (2);
  digitalWrite ( TRIG_PIN,HIGH);
  delayMicroseconds (2);
  digitalWrite ( TRIG_PIN,LOW);
  duration = pulseIn(ECHO_PIN, HIGH, TIME_OUT);
  distance = duration/29.1/2;
  return distance;
}

// ham dieu khien PID
void PID()
{
  double x = GetDistance();
  h_init = 16-x;
  E = h -h_init;
  alpha = 2*T*Kp + Ki*T*T - 2*Kd;
  beta = T*T*Ki - 4*Kd - 2*T*Kp;
  gamma = 2*Kd;
  Output = (alpha*E + beta*E1 + gamma*E2+2*T*LastOutput)/(2*T);
  LastOutput = Output;
  sp = Output*12/255;
  E2=E1;
  E1=E;
  if (h_init <= h)
{
  analogWrite(ena,sp);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
}
else if (h_init > h)
{
  digitalWrite(in1 ,LOW);
  digitalWrite(in2 ,LOW);
}
Serial.println (h_init);
}
void setup ()
{
  Serial.begin(9600);
  pinMode (TRIG_PIN, OUTPUT);
  pinMode (ECHO_PIN, INPUT);
  pinMode (ena, OUTPUT);
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT);

  // xac dinh thong so PID

  Kp =2; Ki =0.08; Kd =3;
  E =0; E1 =0; E2 =0;
  h=4;
  T=0.1;
}

void loop()
{
  delay(100);
  PID();
}
