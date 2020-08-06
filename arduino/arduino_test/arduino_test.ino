#define potEnable 8

int readIn = 0;

void setup(){
  Serial.begin(9600);
  pinMode(potEnable, OUTPUT);

}


void loop(){
  if(Serial.available()>0)
  {
    readIn = Serial.read();
  }

  if(readIn == '1') {
    digitalWrite(potEnable, HIGH);
  }

  else if(readIn == '0'){
    digitalWrite(potEnable, LOW);
  }
}
