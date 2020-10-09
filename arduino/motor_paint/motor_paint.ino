#define potEnable 3

int readIn = 0;

void setup(){
  Serial.begin(38400);
  pinMode(potEnable, OUTPUT);

}


void loop(){
  if(Serial.available()>0)
  {
    readIn = Serial.read();
  }

  if(readIn == '1') {
    delay(1000);
    digitalWrite(potEnable, HIGH);
  }

  else if(readIn == '0'){
    digitalWrite(potEnable, LOW);
  }

  else if(readIn == '2'){
    digitalWrite(potEnable, HIGH);
    delay(1100);
    digitalWrite(potEnable, LOW);
    readIn = '0';
  }
}
