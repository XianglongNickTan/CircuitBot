#define potEnable 3

#define restart '0'


int readIn = 0;

void pump_paint(int interval);

void setup(){
  Serial.begin(9600);
  pinMode(potEnable, OUTPUT);

  //enable readout from potentiometer



}


void loop(){
	// Serial.print("Hello,World!\n");
	if(Serial.available()>0)
	{
		readIn = Serial.read();
	}

	if(readIn == '1') {
		// Serial.println("On");
		digitalWrite(potEnable, HIGH);
		delay(50);
		digitalWrite(potEnable, LOW);
	}

	else if(readIn == '0'){
		// Serial.println()
		digitalWrite(potEnable, LOW);
	}


	// Serial.println(readIn);
}

void pump_paint(int interval){
    digitalWrite(potEnable, HIGH);
    delay(50);
    digitalWrite(potEnable, LOW);
}