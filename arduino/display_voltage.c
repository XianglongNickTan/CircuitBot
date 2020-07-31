// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// You can use any (4 or) 5 pins 
#define SCLK_PIN 2
#define MOSI_PIN 3
#define DC_PIN   4
#define CS_PIN   5
#define RST_PIN  6

// Color definitions
#define  BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>

// Option 1: use any pins but a little slower
// Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);  

// Option 2: must use the hardware SPI pins 
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be 
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);

int voltagePin = A5;

int sensorValue = 0;
float voltage = 0.0;

void setup(void) {
  Serial.begin(9600);
  tft.begin();
  Init();
}

void loop() {
  sensorValue = analogRead(voltagePin);   //读取电位值
  voltage= sensorValue * (5.0 / 1023.0);   //换算成电压

  if(Serial.available()>0)
  {
      readIn = Serial.read();
  }

  if(readIn == '1') {
     Serial.println(voltage, 2);
  }

  PrintVoltage();
  delay(2000);

}

void Init(){
  tft.fillScreen(BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(WHITE);  
  tft.setTextSize(3);
  tft.println("Voltage");
}

void testfillrects(uint16_t color1, uint16_t color2) {
 tft.fillScreen(BLACK);
 for (uint16_t x=tft.height()-1; x > 6; x-=6) {
   tft.fillRect((tft.width()-1)/2 -x/2, (tft.height()-1)/2 -x/2 , x, x, color1);
   tft.drawRect((tft.width()-1)/2 -x/2, (tft.height()-1)/2 -x/2 , x, x, color2);
 }
}

void PrintVoltage() {
  tft.fillRect(0, 60, 120, 120, BLACK);
  tft.setCursor(10, 60);
  tft.setTextColor(MAGENTA);
  tft.setTextSize(4);
  tft.print(voltage,1);
  tft.setTextSize(4);
  tft.print("V");
}
