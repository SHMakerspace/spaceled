// Prototype 

// Define ports for one panel - cable assumes facing downwards
/*
 * D2=grey=1
 * D1=purple=2
 * OE=blue=3
 * LATCH=green=4
 * A0=yellow=5
 * A1=orange=6
 * CLK=red=7
 * NC=brown=8
 * 
 * ----------
 * | 2 4 6 8 |
 * | 1 3 5 7 |
 * ----  -----
 * 
 * Digiled:
 * | D1 LAT A1 NC  |
 * | D2 OE  A0 CLK |
 * 
 * Due:
 * | 28 26 24 22 |
 * | 29 27 25 23 |
 * 
 */

#include <Adafruit_GFX.h>
#include <gfxfont.h>

#define PIN_1D1 28
#define PIN_1D2 29
#define PIN_1LATCH 26 
#define PIN_1OE 27
#define PIN_1A1 24
#define PIN_1A0 25
//#define PIN_NC 22
# define PIN_1CLK 23

#define LED_BLACK 0
#define LED_BLUE 1
#define LED_GREEN 2
#define LED_CYAN 3
#define LED_RED 4
#define LED_MAGENTA 5
#define LED_YELLOW 6
#define LED_WHITE 7

byte frame[4][384];

class LedPanel : public Adafruit_GFX
{
  public:
    LedPanel() : Adafruit_GFX(64,64) {};
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    uint16_t newColor(uint8_t red, uint8_t green, uint8_t blue);
    uint16_t getColor() { return textcolor; }
    void drawBitmapMem(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
};

LedPanel panel;

// bb describes which data lines drive which of the 4 panels.
// By adjusting the order of the bits in the array, you can change the panel order visually.
byte bb[8] = { 0x40, 0x80, 0x10, 0x20, 0x04, 0x08, 0x01, 0x02 };

// Set a pixel to a specific 3 bit colour (8 colours)
// 0b000 = black (off), 0b001 = Blue, 0b010 = Green, 0b100 = Red, 0b011 = Cyan, 0b101 = Magenta, 0b110 = Yellow, 0b111 = White, etc.
void setpixel(byte x, byte y, byte col) {
  int16_t off = (x&7) + (x & 0xf8)*6 + ((y & 4)*2);
//  int16_t off = (x&7) + (x >> 3)*48 + ((y & 4)*2);
  byte row = y & 3;
  byte b = bb[(y&0x3f) >> 3];
  byte *p = & frame[row][off];
  for(byte c = 0; c < 3;c++) {
    if ( col & 1 ) {
      *p |= b;
    } else {
      *p &= ~b;
    }
    col >>= 1;
    p += 16;
  }
}


void LedPanel::drawPixel(int16_t x, int16_t y, uint16_t color) {
  setpixel(x,y,color);
}

uint16_t LedPanel::newColor(uint8_t red, uint8_t green, uint8_t blue) {
  return (blue>>7) | ((green&0x80)>>6) | ((red&0x80)>>5);
}

void LedPanel::drawBitmapMem(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
  int16_t i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(bitmap[j * byteWidth + i / 8] & (128 >> (i & 7))) {
        panel.drawPixel(x+i, y+j, color);
      }
    }
  }
}

int led = 13;
bool stat1 = 0;

void FillBuffer(byte b){
  for(uint8_t x=0; x<4; x++){
    for(uint16_t y=0; y<384; y++){
      frame[x][y] = b;
    }
  }
}

const int delayPeriod = 42000000/1000*5; //42000000 is the "TCCLKS_TIMER_CLOCK1" clock speed / 1000 for 1ms

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  pinMode(PIN_1D1, OUTPUT);
  pinMode(PIN_1D2, OUTPUT);
  pinMode(PIN_1A1, OUTPUT);
  pinMode(PIN_1A0, OUTPUT);
  pinMode(PIN_1OE, OUTPUT);
  pinMode(PIN_1LATCH, OUTPUT);
  pinMode(PIN_1CLK, OUTPUT);

  pinMode(led, OUTPUT);
  
  digitalWrite(PIN_1D1, LOW);
  digitalWrite(PIN_1D2, LOW);
  digitalWrite(PIN_1A1, LOW);
  digitalWrite(PIN_1A0, LOW);

  digitalWrite(PIN_1OE, HIGH);
  digitalWrite(PIN_1LATCH, HIGH);
  digitalWrite(PIN_1CLK, LOW);

  FillBuffer(0x00);         // Set all LEDs off. (Black)
  panel.setTextSize(1);
  panel.setCursor(0, 0);

  panel.setTextColor(LED_MAGENTA);  
  panel.println("Hello!");
  panel.setCursor(1, 8);

  //panel.setTextColor(LED_CYAN);  
  //panel.println("Makerspace");
  // initialize Timer1 ~400Hz
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)TC8_IRQn);
  TC_Configure(TC2, 2,TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_SetRC(TC2, 2, delayPeriod);

  TC2->TC_CHANNEL[2].TC_IER=TC_IER_CPCS;
  TC2->TC_CHANNEL[2].TC_IMR=TC_IMR_CPCS;
  TC2->TC_CHANNEL[2].TC_IDR=~TC_IER_CPCS;

  TC_Start(TC2, 2);
  NVIC_EnableIRQ(TC8_IRQn);
}

uint8_t bank = 0;

void UpdateFrame() {
  byte * f = frame[bank];
  for (uint16_t n = 0; n<384; n++) {
    digitalWrite(PIN_1D1, *f & 0x40);      // We use the low nibble on PortD for Panel 1 & 2
    digitalWrite(PIN_1D2, *f & 0x80);    // We use the high nibble on PortB for Panel 3 & 4
    f++;
    digitalWrite(PIN_1CLK, LOW);
    digitalWrite(PIN_1CLK, HIGH);
    }

  digitalWrite(PIN_1OE,HIGH);     // disable output
  if (bank & 0x01) {
    digitalWrite(PIN_1A0, HIGH);
  } else {
    digitalWrite(PIN_1A0, LOW);
  }
  if (bank & 0x02) {
    digitalWrite(PIN_1A1, HIGH);
  } else {
    digitalWrite(PIN_1A1, LOW);
  }
  digitalWrite(PIN_1LATCH, HIGH);   // toggle latch
  digitalWrite(PIN_1LATCH, LOW);
  digitalWrite(PIN_1OE, LOW);     // enable output

  if (++bank>3) bank=0;
}


void TC8_Handler()
{
  TC_GetStatus(TC2, 2);
  UpdateFrame();
}


String text = "Hello World!!";
int textSize = 1;
int characterWidths[] = {6,12};
int verticalOffsets[] = {4,0};
int panelWidth = 63;
int scrollDelay = 75;
int color = LED_GREEN;
int offset = -20000;
int increment = +3;

void scrollText ()
{
  int characterWidth = characterWidths[textSize-1];
  int correctedOffset = offset;
  int stringLength = text.length();
  int startChar, endChar;


  if(offset<(-1*(stringLength-1)*characterWidth)) {
    offset = panelWidth-characterWidth;
  }
  if(offset>panelWidth-characterWidth) {
    offset = (-1*(stringLength-1)*characterWidth);
  }
  FillBuffer(0x00);         // Set all LEDs off. (Black)
  panel.setTextSize(textSize);
  correctedOffset = offset;
  if(offset>=0) {
    startChar = 0;
  } else {
    startChar = abs(offset-characterWidth)/characterWidth;
    correctedOffset = offset%characterWidth+characterWidth;
  }
  endChar = (panelWidth-offset)/characterWidth;
  if(endChar > stringLength) {
    endChar = stringLength;
  }
  panel.setCursor(correctedOffset, verticalOffsets[textSize-1]);
  panel.setTextColor(color);
  panel.print(text.substring(startChar, endChar));
  offset = offset + increment;

  delay(scrollDelay);
}

void loop() {
  // put your main code here, to run repeatedly:

  // From London Hackspace wiki:
  // 1. Set OE=HIGH & LAT=LOW
  // 2. For each address A0 & A1 = 0x00. 0x01. 0x10. 0x1 :
  // 3. Clock 384 bits of data on to D1 & D2, by toggling the CLK pin from LOW to HIGH
  // 4. Toggling LAT pin from HIGH to LOW to latch the 384 bits in to the driver chips.
  // 5. Toggle the OE pin LOW to turn the LEDs on for that row.
  // 6. Clock 4 x 384 bits of data to light every row of LEDs in sequence.


  // Step 1 - Set output off (OE = high) and LATCH to LOW.
  scrollText();
}
