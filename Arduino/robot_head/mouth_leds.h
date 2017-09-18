#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define NUM_LEDS 8
#define BRIGHTNESS 50


byte neopix_gamma[] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 };

class MouthLeds {
  Adafruit_NeoPixel strip;

  public:
    void setup(int pin) {
      strip =  Adafruit_NeoPixel(NUM_LEDS, pin, NEO_GRB + NEO_KHZ800);
      // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
      #if defined (__AVR_ATtiny85__)
        if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
      #endif
      // End of trinket special code
      strip.setBrightness(BRIGHTNESS);  
      strip.begin(); 
      strip.show(); // Initialize all pixels to 'off'
    }
  
    void setMouthWidth(uint8_t width) {
      // Some example procedures showing how to display to the pixels:
      if (width > 8)
        colorWipe(strip.Color(0, 255, 0), strip.Color(255, 0, 0), width-8); // Green
      else
        colorWipe(strip.Color(255, 0, 0), 0, width); // Red
    }
  private:
    // Fill the dots one after the other with a color
    void colorWipe(uint32_t c_on, uint32_t c_off, uint8_t width) {
      const int index[] = {7,5,3,1,0,2,4,6};
      for(uint16_t i=0; i< strip.numPixels(); i++) {
        //if (i < width)
        if(index[i] < width)
          strip.setPixelColor(i, c_on);
        else
         strip.setPixelColor(i, c_off);
      }
      strip.show();
    }
};

