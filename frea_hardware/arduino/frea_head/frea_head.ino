#include <Adafruit_NeoPixel.h>
#include <Servo.h>

#define NUM_PIXELS 9

// Pin definitions
#define WS2812_PIN 3
#define LEFT_EAR 5 # M1
#define LEFT_MOUTH 6 # M2
#define RIGHT_EAR 9 # M3
#define RIGHT_MOUTH 11 # M4
#define NECK 10 # M5

// Servo objects
Servo left_ear;
Servo left_mouth;
Servo right_ear;
Servo right_mouth;
Servo neck;

// Pixel array pixel values
struct Pixel {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};
Pixel pixels[NUM_PIXELS];
Adafruit_NeoPixel strip(NUM_PIXELS, WS2812_PIN, NEO_GRB | NEO_KHZ800);

// Communication protocol special characters
const uint8_t END_MSG_BYTE  = 0xFF;
const uint8_t LE_SERVO_BYTE = 0x01;
const uint8_t RE_SERVO_BYTE = 0x02;
 uint8_t MOUTH_BYTE    = 0x04;
const uint8_t NECK_BYTE     = 0x08;
const uint8_t PIXELS_BYTE   = 0x0F;

// Communication message buffer
uint8_t buffer_size = 0;
uint8_t buffer[128];

// Function to convert from HSV colour space to RGB colour space - taken from https://www.rapidtables.com/convert/color/hsv-to-rgb.html
void hsvToRgb(uint8_t h, uint8_t s, uint8_t v, uint8_t &r, uint8_t &g, uint8_t &b) {
    // We do the maths in floating point and then convert back
    float fs = s;
    float fv = v;
    float fh = h;
    
    float fr, fg, fb;
    
    // We're 0-100 not 0-1
    fs /= 100.f;
    fv /= 100.f;
    
    float c = fs * fv;
    float m = fv - c;
    
    // We're 0-180 not 0-360
    float x = c * (1 - abs(fmod(fh / 30.f, 2) - 1));
    
    if(h < 30) {
        fr = c;
        fg = x;
        fb = 0;
    }
    else if(h < 60) {
        fr = x;
        fg = c;
        fb = 0;
    }
    else if(h < 90) {
        fr = 0;
        fg = c;
        fb = x;
    }
    else if(h < 120) {
        fr = 0;
        fg = x;
        fb = c;
    }
    else if(h < 150) {
        fr = x;
        fg = 0;
        fb = c;
    }
    else {
        fr = c;
        fg = 0;
        fb = x;
    }
    
    // Now convert back to uint8_t
    r = (fr + m) * 255;
    g = (fg + m) * 255;
    b = (fb + m) * 255;
}

// Function to process a fully received message and update the pixel and servo values etc.
void process_received_instructions() {
    for(uint8_t i = 0; i < buffer_size; i++) {
        uint8_t c = buffer[i];
        if(c == LE_SERVO_BYTE) {
            uint8_t val = buffer[++i];
            float fval = val;
            fval /= 100.f;
            if(fval > 1) fval = 1;
            if(fval < 0) fval = 0;
            int ival = 180 * fval;
            // Left servos are mirrored
            left_ear.write(180 - ival);
        }
        else if(c == RE_SERVO_BYTE) {
            uint8_t val = buffer[++i];
            float fval = val;
            fval /= 100.f;
            if(fval > 1) fval = 1;
            if(fval < 0) fval = 0;
            int ival = 180 * fval;
            right_ear.write(ival);
        }
        else if(c == MOUTH_BYTE) {
            uint8_t val = buffer[++i];
            float fval = val;
            fval /= 100.f;
            if(fval > 1) fval = 1;
            if(fval < 0) fval = 0;
            int ival = 180 * fval;
            right_mouth.write(ival);
            // Left servos are mirrored
            left_mouth.write(180 - ival);
        }
        else if(c == NECK_BYTE) {
            uint8_t val = buffer[++i];
            float fval = val;
            fval /= 100.f;
            if(fval > 1) fval = 1;
            if(fval < 0) fval = 0;
            int ival = 180 * fval;
            neck.write(ival);
        }
        else if(c == PIXELS_BYTE) {
            for(uint8_t j = 0; j < NUM_PIXELS; j++) {
                uint8_t h = buffer[++i];
                uint8_t s = buffer[++i];
                uint8_t v = buffer[++i];
                
                uint8_t r, g, b;
                hsvToRgb(h, s, v, r, g, b);
                
                pixels[j].r = r;
                pixels[j].g = g;
                pixels[j].b = b;
                strip.setPixelColor(j, pixels[j].r, pixels[j].g, pixels[j].b);
            }
            strip.show();
        }
    }    
}

// Function to poll the serial buffer and update our stored message buffer
void update_instructions() {
    while(Serial.available()) {
        uint8_t c = Serial.read();
        if(c == END_MSG_BYTE) {
            process_received_instructions();
            buffer_size = 0;
        }
        else {
            buffer[buffer_size++] = c;
        }
    }
}

void setup() {
    left_ear.attach(LEFT_EAR);
    right_ear.attach(RIGHT_EAR);
    left_mouth.attach(LEFT_MOUTH);
    right_mouth.attach(RIGHT_MOUTH);
    neck.attach(NECK);
    left_ear.write(90);
    right_ear.write(90);
    left_mouth.write(90);
    right_mouth.write(90);
    neck.write(90);

    Serial.begin(38400);
    
    strip.begin();
    for(size_t i = 0; i < NUM_PIXELS; i++) {
        hsvToRgb(120, 75, 25, pixels[i].r, pixels[i].g, pixels[i].b);
        strip.setPixelColor(i, pixels[i].r, pixels[i].g, pixels[i].b);
    }
    strip.show();
}

void loop() {
    while (1) {
        update_instructions();
    }

