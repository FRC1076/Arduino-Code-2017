#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
    #include <avr/power.h>
#endif
#include <VuMeter1076.h>
#include <Math.h>

// Which pin does what with the Arduino (digital pins)
#define LIGHTRING_PIN        6
#define VU_LEFT_PIN          7
#define VU_RIGHT_PIN         8
#define TOP_PIN              9
#define GEAR_SWITCH_PIN     10

// Analog inputs
#define LEFT_MIC            A0
#define RIGHT_MIC           A1

// How many NeoPixels are attached to the Arduino?
#define LR_NUM_PIXELS            16
#define VU_NUM_PIXELS            20
#define TOP_NUM_PIXELS           29
#define NUM_METERS                5

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel left_vu_pixels = 
    Adafruit_NeoPixel(VU_NUM_PIXELS*NUM_METERS, VU_LEFT_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel right_vu_pixels =
    Adafruit_NeoPixel(VU_NUM_PIXELS*NUM_METERS, VU_RIGHT_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel top_pixels =
    Adafruit_NeoPixel(TOP_NUM_PIXELS, TOP_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel lr_pixels =
    Adafruit_NeoPixel(LR_NUM_PIXELS, LIGHTRING_PIN, NEO_GRB + NEO_KHZ800);

/*
 *  5 VuMeters mounted on each side  (20 pixels each)
 */
VuMeter1076 left_meters[NUM_METERS] =  {
    VuMeter1076(left_vu_pixels, 0, 19),
    VuMeter1076(left_vu_pixels, 39, 20),
    VuMeter1076(left_vu_pixels, 40, 59),
    VuMeter1076(left_vu_pixels, 79, 60),
    VuMeter1076(left_vu_pixels, 80, 99),
};

VuMeter1076 right_meters[NUM_METERS] =  {
    VuMeter1076(right_vu_pixels, 0, 19),
    VuMeter1076(right_vu_pixels, 39, 20),
    VuMeter1076(right_vu_pixels, 40, 59),
    VuMeter1076(right_vu_pixels, 79, 60),
    VuMeter1076(right_vu_pixels, 80, 99),
};


void setup() {
    // initialize the lights
    lr_pixels.begin();
    top_pixels.begin();
    // monitor the occupancy switch in the gear slot
    pinMode(GEAR_SWITCH_PIN, INPUT_PULLUP);
    //Serial.begin(115200);
}

/*
 *   Turn the lights either on or off based on occupancy sensor.
 */
void light_top(bool on) {
    uint32_t markColor;

    if (on) {
        markColor = top_pixels.Color(65,0,65);
    } else {
        markColor = top_pixels.Color(0,0,0);
    }

    for (int i=0; i<TOP_NUM_PIXELS; i++) {
        top_pixels.setPixelColor(i, markColor);
    }
}


/*
 *  Remember the occupancy state of the gear in the slot.
 *  If it changes, switch lights on/off as appropriate.
 */
int gearGoneState = HIGH;    /* initial state is gear is gone(HIGH) */

void checkGear() {

    int gearGonePinValue = digitalRead(GEAR_SWITCH_PIN);

    if (gearGonePinValue != gearGoneState) {
        if (gearGonePinValue == HIGH) {
            light_top(false);
        } else {
            light_top(true);
        }
        gearGoneState = gearGonePinValue;
    }
}

/*
 *  Compute log10, but protect against 0 value.
 *  We could maybe even fudge things a little bit
 *  more here if it gave us something that looked
 *  good.
 */
double safeLog10(int sensorValue) {
    if (sensorValue < 10) {
        return 1;
    } else {
        return log10(sensorValue);
    }
}

#define MIN_SCALE_VALUE 50             // never scale below this
int maxValueSoFar = MIN_SCALE_VALUE;   // nominal scaling
#define DECAY_EVERY_NTH 50
uint32_t callCount = 0;

/*
 *   This returns value strictly in the desired range.
 *   It tries to adjust the sensitivity.
 */
int scaledSensorValue(uint32_t sensorValue) {

    double correctionFactor;
    uint32_t logSensorValue;
    uint32_t scaledSensorValue;

    /* re-calibrate with a new ceiling */
    if (sensorValue > maxValueSoFar) {
        maxValueSoFar = sensorValue * 0.9;
        //Serial.print("increase maxValueSoFar =");
        //Serial.println(maxValueSoFar);
        callCount = 0;   /* reset the decay any time there is an increase */
    }

    if (callCount == DECAY_EVERY_NTH) {
        callCount = 0;
        maxValueSoFar *= 0.90;
        if (maxValueSoFar < MIN_SCALE_VALUE) {
            maxValueSoFar = MIN_SCALE_VALUE;
        }
        //Serial.print("decay maxValueSoFar =");
        //Serial.println(maxValueSoFar);
    }

    callCount++;


    /*
     * 33 because of 33 - 13 is 20 pixels?  (0-19 for robot lights)
     * Shift down by 13 to reduce number of lights in low range.
     */ 
    correctionFactor = 33/safeLog10(maxValueSoFar);
    logSensorValue = safeLog10(sensorValue);
    scaledSensorValue = logSensorValue * correctionFactor - 13;
    if (scaledSensorValue < 0) {
        scaledSensorValue = 0;
    }
    return scaledSensorValue;
}

int safeScale(int value) {
    if (value > 200) {
        return value - 200;
    } else {
        return 1;
    }
}

void loop() {

    checkGear();

    /*
     * Display the light ring every time just to be sure.
     */
    for(int i=0;i<16;i++){
        lr_pixels.setPixelColor(i, lr_pixels.Color(0,150,0)); // Moderately bright green color.
    }
    lr_pixels.show();

    /*
     * Distribute the next data samples over all of the meters
     */
    for (int i=0; i<NUM_METERS; i++) {    
        left_meters[i].setMeterValue(scaledSensorValue(safeScale(analogRead(LEFT_MIC))));
        right_meters[i].setMeterValue(scaledSensorValue(safeScale(analogRead(RIGHT_MIC))));
        left_meters[0].show();
        right_meters[0].show();
    }

    //
    // And now do a little dance to simulate
    // decay of the sticking threshold value.
    //
    for (int j = 0; j < 2; j++) {
        for (int i=0; i<NUM_METERS; i++) {
            left_meters[i].hangPixels(left_meters[i].meterValue());
            right_meters[i].hangPixels(right_meters[i].meterValue());
        }
        left_meters[0].show();
        right_meters[0].show();
        delay((1000 / VUMETER_HANG_FPS) - 5);
    }
}