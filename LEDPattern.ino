// // #include <FastLED.h>

// #define NUM_LEDS  47
// #define LED_PIN   2

// CRGB leds[NUM_LEDS];
// uint8_t colorIndex[NUM_LEDS];


// DEFINE_GRADIENT_PALETTE(red_blue) { 
//   0,   255, 100, 0,
//   46,  255,  0,  0,
//   179, 150, 0, 0,
//   255, 250, 75, 0
// };
// DEFINE_GRADIENT_PALETTE(neon_green_bg) { 
//   0,   137,  255, 0,
//   46,  0,  155,  0,
//   179, 12, 250, 0,
//   255, 66, 255, 60
// };

// DEFINE_GRADIENT_PALETTE(humanFloor_bg) { 
//   0,   255,  0, 0,
//   46,  255,  74,  74,
//   179, 124, 0, 0,
//   255, 255, 150, 0
// };

// DEFINE_GRADIENT_PALETTE(humanPlayer_bg) { 
//   0, 0, 0, 255,
//   46,  0,  162, 255,
//   179, 137, 0, 255,
//   255, 0, 27, 117
// };

// CRGBPalette16 humanFloorPattern = humanFloor_bg;
// CRGBPalette16 neon_green = neon_green_bg;
// CRGBPalette16 humanPlayerPattern = humanPlayer_bg;

// uint8_t currentMode = 0x00;
// CRGB Intake = CRGB(255, 0, 0);
// CRGB GeneralPattern = CRGB(0, 200, 0);
// CRGB shooting = CRGB(100, 0, 100);
// CRGB Auto = CRGB(100, 200, 100);
// CRGB Enabled = CRGB(0, 255, 15);
// CRGB Climbing = CRGB(255, 255, 255);

// CRGBPalette16 greenblue = red_blue;

// void setup() {
//   FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
//   FastLED.setBrightness(255);

//   Serial.begin(9600);

//   //Fill the colorIndex array with random numbers
//   for (int i = 0; i < NUM_LEDS; i++) {
//     colorIndex[i] = random8();
//   }
// }

// void loop() {
//   // currentMode = 0b10000000;
//   // bool isBlueAlliance = (currentMode & 0b00000001) == 0b00000000;
//   // bool isRedAlliance = (currentMode & 0b00000001) == 0b00000001; //hubv if binary data at 10 (2) than red
//   bool isEnabled = (currentMode & 0b00000010) == 0b00000010;
//   bool isNoteLoaded = (currentMode & 0b00000100) == 0b00000100;
//   bool isShooting = (currentMode & 0b00001000) == 0b00001000;
//   bool isAuto = (currentMode & 0b00010000) == 0b00010000;
//   bool isFloorSource = (currentMode & 0b00100000) == 0b00100000; // floor source
//   bool isClimbing = (currentMode & 0b01000000) == 0b01000000;
//   bool isRobotSource = (currentMode & 0b10000000) == 0b10000000;
//   if (isRobotSource) {
//     uint8_t sinBeat = beatsin8(30, 50, 255, 0, 0);
//     for (int i = 0; i < NUM_LEDS; i++) {
//       leds[i] = ColorFromPalette(greenblue, colorIndex[i], sinBeat);
//     }
//     FastLED.show();
//   } else if (isFloorSource) {
//     uint8_t sinBeat = beatsin8(30, 50, 255, 0, 0);
//     for (int i = 0; i < NUM_LEDS; i++) {
//       leds[i] = ColorFromPalette(humanFloorPattern, colorIndex[i], sinBeat);
//     }
//     FastLED.show();
//   } else if (isShooting) {
//     for (int i = 255; i >= 0; i--) {
//         for (int j = 0; j < NUM_LEDS; j++) {
//             leds[j] = CRGB(i, 0, 0); // Set red intensity to i, green and blue to 0 for black
//         }
//         FastLED.show();
//         delay(0.3); // Adjust delay to control fade speed
//     }
//   } else if (isClimbing) {


//     uint8_t sinBeat   = beatsin8(30, 0, NUM_LEDS - 1, 0, 0);
//     uint8_t sinBeat2  = beatsin8(30, 0, NUM_LEDS - 1, 0, 85);
//     uint8_t sinBeat3  = beatsin8(30, 0, NUM_LEDS - 1, 0, 170);

//     // If you notice that your pattern is missing out certain LEDs, you
//     // will need to use the higher resolution beatsin16 instead. In this
//     // case remove the 3 lines above and replace them with the following:
//     // uint16_t sinBeat   = beatsin16(30, 0, NUM_LEDS - 1, 0, 0);
//     // uint16_t sinBeat2  = beatsin16(30, 0, NUM_LEDS - 1, 0, 21845);
//     // uint16_t sinBeat3  = beatsin16(30, 0, NUM_LEDS - 1, 0, 43690);

//     leds[sinBeat]   = CRGB::ForestGreen;
//     leds[sinBeat2]  = CRGB::Lime;
//     leds[sinBeat3]  = CRGB::LightSeaGreen;
    
//     fadeToBlackBy(leds, NUM_LEDS, 10);

//     FastLED.show();


//   } else if (isAuto) {
  
//     uint8_t sinBeat = beatsin8(30, 50, 255, 0, 0);
//     // Color each pixel from the palette using the index from colorIndex[]
//     for (int i = 0; i < NUM_LEDS; i++) {
//       leds[i] = ColorFromPalette(greenblue, colorIndex[i], sinBeat);
//     }
//     Serial.println("d");
//     for (int i = 0; i < NUM_LEDS; i++) {
//       leds[i] = ColorFromPalette(neon_green, colorIndex[i], sinBeat);
//     }
//   } else if (isNoteLoaded) {  

//     uint8_t sinBeat = beatsin8(30, 50, 255, 0, 0);
//     // Color each pixel from the palette using the index from colorIndex[]
//     for (int i = 0; i < NUM_LEDS; i++) {
//       leds[i] = ColorFromPalette(greenblue, colorIndex[i], sinBeat);
//     }
    
//   } else if (isEnabled) {
//     uint8_t sinBeat = beatsin8(30, 50, 255, 0, 0);
//     // Color each pixel from the palette using the index from colorIndex[]
//     for (int i = 0; i < NUM_LEDS; i++) {
//       leds[i] = ColorFromPalette(greenblue, colorIndex[i], sinBeat);
//     }
//     Serial.println("d");
//     for (int i = 0; i < NUM_LEDS; i++) {
//       leds[i] = ColorFromPalette(neon_green, colorIndex[i], sinBeat);
//     }
//   }
//   // Color each pixel from the palette using the index from colorIndex[]
//   // for (int i = 0; i < NUM_LEDS; i++) {
//   //   leds[i] = ColorFromPalette(greenblue, colorIndex[i], sinBeat);
//   // }
//   EVERY_N_MILLISECONDS(5){
//     for (int i = 0; i < NUM_LEDS; i++) {
//       colorIndex[i]++;
//     }
//   }
//   EVERY_N_MILLISECONDS(50) {
//     // currentMode = AUTO ;
//     while (Serial.available() > 0) {
//       // read the incoming byte:
//       // Serial.readBytes(buffer, 1) ;
//       uint8_t incomingByte = Serial.read();
//       // Serial.print("byte is ") ;
//       // Serial.println(incomingByte) ;
//       if (incomingByte >= 1 && incomingByte <= 15) {
//         currentMode = incomingByte;
//       }
//       // if (buffer[0] >= 1 && buffer[0] <= 5) {
//       //   currentMode = buffer[0];
//       // }
//       // }
//     }
//     FastLED.show();
//   }
// }