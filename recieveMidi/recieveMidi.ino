#include "pitches.h"

#define MAX_DURATION 60000  //max held note time of 60 seconds
#define OUT_PIN 8

byte commandByte;
byte noteByte;
byte velocityByte;
boolean isPlaying;
int * newPitches;
int outputPin;

void setup(){
  Serial1.begin(31250);  // MIDI sends data at 31250 bps
  newPitches = initPitches();
  outputPin = OUT_PIN;
}

void loop() {
  checkMIDI();
}

void checkMIDI(){
  
  if (Serial1.available() > 2){
    commandByte = Serial1.read();  //read first byte
    noteByte = Serial1.read(); //read next byte
    velocityByte = Serial1.read(); //read final byte

    // if command byte = 1000xxxx, turn note off
    // if command byte  = 1001xxxx, turn on note
    if (bitRead(commandByte,7) == 1 && bitRead(commandByte,6) == 0 && bitRead(commandByte,5) == 0 && bitRead(commandByte,4) == 0) {
      noTone(outputPin);
    } else if (bitRead(commandByte,7) == 1 && bitRead(commandByte,6) == 0 && bitRead(commandByte,5) == 0 && bitRead(commandByte,4) == 1) {
      int freq = newPitches[noteByte];
      tone(outputPin, freq, MAX_DURATION);
    }
  }
}

/*
 * Returns an array of integers representing the reassigned frequency for each MIDI note
 */
int * initPitches() {
  int ret[128];

  //black keys

  //or just use brute force
  //white keys
  ret[0] = NOTE_C_2 / 4;  //dividing a frequency by 2 lowers a pitch by an octave; this pitch is C_4
  ret[2] = NOTE_D_2 / 4;
  ret[4] = NOTE_E_2 / 4;
  ret[5] = NOTE_G_2 / 4;
  ret[7] = NOTE_A_2 / 4;
  ret[9] = NOTE_C_2 / 2; //C_3
  ret[11] = NOTE_D_2 / 2;
  ret[12] = NOTE_E_2 / 2;
  ret[14] = NOTE_G_2 / 2;
  ret[16] = NOTE_A_2 / 2;  //lower pitches than usually handled in MIDI
  ret[17] = NOTE_C_2;
  ret[19] = NOTE_D_2;
  ret[21] = NOTE_E_2;
  ret[23] = NOTE_G_2;
  ret[24] = NOTE_A_2;
  ret[26] = NOTE_C_1;
  ret[28] = NOTE_D_1;
  ret[29] = NOTE_E_1;
  ret[31] = NOTE_G_1;
  ret[33] = NOTE_A_1;
  ret[35] = NOTE_C0;
  ret[36] = NOTE_D0;
  ret[38] = NOTE_E0;
  ret[40] = NOTE_G0;
  ret[41] = NOTE_A0;
  ret[43] = NOTE_C1;
  ret[45] = NOTE_D1;
  ret[47] = NOTE_E1;
  ret[48] = NOTE_G1;
  ret[50] = NOTE_A1;
  ret[52] = NOTE_C2;
  ret[53] = NOTE_D2;
  ret[55] = NOTE_E2;
  ret[57] = NOTE_G2;
  ret[59] = NOTE_A2;
  ret[60] = NOTE_C3;  // Middle C is still middle C
  ret[62] = NOTE_D3;
  ret[64] = NOTE_E3;
  ret[65] = NOTE_G3;
  ret[67] = NOTE_A3;
  ret[69] = NOTE_C4;
  ret[71] = NOTE_D4;
  ret[72] = NOTE_E4;
  ret[74] = NOTE_G4;
  ret[76] = NOTE_A4;
  ret[77] = NOTE_C5;
  ret[79] = NOTE_D5;
  ret[81] = NOTE_E5;
  ret[83] = NOTE_G5;
  ret[84] = NOTE_A5;
  ret[86] = NOTE_C6;
  ret[88] = NOTE_D6;
  ret[90] = NOTE_E6;
  ret[91] = NOTE_G6;
  ret[93] = NOTE_A6;
  ret[95] = NOTE_C7;
  ret[96] = NOTE_D7;
  ret[98] = NOTE_E7;
  ret[100] = NOTE_G7;
  ret[101] = NOTE_A7;
  ret[103] = NOTE_C8;
  ret[105] = NOTE_D8;
  ret[107] = NOTE_E8;
  ret[108] = NOTE_G8;
  ret[110] = NOTE_A8;
  ret[112] = NOTE_C8 * 2;  //higher than MIDI's usual range, doubling the frequency raises the note by an octave
  ret[113] = NOTE_D8 * 2;
  ret[115] = NOTE_E8 * 2;
  ret[117] = NOTE_G8 * 2;
  ret[119] = NOTE_A8 * 2;
  ret[120] = NOTE_C8 * 4; //another octave higher
  ret[122] = NOTE_D8 * 4;
  ret[124] = NOTE_E8 * 4;
  ret[125] = NOTE_G8 * 4;
  ret[127] = NOTE_A8 * 4;

  //black keys
  ret[1] = NOTE_G_1;
  ret[3] = NOTE_A_1;
  ret[6] = NOTE_B_1;
  ret[8] = NOTE_D_1;
  ret[10] = NOTE_E_1;
  ret[13] = NOTE_G_1;
  ret[15] = NOTE_A_1;
  ret[18] = NOTE_B_1;
  ret[20] = NOTE_D0;
  ret[22] = NOTE_E0;
  ret[25] = NOTE_G0;
  ret[27] = NOTE_A0;
  ret[30] = NOTE_B0;
  ret[32] = NOTE_D1;
  ret[34] = NOTE_E1;
  ret[37] = NOTE_G1;
  ret[39] = NOTE_A1;
  ret[42] = NOTE_B1;
  ret[44] = NOTE_D2;
  ret[46] = NOTE_E2;
  ret[49] = NOTE_G2;
  ret[51] = NOTE_A2;
  ret[54] = NOTE_B2;
  ret[56] = NOTE_D3;
  ret[58] = NOTE_E3;
  ret[61] = NOTE_G3; // Middle C#
  ret[63] = NOTE_A3;
  ret[66] = NOTE_B3;
  ret[68] = NOTE_D4;
  ret[70] = NOTE_E4;
  ret[73] = NOTE_G4;
  ret[75] = NOTE_A4;
  ret[78] = NOTE_B4;
  ret[80] = NOTE_D5;
  ret[82] = NOTE_E5;
  ret[85] = NOTE_G5;
  ret[87] = NOTE_A5;
  ret[90] = NOTE_B5;
  ret[92] = NOTE_D6;
  ret[94] = NOTE_E6;
  ret[97] = NOTE_G6;
  ret[99] = NOTE_A6;
  ret[102] = NOTE_B6;
  ret[104] = NOTE_D7;
  ret[106] = NOTE_E7;
  ret[109] = NOTE_G7;
  ret[111] = NOTE_A7;
  ret[114] = NOTE_B7;
  ret[116] = NOTE_E8;
  ret[118] = NOTE_D8;
  ret[121] = NOTE_G8;
  ret[123] = NOTE_A8;
  ret[126] = NOTE_B8;
  
}

/*
Tone generator
Since it turns out the Due does not have an implementation of tone() or noTone()
v1  use timer, and toggle any digital pin in ISR
   funky duration from arduino version
   TODO use FindMckDivisor?
   timer selected will preclude using associated pins for PWM etc.
    could also do timer/pwm hardware toggle where caller controls duration
*/


// timers TC0 TC1 TC2   channels 0-2 ids 0-2  3-5  6-8     AB 0 1
// use TC1 channel 0
#define TONE_TIMER TC1
#define TONE_CHNL 0
#define TONE_IRQ TC3_IRQn

// TIMER_CLOCK4   84MHz/128 with 16 bit counter give 10 Hz to 656KHz
//  piano 27Hz to 4KHz

static uint8_t pinEnabled[PINS_COUNT];
static uint8_t TCChanEnabled = 0;
static boolean pin_state = false ;
static Tc *chTC = TONE_TIMER;
static uint32_t chNo = TONE_CHNL;

volatile static int32_t toggle_count;
static uint32_t tone_pin;

// frequency (in hertz) and duration (in milliseconds).

void tone(uint32_t ulPin, uint32_t frequency, int32_t duration)
{
    const uint32_t rc = VARIANT_MCK / 256 / frequency;
    tone_pin = ulPin;
    toggle_count = 0;  // strange  wipe out previous duration
    if (duration > 0 ) toggle_count = 2 * frequency * duration / 1000;
    else toggle_count = -1;

    if (!TCChanEnabled) {
      pmc_set_writeprotect(false);
      pmc_enable_periph_clk((uint32_t)TONE_IRQ);
      TC_Configure(chTC, chNo,
        TC_CMR_TCCLKS_TIMER_CLOCK4 |
        TC_CMR_WAVE |         // Waveform mode
        TC_CMR_WAVSEL_UP_RC ); // Counter running up and reset when equals to RC
  
      chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
      chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
      NVIC_EnableIRQ(TONE_IRQ);
                         TCChanEnabled = 1;
    }
    if (!pinEnabled[ulPin]) {
      pinMode(ulPin, OUTPUT);
      pinEnabled[ulPin] = 1;
    }
    TC_Stop(chTC, chNo);
                TC_SetRC(chTC, chNo, rc);    // set frequency
    TC_Start(chTC, chNo);
}

void noTone(uint32_t ulPin)
{
  TC_Stop(chTC, chNo);  // stop timer
  digitalWrite(ulPin,LOW);  // no signal on pin
}

// timer ISR  TC1 ch 0
void TC3_Handler ( void ) {
  TC_GetStatus(TC1, 0);
  if (toggle_count != 0){
    // toggle pin  TODO  better
    digitalWrite(tone_pin,pin_state= !pin_state);
    if (toggle_count > 0) toggle_count--;
  } else {
    noTone(tone_pin);
  }
}

