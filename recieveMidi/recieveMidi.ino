#include "pitches.h"

#define MAX_DURATION 60000  //max held note time of 60 seconds
#define OUT_PIN 8
#define STATE_NO_NOTE 0
#define STATE_NOTE_ON 1
#define STATE_PLAYING 2
#define STATE_TURNING_OFF 3

//byte commandByte;
//byte noteByte;
//byte velocityByte;
byte command;
int curNote;
int newPitches[128];
int state;
int freq;
byte readByte;
byte channel;

void setup(){
  Serial1.begin(31250);  // MIDI sends data at 31250 bps, Serial 1 receives signal on RX1 (pin 19 on Due)
  Serial.begin(9600);

  newPitches[0] = NOTE_C_2 / 4;  //dividing a frequency by 2 lowers a pitch by an octave; this pitch is C_4
  newPitches[2] = NOTE_D_2 / 4;
  newPitches[4] = NOTE_E_2 / 4;
  newPitches[5] = NOTE_G_2 / 4;
  newPitches[7] = NOTE_A_2 / 4;
  newPitches[9] = NOTE_C_2 / 2; //C_3
  newPitches[11] = NOTE_D_2 / 2;
  newPitches[12] = NOTE_E_2 / 2;
  newPitches[14] = NOTE_G_2 / 2;
  newPitches[16] = NOTE_A_2 / 2;  //lower pitches than usually handled in MIDI
  newPitches[17] = NOTE_C_2;
  newPitches[19] = NOTE_D_2;
  newPitches[21] = NOTE_E_2;
  newPitches[23] = NOTE_G_2;
  newPitches[24] = NOTE_A_2;
  newPitches[26] = NOTE_C_1;
  newPitches[28] = NOTE_D_1;
  newPitches[29] = NOTE_E_1;
  newPitches[31] = NOTE_G_1;
  newPitches[33] = NOTE_A_1;
  newPitches[35] = NOTE_C0;
  newPitches[36] = NOTE_D0;
  newPitches[38] = NOTE_E0;
  newPitches[40] = NOTE_G0;
  newPitches[41] = NOTE_A0;
  newPitches[43] = NOTE_C1;
  newPitches[45] = NOTE_D1;
  newPitches[47] = NOTE_E1;
  newPitches[48] = NOTE_G1;
  newPitches[50] = NOTE_A1;
  newPitches[52] = NOTE_C2;
  newPitches[53] = NOTE_D2;
  newPitches[55] = NOTE_E2;
  newPitches[57] = NOTE_G2;
  newPitches[59] = NOTE_A2;
  newPitches[60] = NOTE_C3;  // Middle C is still middle C
  newPitches[62] = NOTE_D3;
  newPitches[64] = NOTE_E3;
  newPitches[65] = NOTE_G3;
  newPitches[67] = NOTE_A3;
  newPitches[69] = NOTE_C4;
  newPitches[71] = NOTE_D4;
  newPitches[72] = NOTE_E4;
  newPitches[74] = NOTE_G4;
  newPitches[76] = NOTE_A4;
  newPitches[77] = NOTE_C5;
  newPitches[79] = NOTE_D5;
  newPitches[81] = NOTE_E5;
  newPitches[83] = NOTE_G5;
  newPitches[84] = NOTE_A5;
  newPitches[86] = NOTE_C6;
  newPitches[88] = NOTE_D6;
  newPitches[90] = NOTE_E6;
  newPitches[91] = NOTE_G6;
  newPitches[93] = NOTE_A6;
  newPitches[95] = NOTE_C7;
  newPitches[96] = NOTE_D7;
  newPitches[98] = NOTE_E7;
  newPitches[100] = NOTE_G7;
  newPitches[101] = NOTE_A7;
  newPitches[103] = NOTE_C8;
  newPitches[105] = NOTE_D8;
  newPitches[107] = NOTE_E8;
  newPitches[108] = NOTE_G8;
  newPitches[110] = NOTE_A8;
  newPitches[112] = NOTE_C8 * 2;  //higher than MIDI's usual range, doubling the frequency raises the note by an octave
  newPitches[113] = NOTE_D8 * 2;
  newPitches[115] = NOTE_E8 * 2;
  newPitches[117] = NOTE_G8 * 2;
  newPitches[119] = NOTE_A8 * 2;
  newPitches[120] = NOTE_C8 * 4; //another octave higher
  newPitches[122] = NOTE_D8 * 4;
  newPitches[124] = NOTE_E8 * 4;
  newPitches[125] = NOTE_G8 * 4;
  newPitches[127] = NOTE_A8 * 4;

  //black keys
  newPitches[1] = NOTE_G_1;
  newPitches[3] = NOTE_A_1;
  newPitches[6] = NOTE_B_1;
  newPitches[8] = NOTE_D_1;
  newPitches[10] = NOTE_E_1;
  newPitches[13] = NOTE_G_1;
  newPitches[15] = NOTE_A_1;
  newPitches[18] = NOTE_B_1;
  newPitches[20] = NOTE_D0;
  newPitches[22] = NOTE_E0;
  newPitches[25] = NOTE_G0;
  newPitches[27] = NOTE_A0;
  newPitches[30] = NOTE_B0;
  newPitches[32] = NOTE_D1;
  newPitches[34] = NOTE_E1;
  newPitches[37] = NOTE_G1;
  newPitches[39] = NOTE_A1;
  newPitches[42] = NOTE_B1;
  newPitches[44] = NOTE_D2;
  newPitches[46] = NOTE_E2;
  newPitches[49] = NOTE_G2;
  newPitches[51] = NOTE_A2;
  newPitches[54] = NOTE_B2;
  newPitches[56] = NOTE_D3;
  newPitches[58] = NOTE_E3;
  newPitches[61] = NOTE_G3; // Middle C#
  newPitches[63] = NOTE_A3;
  newPitches[66] = NOTE_B3;
  newPitches[68] = NOTE_D4;
  newPitches[70] = NOTE_E4;
  newPitches[73] = NOTE_G4;
  newPitches[75] = NOTE_A4;
  newPitches[78] = NOTE_B4;
  newPitches[80] = NOTE_D5;
  newPitches[82] = NOTE_E5;
  newPitches[85] = NOTE_G5;
  newPitches[87] = NOTE_A5;
  newPitches[90] = NOTE_B5;
  newPitches[92] = NOTE_D6;
  newPitches[94] = NOTE_E6;
  newPitches[97] = NOTE_G6;
  newPitches[99] = NOTE_A6;
  newPitches[102] = NOTE_B6;
  newPitches[104] = NOTE_D7;
  newPitches[106] = NOTE_E7;
  newPitches[109] = NOTE_G7;
  newPitches[111] = NOTE_A7;
  newPitches[114] = NOTE_B7;
  newPitches[116] = NOTE_E8;
  newPitches[118] = NOTE_D8;
  newPitches[121] = NOTE_G8;
  newPitches[123] = NOTE_A8;
  newPitches[126] = NOTE_B8;
  
  state = STATE_NO_NOTE;
  curNote = -1;

  Serial.println("Hello World!");

  delay(100);
}

void loop() {
  checkMIDI();
}

void checkMIDI(){
  
  if (Serial1.available() > 0){

    readByte = Serial1.read();

    if(readByte > 0) {
      Serial.print("Read: ");
      Serial.println(readByte, BIN);
    }

    if(readByte == 0xf8) {
      Serial.println("is clock byte: tic");
    }

    if(state == STATE_NO_NOTE) {
      // if there not a note playing, then this could be a command byte
      command = readByte & B11110000;
      channel = readByte & B00001111;
      if (command == B10010000) { //on
        state = STATE_NOTE_ON;
      } else if (command == 128) { //off
        Serial.println("Stopping");
        state = STATE_TURNING_OFF;
      }
      
    } else if (state == STATE_NOTE_ON) {
      Serial.print("Note: ");
      Serial.println(readByte);
      if (readByte < 128 && readByte > -1) {
        freq = newPitches[readByte];
        tone(OUT_PIN, freq, MAX_DURATION);
        curNote = readByte;
        Serial.print("Playing Frequency: ");
        Serial.println(freq);
      } else {
        Serial.println("Bad Note, not playing.");
      }
      state = STATE_PLAYING;
      
    }else if (state == STATE_PLAYING) {
      //ignore velocity for now
      Serial.print("Velocity: ");
      Serial.println(command, BIN);
      state = STATE_NO_NOTE;
    } else if( state == STATE_TURNING_OFF) {
      // turn off the note if it is the current one
      if (readByte == curNote) {
        noTone(OUT_PIN);
        curNote = -1;
        state = STATE_PLAYING;
      } else {
        state = STATE_NO_NOTE;
      }
    }
  }
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

