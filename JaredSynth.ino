#include <MIDI.h>
#include <LiquidCrystal.h>

#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <Smooth.h>
#include <ADSR.h>

#include <mozzi_midi.h>

/******************************************************************************************************
 * This set of readings is taken from the "Ctrl Values" panel when reading at different voltages.
 * They are used for calibrating the inputs.
 */
#define A0_1V_READING   (184)
#define A0_5V_READING   (967)

#define A1_1V_READING   (192)
#define A1_5V_READING   (1000)

#define A2_1V_READING   (185)
#define A2_5V_READING   (975)

/******************************************************************************************************
 * Maximum value on dials.
 */
#define DIAL_READING (920)

/******************************************************************************************************
 * Smoothing values for inputs.
 * Nearer 0.0 - More jittery, more responsive to change.
 * Nearer 1.0 - Less jittery, but laggy.
 */
#define CV0_SMOOTHING (0.2)
#define CV1_SMOOTHING (0.1)
#define CV2_SMOOTHING (0.1)
#define DIAL0_SMOOTHING (0.5)
#define DIAL1_SMOOTHING (0.95)

/******************************************************************************************************
 * How sensitive are CVs as gates?
 * 0 = very. 512 = basically not.
 */
#define CV_GATE_SENSITIVITY (128)


/******************************************************************************************************
 * CV & dial configuration.
 * Which CV inputs do what.
 */
#define DISABLED       (-1)

#define CV_PITCH        (0)
#define CV_TRIGGER      (1)
#define CV_TUNE         DISABLED
#define CV_DETUNE       (2)
#define CV_UNISON       DISABLED
#define CV_BITCRUNCH    DISABLED

#define DIAL_PITCH      DISABLED
#define DIAL_TUNE       DISABLED
#define DIAL_DETUNE     DISABLED
#define DIAL_UNISON     (0)
#define DIAL_BITCRUNCH  (1)

 
/******************************************************************************************************
 * MIDI config.
 * These are for other MIDI devices to control it.
 */
#define MIDI_DEFAULT_CHANNEL  (5)

#define MIDI_CTRL_DETUNE      (0)
#define MIDI_CTRL_UNISON      (1)
#define MIDI_CTRL_BITCRUNCH   (2)

/******************************************************************************************************
 * Misc config.
 */
 
// For each 2 levels of unison, how much to multiply by.
const Q16n16 DETUNE_MULTIPLIER = float_to_Q16n16(1.05f);

/******************************************************************************************************
 * Wavetable to use.
 * Comment out all but the one you want to use.
 */

//#define USE_SINE
#define USE_SAWTOOTH
//#define USE_SQUARE
//#define USE_TRIANGLE
//#define USE_PINK_NOISE

/******************************************************************************************************
 * Setup defines to wavetables based on simpler above defines.
 */
#if defined(USE_SIN)
#  include <tables/sin8192_int8.h>
#  define WAVETABLE_NUM_CELLS SIN8192_NUM_CELLS
#  define WAVETABLE_DATA SIN8192_DATA
#elif defined(USE_SAWTOOTH)
#  include <tables/saw8192_int8.h>
#  define WAVETABLE_NUM_CELLS SAW8192_NUM_CELLS
#  define WAVETABLE_DATA SAW8192_DATA
#elif defined(USE_TRIANGLE)
#  include <tables/triangle2048_int8.h>
#  define WAVETABLE_NUM_CELLS TRIANGLE2048_NUM_CELLS
#  define WAVETABLE_DATA TRIANGLE2048_DATA
#elif defined(USE_SQUARE)
#  include <tables/square_no_alias_2048_int8.h>
#  define WAVETABLE_NUM_CELLS SQUARE_NO_ALIAS_2048_NUM_CELLS
#  define WAVETABLE_DATA SQUARE_NO_ALIAS_2048_DATA
#elif defined(USE_PINK_NOISE)
#  include <tables/pinknoise8192_int8.h>
#  define WAVETABLE_NUM_CELLS PINKNOISE8192_NUM_CELLS
#  define WAVETABLE_DATA PINKNOISE8192_DATA
#else
#  error "No wavetable defined!"
#endif

/******************************************************************************************************
 * Math utilities.
 */
inline Q8n8 Q8n8_mult(Q8n8 a, Q8n8 b)
{
  return ((uint16_t)((((uint32_t)(a))*(b))>>8));
}

inline Q16n16 Q16n16_mult(Q16n16 a, Q16n16 b)
{
  return ((uint32_t)((((uint64_t)(a))*(b))>>16));
}

inline Q15n16 Q15n16_mult(Q15n16 a, Q15n16 b)
{
  return ((uint32_t)((((uint64_t)(a))*(b))>>16));
}

inline Q16n16 Q16n16_mult_fast(Q16n16 a, Q16n16 b)
{
  return (a >> 8)*(b >> 8);
}

inline Q15n16 Q15n16_mult_fast(Q15n16 a, Q15n16 b)
{
  return (a >> 8)*(b >> 8);
}


/******************************************************************************************************
 * LUTs
 */

static const Q15n16 UNISON_DIVISOR_LUT[9] = 
{
  float_to_Q15n16(1.0f / 1.00f),
  float_to_Q15n16(1.0f / 2.00f),
  float_to_Q15n16(1.0f / 2.50f),
  float_to_Q15n16(1.0f / 3.00f),
  float_to_Q15n16(1.0f / 3.50f),
  float_to_Q15n16(1.0f / 4.00f),
  float_to_Q15n16(1.0f / 4.50f),
  float_to_Q15n16(1.0f / 5.00f),
  float_to_Q15n16(1.0f / 5.50f),
};

/******************************************************************************************************
 * Conversion utilities. 
 */

// Convert control voltage to MIDI note.
inline Q8n8 Q8n8_ConvertCVtoMIDI_Q8n8(Q8n8 cv, uint8_t root)
{
   return Q8n8_mult(cv, 12 << 8) + (root << 8);
}

// Convert control voltage to frequency.
inline Q16n16 Q16n16_ConvertCVtoFreq_Q8n8(Q8n8 cv, uint8_t root)
{
   return Q16n16_mtof((Q16n16)Q8n8_ConvertCVtoMIDI_Q8n8(cv, root) << 8);
}

/******************************************************************************************************
 * Global vars for hardware interface.
 */
MIDI_CREATE_DEFAULT_INSTANCE();

// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // powers of 2 please

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define LED0 13
#define LED1 13

#define BUTTON0 6
#define BUTTON1 7

/******************************************************************************************************
 * This struct encapsulates all the logic for handling input.
 */
struct InputCore
{ 
  inline bool DidCVGoHigh(int idx) const
  {
    return m_CVIsHigh[idx] && !m_CVIsHighPrev[idx];
  }

  inline bool DidCVGoLow(int idx) const
  {
    return !m_CVIsHigh[idx] && m_CVIsHighPrev[idx];
  }

  inline bool WasButtonPressed(int idx)
  {
    return m_ButtonsCurr[idx] && !m_ButtonsPrev[idx];
  }

  void Update()
  {
    UpdateCV<A0, 0, A0_1V_READING, A0_5V_READING>();
    UpdateCV<A1, 1, A1_1V_READING, A1_5V_READING>();
    UpdateCV<A2, 2, A2_1V_READING, A2_5V_READING>();
    UpdateDial<A3, 0, DIAL_READING>();
    UpdateDial<A4, 1, DIAL_READING>();

    m_ButtonsPrev[0] = m_ButtonsCurr[0];
    m_ButtonsPrev[1] = m_ButtonsCurr[1];
    m_ButtonsCurr[0] = digitalRead(BUTTON0);
    m_ButtonsCurr[1] = digitalRead(BUTTON1);
  }

  // Raw CV values.
  uint16_t m_CV[3] = {};
  uint16_t m_CVPrev[3] = {};
  bool m_CVIsHigh[3] = {};
  bool m_CVIsHighPrev[3] = {};

  // Input voltages on CV inputs. Should be between 0 and 5.
  Q8n8 m_CVVoltage[3];
  Q8n8 m_CVVoltageSmooth[3];

  // Raw dial values.
  uint16_t m_Dial[2] = {};

  // Processed dial values. Should be between 0 and 5.
  Q8n8 m_DialValue[2] = {};
  Q8n8 m_DialValueSmooth[2] = {};

  // Buttons.
  int m_ButtonsPrev[2] = {};
  int m_ButtonsCurr[2] = {};

private:
  /**
   * Templating used to try force the compile to generate code specially for each CV input.
   */
  template<int PIN, int IDX, long PIN_1V_READING, long PIN_5V_READING>
  void UpdateCV()
  {
    m_CVPrev[IDX] = m_CV[IDX];
    m_CVIsHighPrev[IDX] = m_CVIsHigh[IDX];

    const uint16_t pinValue = mozziAnalogRead(PIN);
    m_CV[IDX] = pinValue;

    // Remap the pin value from the measured values for 1 & 5v, and clamp between the min/max values supported by Q8n8.
    const Q8n8 voltage = max(0, min(0xffff, map(pinValue, PIN_1V_READING, PIN_5V_READING, float_to_Q8n8(1.0f), float_to_Q8n8(5.0f))));
    m_CVVoltage[IDX] = voltage;
    m_CVVoltageSmooth[IDX] = m_CVVoltageSmoothing[IDX].next(voltage);

    // Check if we're high or low.
    if(m_CVIsHigh[IDX])
    {
      if(m_CV[IDX] < (CV_GATE_SENSITIVITY))
        m_CVIsHigh[IDX] = false;
    }
    else
    {
      if(m_CV[IDX] > (1024 - CV_GATE_SENSITIVITY))
        m_CVIsHigh[IDX] = true;
    }
  }


  template<int PIN, int IDX, long PIN_MAX>
  void UpdateDial()
  {
    const uint16_t pinValue = mozziAnalogRead(PIN);
    m_Dial[IDX] = pinValue;

    const Q8n8 value = max(0, min(0xffff, map(pinValue, 0, PIN_MAX, float_to_Q8n8(0.0f), float_to_Q8n8(5.0f))));
    m_DialValue[IDX] = value;
    m_DialValueSmooth[IDX] = m_DialValueSmoothing[IDX].next(value);
  }

  Smooth<Q8n8> m_CVVoltageSmoothing[3] = { CV0_SMOOTHING, CV1_SMOOTHING, CV2_SMOOTHING };
  Smooth<Q8n8> m_DialValueSmoothing[2] = { DIAL0_SMOOTHING, DIAL1_SMOOTHING };
};


/******************************************************************************************************
 * This is the synth core. It is responsible for making all the noises.
 */
struct SynthCore
{

  SynthCore()
  {
    m_Envelope.setADLevels(255,128);
    m_Envelope.setTimes(50,200,10000,200);
  }

  /**
   * Set note + tuning.
   */
  void SetNote(Q8n8 note, Q7n8 tune)
  {
    // Convert to frequency taking tuning into account.
    Q16n16 freq = Q16n16_mtof(Q8n8_to_Q16n16(note + tune));
    if(freq != m_Freq)
    {
      m_Freq = freq;
      m_OscilMain.setFreq_Q16n16(m_Freq);
      m_RecalculateDetune = true;
    }
  }


  inline void SetDetune(Q16n16 detune)
  {
    if(m_Detune != detune)
    {
      m_Detune = detune;
      m_RecalculateDetune = true;
    }
  }

  inline void SetUnison(int unison)
  {
    m_Unison = min(8, (unison + 1) >> 4);
  }

  inline void SetBitcrunch(int8_t bitcrunch)
  {   
    m_NumBitReductionA = bitcrunch >> 4;
    m_NumBitReductionB = max(0, bitcrunch - 8) >> 4;
  }

  inline void NoteOn(byte velocity)
  {
    m_Envelope.setADLevels((velocity * 2) + 1, velocity);
    m_Envelope.noteOn();
  }
  
  inline void NoteOff()
  {
    m_Envelope.noteOff();
  }

  inline void Update()
  {
    m_Envelope.update();
  }

  inline int UpdateAudio()
  {
    // Recalculate detune if we need to.
    if(m_RecalculateDetune)
    {
      RecalculateDetune();
    }

    // Sample the main WAVETABLE. Will be -128 to 127 in value.
    Q15n0 wave = m_OscilMain.next();
        
    // Manually unroll loop and rely on case statement fallthrough to avoid
    // overhead from doing a forloop.
    switch(m_Unison)
    {
    case 8:  wave += m_OscilUnison[7].next();
    case 7:  wave += m_OscilUnison[6].next();
    case 6:  wave += m_OscilUnison[5].next();
    case 5:  wave += m_OscilUnison[4].next();
    case 4:  wave += m_OscilUnison[3].next();
    case 3:  wave += m_OscilUnison[2].next();
    case 2:  wave += m_OscilUnison[1].next();
    case 1:  wave += m_OscilUnison[0].next();
    default:
      break;
    }
  
    // Instead of dividing, having a lookup table to multiply to save performance. 
    wave = Q15n16_to_Q15n0(Q15n16_mult_fast(Q15n0_to_Q15n16(wave), UNISON_DIVISOR_LUT[m_Unison]));

    // Clip to 8-bit range.
    if(wave > 127)
      wave = 127;
    else if(wave < -128)
      wave = -128;

    // Crunch the bits. We do it twice to get a smoother transition, rather than 8 solid levels.
    Q15n0 bitCrunchA = wave >> m_NumBitReductionA;
    Q15n0 bitCrunchB = wave >> m_NumBitReductionB;

    // Mix crunch.
    wave = (bitCrunchA << m_NumBitReductionA) + (bitCrunchB << m_NumBitReductionB);
    wave >>= 1;
    
    // The "- 6" is because we'd otherwise be returning a 8-bit value -  "14 - 8 = 6". We can output 14-bit audio, and we'd otherwise throw it away.
    // The last few bits of precision are useful for the envelope afer all!
    int finalMix = (m_Envelope.next() * wave) >> (8 - 6);
    
    return finalMix;
  }

  void RecalculateDetune()
  {
    m_RecalculateDetune = false;
    
#if 1
    // Optimized.
    Q16n16 detune = m_Detune + (16 << 6); // Offset a little to help avoid artifacts when detune is too low.
    m_OscilUnison[0].setFreq_Q16n16(Q16n16_mult_fast(m_Freq, Q16n16_FIX1 + detune));
    m_OscilUnison[1].setFreq_Q16n16(Q16n16_mult_fast(m_Freq, Q16n16_FIX1 - detune));
    detune = Q16n16_mult(detune, DETUNE_MULTIPLIER);
    m_OscilUnison[2].setFreq_Q16n16(Q16n16_mult_fast(m_Freq, Q16n16_FIX1 + detune));
    m_OscilUnison[3].setFreq_Q16n16(Q16n16_mult_fast(m_Freq, Q16n16_FIX1 - detune));
    detune = Q16n16_mult(detune, DETUNE_MULTIPLIER);
    m_OscilUnison[4].setFreq_Q16n16(Q16n16_mult_fast(m_Freq, Q16n16_FIX1 + detune));
    m_OscilUnison[5].setFreq_Q16n16(Q16n16_mult_fast(m_Freq, Q16n16_FIX1 - detune));
    detune = Q16n16_mult(detune, DETUNE_MULTIPLIER);
    m_OscilUnison[6].setFreq_Q16n16(Q16n16_mult_fast(m_Freq, Q16n16_FIX1 + detune));
    m_OscilUnison[7].setFreq_Q16n16(Q16n16_mult_fast(m_Freq, Q16n16_FIX1 - detune));

#else
    // Reference.   
    Q16n16 unisonDetune = m_Detune;
    for(int i = 0; i < 8; ++i)
    {
      if((i & 1) == 0)
      {
        m_OscilUnison[i].setFreq_Q16n16(Q16n16_mult(m_Freq, float_to_Q16n16(1.0f) + unisonDetune));
      }
      else
      {
        m_OscilUnison[i].setFreq_Q16n16(Q16n16_mult(m_Freq, float_to_Q16n16(1.0f) - unisonDetune));
        unisonDetune = Q16n16_mult(unisonDetune, DETUNE_MULTIPLIER);
      }      
    }
#endif    
  }

  // Main oscillator.
  Oscil <WAVETABLE_NUM_CELLS, AUDIO_RATE> m_OscilMain = WAVETABLE_DATA;

  // Extra oscillators for unison.
  Oscil <WAVETABLE_NUM_CELLS, AUDIO_RATE> m_OscilUnison[8] = {
    WAVETABLE_DATA,
    WAVETABLE_DATA,
    WAVETABLE_DATA,
    WAVETABLE_DATA,
    WAVETABLE_DATA,
    WAVETABLE_DATA,
    WAVETABLE_DATA,
    WAVETABLE_DATA,
  };

  Q16n16 m_Freq = 0;
  Q16n16 m_Detune = 0;
  uint8_t m_NumBitReductionA = 0;
  uint8_t m_NumBitReductionB = 0;
  uint8_t m_Unison = 0;
  bool m_RecalculateDetune = false;
  
  // Envelope for note.
  ADSR <CONTROL_RATE, AUDIO_RATE> m_Envelope;
};

/******************************************************************************************************
 * Global instances of our cores.
 */
InputCore inputCore;
SynthCore synthCore;

/******************************************************************************************************
 * Very messy UI/UX code.
 */
enum DisplayMode
{
  DM_INVALID = 0 ,
  DM_INIT,
  DM_MIDI_CONFIG,
  DM_DIAL_VALUES,
  DM_CTRL_VALUES,

  DM_LAST
};

int displayMode = DM_INIT;
int lastDisplayMode = DM_INVALID;


int ctrlValueMode = 0;
int dialValueMode = 0;

int midiChannel = MIDI_DEFAULT_CHANNEL;

const char* displayNames[] = 
{
  "                ",
  "    Welcome!    ",
  "  MIDI Config:  ",  
  "  Dial Values:  ",  
  "  Ctrl Values:  ",  
};

void UpdateDisplay()
{
  bool forceUpdate = false;
  char buf[32] = {};
  if(lastDisplayMode != displayMode)
  {
    lastDisplayMode = displayMode;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(displayNames[displayMode]);    

    forceUpdate = true;
  }

  // If other button is pressed force an update.
  forceUpdate |= inputCore.WasButtonPressed(1);

  static int updateCounter = 0;
  updateCounter++;
  if(forceUpdate || (updateCounter & 0x4) == 0x4)
  {
    switch(displayMode)
    {
      case DM_INIT:
        if(forceUpdate)
        {
            lcd.setCursor(0, 1);
            lcd.print("Make some noise!");
        }
        break;
      case DM_MIDI_CONFIG:
        {
          if(inputCore.WasButtonPressed(1))
          {
            midiChannel++;
            if(midiChannel > 16)
            {
              midiChannel = 1;
            }

            MIDI.begin(midiChannel);
          }
  
          if(forceUpdate)
          {
            lcd.setCursor(0, 1);
            lcd.print("                ");
            lcd.setCursor(0, 1);
            lcd.print("Channel: ");
            lcd.setCursor(11, 1);
            lcd.print(midiChannel);
          }
        }
        break;
  
      case DM_DIAL_VALUES:
        {
          sprintf(buf, "< %04d -- %04d >", inputCore.m_Dial[0], inputCore.m_Dial[1]);
          lcd.setCursor(0, 1);
          lcd.print(buf);
        }
        break;
  
      case DM_CTRL_VALUES:
        {
          switch(ctrlValueMode)
          {
          case 0:
            lcd.setCursor(15, 0);
            lcd.print("R");
            sprintf(buf, "%04d  %04d  %04d", inputCore.m_CV[0], inputCore.m_CV[1], inputCore.m_CV[2]);
            break;

          case 1:
            {
              lcd.setCursor(15, 0);
              lcd.print("V");

              // Cache CV voltages.
              const uint16_t v[3] = { inputCore.m_CVVoltageSmooth[0], inputCore.m_CVVoltageSmooth[1], inputCore.m_CVVoltageSmooth[2] };

              // Division is bad, so we'll multiply by (1/2.56) instead.
              const Q8n8 invDivisor = float_to_Q8n8(1.0f / 2.56f);

              // Calculate the value of the decimal place. This is stored in 8 bits, so 0-255 needs to be scaled down to 0-99.
              const uint16_t dv[3] = {
                Q8n8_mult(((v[0] & 0xff) << 8), invDivisor),
                Q8n8_mult(((v[1] & 0xff) << 8), invDivisor),
                Q8n8_mult(((v[2] & 0xff) << 8), invDivisor),
              };

              sprintf(buf, "%01d.%02d  %01d.%02d  %01d.%02d", v[0] >> 8, dv[0] >> 8, v[1] >> 8, dv[1] >> 8, v[2] >> 8, dv[2] >> 8);
            }
            break;
          }
          lcd.setCursor(0, 1);
          lcd.print(buf);

          if(inputCore.WasButtonPressed(1))
          {
            ctrlValueMode += 1;
            if(ctrlValueMode > 1)
              ctrlValueMode = 0;
          }
        }
        break;
    }
  }
}

/******************************************************************************************************
 * Globals for MIDI stuff.
 */
byte lastNote = 0;
Q7n8 tuneValue = 0;
int midi_clk = 0;

/******************************************************************************************************
 * Called on MIDI clock. This is called 24 times per quarter note.
 */
void HandleClock()
{ 
  
  if(midi_clk++ == 24)
  {
    digitalWrite(LED0,HIGH);
    midi_clk = 0;
  }
  else if(midi_clk > 4)
  {
    digitalWrite(LED0,LOW);
  }
}

/******************************************************************************************************
 * Called on MIDI note on.
 */
void HandleNoteOn(byte channel, byte note, byte velocity)
{ 
  synthCore.SetNote(note << 8, tuneValue);
  synthCore.NoteOn(velocity);

  lastNote = note;
    
  digitalWrite(LED1,HIGH);
}

/******************************************************************************************************
 * Called on MIDI note off.
 */
void HandleNoteOff(byte channel, byte note, byte velocity)
{ 
  if(note == lastNote)
  {
    synthCore.m_Envelope.noteOff();
    digitalWrite(LED1,LOW);
  }
}

/******************************************************************************************************
 * Called on MIDI pitch bend.
 */
void HandlePitchBend(byte channel, int bend)
{
  tuneValue = (bend - 8192) >> 2;
  synthCore.SetNote(lastNote << 8, tuneValue);
}

/******************************************************************************************************
 * Called on MIDI system reset.
 */
void HandleSystemReset()
{
  lastNote = 0;
  synthCore.m_Envelope.noteOff();
}

/******************************************************************************************************
 * Called on MIDI control change message. Setup control parameters here.
 */
void HandleControlChange(byte channel, byte number, byte value)
{
  switch(number)
  {
  case MIDI_CTRL_DETUNE:
    {
      const Q16n16 detuneValue = (int)value << 6;
      synthCore.SetDetune(detuneValue);
    }
    break;

  case MIDI_CTRL_UNISON:
    {
      synthCore.SetUnison(value);
    }
    break;

  case MIDI_CTRL_BITCRUNCH:
    {
      synthCore.SetBitcrunch(value);
    }
    break;
  }
}

/******************************************************************************************************
 * Initial setup of the Arduino.
 */
void setup()
{
  pinMode(LED0, OUTPUT);  
  pinMode(LED1, OUTPUT);

  pinMode(BUTTON0, INPUT);
  pinMode(BUTTON1, INPUT);

  // Analog pins are setup to divide down ~5v to 1.1v so use the internal 1.1v reference.
  analogReference(INTERNAL);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // Initial display update.
  UpdateDisplay();


  // Setup all MIDI handling.
  MIDI.setHandleNoteOn(HandleNoteOn);
  MIDI.setHandleNoteOff(HandleNoteOff);
  MIDI.setHandleClock(HandleClock);
  MIDI.setHandlePitchBend(HandlePitchBend);
  MIDI.setHandleSystemReset(HandleSystemReset);
  MIDI.setHandleControlChange(HandleControlChange);
  
  // Initiate MIDI communications, listen to all channels (not needed with Teensy usbMIDI)
  MIDI.begin(midiChannel);

  startMozzi(CONTROL_RATE); 
}

/******************************************************************************************************
 * Called several times a second, as defined by CONTROL_RATE
 */
void updateControl()
{
  inputCore.Update();

  if(inputCore.WasButtonPressed(0))
  {
    displayMode++;
    if(displayMode == DM_LAST)
    {
      displayMode = DM_INIT;
    }
  }

  //

  // Example:
  // CV0 = Pitch (1v/oct)
  // CV1 = Gate
  // CV2 = Tune
  
  if((CV_TRIGGER != -1 && inputCore.m_CV[CV_TRIGGER] > 128))
  {
    // Calculate MIDI note.
    Q8n8 midiNote = 0;
    
    if(CV_PITCH != DISABLED)
      midiNote += Q8n8_ConvertCVtoMIDI_Q8n8(inputCore.m_CVVoltageSmooth[CV_PITCH], (12 * 2));
    if(DIAL_PITCH != DISABLED)
      midiNote += Q8n8_ConvertCVtoMIDI_Q8n8(inputCore.m_DialValueSmooth[DIAL_PITCH], (12 * 2));
  
    // Convert from 0-5 range to -6 to 6.
    Q7n8 tuneValue = 0;
    
    if(CV_TUNE != DISABLED)
      tuneValue += map(inputCore.m_CVVoltageSmooth[CV_TUNE], 0, 5 << 8, 0, 13 << 8) - (6 << 8);
    if(DIAL_TUNE != DISABLED)
      tuneValue += map(inputCore.m_DialValueSmooth[DIAL_PITCH], 0, 5 << 8, 0, 13 << 8) - (6 << 8);

    synthCore.SetNote(midiNote, tuneValue);
  }

  // Set all parameters.
  {   
    static Q16n16 detuneValuePrev = 0;
    Q16n16 detuneValue = 0;
    bool setDetune = false;
    if(CV_DETUNE != DISABLED)
    {
      detuneValue += inputCore.m_CVVoltageSmooth[DIAL_DETUNE] << 2;
      setDetune = true;
    }
    if(DIAL_DETUNE != DISABLED)
    {
      detuneValue += inputCore.m_DialValueSmooth[DIAL_DETUNE] << 2;
      setDetune = true;
    }
    if(setDetune && detuneValuePrev != detuneValue)
    {
      synthCore.SetDetune(detuneValue);
      detuneValuePrev = detuneValue;
    }

    static int unisonValuePrev = 0;
    int unisonValue = 0;
    bool setUnison = false;
    
    if(CV_UNISON != DISABLED)
    {
      unisonValue += inputCore.m_CVVoltageSmooth[DIAL_UNISON] << 4;
      setUnison = true;
    }
    if(DIAL_UNISON != DISABLED)
    {
      unisonValue += inputCore.m_DialValueSmooth[DIAL_UNISON] << 4;
      setUnison = true;
    }
    if(setUnison && unisonValuePrev != unisonValue)
    {
      synthCore.SetUnison(unisonValue);
      unisonValuePrev = unisonValue;
    }

    // Grab the 2nd dial's value. We want to take the highest bit set,
    static int8_t bitCrunchPrev = 0;
    int8_t bitCrunch = 0; 
    bool setBitCrunch = false;    
    if(CV_BITCRUNCH != DISABLED)
    {
      bitCrunch += inputCore.m_CVVoltageSmooth[CV_BITCRUNCH] >> 2;
      setBitCrunch = true;
    }
    if(DIAL_BITCRUNCH != DISABLED)
    {
      bitCrunch += inputCore.m_DialValueSmooth[DIAL_BITCRUNCH] >> 2;      
      setBitCrunch = true;
    }

    if(setBitCrunch && bitCrunchPrev != bitCrunch)
    {
      synthCore.SetBitcrunch(bitCrunch);
      bitCrunchPrev = bitCrunch;
    }
  }

  if(inputCore.DidCVGoHigh(1))
  {
    // Turn the note on.
    synthCore.NoteOn(127);
  }
  
  if(inputCore.DidCVGoLow(1))
  {
    synthCore.NoteOff();
  }


  synthCore.Update();
  
  
  UpdateDisplay();
}

/******************************************************************************************************
 * Called to fill up audio buffer.
 */
int updateAudio()
{
  return synthCore.UpdateAudio();
}


/******************************************************************************************************
 * Constantly called at runtime.
 */
void loop()
{
  MIDI.read();
  audioHook(); // required here
} 



