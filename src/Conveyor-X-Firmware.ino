#include <Arduino.h>
#include <EEPROM.h>
#include "fastio.h"

#define LED_PIN LED_BUILTIN

// stepper motor configurations
#define VOLUME_PIN A0

#define DIR_PIN 10
#define STEP_PIN 11
#define EN_PIN 12

// stepper timer parameters
#define COMPARE_VALUE_TIMER OCR1A

#define ResumeStepperTimer (TIMSK1 |= (1 << OCIE1A))
#define StopStepperTimer (TIMSK1 &= ~(1 << OCIE1A))

#define READ_VOLUME_TIME_MS 10

#define SERIAL_MODE true
#define VOLUME_MODE false

#define COMMAND_PORT Serial

#define MAX_SPEED 600
#define DEFAULT_SPEED 30

#define STEP_PER_MM 78.3532
#define SPEED_TO_CYCLE(x) (1000000.0 / (STEP_PER_MM * x))

// encoder congigurations

#define PIN_A 2
#define PIN_B 3

// encoder parameters
#define DIA 64   // diameter (mm)
#define PPR 1024 // pulse per revolution

#define PULSE_PER_MM_ADDRESS 10
#define DEFAULT_PULSE_PER_MM 10.24f

#define ResumeEncoderTimer (TIMSK2 |= (1 << OCIE2A))
#define StopEncoderTimer (TIMSK2 &= ~(1 << OCIE2A))

// #define INVERT_STEPPER_DIR
// #define INVERT_ENCODER_DIR

String inputString;
bool stringComplete;
float DesireSpeed;
float DesirePosition;
long DesireStepPosition;
long CurrentStepPosition;
float OldSpeed;
unsigned long LedBlinkMillis;

byte VolumeValueCounter;
float VolumeValue;

float DefaultSpeed;
bool IsConveyorRun;

volatile uint8_t timer2_loop_num = 1;
volatile uint8_t timer2_loop_index = 1;

volatile int64_t absolute_pulse;
int64_t last_absolute_pulse;
int32_t incremental_pulse;
long period;
long timer_counter;
float pulse_per_mm;
bool e_stt;
bool is_auto_send_e_stt;
bool is_absolute_mode = true;
volatile bool is_encoder_timer_running = false;
bool led_stt = false;

float S, R, E, P;

void setup()
{
  InitIO();
  InitData();
  StepperTimerInit();
  EncoderTimerInit();
}

void loop()
{
  SerialExecute();
  LedBlink();
}

void InitIO()
{
  pinMode(VOLUME_PIN, INPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(EN_PIN, 0);

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A), intterupt_a, CHANGE);
}

void intterupt_a()
{
  if (READ(PIN_B))
  {
    if (READ(PIN_A))
    {
      absolute_pulse--;
    }
    else
    {
      absolute_pulse++;
    }
  }
  else
  {
    if (READ(PIN_A))
    {
      absolute_pulse++;
    }
    else
    {
      absolute_pulse--;
    }
  }
}

void InitData()
{
  COMMAND_PORT.begin(115200);
  EEPROM.begin();
  OldSpeed = DEFAULT_SPEED;
  get(PULSE_PER_MM_ADDRESS, pulse_per_mm);
  delay(100);
}

void StepperTimerInit()
{
  noInterrupts();

  // Reset register relate to Timer 1
  // Reset register relate
  TCCR1A = TCCR1B = TCNT1 = 0;
  // Set CTC mode to Timer 1
  TCCR1B |= (1 << WGM12);
  // Set prescaler 1 to Timer 1
  TCCR1B |= (1 << CS10);
  // Normal port operation, OCxA disconnected
  TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0));

  interrupts();

  StopStepperTimer;
}

// set the timer interrupt cycle at 1ms
void EncoderTimerInit()
{
  noInterrupts();

  TCCR2A = TCCR2B = TCNT2 = 0;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS20);

  interrupts();

  StopEncoderTimer;
  setEncoderTimerPeriod(1000);
}

void LedBlink()
{
  if (DesireSpeed == 0 && DesireStepPosition == 0)
  {
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  if (millis() - LedBlinkMillis > COMPARE_VALUE_TIMER / 16)
  {
    LedBlinkMillis = millis();
    led_stt = !led_stt;
    digitalWrite(LED_BUILTIN, led_stt);
  }
}

void ConveyorExecute()
{

  if (DesireSpeed < 0.02 && DesireSpeed > -0.02)
  {
    DesireSpeed = 0;
  }

#ifdef INVERT_STEPPER_DIR
  if (DesireSpeed > 0)
  {
    digitalWrite(DIR_PIN, 1);
  }
  else if (DesireSpeed < 0)
  {
    digitalWrite(DIR_PIN, 0);
  }
#else
  if (DesireSpeed > 0)
  {
    digitalWrite(DIR_PIN, 0);
  }
  else if (DesireSpeed < 0)
  {
    digitalWrite(DIR_PIN, 1);
  }
#endif

  DesireSpeed = abs(DesireSpeed);
  setStepperTimerPeriod(SPEED_TO_CYCLE(DesireSpeed));
  DesireStepPosition += roundf(DesirePosition * STEP_PER_MM);
  DesirePosition = 0;

#ifdef INVERT_STEPPER_DIR
  if (DesireStepPosition > 0)
  {
    digitalWrite(DIR_PIN, 1);
  }
  else if (DesireStepPosition < 0)
  {
    digitalWrite(DIR_PIN, 0);
    DesireStepPosition = -DesireStepPosition;
  }
#else
  if (DesireStepPosition > 0)
  {
    digitalWrite(DIR_PIN, 0);
  }
  else if (DesireStepPosition < 0)
  {
    digitalWrite(DIR_PIN, 1);
    DesireStepPosition = -DesireStepPosition;
  }
#endif

  if (DesireStepPosition != 0)
  {
    setStepperTimerPeriod(SPEED_TO_CYCLE(OldSpeed));
  }

  if (DesireSpeed == 0 && DesireStepPosition == 0)
  {
    StopStepperTimer;
    return;
  }

  ResumeStepperTimer;
}

// intCycle us
void setStepperTimerPeriod(float period)
{
  int prescaler;

  if (period > 4000)
  {
    TCCR1B |= (1 << CS11);
    TCCR1B &= ~(1 << CS10);
    prescaler = 8;
  }
  else
  {
    TCCR1B &= ~(1 << CS11);
    TCCR1B |= (1 << CS10);
    prescaler = 1;
  }

  COMPARE_VALUE_TIMER = roundf(period * F_CPU / (1000000.0 * prescaler)) - 1;
}

void setEncoderTimerPeriod(int _period)
{
  int prescaler;
  if (_period < 16)
  {
    TCCR2B |= (1 << CS20);
    TCCR2B &= ~(1 << CS22);
    prescaler = 1;
    timer2_loop_num = 1;
    OCR2A = roundf(_period * 16 / prescaler - 1);
  }
  else if (_period < 1020)
  {
    TCCR2B |= (1 << CS22);
    TCCR2B &= ~(1 << CS20);
    prescaler = 64;
    timer2_loop_num = 1;
    OCR2A = roundf(_period * 16 / prescaler - 1);
  }
  else
  {
    TCCR2B |= (1 << CS22);
    TCCR2B &= ~(1 << CS20);
    prescaler = 64;
    timer2_loop_num = _period / 1000 + 1;
    OCR2A = roundf((_period / timer2_loop_num) * 16 / prescaler - 1);
  }
}

ISR(TIMER1_COMPA_vect)
{
  digitalWrite(STEP_PIN, 0);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, 1);

  if (DesireStepPosition != 0)
  {
    CurrentStepPosition++;
    if (DesireStepPosition == CurrentStepPosition)
    {
      COMMAND_PORT.println("Ok");
      StopStepperTimer;
      DesireStepPosition = 0;
      CurrentStepPosition = 0;
      // digitalWrite(EN_PIN, 1);
    }
  }
}

ISR(TIMER2_COMPA_vect)
{
  if (!is_encoder_timer_running)
    return;

  if (timer2_loop_index == timer2_loop_num)
  {
    timer2_loop_index = 1;

    timer_counter += 1;
    if (timer_counter == period)
    {
      timer_counter = 0;
      COMMAND_PORT.print("P:");

      if (is_absolute_mode)
      {
        COMMAND_PORT.println(absolute_pulse / pulse_per_mm, 3);
      }
      else
      {
        COMMAND_PORT.println((absolute_pulse - last_absolute_pulse) / pulse_per_mm, 3);
      }
      last_absolute_pulse = absolute_pulse;
    }
  }
  else
  {
    timer2_loop_index++;
  }
}

void SerialExecute()
{
  while (COMMAND_PORT.available())
  {
    char inChar = (char)COMMAND_PORT.read();

    if (inChar == '\n')
    {
      stringComplete = true;
      break;
    }
    else if (inChar != '\r')
      inputString += inChar;
  }

  if (!stringComplete)
    return;

  if (inputString == "IsXConveyor")
  {
    inputString = "";
    stringComplete = false;
    COMMAND_PORT.println("YesXConveyor");
    return;
  }
  else if (inputString == "IsXEncoder")
  {
    COMMAND_PORT.println("YesXEncoder");
    inputString = "";
    stringComplete = false;
    return;
  }

  String messageBuffer = inputString.substring(0, 4);
  if (messageBuffer == "M310")
  {
    float mode = inputString.substring(5).toFloat();
    if (int(mode) < 3 && int(mode) > 0)
    {
      mode = int(mode);
      COMMAND_PORT.println("Ok");
    }
    else
    {
      COMMAND_PORT.println("Unknown: Invalid mode!");
    }
  }
  else if (messageBuffer == "M311")
  {
    float speed = inputString.substring(5).toFloat();
    if (abs(speed) < MAX_SPEED)
    {
      DesireSpeed = speed;
      DesirePosition = 0;
      COMMAND_PORT.println("Ok");
    }
    else
    {
      COMMAND_PORT.println("Error: Exceeds max speed!");
    }
  }

  else if (messageBuffer == "M312")
  {
    DesirePosition = inputString.substring(5).toFloat();
    DesireSpeed = 0;
    if (DesirePosition == 0)
      COMMAND_PORT.println("Ok");
  }

  else if (messageBuffer == "M313")
  {
    float speed = inputString.substring(5).toFloat();
    if (abs(speed) < MAX_SPEED)
    {
      OldSpeed = speed;
      COMMAND_PORT.println("Ok");
    }
    else
    {
      COMMAND_PORT.println("Error: Exceeds max speed!");
    }
  }
  else if (messageBuffer == "M315")
  {
    
  }
  else if (messageBuffer == "M316")
  {
    absolute_pulse = last_absolute_pulse = 0;
    incremental_pulse = 0;
    if (inputString.length() > 4)
    {
      float _val = inputString.substring(5).toFloat();
      if (_val == 0)
      {
        is_absolute_mode = true;
      }
      else if (_val == 1)
      {
        is_absolute_mode = false;
      }
    }
    COMMAND_PORT.println("Ok");
  }
  else if (messageBuffer == "M317")
  {
    if (inputString.length() < 5)
    {
      COMMAND_PORT.print('P');
      if (is_absolute_mode)
      {
        COMMAND_PORT.println(absolute_pulse / pulse_per_mm, 3);
        last_absolute_pulse = absolute_pulse;
      }
      else
      {
        incremental_pulse = absolute_pulse - last_absolute_pulse;
        last_absolute_pulse = absolute_pulse;
        COMMAND_PORT.println(incremental_pulse / pulse_per_mm, 3);
      }
    }
    else
    {
      is_encoder_timer_running = false;
      StopEncoderTimer;
      int _per = inputString.substring(6).toInt(); // eg: "M317 T100" -> period = 100
      if (_per > 50)
      {
        period = _per;
        timer_counter = 0;
        is_encoder_timer_running = true;
        ResumeEncoderTimer;
      }
      else
      {
        timer_counter = 0;
      }

      COMMAND_PORT.println("Ok");
    }
  }
  else if (messageBuffer == "M318")
  {
    if (inputString.length() < 5)
    {
      COMMAND_PORT.println(pulse_per_mm);
    }
    else
    {
      pulse_per_mm = inputString.substring(6).toFloat();
      put(PULSE_PER_MM_ADDRESS, pulse_per_mm);
      COMMAND_PORT.println("Ok");
    }
  }

  inputString = "";
  stringComplete = false;

  ConveyorExecute();
}

void get(int idx, float &f)
{
  uint8_t *ptr = (uint8_t *)&f;
  for (int count = 0; count < 4; ++count)
  {
    ptr[count] = EEPROM.read(idx + count);
  }
}

void put(int idx, const float &f)
{
  const uint8_t *ptr = (const uint8_t *)&f;
  for (int count = 0; count < 4; ++count)
  {
    EEPROM.update(idx + count, ptr[count]);
  }
}