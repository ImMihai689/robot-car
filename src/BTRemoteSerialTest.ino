#include <SerialBT.h>
#include <pico/time.h>

#define WHEEL_SPEED_SAMPLE_FREQUENCY 2
#define WHELL_SPEED_AVG_SAMPLES 5
#define WHEEL_ENCODE_FREQUENCY 130
#define ENCODER_TICK_TO_DISTANCE_K 0.0344827586206896F
#define TURN_SMOOTHING_K 0.7F
#define PD_SEEDING_MICROS 300000

#define ENCODER_1_PIN 4
#define ENCODER_2_PIN 3

#define DRV_PWMA 14
#define DRV_AIN2 22
#define DRV_AIN1 21
#define DRV_STBY 20
#define DRV_BIN1 19
#define DRV_BIN2 18
#define DRV_PWMB 17



bool wheel_encode(repeating_timer_t *rt);

volatile bool encode_1_state = false, encode_2_state = false;
volatile uint wheel_1_count = 0, wheel_2_count = 0;

repeating_timer_t wheel_speed_sampler_timer;
repeating_timer_t wheel_encode_timer;
volatile uint lmic_1 = 0, lmic_2 = 0, loop_last_mic = 0;
volatile uint t1 = 0, t2 = 0; // microseconds between encoder ticks

volatile int motor_1_speed = 0, motor_2_speed = 0;

const float Kp = 0.6;
const float Kd =  1.0;
float prev_err1 = 0, prev_err2 = 0;

uint star_of_movement_mic = 0;


void setup() {
  SerialBT.setName("MRobot");
  SerialBT.begin();
  Serial.begin();

  while(!SerialBT.availableForWrite()) {}
}

void loop() {
  if (SerialBT.available())
  {
    String n = SerialBT.readStringUntil('\n');
    SerialBT.flush();
    String n1 = "", n2 = "";
    int i = 0;
    while(n[i] != ',')
    {
      n1 += n[i];
      i++;
    }
    i++;
    while(i < n.length())
    {
      n2 += n[i];
      i++;
    }

    float x = n1.toFloat(), y = n2.toFloat();

    motor_1_speed = (int)(x * 1023);
    motor_2_speed = (int)(y * 1023);

    Serial.print(motor_1_speed);
    Serial.print(",  ");
    Serial.println(motor_2_speed);
  }

}

void setup1()
{
  pinMode(ENCODER_1_PIN, INPUT);
  pinMode(ENCODER_2_PIN, INPUT);

  pinMode(DRV_PWMA, OUTPUT);
  pinMode(DRV_AIN2, OUTPUT);
  pinMode(DRV_AIN1, OUTPUT);
  pinMode(DRV_STBY, OUTPUT);
  pinMode(DRV_BIN1, OUTPUT);
  pinMode(DRV_BIN2, OUTPUT);
  pinMode(DRV_PWMB, OUTPUT);

  analogWriteFreq(16 * 1000);
  analogWriteRange(1023);


  add_repeating_timer_us(-1000000 / WHEEL_ENCODE_FREQUENCY, wheel_encode, NULL, &wheel_encode_timer);

  analogWrite(DRV_PWMA, 0);
  digitalWrite(DRV_AIN2, HIGH);
  digitalWrite(DRV_AIN1, LOW);
  digitalWrite(DRV_STBY, HIGH);
  digitalWrite(DRV_BIN1, HIGH);
  digitalWrite(DRV_BIN2, LOW);
  analogWrite(DRV_PWMB, 0);
  delay(1000);
}

void loop1()
{
  int p1 = 0, p2 = 0;

  try
  {
    if(motor_1_speed != 0 && motor_2_speed != 0)
    {

      if (abs(motor_1_speed) >= abs(motor_2_speed)) {
        p2 = motor_2_speed;

        int wanted_t1 = motor_1_speed/motor_2_speed * t2;

        double power_diff = (double)t1 / (double)wanted_t1 * Kp;

        p1 = (int)((double)motor_1_speed * power_diff);
      }
      else {
        p1 = motor_1_speed;

        int wanted_t2 = motor_2_speed/motor_1_speed * t1;

        double power_diff = (double)t2 / (double)wanted_t2 * Kp;

        p2 = (int)((double)motor_2_speed * power_diff);
      }
    }
    else
    {
      p1 = motor_1_speed;
      p2 = motor_2_speed;
    }
  }
  catch(std::exception e)
  {

  }
  if(abs(p1) > 1 || abs(p2) > 1)
  {
    if(motor_1_speed >= 0)
    {
      digitalWrite(DRV_AIN2, HIGH);
      digitalWrite(DRV_AIN1, LOW);
    }
    else
    {
      digitalWrite(DRV_AIN2, LOW);
      digitalWrite(DRV_AIN1, HIGH);
    }

    if(motor_2_speed >= 0)
    {
      digitalWrite(DRV_BIN1, HIGH);
      digitalWrite(DRV_BIN2, LOW);
    }
    else
    {
      digitalWrite(DRV_BIN1, LOW);
      digitalWrite(DRV_BIN2, HIGH);
    }

    if(false)//(micros() > star_of_movement_mic + PD_SEEDING_MICROS && (motor_1_speed > 300 && motor_2_speed > 300))
    {
      analogWrite(DRV_PWMA, abs(p1));
      analogWrite(DRV_PWMB, abs(p2));
    }
    else
    {
      analogWrite(DRV_PWMA, abs(motor_1_speed));
      analogWrite(DRV_PWMB, abs(motor_2_speed));
    } 
  }
  else
  {
    star_of_movement_mic = micros();

    analogWrite(DRV_PWMA, 0);
    analogWrite(DRV_PWMB, 0);
  }
  motor_1_speed = 0;
  motor_2_speed = 0;
  
  delay(100);
}



bool wheel_encode(repeating_timer_t *rt)
{
  
  if(digitalReadFast(ENCODER_1_PIN) != encode_1_state)
  {
    encode_1_state = !encode_1_state;
    if(encode_1_state == false)
    {
      uint mic = micros();
      t1 = mic - lmic_1;
      lmic_1 = mic;
    }
  }

  if(digitalReadFast(ENCODER_2_PIN) != encode_2_state)
  {
    encode_2_state = !encode_2_state;
    if(encode_2_state == false)
    {
      uint mic = micros();
      t2 = mic - lmic_2;
      lmic_2 = mic;
    }
  }
  
  return true;
}



