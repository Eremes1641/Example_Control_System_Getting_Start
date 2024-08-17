#include <Encoder.h>

// --- define ---
// encoder
#define ENC_A_PIN 2
#define ENC_B_PIN 3
#define ENCODER_CPR 2000
#define GEAR_RATIO 6.36f
// L298N
#define IN1_PIN 4
#define IN2_PIN 5
#define PWM_PIN 6
#define MOTOR_VOLT 12.0f

// --- configuration ---
// loop
unsigned long dt_us = 10 * 1000;  // [usec]
// encoder
float enc_bandwidth = 100.0f;  // bandwidth of LPF for suppressing vel noise [rad/s]
// excitation signal
float direction = 1.0f; // (1, 0, -1)
float amp = 0.0f;       // [volt]

// --- global variable ---
// loop
unsigned long us_pre = 0; // [usec]
float dt = float(dt_us) / 1000000.0f;  // [sec]
// encoder
Encoder myEnc(ENC_A_PIN, ENC_B_PIN);
float pos_pre = 0.0f; // [rad]
float vel_pre = 0.0f; // [rad/s]
// serial
int serial_cnt = 0;
int down_sample = 10;

// --- function ---
void L298N_write_volt(float volt);
float Enc_calculate_vel(float pos_now, float pos_pre, float vel_filted_pre, float bandwidth, float dt);
float Enc_read_pos(Encoder Enc_obj, long enc_cpr, float gear_ratio);
float LPF(float x, float y_pre, float bandwidth, float dt);


void setup() {
  Serial.begin(921600);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
}

void loop() {
  // loop
  unsigned long us_now = micros();

  if ((us_now - us_pre) >= dt_us) {
    // Encoder
    long pos_count = myEnc.read();
    float pos = Enc_read_pos(myEnc, ENCODER_CPR, GEAR_RATIO);
    float vel = Enc_calculate_vel(pos, pos_pre, vel_pre, enc_bandwidth, dt);

    // L298N
    float volt = amp * direction;
    L298N_write_volt(volt);

    // down sampling print
    if (serial_cnt > down_sample) {
      // Serial.print("loop dt [us] = "); Serial.println(us_now - us_pre);
      // Serial.print("Encoder pos [count] = ");  Serial.println(pos_count);
      // Serial.print("Encoder pos [rad] = ");  Serial.println(pos);
      Serial.print("Encoder vel [rad/s] = "); Serial.println(vel);
      serial_cnt = 0;
    }

    // save data
    pos_pre = pos;
    vel_pre = vel;
    us_pre = us_now;
    serial_cnt++;
  }
}

void L298N_write_volt(float volt) {
  float volt_temp = 0.0f;

  // direction
  if (volt >= 0.0f) {
    volt_temp = volt;
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    volt_temp = -volt;
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }

  // volt
  float duty_cycle = volt_temp / MOTOR_VOLT;
  int pwm_value = int(trunc(255.0f * duty_cycle));
  analogWrite(PWM_PIN, pwm_value);
}

float LPF(float x, float y_pre, float bandwidth, float dt) {
  if (bandwidth > 1 / dt) {
    bandwidth = 1 / dt;
  }
  float y = bandwidth * dt * x + (1 - bandwidth * dt) * y_pre;
  return y;
}

float Enc_calculate_vel(float pos_now, float pos_pre, float vel_filted_pre, float bandwidth, float dt) {
  float vel = (pos_now - pos_pre) / dt;
  float vel_filted = LPF(vel, vel_filted_pre, bandwidth, dt);
  return vel_filted;
}

float Enc_read_pos(Encoder Enc_obj, long enc_cpr, float gear_ratio) {
  long pos_count = Enc_obj.read();
  float pos = float(pos_count) / float(enc_cpr) / gear_ratio * 2.0f * PI;
  return pos;
}


