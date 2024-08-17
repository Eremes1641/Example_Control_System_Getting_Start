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
long loop_max_time = 10.0f;       // [sec]
// encoder
float enc_bandwidth = 100.0f;  // bandwidth of LPF for suppressing vel noise [rad/s]
// excitation signal
float pos_upper = 5 * 2 * PI;   // upper bounadary [rad]
float pos_lower = -5 * 2 * PI;  // lower bounadary [rad]
float amp = 3.0f;               // excitation voltage [volt]

// --- global variable ---
// loop
unsigned long us_pre = 0;              // [usec]
float dt = float(dt_us) / 1000000.0f;  // [sec]
long loop_iter = 0;
// encoder
Encoder myEnc(ENC_A_PIN, ENC_B_PIN);
float pos_pre = 0.0f;  // [rad]
float vel_pre = 0.0f;  // [rad/s]
// excitation signal
float direction = 1.0f;  // (1, 0, -1)


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

  if ((us_now - us_pre) >= dt_us && loop_iter * dt < loop_max_time) {
    // Encoder
    long pos_count = myEnc.read();
    float pos = Enc_read_pos(myEnc, ENCODER_CPR, GEAR_RATIO);
    float vel = Enc_calculate_vel(pos, pos_pre, vel_pre, enc_bandwidth, dt);

    // excitation signal
    if (pos_pre >= pos_upper) {
      direction = -1.0f;
    } else if (pos_pre <= pos_lower) {
      direction = 1.0f;
    }

    // L298N
    float volt = amp * direction;
    L298N_write_volt(volt);

    // print
    Serial.print(loop_iter);
    Serial.print(", ");
    Serial.print(vel);
    Serial.print(", ");
    Serial.println(volt);

    // save data
    pos_pre = pos;
    vel_pre = vel;
    us_pre = us_now;
    loop_iter++;
  }

  if (loop_iter * dt >= loop_max_time) {
    L298N_write_volt(0.0f);
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
