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
#define MOTOR_VOLT 12.0f      // [volt]
#define MOTOR_VOLT_LIM 3.0f   // [volt] Avoid the speed being too high

// --- configuration ---
// loop
unsigned long dt_us = 10 * 1000;  // [usec]
long loop_max_time = 20.0f;       // [sec]
// serial
int down_sample = 10;
// encoder
float enc_bandwidth = 100.0f;  // bandwidth of LPF for suppressing vel noise [rad/s]
// desired signal
float amp = PI;          // desired position [rad]
float signal_switch_time = 2.0f;  // time for changing direction [sec]
// model
//   b
// -----
// s + a
float model_b = 24.49;
float model_a = 2.572;  // [rad/s]
// P-PI controller
float bw_pos = 20.0f;          // position bandwidth [rad/s]
float decay_rario = 0.98f;     // anti-windup (0 ~ 1)

// --- global variable ---
// loop
unsigned long us_pre = 0; // [usec]
float dt = float(dt_us) / 1000.0f / 1000.0f;  // [sec]
long loop_iter = 0;
// serial
int serial_cnt = 0;
// encoder
Encoder myEnc(ENC_A_PIN, ENC_B_PIN);
float pos_pre = 0.0f; // [rad]
float vel_pre = 0.0f; // [rad/s]
// desired signal
float direction = 1.0f;   // (1, 0, -1)
float signal_time = 0.0f;
// P-PI controller
float bw_vel = 2.0f * bw_pos;  // velocity bandwidth [rad/s]
float kp_pos = bw_vel / 4.0f;
float kp = 1 / model_b * bw_vel;
float ki = model_a / model_b * bw_vel;
float Sum_e_vel = 0.0f;


// --- function ---
void L298N_write_volt(float volt);
float saturation(float x, float limit);
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

    // desired signal
    if (signal_time >= signal_switch_time) {
      direction = -1.0f * direction;
      signal_time = 0.0f;
    }
    float pos_desired = direction * amp;

    // P-PI controller
    float e_pos = pos_desired - pos;
    float vel_desired = kp_pos * e_pos;
    float e_vel = vel_desired - vel;
    float u = kp * e_vel + ki * Sum_e_vel;
    // limit volt and anti-windup
    if (u > MOTOR_VOLT_LIM) {
      u = MOTOR_VOLT_LIM;
      Sum_e_vel *= decay_rario;
    } else if (u < -1.0f*MOTOR_VOLT_LIM) {
      u = -MOTOR_VOLT_LIM;
      Sum_e_vel *= decay_rario;
    } else {
      Sum_e_vel += e_vel * dt;
    }

    // L298N
    L298N_write_volt(u);

    // down sampling print
    if (serial_cnt > down_sample) {
      Serial.print(pos_desired);
      Serial.print(", ");
      Serial.print(pos);
      Serial.print(", ");
      Serial.println(u);
      serial_cnt = 0;
    }

    // save data
    pos_pre = pos;
    vel_pre = vel;
    us_pre = us_now;
    serial_cnt++;
    loop_iter++;
    signal_time += dt;
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

float saturation(float x, float limit) {
  if (x > limit) {
    x = limit;
  } else if (x < -limit) {
    x = -limit;
  }
  return x;
}
