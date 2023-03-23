#define AI1 15
#define AI2 18
#define PWMA 4
#define BI1 22
#define BI2 23
#define PWMB 5

#define POT 27

#define IR_1 13
#define IR_2 12
#define IR_3 14

#define LOG 0

class PID_CONTROLLER {
private:
  double p;
  double i;
  double d;

  double integrator = 0;
  double integrator_cap = 1000;
  double z_prev = NULL;
  unsigned long last_time = NULL;

  int sample_rate_ms = 1;  //1khz

public:
  double output = 0;

  PID_CONTROLLER() {
    p = 0;
    i = 0;
    d = 0;
  }

  PID_CONTROLLER(double _p, double _i, double _d) {
    p = _p;
    i = _i;
    d = _d;

    if (_i != 0) {
      integrator_cap = 1 / i;
    }
  }

  void step(double u, double z) {
    unsigned long now = millis();
    if (z_prev == NULL) {
      z_prev = z;
    }
    if (last_time == NULL) {
      last_time = millis();
    }
    int dt = now - last_time;
    if (dt > sample_rate_ms) {
      double err = u - z;

      integrator += err;

      integrator = max(-integrator_cap, min(integrator_cap, integrator));

      double dz = z - z_prev;
      double der = -dz / dt;

      last_time = millis();
      z_prev = z;

      output = p * err + i * integrator + d * der;
    }
  }

  void set_gains(double _p, double _i, double _d) {
    p = _p;
    i = _i;
    d = _d;
  }

  void set_p_gain(double _p) {
    p = _p;
  }

  void set_i_gain(double _i) {
    i = _i;
  }

  void set_d_gain(double _d) {
    d = _d;
  }
};

void set_wheel_output(double left, double right) {
    if(left > 1.0) left = 1.0;
    if(left < -1.0) left = -1.0;
    if(right > 1.0) right = 1.0;
    if(right < -1.0) right = -1.0;

    int deadband = 20;
    int remaining_band = 255 - deadband;
    int left_sign = left / abs(left);
    int right_sign = right / abs(right);

    int output_left = left_sign * deadband + left * remaining_band;
    int output_right = right_sign * deadband + right * remaining_band;

    if (left == 0.0) output_left = 0.0;
    if (right == 0.0) output_right = 0.0;

    analogWrite(PWMA, right_sign*output_right);
    digitalWrite(AI1, max(right_sign, 0));
    digitalWrite(AI2, -min(right_sign, 0));


    analogWrite(PWMB, left_sign*output_left);
    digitalWrite(BI1, -min(left_sign, 0));
    digitalWrite(BI2, max(left_sign, 0));
  }

PID_CONTROLLER angle_controller(1.0 / 3000.0, 0.0, 0.0);
PID_CONTROLLER offset_controller(1.0 / 3000.0, 0.0, 0.0);

void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(POT, INPUT);

  pinMode(IR_1, INPUT);
  pinMode(IR_2, INPUT);
  pinMode(IR_3, INPUT);
}

void loop() {
  int pot_val = analogRead(POT) /  16;

  int IR_1_val = analogRead(IR_1);
  int IR_2_val = analogRead(IR_2);
  int IR_3_val = analogRead(IR_3);

  double angleDiff = IR_1_val - IR_2_val;
  double offsetDiff = IR_3_val - 2000 - ((IR_1_val - 2000) - (IR_2_val - 2000)) / 2 ;

  angle_controller.step(0, angleDiff);
  offset_controller.step(0, offsetDiff);
  double right_out = 0.7+offset_controller.output;
  double left_out = 0.7-offset_controller.output;
  
  //set_wheel_output(-angle_controller.output, angle_controller.output);
  set_wheel_output(right_out, left_out);

  Serial.print("");
  Serial.print(offset_controller.output);
  Serial.print(" ");
  Serial.print(angle_controller.output);
  Serial.print(" ");
  Serial.print(left_out);
  Serial.print(" ");
  Serial.println(right_out);

  if(LOG){
    Serial.print("ADC READS: ");
    Serial.print(IR_1_val);
    Serial.print(" ");
    Serial.print(IR_2_val);
    Serial.print(" ");
    Serial.print(IR_3_val);
    Serial.print(" ");
    Serial.println(pot_val);
  }
}
