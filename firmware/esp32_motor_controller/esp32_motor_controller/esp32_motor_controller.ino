// PINY
const int ENC1_A = 17;
const int ENC1_B = 13;
const int DIR1   = 5;
const int PWM1   = 4;

const int ENC2_A = 15;
const int ENC2_B = 16;
const int DIR2   = 7;
const int PWM2  = 6;

//Stałe
const int CPR = 9600; //narazie 64 ale trzeba zorbic test
const float Ts = 0.010f;
const float Tc = 0.05f; // trzeba dobrac idk jak
const float tau = 0.0315f; //czas narastania do 63% prędkości końcowej - treba zrobic test 
const float K = 0.0565f;
volatile float u0 = 1.0f; // K i u0 z eksperymentu normalnie
const float Kp = tau/(K*Tc);
const float Ki = Kp/tau;
const float Ti = tau;
const float alpha = 0.5f; //do filtra w sprzężeniu zwrotnym
const float U_MAX = 90.0f; //chyba? to jest do saturacji max napiecie na silniki?
//Prędkość zadana - ale czy nie powinna iść z ROS? narazie dajmy na wysokosci ESP

volatile float w_zad_L = 0.0f;
volatile float w_zad_P = 0.0f;

volatile float omega_L = 0.0f;
volatile float omega_P = 0.0f;
volatile float error_L = 0.0f;
volatile float error_P = 0.0f;

volatile float omega_L_hat = 0.0f;
volatile float u_L_sat = 0.0f;
volatile float omega_P_hat = 0.0f;
volatile float u_P_sat = 0.0f;

//Przerwania na ticki z enkoderow
volatile long ticks_L = 0;
volatile long ticks_P = 0;

void IRAM_ATTR isrA1() { ticks_L += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1; }
void IRAM_ATTR isrB1() { ticks_L += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : 1; }
void IRAM_ATTR isrA2() { ticks_P += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? 1 : -1; }
void IRAM_ATTR isrB2() { ticks_P += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : 1; }
//Funkcja pętli regulatora


const float ENC_SIGN_L = 1.0f;
const float ENC_SIGN_P = -1.0f;
const TickType_t T_TICKS = pdMS_TO_TICKS(10);

float sgn(float x) { return (x > 0) - (x < 0); }


void controlTask(void*)
{
  TickType_t lastWake = xTaskGetTickCount();
  float omega_L_hat_old = 0.0f;
  float omega_P_hat_old = 0.0f;
  float Ik_old_L = 0.0f;
  float Ik_old_P = 0.0f;
  long ticks_old_L = ticks_L;
  long ticks_old_P = ticks_P;

  for (;;)
  {
    vTaskDelayUntil(&lastWake, T_TICKS);

    long ticks_now_L = ticks_L;
    long ticks_now_P = ticks_P;
    long delta_N_L = ticks_now_L - ticks_old_L;
    long delta_N_P = ticks_now_P - ticks_old_P;

    omega_L = ENC_SIGN_L*2*PI*delta_N_L/(CPR*Ts);
    omega_P = ENC_SIGN_P*2*PI*delta_N_P/(CPR*Ts);

    omega_L_hat = alpha*omega_L_hat_old + (1-alpha)*omega_L;
    omega_P_hat = alpha*omega_P_hat_old + (1-alpha)*omega_P; //filtrowane predkosci rzeczywiste 

    error_L = w_zad_L - omega_L_hat;
    error_P = w_zad_P - omega_P_hat;

    float u_FF_L = w_zad_L/K + u0*sgn(w_zad_L);
    float u_FF_P = w_zad_P/K + u0*sgn(w_zad_P);

    float u_L = Kp*error_L + Ik_old_L + u_FF_L;
    float u_P = Kp*error_P + Ik_old_P + u_FF_P;

    u_L_sat = constrain(u_L, -U_MAX, U_MAX);
    u_P_sat = constrain(u_P, -U_MAX, U_MAX);

    float Ik_L = Ik_old_L;
    float Ik_P = Ik_old_P;
    if (u_L == u_L_sat) Ik_L += Ki*Ts*error_L;
    if (u_P == u_P_sat) Ik_P += Ki*Ts*error_P;

    digitalWrite(DIR1, u_L_sat >= 0 ? HIGH : LOW);
    analogWrite(PWM1, (int)(fabsf(u_L_sat)*2.55f));
    digitalWrite(DIR2, u_P_sat >= 0 ? HIGH : LOW);
    analogWrite(PWM2, (int)(fabsf(u_P_sat)*2.55f));

    if (w_zad_L == 0.0f) Ik_L = 0.0f;
    if (w_zad_P == 0.0f) Ik_P = 0.0f;

    // przypisywanie na nowo do petli

    omega_L_hat_old = omega_L_hat;
    omega_P_hat_old = omega_P_hat;
    ticks_old_L = ticks_now_L;
    ticks_old_P = ticks_now_P;
    Ik_old_L = Ik_L;
    Ik_old_P = Ik_P;
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isrA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), isrB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isrA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), isrB2, CHANGE);

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);

  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, nullptr, 5, nullptr, 1);

}

const float profile[][2] = {
  {1.0f, 2.0f}, {3.0f, 3.0f}, {4.5f, 4.5f}, {0.0f, 0.0f},
  {-2.0f, -2.0f}, {0.0f, 0.0f}, {2.0f, -2.0f}, {0.0f, 0.0f}
};
const int N_PROFILE = sizeof(profile) / sizeof(profile[0]);

void loop() {
  static uint32_t tNext = 0;
  static int idx = 0;
  uint32_t now = millis();
  if (now >= tNext) {
    w_zad_L = profile[idx][0];
    w_zad_P = profile[idx][1];
    idx = (idx + 1) % N_PROFILE;
    tNext = now + 3000;
  }
  Serial.printf("%lu,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f\n", now,
                w_zad_L, omega_L, u_L_sat, w_zad_P, omega_P, u_P_sat);
  delay(20);
}