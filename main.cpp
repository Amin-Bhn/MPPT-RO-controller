#include <Arduino.h>

const int PIN_VOC = A0;
const int PIN_TEMP = A1;
const int NUM_MODULES = 10;
const int roPins[NUM_MODULES] = {2,3,4,5,6,7,8,9,10,11};

const float ADC_REF_VOLT = 5.0;
const float ADC_MAX_VAL = 1024.0;
const float VOC_SCALE = 8.0;
const float TEMP_V_PER_C = 0.01;
const float KELVIN_OFFSET = 273.15;

float x1 = 50.0;
float x2 = 1.0;
float x3 = 1.1;
float x4 = 0.0;

float nCell = 36.0;
float k_const = 1.380649e-23;
float q_const = 1.602176634e-19;
float Isc = 5.0;
float I_old = 0.0;
float P_RO = 25.0;
float eta_inv = 0.95;
float eta_conv = 0.90;
float Vpv_old = 0.0;
float I_sat = 1.0e-9;

const float K_MARGIN = 1.05;
const float MAX_EXP_ARG = 80.0;
const float MIN_LOG_ARG = 1e-9;

float readVoc() {
  int adc = analogRead(PIN_VOC);
  float v = (adc * ADC_REF_VOLT) / ADC_MAX_VAL;
  return v * VOC_SCALE;
}

float readTemperatureKelvin() {
  int adc = analogRead(PIN_TEMP);
  float v = (adc * ADC_REF_VOLT) / ADC_MAX_VAL;
  float celsius = v / TEMP_V_PER_C;
  return celsius + KELVIN_OFFSET;
}

float calc_Vmpp(float Voc, float T_kelvin) {
  if (T_kelvin <= 0.0) T_kelvin = 1e-3;
  float argExp = (x3 * Voc) / T_kelvin;
  if (argExp > MAX_EXP_ARG) argExp = MAX_EXP_ARG;
  float inner = x2 * exp(argExp) - x4;
  if (inner <= 0.0) inner = MIN_LOG_ARG;
  return Voc - x1 * log(inner);
}

float calc_Vpv_and_Iload(float T_kelvin, float &I_load_out) {
  if (T_kelvin <= 0.0) T_kelvin = 1e-3;
  float P_in = P_RO / (eta_inv * eta_conv);
  float I_load = (Vpv_old > 0.1) ? (P_in / Vpv_old) : 0.0;
  I_load_out = I_load;
  float numerator = Isc - I_old - I_load + I_sat;
  if (numerator <= 0.0) numerator = MIN_LOG_ARG;
  float ratio = numerator / I_sat;
  if (ratio <= 0.0) ratio = MIN_LOG_ARG / I_sat;
  return (nCell * k_const * T_kelvin / q_const) * log(ratio);
}

void runMPPT_RO_Algorithm() {
  float Voc = readVoc();
  float T_kelvin = readTemperatureKelvin();
  float Vmpp = calc_Vmpp(Voc, T_kelvin);
  float threshold = K_MARGIN * Vmpp;

  Serial.println(F("\n===== New MPPT Cycle ====="));
  Serial.print(F("Voc = ")); Serial.print(Voc, 1); Serial.println(F(" V"));
  Serial.print(F("T   = ")); Serial.print(T_kelvin, 1); Serial.println(F(" K"));
  Serial.print(F("Vmpp = ")); Serial.print(Vmpp, 2); Serial.println(F(" V"));
  Serial.print(F("Threshold = ")); Serial.println(threshold, 2);
  Serial.println(F("Mod | V_est(V) | I_load(A) | State"));

  I_old = 0.0;
  Vpv_old = 0.0;

  for (int i = 0; i < NUM_MODULES; i++) {
    float I_load_mod;
    float V_est = calc_Vpv_and_Iload(T_kelvin, I_load_mod);
    bool active = (V_est >= threshold);
    digitalWrite(roPins[i], active ? HIGH : LOW);

    Serial.print(i+1);
    Serial.print(F("   | "));
    Serial.print(V_est, 2);
    Serial.print(F("   | "));
    Serial.print(I_load_mod, 2);
    Serial.print(F("     | "));
    Serial.println(active ? F("ON") : F("OFF"));

    if (active) I_old += I_load_mod;
    Vpv_old = V_est;
  }
}

void setup() {
  for (int i = 0; i < NUM_MODULES; i++) {
    pinMode(roPins[i], OUTPUT);
    digitalWrite(roPins[i], LOW);
  }
  pinMode(PIN_VOC, INPUT);
  pinMode(PIN_TEMP, INPUT);
  Serial.begin(9600);
  delay(500);
  runMPPT_RO_Algorithm();
}

void loop() {
  runMPPT_RO_Algorithm();
  delay(5000);
}
