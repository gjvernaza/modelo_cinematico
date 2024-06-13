#include "BluetoothSerial.h"

//////// valores de los pines del motor 1 //////////
const int phase_A_M1 = 35;
const int phase_B_M1 = 34;
const int ENA_M1 = 5;
const int IN1_M1 = 18;
const int IN2_M1 = 19;

//////// valores de los pines del motor 2 //////////
const int phase_A_M2 = 33;
const int phase_B_M2 = 32;
const int ENB_M2 = 23;
const int IN3_M2 = 21;
const int IN4_M2 = 22;

//////// valores de los pines del motor 3 //////////
const int phase_A_M3 = 26;
const int phase_B_M3 = 25;
const int ENA_M3 = 2;
const int IN1_M3 = 15;
const int IN2_M3 = 0;

//////// valores de los pines del motor 4 //////////
const int phase_A_M4 = 14;
const int phase_B_M4 = 27;
const int ENB_M4 = 17;
const int IN3_M4 = 4;
const int IN4_M4 = 16;

volatile int pulsos_M1 = 0;
volatile int pulsos_M2 = 0;
volatile int pulsos_M3 = 0;
volatile int pulsos_M4 = 0;

volatile byte anterior_M1 = 0;
volatile byte actual_M1 = 0;
volatile byte anterior_M2 = 0;
volatile byte actual_M2 = 0;
volatile byte anterior_M3 = 0;
volatile byte actual_M3 = 0;
volatile byte anterior_M4 = 0;
volatile byte actual_M4 = 0;

const int freq = 1000;
const int pwmChannel_0 = 0;
const int pwmChannel_1 = 1;
const int pwmChannel_2 = 2;
const int pwmChannel_3 = 3;

const int resolution = 12;
int control_value_M1 = 0;
int control_value_M2 = 0;
int control_value_M3 = 0;
int control_value_M4 = 0;

double rpm_M1 = 0;  // Velocidad rpm M1
double rpm_M2 = 0;  // Velocidad rpm M2
double rpm_M3 = 0;  // Velocidad rpm M3
double rpm_M4 = 0;  // Velocidad rpm M4

double w1 = 0;    // Velocidad rad/s M1
double w2 = 0;    // Velocidad rad/s M2
double w3 = 0;    // Velocidad rad/s M3
double w4 = 0;    // Velocidad rad/s M4

float w1_ref = 0; // Velocidad de referencia M1
float w2_ref = 0; // Velocidad de referencia M2
float w3_ref = 0; // Velocidad de referencia M3
float w4_ref = 0; // Velocidad de referencia M4


double R = 4320; // Resolución experimental

unsigned long lastTime = 0;
unsigned long sampleTime = 100;

// variables para el PID


float u_k1_M1 = 0;
float u_k2_M1 = 0;
float e_k_M1 = 0;
float e_k1_M1 = 0;
float e_k2_M1 = 0;


float u_k1_M2 = 0;
float u_k2_M2 = 0;
float e_k_M2 = 0;
float e_k1_M2 = 0;
float e_k2_M2 = 0;


float u_k1_M3 = 0;
float u_k2_M3 = 0;
float e_k_M3 = 0;
float e_k1_M3 = 0;
float e_k2_M3 = 0;

float u_k1_M4 = 0;
float u_k2_M4 = 0;
float e_k_M4 = 0;
float e_k1_M4 = 0;
float e_k2_M4 = 0;

float N = 7.65; // coeficiente de filtro
float kp = 20 / N;
float ti = 0.02 / N;
float td = 0.02 / N;

float T = 0.1;

float q0 = 0;
float q1 = 0;
float q2 = 0;

/////////////////////////// RECEPCIÓN Serial //////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 3;
float data[dataLength];
float outValue = 0;

/////////////////////////// variables dinamicas del robot ////////////////////////
float radius_wheel = 0.04; // Radio de las ruedas
float a = 0.1063; // Distancia vertical desde la rueda frontal hasta el centro del robot
float b = 0.095; // Distancia horizontal desde la rueda trasera hasta el centro del robot

float vf = 0; // Velocidad frontal
float vl = 0; // Velocidad lateral
float w = 0; // Velocidad angular total

float vf_ref = 0; // Velocidad frontal de referencia
float vl_ref = 0; // Velocidad lateral de referencia
float w_ref = 0; // Velocidad angular total de referencia

float theta = 0; // Angulo de orientación del robot

BluetoothSerial SerialBT;

void setup(){

    Serial.begin(115200);
    SerialBT.begin("ESP32_Master");
    
    pinMode(phase_A_M1, INPUT);
    pinMode(phase_B_M1, INPUT);
    pinMode(ENA_M1, OUTPUT);
    pinMode(IN1_M1, OUTPUT);
    pinMode(IN2_M1, OUTPUT);

    pinMode(phase_A_M2, INPUT);
    pinMode(phase_B_M2, INPUT);
    pinMode(ENB_M2, OUTPUT);
    pinMode(IN3_M2, OUTPUT);
    pinMode(IN4_M2, OUTPUT);

    pinMode(phase_A_M3, INPUT);
    pinMode(phase_B_M3, INPUT);
    pinMode(ENA_M3, OUTPUT);
    pinMode(IN1_M3, OUTPUT);
    pinMode(IN2_M3, OUTPUT);

    pinMode(phase_A_M4, INPUT);
    pinMode(phase_B_M4, INPUT);
    pinMode(ENB_M4, OUTPUT);
    pinMode(IN3_M4, OUTPUT);
    pinMode(IN4_M4, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(phase_A_M1), encoder_M1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(phase_B_M1), encoder_M1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(phase_A_M2), encoder_M2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(phase_B_M2), encoder_M2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(phase_A_M3), encoder_M3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(phase_B_M3), encoder_M3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(phase_A_M4), encoder_M4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(phase_B_M4), encoder_M4, CHANGE);

    digitalWrite(IN1_M1, LOW);
    digitalWrite(IN2_M1, LOW);

    digitalWrite(IN3_M2, LOW);
    digitalWrite(IN4_M2, LOW);

    digitalWrite(IN1_M3, LOW);
    digitalWrite(IN2_M3, LOW);

    digitalWrite(IN3_M4, LOW);
    digitalWrite(IN4_M4, LOW);

    lastTime = millis();
    ledcSetup(pwmChannel_0, freq, resolution);
    ledcSetup(pwmChannel_1, freq, resolution);
    ledcSetup(pwmChannel_2, freq, resolution);
    ledcSetup(pwmChannel_3, freq, resolution);
    ledcAttachPin(ENA_M1, pwmChannel_0);
    ledcAttachPin(ENB_M2, pwmChannel_1);
    ledcAttachPin(ENA_M3, pwmChannel_2);
    ledcAttachPin(ENB_M4, pwmChannel_3);
    q0 = kp * (1 + (T / (2 * ti)) + (td / T));
    q1 = kp * ((T / (2 * ti)) - ((2 * td) / T) - 1);
    q2 = kp * (td / T);

}

//void serialEvent()
//{
//    while (Serial.available())
//    {
//        char inChar = (char)Serial.read();
//        inputString += inChar;
//
//        if (inChar == '\n')
//        {
//            stringComplete = true;
//        }
//    }
//}

void encoder_M1()
{
    anterior_M1 = actual_M1;

    if (digitalRead(phase_A_M1))
    {
        bitSet(actual_M1, 1);
    }
    else
        bitClear(actual_M1, 1);

    if (digitalRead(phase_B_M1))
    {
        bitSet(actual_M1, 0);
    }
    else
        bitClear(actual_M1, 0);

    if (anterior_M1 == 2 && actual_M1 == 0)
        pulsos_M1++;
    if (anterior_M1 == 0 && actual_M1 == 1)
        pulsos_M1++;
    if (anterior_M1 == 3 && actual_M1 == 2)
        pulsos_M1++;
    if (anterior_M1 == 1 && actual_M1 == 3)
        pulsos_M1++;

    if (anterior_M1 == 1 && actual_M1 == 0)
        pulsos_M1--;
    if (anterior_M1 == 3 && actual_M1 == 1)
        pulsos_M1--;
    if (anterior_M1 == 0 && actual_M1 == 2)
        pulsos_M1--;
    if (anterior_M1 == 2 && actual_M1 == 3)
        pulsos_M1--;
}

void encoder_M2()
{
    anterior_M2 = actual_M2;

    if (digitalRead(phase_A_M2))
    {
        bitSet(actual_M2, 1);
    }
    else
        bitClear(actual_M2, 1);

    if (digitalRead(phase_B_M2))
    {
        bitSet(actual_M2, 0);
    }
    else
        bitClear(actual_M2, 0);

    if (anterior_M2 == 2 && actual_M2 == 0)
        pulsos_M2--;
    if (anterior_M2 == 0 && actual_M2 == 1)
        pulsos_M2--;
    if (anterior_M2 == 3 && actual_M2 == 2)
        pulsos_M2--;
    if (anterior_M2 == 1 && actual_M2 == 3)
        pulsos_M2--;

    if (anterior_M2 == 1 && actual_M2 == 0)
        pulsos_M2++;
    if (anterior_M2 == 3 && actual_M2 == 1)
        pulsos_M2++;
    if (anterior_M2 == 0 && actual_M2 == 2)
        pulsos_M2++;
    if (anterior_M2 == 2 && actual_M2 == 3)
        pulsos_M2++;
}

void encoder_M3()
{
    anterior_M3 = actual_M3;

    if (digitalRead(phase_A_M3))
    {
        bitSet(actual_M3, 1);
    }
    else
        bitClear(actual_M3, 1);

    if (digitalRead(phase_B_M3))
    {
        bitSet(actual_M3, 0);
    }
    else
        bitClear(actual_M3, 0);

    if (anterior_M3 == 2 && actual_M3 == 0)
        pulsos_M3--;
    if (anterior_M3 == 0 && actual_M3 == 1)
        pulsos_M3--;
    if (anterior_M3 == 3 && actual_M3 == 2)
        pulsos_M3--;
    if (anterior_M3 == 1 && actual_M3 == 3)
        pulsos_M3--;

    if (anterior_M3 == 1 && actual_M3 == 0)
        pulsos_M3++;
    if (anterior_M3 == 3 && actual_M3 == 1)
        pulsos_M3++;
    if (anterior_M3 == 0 && actual_M3 == 2)
        pulsos_M3++;
    if (anterior_M3 == 2 && actual_M3 == 3)
        pulsos_M3++;
}

void encoder_M4()
{
    anterior_M4 = actual_M4;

    if (digitalRead(phase_A_M4))
    {
        bitSet(actual_M4, 1);
    }
    else
        bitClear(actual_M4, 1);

    if (digitalRead(phase_B_M4))
    {
        bitSet(actual_M4, 0);
    }
    else
        bitClear(actual_M4, 0);

    if (anterior_M4 == 2 && actual_M4 == 0)
        pulsos_M4++;
    if (anterior_M4 == 0 && actual_M4 == 1)
        pulsos_M4++;
    if (anterior_M4 == 3 && actual_M4 == 2)
        pulsos_M4++;
    if (anterior_M4 == 1 && actual_M4 == 3)
        pulsos_M4++;

    if (anterior_M4 == 1 && actual_M4 == 0)
        pulsos_M4--;
    if (anterior_M4 == 3 && actual_M4 == 1)
        pulsos_M4--;
    if (anterior_M4 == 0 && actual_M4 == 2)
        pulsos_M4--;
    if (anterior_M4 == 2 && actual_M4 == 3)
        pulsos_M4--;
}

int PID_M1(float error_M1)
{
    float e_k_M1 = error_M1;
    int u_k_M1 = (q0 * e_k_M1) + (q1 * e_k1_M1) + (q2 * e_k2_M1) + u_k1_M1;
    u_k1_M1 = u_k_M1;
    e_k2_M1 = e_k1_M1;
    e_k1_M1 = e_k_M1;
    if (u_k_M1 > 4095)
    {
        u_k_M1 = 4095;
    }
    if (u_k_M1 < -4095)
    {
        u_k_M1 = -4095;
    }    
    
    return u_k_M1;
}

int PID_M2(float error_M2)
{
    float e_k_M2 = error_M2;
    int u_k_M2 = (q0 * e_k_M2) + (q1 * e_k1_M2) + (q2 * e_k2_M2) + u_k1_M2;
    u_k1_M2 = u_k_M2;
    e_k2_M2 = e_k1_M2;
    e_k1_M2 = e_k_M2;
    if (u_k_M2 > 4095)
    {
        u_k_M2 = 4095;
    }
    if (u_k_M2 < -4095)
    {
        u_k_M2 = -4095;
    }
   
    
    return u_k_M2;
}

int PID_M3(float error_M3)
{
    float e_k_M3 = error_M3;
    int u_k_M3 = (q0 * e_k_M3) + (q1 * e_k1_M3) + (q2 * e_k2_M3) + u_k1_M3;
    u_k1_M3 = u_k_M3;
    e_k2_M3 = e_k1_M3;
    e_k1_M3 = e_k_M3;
    if (u_k_M3 > 4095)
    {
        u_k_M3 = 4095;
    }
    if (u_k_M3 < -4095)
    {
        u_k_M3 = -4095;
    }
    
    return u_k_M3;
    
}

int PID_M4(float error_M4)
{
    float e_k_M4 = error_M4;
    int u_k_M4 = (q0 * e_k_M4) + (q1 * e_k1_M4) + (q2 * e_k2_M4) + u_k1_M4;
    u_k1_M4 = u_k_M4;
    e_k2_M4 = e_k1_M4;
    e_k1_M4 = e_k_M4;
    if (u_k_M4 > 4095)
    {
        u_k_M4 = 4095;
    }
    if (u_k_M4 < -4095)
    {
        u_k_M4 = -4095;
    }    
    
    return u_k_M4;
}


void clock_wise(int pin_1, int pin_2, int channel, int out_value)
{
    int duty = out_value;
    digitalWrite(pin_1, LOW);
    digitalWrite(pin_2, HIGH);
    if(duty <1000){
        duty = 0;
    }
    ledcWrite(channel, duty);
}

void counter_clock_wise(int pin_1, int pin_2, int channel, int out_value)
{

    int duty = abs(out_value);
    digitalWrite(pin_1, HIGH);
    digitalWrite(pin_2, LOW);
    if(duty <1000){
        duty = 0;
    }
    ledcWrite(channel, duty);
}

void get_data(String request)
{
    for (int i = 0; i < dataLength; i++)
    {
        int pos = request.indexOf(separator);
        data[i] = request.substring(0, pos).toFloat();
        request = request.substring(pos + 1);
    }
    vf_ref = data[0];
    vl_ref = data[1];
    w_ref = data[2];
    inverse_kinematics(vf_ref, vl_ref, w_ref); // Actualización de las velocidades de referencia
}

void direct_kinematics(float w_1, float w_2, float w_3, float w_4)
{
    vf = radius_wheel * (w_1 + w_2 + w_3 + w_4) / 4;
    vl = radius_wheel * (-w_1 + w_2 - w_3 + w_4) / 4;
    w  = radius_wheel * (w_1 - w_2 - w_3 + w_4) / (4*(a+b));
}

void inverse_kinematics(float v_f, float v_l, float w_t)
{
    w1_ref = (v_f - v_l + ((a+b) * w_t)) / radius_wheel;
    w2_ref = (v_f + v_l - ((a+b) * w_t)) / radius_wheel;
    w3_ref = (v_f - v_l - ((a+b) * w_t)) / radius_wheel;
    w4_ref = (v_f + v_l + ((a+b) * w_t)) / radius_wheel;
    
    
}

void loop(){

    if (SerialBT.available())
    {
        char inChar = (char)SerialBT.read();
        inputString += inChar;

        if (inChar == '\n')
        {
            stringComplete = true;
        }
    }
    if(stringComplete){
        get_data(inputString);
        stringComplete = false;
        inputString = "";
    }
    
    if(millis()-lastTime >= sampleTime){
        //rpm_M1 = (pulsos_M1 * 60.0*1000) / (R * sampleTime);
        //rpm_M2 = (pulsos_M2 * 60.0 * 1000) / (R * sampleTime);
        //rpm_M3 = (pulsos_M3 * 60.0 * 1000) / (R * sampleTime);
        //rpm_M4 = (pulsos_M4 * 60.0 * 1000) / (R * sampleTime);
        w1 = (2 * PI * 1000.0 * pulsos_M1) / ((millis() - lastTime) * R); // Velocidad angular M1
        w2 = (2 * PI * 1000.0 * pulsos_M2) / ((millis() - lastTime) * R); // Velocidad angular M2 
        w3 = (2 * PI * 1000.0 * pulsos_M3) / ((millis() - lastTime) * R); // Velocidad angular M3
        w4 = (2 * PI * 1000.0 * pulsos_M4) / ((millis() - lastTime) * R); // Velocidad angular M4
        

        //w2 = (2 * 3.1416 * rpm_M2) / 60.0;
        //w3 = (2 * 3.1416 * rpm_M3) / 60.0;
        //w4 = (2 * 3.1416 * rpm_M4) / 60.0;
        
        lastTime = millis();
        pulsos_M1 = 0;
        pulsos_M2 = 0;
        pulsos_M3 = 0;
        pulsos_M4 = 0;
        direct_kinematics(w1, w2, w3, w4); // Actualización de las velocidades del robot
        theta += w * T; // Actualización del angulo de orientación
        
        Serial.print("vf_ref,"+String(vf_ref));
        Serial.print("  vl_ref,"+String(vl_ref));
        Serial.println("    w_ref,"+String(w_ref));
        Serial.print("vf,"+String(vf));
        Serial.print("  vl,"+String(vl));
        Serial.println("  w,"+String(w));       

        float error1 = w1_ref - w1;
        float error2 = w2_ref - w2;
        float error3 = w3_ref - w3;
        float error4 = w4_ref - w4;
        control_value_M1 = PID_M1(error1);
        control_value_M2 = PID_M2(error2);
        control_value_M3 = PID_M3(error3);
        control_value_M4 = PID_M4(error4);
        if(control_value_M1 > 0){
            clock_wise(IN1_M1, IN2_M1, pwmChannel_0, control_value_M1);
        }else if (control_value_M1 < 0){
            counter_clock_wise(IN1_M1, IN2_M1, pwmChannel_0, control_value_M1);
        }
        if(control_value_M2 > 0){
            clock_wise(IN3_M2, IN4_M2, pwmChannel_1, control_value_M2);
        }else if(control_value_M2 < 0){
            counter_clock_wise(IN3_M2, IN4_M2, pwmChannel_1, control_value_M2);
        }
        if(control_value_M3 > 0){
            clock_wise(IN1_M3, IN2_M3, pwmChannel_2, control_value_M3);
        }else if(control_value_M3 < 0){
            counter_clock_wise(IN1_M3, IN2_M3, pwmChannel_2, control_value_M3);
        }
        if(control_value_M4 > 0){
            clock_wise(IN3_M4, IN4_M4, pwmChannel_3, control_value_M4);
        }else if(control_value_M4 < 0){
            counter_clock_wise(IN3_M4, IN4_M4, pwmChannel_3, control_value_M4);
        }
        
        
        



    }
    
}