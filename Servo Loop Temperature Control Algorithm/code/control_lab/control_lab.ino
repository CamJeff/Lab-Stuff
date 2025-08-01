// comment out the following line to actually run on hardware
// #define SIMULATION_MODE
#define SIM_TIME_MULTIPLIER 10

#include <Arduino.h>
#ifndef SIMULATION_MODE
#include <Adafruit_MotorShield.h>
#include "Adafruit_MAX31856.h"
#endif

#define BAUD 115200

#define HEATER_MAX 62500 // CONFIRMED: works
uint16_t heater = 0;

bool fanOn = false;



// ---------------------------- hardware interface -----------------------------
#ifndef SIMULATION_MODE

// Thermocouple:
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(5,4,3,2); // Use software SPI: CS, DI, DO, CLK

// Motor shield:
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *fanMotor = AFMS.getMotor(1); // CONFIRMED: works

void heaterSetup() {
    AFMS.begin();  // Initialize motor shield

    thermocouple.begin();    // Enable MAX31856
    thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);  // Set thermocouple as K type
    thermocouple.setConversionMode(MAX31856_CONTINUOUS);  // Use continuous conversion mode

    /* Configure pin 9 for slow (1Hz) PWM (DO NOT MODIFY THIS!)*/
    pinMode(9, OUTPUT);
    TCCR1A = _BV(COM1A1) | _BV(WGM11);  // Enable the PWM output OC1A (Arduino digital pin 9)
    TCCR1B = _BV(WGM13) | _BV(WGM12);   // Set fast pwm mode w/ ICR1 as top
    TCCR1B = TCCR1B | _BV(CS12);        // Set prescaler @ 256
    ICR1 = 62499;                       // Set the PWM frequency to 1Hz: 16MHz/(256 * 1Hz) - 1 = 62499
    OCR1A = 0;                          // Output compare register for setting duty cycle (This can be conveniently set with analogWrite())

    analogWrite(9,heater);              // Set a fixed heater power

    // Prepare fan motor:
    fanMotor->setSpeed(255); // max speed
    fanMotor->run(RELEASE);  // start off
}

float getTemp() {
    return thermocouple.readThermocoupleTemperature();
}

#else // ---------------------------- sim interface ----------------------------



float T_heater = 25.0;  // Heater element temperature
float T_block  = 25.0;  // Bulk metal (sensor) temperature

const float C_heater = 12.0;
const float C_block = 26.1;
const float R_heater_to_block = 5.0;
const float R_block_ambient = 9.0;
const float T_ambient = 25.0;
const float P_max = 20.0;
unsigned long lastSimTime = 0;

void heaterSetup() {
    T_heater = T_block = T_ambient;
    lastSimTime = millis();
}

float getTemp() {
    unsigned long time = millis();
    float dt = (time - lastSimTime) * SIM_TIME_MULTIPLIER / 1000.0;
    lastSimTime = time;
    float P_effective = ((float) heater / HEATER_MAX) * P_max;

    // Update heater element temperature
    T_heater += dt * (P_effective - (T_heater - T_block) / R_heater_to_block) / C_heater;

    // Update block (sensor) temperature
    T_block += dt * ((T_heater - T_block) / R_heater_to_block - (T_block - T_ambient) / R_block_ambient) / C_block;

    return T_block;
}

#endif








// --------------------------- Controller Parameters ---------------------------

enum ControlModeType {
    MODE_COOL_DOWN,
    MODE_OPEN_LOOP,
    MODE_ON_OFF,
    MODE_PROPORTIONAL,
    MODE_ON_OFF_HYST,
    MODE_PI,
    MODE_PID
};


struct ControlModeParams {
    ControlModeType mode;
    union {
        float openLoopPercentage; // For MODE_OPEN_LOOP: 0.0 to 100.0 (% of total power)
        float onOffTarget;        // For MODE_ON_OFF: target temperature
        struct {
            float target;
            float band_width;     // For MODE_PROPORTIONAL: width of the band in which there's proportional control, degrees c, centered around target
        } prop;

        struct {
            float target;
            float hyst_width;     // For MODE_ON_OFF_HYST: hysteresis window width, degrees c
            bool heater_on;       // <- state variable storing whether we last were on or off
        } onOffHyst;              // MODE_ON_OFF_HYST

        struct { // For PI/PID, store parameters and persistent state
            float target; // target temperature
            float kp;     // proportional gain
            float ki;     // integral gain
            float kd;     // derivative gain (only used in PID)
            bool i_inBandOnly; // only used in PI, assumed true for PID
            float integral;
            float lastError;
	    float dRunningAvg;
        } pid;
    };
};

ControlModeParams params = { MODE_COOL_DOWN, {0} };





// -------------------------------- application --------------------------------

char buffer[256];

unsigned long startTime;
void experimentSetup() {
    startTime = millis();
    params.onOffHyst.heater_on = false;
    params.pid.integral = 0.0f;
    params.pid.lastError = 0.0f;
    params.pid.lastError = 0.0f;
    params.pid.dRunningAvg = 0.0f;
}

void setup() {
    Serial.begin(BAUD);
    heaterSetup();
    experimentSetup();
}


// the windows version of the standard library's sprintf seems a bit broken, this is a workaround
void uintToHexStr(uint64_t value, char *b, int bits) {
    const char hexDigits[] = "0123456789ABCDEF";
    int count = bits / 4;
    b += count;
    *b = '\0';
    while(count--) {
        b--;
        *b = hexDigits[value & 0xF];
        value >>= 4;
    }
}

void loop() {

    // --------------------------- process commands ----------------------------



    if (Serial.available() > 0) {

        size_t len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[len] = '\0';

        Serial.print("LOG: Received line -> [");
        Serial.print(buffer);
        Serial.println("]");

        // Tokenize input (space delimited) to support commands with parameters.
        char* token = strtok(buffer, " ");
        if (token) {
            if (strcmp(token, "COOL-DOWN") == 0) {
                params.mode = MODE_COOL_DOWN;
                Serial.println("COMMAND RECEIVED: COOL-DOWN");

            } else if (strcmp(token, "OPEN-LOOP") == 0) {
                char* param = strtok(NULL, " ");
                if (param) {
                    experimentSetup();
                    params.openLoopPercentage = atof(param);
                    params.mode = MODE_OPEN_LOOP;
                    Serial.println("COMMAND RECEIVED: OPEN-LOOP");
                    Serial.print("LOG: percentage = ");
                    Serial.println(params.openLoopPercentage);
                } else {
                    Serial.println("LOG: OPEN-LOOP command missing parameter");
                }

            } else if (strcmp(token, "ON-OFF") == 0) {
                char* param = strtok(NULL, " ");
                if (param) {
                    experimentSetup();
                    params.onOffTarget = atof(param);
                    params.mode = MODE_ON_OFF;
                    Serial.println("COMMAND RECEIVED: ON-OFF");
                    Serial.print("LOG: target temp = ");
                    Serial.println(params.onOffTarget);
                } else {
                    Serial.println("LOG: ON-OFF command missing parameter");
                }

            } else if (strcmp(token, "PROPORTIONAL") == 0) {
                char* param1 = strtok(NULL, " ");
                char* param2 = strtok(NULL, " ");
                if (param1 && param2) {
                    experimentSetup();
                    params.prop.target = atof(param1);
                    params.prop.band_width = atof(param2);
                    params.mode = MODE_PROPORTIONAL;

                    Serial.println("COMMAND RECEIVED: PROPORTIONAL");
                    Serial.print("LOG: target = ");
                    Serial.print(params.prop.target);
                    Serial.print(", band_width = ");
                    Serial.println(params.prop.band_width);
                } else {
                    Serial.println("LOG: PROPORTIONAL command requires two parameters");
                }

            } else if (strcmp(token, "ON-OFF-HYST") == 0) {
                char* p_target = strtok(NULL, " ");
                char* p_hyst   = strtok(NULL, " ");
                if (p_target && p_hyst) {
                    experimentSetup();
                    params.mode = MODE_ON_OFF_HYST;
                    params.onOffHyst.target       = atof(p_target);
                    params.onOffHyst.hyst_width   = atof(p_hyst);
                    params.onOffHyst.heater_on    = false;
                    Serial.println("COMMAND RECEIVED: ON-OFF-HYST");
                    Serial.print("LOG: target = ");
                    Serial.print(params.onOffHyst.target);
                    Serial.print(", hyst_width = ");
                    Serial.println(params.onOffHyst.hyst_width);
                } else {
                    Serial.println("LOG: ON-OFF-HYST command missing param(s). Usage: ON-OFF-HYST <target> <hysteresis>");
                }

            } else if (strcmp(token, "PI") == 0) {
                // usage: PI <target> <Kp> <Ki> <i_inBandOnly=0 or 1>
                char* p_target  = strtok(NULL, " ");
                char* p_kp      = strtok(NULL, " ");
                char* p_ki      = strtok(NULL, " ");
                char* p_inBand  = strtok(NULL, " ");
                if (p_target && p_kp && p_ki && p_inBand) {
                    experimentSetup();
                    params.mode = MODE_PI;
                    params.pid.target      = atof(p_target);
                    params.pid.kp          = atof(p_kp);
                    params.pid.ki          = atof(p_ki);
                    params.pid.kd          = 0.0f; // not used in PI
                    params.pid.i_inBandOnly= (atoi(p_inBand) != 0);
                    params.pid.integral    = 0.0f;
                    params.pid.lastError   = 0.0f;
		    params.pid.dRunningAvg = 0.0f;

                    Serial.println("COMMAND RECEIVED: PI");
                } else {
                    Serial.println("LOG: PI command requires 4 parameters");
                }

            } else if (strcmp(token, "PID") == 0) {
                // usage: PID <target> <Kp> <Ki> <Kd>
                char* p_target  = strtok(NULL, " ");
                char* p_kp      = strtok(NULL, " ");
                char* p_ki      = strtok(NULL, " ");
                char* p_kd      = strtok(NULL, " ");
                if (p_target && p_kp && p_ki && p_kd) {
                    experimentSetup();
                    params.mode = MODE_PID;
                    params.pid.target       = atof(p_target);
                    params.pid.kp           = atof(p_kp);
                    params.pid.ki           = atof(p_ki);
                    params.pid.kd           = atof(p_kd);
                    params.pid.i_inBandOnly = 1; // assumed to be true for our PID
                    params.pid.integral     = 0.0f;
                    params.pid.lastError    = 0.0f;
		    params.pid.dRunningAvg = 0.0f;

                    Serial.println("COMMAND RECEIVED: PID");
                } else {
                    Serial.println("LOG: PID command requires 4 parameters");
                }

            } else {
                Serial.println("LOG: Unrecognized command");
            }
        }
    }



    // --------------------------- make observations ---------------------------

    // read temperature and time
    float temp = getTemp();
    uint64_t timeNow = millis();
    uint64_t timeSinceStart = timeNow - startTime;
#ifdef SIMULATION_MODE
    timeSinceStart *= SIM_TIME_MULTIPLIER;
#endif

    // ----------------------------- control logic -----------------------------

    fanOn = false;

    switch (params.mode) {

        case MODE_COOL_DOWN:
            heater = 0;
            fanOn = true;
            if (temp < 55.0) { // If the block is cool enough, let python code know we're good to start another test.
                Serial.println("SYSTEM READY");
            }
            break;

        case MODE_OPEN_LOOP: // Map percentage (0–100) to a heater value between 0 and HEATER_MAX.
            heater = (uint16_t)((params.openLoopPercentage / 100.0) * HEATER_MAX);
            break;

        case MODE_ON_OFF: // If above target, cut heat. If below, full power.
            heater = (temp < params.onOffTarget) ? HEATER_MAX : 0;
            break;

        case MODE_PROPORTIONAL: {
            float fraction = (params.prop.target - temp) / params.prop.band_width + 0.5;
            if (fraction < 0) fraction = 0;
            if (fraction > 1) fraction = 1;
            heater = (uint16_t)(fraction * HEATER_MAX);
        } break;

        case MODE_ON_OFF_HYST: {
            // as alluded to by the lab manual, there's some technical terms to get right here
            float tLow  = params.onOffHyst.target - params.onOffHyst.hyst_width / 2.0f;
            float tHigh = params.onOffHyst.target + params.onOffHyst.hyst_width / 2.0f;

            if (temp < tLow) {
                params.onOffHyst.heater_on = true;
            } else if (temp > tHigh) {
                params.onOffHyst.heater_on = false;
            }
            heater = params.onOffHyst.heater_on ? HEATER_MAX : 0;
        } break;

        case MODE_PI: {

            float error = params.pid.target - temp;

#ifndef SIMULATION_MODE
            params.pid.integral += error * 0.1f; // loop is 100ms
#else
            params.pid.integral += error * 0.1f * SIM_TIME_MULTIPLIER;
#endif

            float P = params.pid.kp * error;
            if (params.pid.i_inBandOnly && (P < -1.0f || P > 1.0f)) {
                params.pid.integral = 0.0f;
            }

            float I = params.pid.ki * params.pid.integral;
            float out = P + I;
            if (out < 0.0f) out = 0.0f;
            if (out > 1.0f) out = 1.0f;

            Serial.print("LOG: <P:");
            Serial.print(P);
            Serial.print(", I:");
            Serial.print(I);
            Serial.print(", out:");
            Serial.print(out);
            Serial.print(", i_inBandOnly:");
            Serial.print(params.pid.i_inBandOnly);
            Serial.println(">");

            heater = (uint16_t)(out * HEATER_MAX);

        } break;

        case MODE_PID: {

            float error = params.pid.target - temp;

#ifndef SIMULATION_MODE
            float dError = (error - params.pid.lastError) / 0.1f; // loop is 100ms
            params.pid.integral += error * 0.1f;
#else
            float dError = (error - params.pid.lastError) / (0.1f * SIM_TIME_MULTIPLIER);
            params.pid.integral += error * 0.1f;
#endif

            float P = params.pid.kp * error;
            if (params.pid.i_inBandOnly && (P < -1.0f || P > 1.0f)) {
                params.pid.integral = 0.0f;
            }
            float I = params.pid.ki * params.pid.integral;
            float D = params.pid.kd * dError / 0.1f; // if we assume loop is ~0.1s
            params.pid.dRunningAvg = params.pid.dRunningAvg * 0.985f + D * 0.015f;
            float out = P + I + params.pid.dRunningAvg;
            if (out < 0.0f) out = 0.0f;
            if (out > 1.0f) out = 1.0f;
            heater = (uint16_t)(out * HEATER_MAX);

            Serial.print("LOG: <P:");
            Serial.print(P);
            Serial.print(", I:");
            Serial.print(I);
            Serial.print(", D:");
            Serial.print(D);
            Serial.print(", D avg:");
            Serial.print(params.pid.dRunningAvg);
            Serial.print(", out:");
            Serial.print(out);
            Serial.print(", i_inBandOnly:");
            Serial.print(params.pid.i_inBandOnly);
            Serial.println(">");

            params.pid.lastError = error;
        } break;
    }

    // -------------------------------- output ---------------------------------

    // Hard cut-off for safety: if the last measured temperature is >= 150°C, turn off the heater.
    if (temp >= 150.0) {
        heater = 0;
    }

#ifndef SIMULATION_MODE
    analogWrite(9, heater);
    if (fanOn) {
        fanMotor->run(FORWARD);
    } else {
        fanMotor->run(RELEASE);
    }
#endif


    // Convert float values to 32-bit hex for output.
    union FloatToBits { float f; uint32_t u; };
    FloatToBits temperatureBits, fractionHeaterBits;
    temperatureBits.f = temp;
    fractionHeaterBits.f = (float) heater / HEATER_MAX;

    // Gah, this works fine without any of this garbage compiling on mac
    char time_print_buffer[32];
    uintToHexStr(timeSinceStart, time_print_buffer, 64);
    char heater_print_buffer[16];
    uintToHexStr(fractionHeaterBits.u, heater_print_buffer, 32);
    char temp_print_buffer[16];
    uintToHexStr(temperatureBits.u, temp_print_buffer, 32);

    char print_buffer[128];
    sprintf(
        print_buffer,
        "{\"time_u64\":\"%s\",\"heater_f32\":\"%s\",\"temperature_f32\":\"%s\"}\n",
        time_print_buffer,
        heater_print_buffer,
        temp_print_buffer
    );
    Serial.print(print_buffer);

    // 100 ms total cycle time to give the MAX31856 time for next temperature conversion (see datasheet!)
    delay(100 - (millis() - timeNow));
}
