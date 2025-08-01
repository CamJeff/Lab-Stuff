const byte geigerCounterPin = 2; // manually set up in a pull-down configuration

void setup() {
    pinMode(geigerCounterPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(geigerCounterPin), handle, RISING);
    Serial.begin(115200);
}

char buffer[256]; // for reading input from serial
unsigned int bufferPos = 0;
unsigned int mode = 0; // 0 = unset, 1 = interval, 2 = count

volatile long count = 0;
int lastCount = 0;
long lastTime = -1l;

void loop() {

    if (mode == 0) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c != '\n') {
                buffer[bufferPos++] = c;
            } else {
                buffer[bufferPos] = '\0';
                if (strcmp(buffer, "interval") == 0) {
                    mode = 1;
                    Serial.println("mode set: interval");
                } else if (strcmp(buffer, "count") == 0) {
                    mode = 2;
                    Serial.println("mode set: count");
                } else {
                    Serial.println("invalid mode");
                }
                bufferPos = 0;
            }
        }
    }

    if (mode == 1) {
        // don't do anything within loop in interval mode
    }

    if (mode == 2) {

        long time = millis();

        if (lastTime == -1l || time - lastTime >= 1000) {

            Serial.print("count (");
            Serial.print(time - lastTime);
            Serial.print("ms): ");
            Serial.println(count - lastCount);

            lastCount = count;
            lastTime = time;

        }
        if (time - lastTime < 950) {
            // only delay for the first 950ms of the second, and spin for the last
            // 50ms, so we can hit the 1s mark more accurately
            delay(950 - (time - lastTime));
        }

    }
}

// TODO: check if we need to debounce
void handle() {
    if (mode == 2) { // simple count
        count++;
    }
    if (mode == 1) {
        unsigned long time = millis();

        if (lastTime == -1l) {
            lastTime = time;
            return;
        }

        Serial.print("interval: ");
        Serial.print(time - lastTime);
        Serial.println("ms");

        lastTime = time;
    }
}

