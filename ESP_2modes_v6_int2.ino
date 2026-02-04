// this version is compatible with PY_2modes_v5_Hands and PY_2modes_v6_results, 
// and does not have the piezoelectric buzzer activation added yet

#include <Arduino.h>
// 3 byte header to distinguish samples from messages
#define SAMPLE_HEADER_0 0xAA
#define SAMPLE_HEADER_1 0x55
#define PACKET_TYPE_SAMPLE 0x01

#include <Wire.h>
#include "Adafruit_DRV2605.h"

#define TCA_ADDR 0x70 // Default I2C address for TCA9548A
#define NUM_LRAS 5

Adafruit_DRV2605 drv;

// Pin setup (change based on our electrical setup)
const int numLRAs = 10;
// const int lraPins[10] = {2, 4, 5, 18, 19, 21, 22, 23, 25, 26};
// ADC pins for force sensors 
// const int forcePins[10] = {32, 33, 34, 35, 36, 39, 12, 13, 14, 27}; - old, not compatible with the MUX
// ----- Pin definitions -----
const int SIG_PIN = 34; // ADC input

const int S0_PIN = 33;
const int S1_PIN = 27;
const int S2_PIN = 25;
const int S3_PIN = 26;


// Timing settings, each phase should be 60s (60000ms) 
// but I have them set to 10s atm for faster example trials and debugging
// I wouldnt recommend going much faster bc the data collection may start to overlap with the next trial 
const int precond_phase = 10000;   // pre condition phase
const int active_phase = 10000;   // stimulation phase
const int rest_phase = 10000;      // rest phase     

// set data sampling rate in microseconds (1000us = 1000 Hz)
const int SAMPLE_RATE_US = 1000;  

// there is probably a cleaner way to do this but I wanted to be thorough
// default to training mode 
bool assessment_mode = false;
bool precond_flag = true; // start at this step
bool stim_flag = false; // dont start at the other steps
bool rest_flag = false;
bool msg_flag = false;
uint32_t start_time = 0;
// assessment mode initialization
bool data_flag = false;
int counter = 0;
bool fresh_start = true;
bool rest_start_unsent = true; // establish rest message flag to avoid sending multiple times per trial
bool LRA_ON = false;
uint32_t TIMEOUT = millis() + 180000; // 3 minute timeout if no message is received to default back to training mode
uint8_t active_digit = 13; // outside of the range intentionally, so we know if this isnt updated
uint16_t random_delay = 333333; // ms ^
// establishing vars for event scheduling  - these will all fill at the start of the trial
uint32_t activation_time = 0;
uint32_t activation_end = 0;
uint32_t next_sample = 0; // time of next sample in us (first sample here)
uint32_t rest_start = 0; // rest phase start
uint32_t trial_end = 250; // end of trial

// Select a MUX channel (0–15)
void selectMuxChannel(uint8_t channel) {
digitalWrite(S0_PIN, channel & 0x01);
digitalWrite(S1_PIN, channel & 0x02);
digitalWrite(S2_PIN, channel & 0x04);
digitalWrite(S3_PIN, channel & 0x08);

delayMicroseconds(10); // allow channel to settle
}
// establishes the format for and streams the data
// only used in assessment mode
void streamForceSample(uint32_t t_ms, int active_digit, int random_delay) {
    // 30 bytes per sample (3 byte header/type and 27 byte package)
    uint8_t buf[30];
    int idx = 0;

    // Header, as established at the top 
    buf[idx++] = SAMPLE_HEADER_0;
    buf[idx++] = SAMPLE_HEADER_1;
    buf[idx++] = PACKET_TYPE_SAMPLE;

    // Timestamp (uint32_t) 
    memcpy(&buf[idx], &t_ms, sizeof(uint32_t));
    idx += sizeof(uint32_t);

    // Force sensors (uint16_t × 10) 
    for (int i = 0; i < 10; i++) {
        selectMuxChannel(i);
        uint16_t f = analogRead(SIG_PIN);  // 0–4095
        memcpy(&buf[idx], &f, sizeof(uint16_t));
        idx += sizeof(uint16_t);
    }
    // Active digit (uint8_t) 
    uint8_t ad = (uint8_t)active_digit;
    buf[idx++] = ad;

    // Random delay (uint16_t)
    uint16_t rd = (uint16_t)random_delay;
    memcpy(&buf[idx], &rd, sizeof(uint16_t));
    idx += sizeof(uint16_t);

    // (Sanity check) idx should be exactly 30
    Serial.write(buf, idx);
}

// LRA functions
/* Select I2C multiplexer channel */
void tcaSelect(uint8_t channel) {
if (channel > 7) return;
Wire.beginTransmission(TCA_ADDR);
Wire.write(1 << channel);
Wire.endTransmission();
}
/* Initialize DRV2605 on currently selected mux channel */
void initDRV() {
if (!drv.begin()) {
Serial.println("Could not initialize DRV2605 on this channel!");
return;
}
drv.selectLibrary(6); // LRA library
drv.setMode(DRV2605_MODE_INTTRIG);

// Put DRV2605 into LRA mode
drv.writeRegister8(0x1A, 0xB0);

Serial.println("DRV2605 initialized in LRA mode.");
}

// this OLD  function for LRA vibration is only used in the training mode 
// not used in assessment mode bc we dont want 500ms delays during data collection
//void vibrateLRA(int index, int duration_ms = 500) {
//    if (index < 0 || index >= numLRAs) return;
//    digitalWrite(lraPins[index], HIGH);
//    delay(duration_ms);
//    digitalWrite(lraPins[index], LOW);
//}

/* Vibrate one LRA */
void vibrateContinuous(float seconds, uint8_t intensity) {

drv.writeRegister8(0x16, intensity); // Rated voltage
drv.writeRegister8(0x17, intensity); // Overdrive voltage

drv.setWaveform(0, 14); // Buzz
drv.setWaveform(1, 0);

unsigned long duration = seconds * 1000;
unsigned long startTime = millis();

while (millis() - startTime < duration) {
drv.go();
delay(1);
}

drv.stop();
}



// SETUP ______________________________________________________________________________________________
void setup() {
    Serial.begin(921600); // important: must match python and must be fast enough to keep up with 1000Hz data streaming
    Wire.begin();

    pinMode(S0_PIN, OUTPUT);
    pinMode(S1_PIN, OUTPUT);
    pinMode(S2_PIN, OUTPUT);
    pinMode(S3_PIN, OUTPUT);

    // Full ADC range (0–3.3 V)
    analogSetAttenuation(ADC_11db);

    //for (int i = 0; i < numLRAs; i++) {
    //    pinMode(lraPins[i], OUTPUT);
    //    digitalWrite(lraPins[i], LOW);
    //}

    for (uint8_t ch = 0; ch < NUM_LRAS; ch++) {
    Serial.print("Channel ");
    Serial.println(ch);
    tcaSelect(ch);
    initDRV();
    delay(10);
    }

    Serial.println("ESP32 READY (training mode default)");
}

// the actual loop ________________________________________________________________________________
// this has been restructured - it used to be while statements that caused problematic waiting, changed to if statements to allow the esp to stay active and not trigger the watchdog
void loop() {
    if (assessment_mode) { // run in assessment mode
        // ASSESSMENT MODE
        if (!data_flag) { // if we are not actively collecting data, 
            if (fresh_start) { // at the start of the trial
                // check for COMBO
                    // if no message
                    if (!Serial.available()) {
                        // has the TIMEOUT expired?
                        if (millis()>= TIMEOUT) { // if yes, switch back to training mode
                            assessment_mode = false; 
                            Serial.println("#TIMEOUT_NO_COMMAND"); // notify (all messages start with # to indicate to the python decoder that they are messages)
                            precond_flag = true; // start at this step
                            stim_flag = false; // dont start at the other steps
                            rest_flag = false;
                            msg_flag = false;
                            return; // exit and reenter the loop 
                            }
                        // if no, (send OK,) wait, return and check again
                        if (millis()< TIMEOUT) {
                            // Serial.println("#OK");  //signal to python
                            delay(5);
                            return;
                        }
                    }
                    // if there is a message from python
                    if (Serial.available()) {
                        uint32_t t0_ms = millis(); // save the time of the trial start
                        uint64_t t0_us = micros();

                        // we are trying to get the digit combo from python
                        String combo = Serial.readStringUntil('\n');
                        combo.trim();  // e.g. "0,1,1,1,0,0,0,0,0,0"

                        // if the message is not the digit combo, respond according to the message
                        if (combo == "GUI_CLOSED") { assessment_mode=false; Serial.println("#GUI_CLOSED"); 
                            precond_flag = true; // start at this step
                            stim_flag = false;
                            rest_flag = false;
                            msg_flag = false; return;}
                        if (combo == "WAIT_COMBO" || combo == "WAIT_COMBO" || combo == "WAIT_COMBO") { Serial.println("#WAIT_COMBO"); return;}
                        if (combo == "ASSESS") { Serial.println("#OK"); return;}
                        if (combo == "" || combo == "COMBO") {
                            combo = Serial.readStringUntil('\n'); // read the next incoming line
                            combo.trim();
                            }
                        // at this point the saved combo should be the digit combo
                        // add auditory buzzer activation here, placeholder is to print:
                        Serial.println("#START TRIAL");
                        // print the combo to confirm it was correctly recieved
                        Serial.print("#COMBO=");
                        Serial.println(combo);

                        // Parse the combination, convert string into 10 int array
                        int active_list[10];
                        int n_active = 0; 
                        char buf[64];
                        combo.toCharArray(buf, 64); // copy string to a C char array
                        char *tok = strtok(buf, ","); // tokenize (split) the string by commas
                        int idx = 0; 
                        while (tok != NULL && idx < 10) { // loop through each token
                            int v = atoi(tok); // convert to integers
                            active_list[idx] = v; // store value in array
                            if (v == 1) n_active++; // n_active counts active entries of the combo (1s)
                            tok = strtok(NULL, ","); // get the next token (continue from where it left off)
                            idx++;
                        }

                        if (n_active == 0) {
                            Serial.println("#ERR_NO_ACTIVE"); // if there are no active digits in the combo something is wrong, flag
                            return;
                        }

                        // Create an array of the indices of the active digits
                        int picks[n_active]; 
                        int pi = 0;
                        for (int i = 0; i < 10; i++) {
                            if (active_list[i] == 1) picks[pi++] = i; // every time the entry is 1, put the index into picks and increment pi
                        }

                        // Choose random active index
                        active_digit = picks[random(0, n_active)];
                        // choose random delay 
                        random_delay = random(0, active_phase); // ms
                        // print digit and random delay
                        Serial.print("#ACTIVE_DIGIT=");
                        Serial.println(active_digit);
                        Serial.print("#random_delay_MS=");
                        Serial.println(random_delay);

                        // event scheduling
                        // if there are issues here, change everything to be in ms (no more us)
                        Serial.print("#t0_ms");
                        Serial.println(t0_ms);
                        Serial.print("#t0_us");
                        Serial.println(t0_us);
                        activation_time = t0_us + precond_phase*1000 + random_delay*1000; // time of vibration
                        activation_end = activation_time+500000;
                        Serial.print("#activation time");
                        Serial.println(activation_time);
                        next_sample = activation_time - 1000000; // time of next sample in us (first sample here)
                        Serial.print("#next sample = ");
                        Serial.println(next_sample);
                        rest_start = t0_ms + precond_phase + active_phase; // rest phase start
                        Serial.print("#rest_start = ");
                        Serial.println(rest_start);
                        trial_end = rest_start + rest_phase; // end of trial
                        Serial.print("#trial_end = ");
                        Serial.println(trial_end);
                        rest_start_unsent = true; // establish rest message flag 
                        LRA_ON = false;
                        data_flag = true;
                        fresh_start=false;
                        return; 
                    }
            }
            if (!fresh_start){ // gray area (after data collection but not at the start of a new trial)
                if (millis()> trial_end - 250) {// check if enough time has passed for all 3 phases "trial_end" (maybe minus a little time in case combo receiving causes delays)
                    // if yes, begin new trial
                        fresh_start = true; 
                        TIMEOUT = millis() + 60000; // reset the timeout
                        Serial.println("#TRIAL_DONE");
                        delay(5);
                        Serial.println("#WAIT_COMBO");
                }
                if (millis()< trial_end - 250) { // if no, check if rest phase has started, 
                    if (millis()> rest_start && rest_start_unsent) { // if it has, send rest msg - and only send once! (set rest_start_unsent to false)
                        // send rest message
                        rest_start_unsent = false;
                        Serial.flush();
                        Serial.println("#REST_START");

                    }
                }
                return; // break out and re enter
            } 
        }  
        if (data_flag && counter < 6000) { // if we are actively collecting data
            uint64_t now = micros(); // update the time in us
            if (now + 1500000ULL < next_sample) {delay(1000); return;} // if theres still at least 1.5s before the first sample, delay by a second, exit and re enter
            if (now >= next_sample) { // it's time to collect a sample
                streamForceSample(millis(), active_digit, random_delay);
                next_sample = next_sample + SAMPLE_RATE_US; // update next sample time  (previous sample time + sample interval)
                counter = counter + 1; // update counter so we know when we have sent 6000 samples
                if (!LRA_ON && now >= activation_time && now < activation_end){ // if the LRA is off and its time to activate (and less than activation end bc we dont want to turn it back on after we have already turned it off for that trial)
                    // start vibration
                    // digitalWrite(lraPins[active_digit], HIGH);
                    tcaSelect(active_digit);
                    drv.writeRegister8(0x16, 90); // Rated voltage
                    drv.writeRegister8(0x17, 90); // Overdrive voltage

                    drv.setWaveform(0, 14); // Buzz
                    drv.setWaveform(1, 0);
                    drv.go();
                    // ^ i think this might be an issue 
                    LRA_ON = true;
                    //Serial.println("#LRA ON"); - commented out bc its best practice to avoid sending messages during data streaming
                    return;
                }
                if (LRA_ON && now >= activation_end){ // if the LRA is on and its time to turn it off
                    // stop vibration
                    // digitalWrite(lraPins[active_digit], LOW); 
                    drv.stop();
                    //Serial.println("#LRA OFF");
                    LRA_ON = false;
                    return;
                }
            }
            return;
        }
        // if we have sent 6000 samples, data streaming is done.
        if (counter == 6000) {fresh_start=false; data_flag = false; counter = 0; Serial.flush(); Serial.println("#6000 samples"); return;}  
    }
    if (!assessment_mode) { // if you werent told to run in assessment mode, run in training mode
        // TRAINING MODE
        if (precond_flag) {// precond_flag = true
            start_time = millis(); // collect start time for proper scheduling
            // add buzzer activation here, placeholder is to print:
            Serial.println("Start Trial");
            // (no combination selection because there is no way to communicate the combination to the user in training mode 
            //  and the combinations result in some fingers being selected more frequently than others --> uneven training)
            // randomly select one digit: 
            active_digit = random(0, 10);
            // randomly select a delay in ms within the active phase, determines when the vibration is sent
            random_delay = random(0, active_phase); 
            // user would not be able to see this (or any other training mode prints) without connecting to a computer, 
            // but it is helpful for debugging and monitoring system behavior
            Serial.print("TRAINING: digit=");
            Serial.print(active_digit);
            Serial.print(" delay=");
            Serial.println(random_delay);
            // set flags
            precond_flag = false;
            stim_flag = true;
            return;
        }
        if (stim_flag) { // stim flag
            if (millis() < start_time + random_delay + precond_phase) { // too early to stimulate, check messages
                msg_flag = true;
            } else { // good to go
                // vibrate LRA ()
                tcaSelect(active_digit);
                vibrateContinuous(0.5, 90); // .5 seconds, intensity 90
                //vibrateLRA(active_digit);
                Serial.println("vibration sent");
                // set flag
                stim_flag = false;
                rest_flag = true;
                return;
            }
        }
        if (rest_flag) { // rest flag
            if (millis() < start_time + precond_phase + active_phase + rest_phase) { // if too early to reset the trial, check for messages
                msg_flag = true;
            } else { // reset to first step of the trial
                precond_flag = true;
                rest_flag = false;
                return;
            }
        }
        if (msg_flag) { // msg flag
            // Serial.println("checking for messages");
            // check for python messages and switch to assessment mode if they are detected
            if (Serial.available()) {
                String cmd = Serial.readStringUntil('\n');
                cmd.trim(); 
                if (cmd == "ASSESS" || cmd =="SESSION START" || cmd == "WAIT_COMBO") {
                    assessment_mode = true; // Python detected → switch modes
                    fresh_start = true;
                    TIMEOUT = millis() + 180000;
                    Serial.println("OK");  // signal Python
                }
            }
            // turn msg flag false
            msg_flag = false;
            delay(5);
            return;
        }
    } 
}
