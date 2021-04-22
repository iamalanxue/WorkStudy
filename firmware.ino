 /**
 * Create Date: 13.01.2021
 * Design Name: OCT DAQ Firmware v1.0
 * Created By : Wei Hong. FOIL, OCT.
 *
 * Description:
 * Firmware for ADQ for OCT to control galvo and camera triggers.
 * 
 * Revision:
 * Revision v0.1 - File created
 * Revision v0.2 - Communicates with python
 *
 * Additional Comments:
 * 
 */

#include <SPI.h>
#include <SAMD51_InterruptTimer.h>
#include "CONSTANTS.h"
/**************************************************
 * SETUP
 * 
 * Initializes serial, SPI, ADC and DAC chips.
 **************************************************/
void setup() {

  // setup serial port
  Serial.begin(2000000);
  while (! Serial);
  Serial.setTimeout(10);
  Serial.print("init::version ");
  Serial.println(VERSION_NUM);
  Serial.println("done::metro m4 ready!");
  delay(10);

  // initialize SPI pins
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);

  // initialize CS pins of SPI
  pinMode(GALVO_X, OUTPUT);
  pinMode(GALVO_Y, OUTPUT);

  digitalWrite(GALVO_X, LOW);
  digitalWrite(GALVO_Y, LOW);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  SPISettings settings_DAC(2000000, MSBFIRST, SPI_MODE1);
  SPI.beginTransaction(settings_DAC);
  
  // initialize galvo DACs at -5V;
  dac2(0x0300, 0x0300, GALVO_X, GALVO_Y);

  uint8_t state = 0;
  uint8_t print_once = 0;
}

void loop() {
  switch (state)
  {
  case STATE_WAIT_READ_BYTE:
    if (print_once == 0) {
      // print current state to serial port
      send_serial_info(SCAN, *scan_mode);
      send_serial_info(CURR, state);
      print_once++;
    }

    if (Serial.available() >= 1) {
      // read setting_byte on serial port

      digitalWrite(13, HIGH); //for debugging to check if the arduino has recieved input
      setting_byte = Serial.read();
      clear_serial_data();

      // print next state to serial port
      state    = setting_byte >> 5;
      selected = setting_byte & 0x3F;

      // corrects state 5 -> 4, 7 -> 6.
      if (state > 4) {
        state = state & 0x6; 
      }
      print_once = 0;
      send_serial_info(GOTO, state);
    }
    break;
  case STATE_SEND_SETTING:
    if (print_once == 0) {
      send_serial_info(CURR, state);
      send_serial_info(TSEL, selected);
      print_once++;
    }

    send_serial_info16(DATA, parameters[selected]);

    // print next state to serial port
    state = STATE_WAIT_READ_BYTE;
    print_once = 0;
    send_serial_info(GOTO, state);
    break; 
  case STATE_RECV_UPDATE_SETTING:
    if (print_once == 0) {
      send_serial_info(CURR, state);
      send_serial_info(TSEL, selected);
      print_once++;
    }
    
    if (Serial.available() >= 2) {
      // get parameter data from serial port and save
      buffer16 = Serial.read();
      buffer16 = buffer16 << 8 | ((uint16_t) (Serial.read()));
      clear_serial_data();
      parameters[selected] = buffer16;

      // print next state to serial port
      state = STATE_WAIT_READ_BYTE;
      print_once = 0;
      send_serial_info(GOTO, state);
    }
    break;
  case STATE_PREVIEW_SCAN:
    if (print_once == 0) {
      send_serial_info(CURR, state);
      print_once++;
    }

// preview loop here.
    // activate timer and hooks
    // check if 
    state = STATE_WAIT_READ_BYTE;
    print_once = 0;
    send_serial_info(GOTO, state);
    break; 
  case STATE_ACQUISITION_SCAN:
    if (print_once == 0) {
      send_serial_info(CURR, state);
      print_once++;
    }

// activate acquisition loops here.
    for (uint16_t i_y = 0; i_y < n_y; i_y++) { //this for loop is for stepping up wave by one step 
        dac_y = 768 + 6400 + (i_y * range_y * 6400 / (n_y - 1));
        if (i_y % 2 == 0) {
            for (uint16_t i_x = 0; i_x < n_x; i_x++) { //wave going in the right direction
                dac_x = 768 + 6400 + (i_x * range_x * 6400 / (n_x - 1));
                if (scan_mode == ACQUISITION) {
                    dac2(dac_x, dac_y, 10, 11); 
                    // cout << dac_x << ", " << dac_y << endl;
                    continue;
                } //if mode is acquisition do not perform the rising and falling edge 
                if (i_x % 2 == 0) {
                    for (int y = -2; y < 3; y++) {  //output rising edge for pwm wave 
                        high = dac_y + y * del_s;
                        //cout << "Rising Edge: " << high << endl;
                        dac2(dac_x, high, 10, 11); 
                        // cout << dac_x << ", " << high << endl; 
                    }
                }
                else {
                    for (int y = 2; y > -3; y--) { //output falling edge for pwm wave 
                        low = dac_y + y * del_s;
                        //cout << "Falling Edge: " << low << endl;
                        dac2(dac_x, low, 10, 11); 
                        // cout << dac_x << ", " << low<< endl;
                    }
                }
            }
        }
        else {
            for (uint16_t i_x = n_x; i_x > 0; i_x--) { //wave going in the left direction 
                dac_x = 768 + 6400 + ((i_x - 1) * range_x * 6400 / (n_x - 1));
                if (scan_mode == ACQUISITION || scan_mode == INTERLEAVED) {
                    dac2(dac_x, dac_y, 10, 11); 
                    // cout << dac_x << ", " << dac_y << endl; 
                    continue; 
                }
                if (i_x % 2 == 0) {
                    for (int y = -2; y < 3; y++) {  //output rising edge for pwm wave 
                        high = dac_y + y * del_s;
                        //cout << "Rising Edge: " << high << endl;
                        dac2(dac_x, high, 10, 11); 
                        // cout << dac_x << ", " << high << endl;
                    }
                }
                else {
                    for (int y = 2; y > -3; y--) { //output falling edge for pwm wave 
                        low = dac_y + y * del_s;
                        //cout << "Falling Edge: " << low << endl;
                        dac2(dac_x, low, 10, 11); 
                        // cout << dac_x << ", " << low << endl;
                    }
                }
            }
        }
    }
    // end of the acquisition state
    
    // print next state to serial port
    state = STATE_RETURN;
    print_once = 0;
    send_serial_info(GOTO, state);
    break; 
  case STATE_RETURN:
    if (print_once == 0) {
      send_serial_info(CURR, state);
      print_once++;
    }

    // return the laser to -5,-5
    state = STATE_WAIT_READ_BYTE;
    print_once = 0;
    send_serial_info(GOTO, state);
    break; 
  }
}