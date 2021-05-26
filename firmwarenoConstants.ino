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


/*************************
 * CONSTANT DECLARATIONS *
 *************************/

// version number.
const float   VERSION_NUM = 0.1;

// SPI pins for ADC/DAC.
const uint8_t GALVO_X = 10;
const uint8_t GALVO_Y = 11;

// Constants for state
const uint8_t STATE_WAIT_READ_BYTE      = 0;
const uint8_t STATE_PREVIEW_SCAN        = 1;
const uint8_t STATE_ACQUISITION_SCAN    = 2;
const uint8_t STATE_RETURN              = 8;

const uint8_t STATE_SEND_SETTING        = 4;
const uint8_t STATE_RECV_UPDATE_SETTING = 6;

// Constants for serial information classification
const uint8_t INIT = 0;
const uint8_t CURR = 1;
const uint8_t GOTO = 2;
const uint8_t SCAN = 3;
const uint8_t TSEL = 4;
const uint8_t DATA = 5;

// constants for timer
volatile bool toggle = true;
volatile bool updated = false;
volatile uint32_t dT = 0;

/************************* 
 * PARAMETER DECLARATION *
 *************************/

// Global parameters
uint8_t  state = 0;
uint8_t  print_once = 0;
uint8_t  setting_byte = 0;
uint8_t  selected = 0;
uint8_t  buffer8_2[2] = {0};
uint16_t buffer16 = 0;
uint16_t parameters[32] = { // NOTE: can be extended to 64 parameters by design.
  25600, // 0b00000: preview: max_x = 4 x 6400 mV
  25600, // 0b00001: preview: max_y = 4 x 6400 mV
  8,     // 0b00010: preview: samples_x_n = 2^8+1 = 257
  5,     // 0b00011: preview: samples_y_n = 2^5   = 32
  8192,  // 0b00100: preview: rad_s = 2 x 4096 mV
  16384, // 0b00101: preview: rad_l = 4 x 4096 mV
  10,    // 0b00110: preview: samples_r_n = 2^10 = 1024
  250,   // 0b00111: speckle reduction: del_s, 250/6400 for raster, /4096 for circular
  25600, // 0b01000: acquisition: max_x = 4 x 6400 mV
  25600, // 0b01001: acquisition: max_y = 4 x 6400 mV
  9,     // 0b01010: acquisition: samples_x_n = 2^9+1 = 513
  9,     // 0b01011: acquisition: samples_y_n = 2^9   = 512
  8192,  // 0b01100: acquisition: rad_s = 2 x 4096 mV
  16384, // 0b01101: acquisition: rad_l = 4 x 4096 mV
  10,    // 0b01110: acquisition: samples_r_n = 2^10 = 1024
  4,     // 0b01111: speckle reduction: samples_s_n, 2^2 = 4
  1000,  // 0b10000: transition settings: del_x = 1000 dac values
  1000,  // 0b10001: transition settings: del_y = 1000 dac values
  0,     // 0b10010: 
  0,     // 0b10011: 
  0,     // 0b10100: 
  0,     // 0b10101: 
  0,     // 0b10110: 
  0,     // 0b10111: 
  20,    // 0b11000: scan mode: first nibble for preview, second nibble acquisition
  20,    // 0b11001: acquisition rate: 20 kHz 
  32768, // 0b11010: dac_x offset: 32768 
  32768, // 0b11011: dac_y offset: 32768 
  0,     // 0b11100: 
  0,     // 0b11101: 
  0,     // 0b11110: 
  0      // 0b11111: save all settings to eeprom (not implemented)
};       
uint16_t *scan_mode = &parameters[24];

// precalculated arrays
int16_t  ref_s[16]   = {0};
uint16_t ref_r[1024] = {0};
uint16_t ref_x[8192] = {0};
uint16_t ref_y[512]  = {0};

/************************* 
 * FUNCTION DECLARATIONS *
 *************************/

inline void     send_serial_info(uint8_t code, uint8_t n);
inline void     send_serial_info16(uint8_t code, uint16_t n);
inline void     clear_serial_data(void);

void dac2(uint16_t nx, uint16_t ny, uint8_t pinx, uint8_t piny);
void myISR(void);
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


inline void send_serial_info(uint8_t code, uint8_t n) {
/**
 *  helper function to print current state to serial port
 */

  if (code == INIT) {
    Serial.print("init");
  } else if (code == CURR) {
    Serial.print("curr");
  } else if (code == GOTO) {
    Serial.print("goto");
  } else if (code == SCAN) {
    Serial.print("scan");
  } else if (code == TSEL) {
    Serial.print("tsel");
  } else if (code == DATA) {
    Serial.print("data");
  }

  Serial.print("::");
  Serial.println(n);

  return;
}

inline void send_serial_info16(uint8_t code, uint16_t n) {
/**
 *  helper function to print current state to serial port
 */

  if (code == INIT) {
    Serial.print("init");
  } else if (code == CURR) {
    Serial.print("curr");
  } else if (code == GOTO) {
    Serial.print("goto");
  } else if (code == SCAN) {
    Serial.print("scan");
  } else if (code == TSEL) {
    Serial.print("tsel");
  } else if (code == DATA) {
    Serial.print("data");
  }

  Serial.print("::");
  Serial.println(n);

  return;
}

inline void clear_serial_data(void) {
/**
 *  helper function to clear excess serial data from the buffer
 */

  while (Serial.available() > 0) {
    Serial.read();
  }

  return;
}

/**
void spi_begin(void) {
/**
 *  helper function to initialize spi transfers
 /

  SPI.beginTransaction(settings_DAC);

  return;
}

void spi_end(void) {
/**
 *  helper function to end spi transfers
 /

  SPI.endTransaction();

  return;
}

void myISR(void) {

}

*/
void dac2(uint16_t nx, uint16_t ny, uint8_t pinx, uint8_t piny) {
/**
 *  helper function for writing to two DACs
 *  
 *  @param uint16_t nx   : 16-bit integer to be sent to DAC_x
 *  @param uint16_t ny   : 16-bit integer to be sent to DAC_y
 *  @param uint8_t  pinx : CS pin of DAC_x
 *  @param uint8_t  piny : CS pin of DAC_y
 */

  digitalWrite(pinx, HIGH);
  SPI.transfer16(nx);
  digitalWrite(pinx, LOW);

  digitalWrite(piny, HIGH);
  SPI.transfer16(ny);
  digitalWrite(piny, LOW);

  return;
}

