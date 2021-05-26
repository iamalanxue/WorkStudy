"""
Create Date: 2021.01.13
Design Name: OCT DAQ Controller Library
Created By : Wei Hong Yeo. Northwestern University, FOIL.
Modified By: Alan Xue. Northwestern University.

Description:
Library for OCT DAQ Control

Revisions  :
Revision v0.1 - Initial file created

Additional Comments:

"""


import serial
import serial.tools.list_ports
import struct
import time

STATE_IDLE = 0


class GalvoCtrl:
    def __init__(self, comport='auto'):
        self.debug = True
        self.valid_dev = False
        self.serial = None

        if comport == 'auto':
            self.valid_dev = self.autodetect()

        else:
            try:
                self.serial = serial.Serial(port=comport, baudrate=2000000, timeout=10)
                time.sleep(1)
                self.valid_dev = self.check()
                if not self.valid_dev:
                    self.serial.close()

            except FileNotFoundError:
                self.serial = None
                print("could not open %s, no such port available." % comport)

        if self.serial is not None:
            data_in = int(self.clear_buffer(clear_until="curr"))
            if data_in == STATE_IDLE:
                print("Metro M4 board is ready.")

            else:
                print("Metro M4 is in the wrong state.")
                if self.debug:
                    print("expected {}, but got {} instead.".format(STATE_IDLE, data_in))

    def autodetect(self):
        print("Please wait... Detecting Metro M4...")
        valid_dev = False
        for dev_info in serial.tools.list_ports.comports():
            if not valid_dev:
                self.serial = serial.Serial(dev_info.device, baudrate=2000000, timeout=10)
                time.sleep(1)
                valid_dev = self.check()
                if valid_dev:
                    print("Metro M4 board detected successfully.")
                    return True
                else:
                    self.serial.close()
        print("Metro M4 board not found. Try inputting the COM port number manually instead.")
        self.serial = None
        return False

    def check(self):
        data_in = self.clear_buffer(clear_until="done")
        if data_in == "metro m4 ready!":
            return True
        return False

    def read_settings(self, setting):
        if not setting.valid:
            return False

        state_data_to_python = 4

        self.write8((state_data_to_python << 5) + setting.byte)

        if self.debug:
            print("sending byte: " + str((state_data_to_python << 5) + setting.byte))

        if self.wrong_state(target_state=state_data_to_python):
            return False

        if self.wrong_parameter(target_parameter=setting.identifier):
            return False

        setting.value = int(self.clear_buffer(clear_until="data"))

        if self.wrong_state(target_state=STATE_IDLE):
            return False

        return True

    def wrong_state(self, target_state):  #this checks 
        data_in = int(self.clear_buffer(clear_until="curr"))
        if target_state != data_in:
            print("metro m4 is in the wrong state.")
            if self.debug:
                print("expected {}, but got {} instead.".format(target_state, data_in))
            return True
        return False

    def wrong_parameter(self, target_parameter):
        data_in = int(self.clear_buffer(clear_until="tsel"))
        if target_parameter != data_in:
            print("metro m4 is selecting the wrong parameter.")
            if self.debug:
                print("expected {}, but got {} instead.".format(target_parameter, data_in))
            return True
        return False

    def write_settings(self, setting):
        if not setting.valid:
            return False

        state_data_from_python = 6

        self.write8((state_data_from_python << 5) + setting.byte)

        if self.debug:
            print("sending byte: " + str((state_data_from_python << 5) + setting.byte))

        if self.wrong_state(target_state=6):
            return False

        if self.wrong_parameter(target_parameter=setting.identifier):
            return False

        self.write16(setting.value)

        if self.wrong_state(target_state=STATE_IDLE):
            return False
        return True

    def update_settings(self, settings): 
        #takes in an array of settings and updates them from the default values 
        for setting in settings:
            self.write_settings(setting)
            #provide a check for a potential transfer error 
            self.read_settings(setting)
        return 

    def clear_buffer(self, clear_until=None):
        timeout = 0.1
        t0 = time.time()

        # clear with timeout
        while time.time() - t0 < timeout:
            if clear_until is None:
                if self.serial.in_waiting:
                    t0 = time.time()
                    string_in = self.read()
                    if self.debug:
                        print("cleared: " + string_in)

            else:
                if self.serial.in_waiting:
                    message_data = self.read(message_flag_filter=clear_until)
                    if message_data is None:
                        t0 = time.time()
                    else:
                        return message_data
        return

    def read(self, message_flag_filter=None):
        string_in = self.serial.readline().decode().strip()
        double_colon_position = string_in.find("::")

        if double_colon_position < 0:
            return None
        else:
            message_type = string_in[:double_colon_position]
            message_data = string_in[double_colon_position + 2:]
            if message_flag_filter is None:
                if self.debug:
                    print(string_in)
                return message_data
            else:
                if self.debug:
                    print("Looking for " + message_flag_filter)
                    print("obtained " + message_type)
                    print(">>> " + message_data)
                if message_flag_filter == message_type:
                    return message_data
                else:
                    return None

    def write8(self, data):
        self.serial.write(struct.pack(">B", data)) #unsigned char big-endian or most significant digit stored first
        return

    def write16(self, data):
        self.serial.write(struct.pack(">H", data)) #unsigned short 
        return

    def sendInstructionByte(self, byte): #used for going into the different modes
        self.serial.write(byte) #write an unsigned 8 int for arduino to read
        return 

class Settings:
    def __init__(self, parameter, value=None):
        identifiers = {
            "pre_max_x":    0b00000,
            "pre_max_y":    0b00001,
            "pre_n_x":      0b00010,
            "pre_n_y":      0b00011,
            "pre_rad_s":    0b00100,
            "pre_rad_l":    0b00101,
            "pre_n_r":      0b00110,
            "acq_max_x":    0b00111,
            "acq_max_y":    0b01000,
            "acq_n_x":      0b01001,
            "acq_n_y":      0b01010,
            "acq_rad_s":    0b01011,
            "acq_rad_l":    0b01100,
            "acq_n_r":      0b01101,
            "sr_del_s":     0b01110,
            "sr_n_s":       0b01111,
            "max_del_x":    0b10000,
            "max_del_y":    0b10001,
            "scan_mode":    0b11000,
            "acq_rate":     0b11001,
            "dac_x_zero":   0b11010,
            "dac_y_zero":   0b11011
        }

        default_values = {
            "pre_max_x": 25600,
            "pre_max_y": 25600,
            "pre_n_x": 8,
            "pre_n_y": 5,
            "pre_rad_s": 8192,
            "pre_rad_l": 16384,
            "pre_n_r": 10,
            "acq_max_x": 250,
            "acq_max_y": 25600,
            "acq_n_x": 25600,
            "acq_n_y": 9,
            "acq_rad_s": 9,
            "acq_rad_l": 8192,
            "acq_n_r": 16384,
            "sr_del_s": 10,
            "sr_n_s": 4,
            "max_del_x": 1000,
            "max_del_y": 1000,
            "scan_mode": 20,
            "acq_rate": 20,
            "dac_x_zero": 32768,
            "dac_y_zero": 32768
        }

        scan_modes = {
            "raster_acquisition":   0b00010100,
            "raster_interleaved":   0b00010110,
            "raster_speckle rdn":   0b00010111,
            "circular_acquisition": 0b00111000,
            "circular_interleaved": 0b00111010,
            "circular_speckle rdn": 0b00111011
        }

        self.valid = False

        if parameter in identifiers.keys():
            self.valid = True
            self.parameter = parameter
            self.identifier = identifiers[parameter]
            self.byte = self.identifier

            if value is None:
                self.value = None
            elif self.parameter == "scan_mode":
                self.value = scan_modes[value]
            else:
                self.value = value

    def __repr__(self):
        return self.parameter + " <- " + str(self.value)
