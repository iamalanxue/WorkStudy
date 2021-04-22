"""
Create Date: 2021.01.13
Design Name: OCT DAQ Controller
Created By : Wei Hong Yeo. Northwestern University, FOIL.
Modified By: Alan Xue. Northwestern University.

Description:
PC script for OCT DAQ Control on Metro M4 board.

Revisions  :
Revision v0.1 - Initial file created

Additional Comments:

"""

from communicate import *
instructionBytes = {
    "STATE_WAIT_READ_BYTE": 0b000, 
    "STATE_PREVIEW_SCAN": 0b001, 
    "STATE_ACQUISITION_SCAN": 0b010,
    "STATE_SEND_SETTING": 0b10, 
    "STATE_RECV_UPDATE_SETTING": 0b11 
}

g = GalvoCtrl()


s1 = Settings("acq_rate", 10)
s2 = Settings("acq_rate")
s3 = Settings("scan_mode", "circular_speckle rdn")

g.write_settings(s1)
g.read_settings(s2)
g.read_settings(s3)

g.write_settings(s3)







instruction = False 
while not instruction:
    #currently arduino in idle mode waiting for instruction byte 
    print('ARDUINO STATES INTERFACE')
    #display the arduino states options 
    print('a: WAIT_READ_BYTE \t b: PREVIEW_SCAN'
          '\nc: ACQUISITION_SCAN \td: SEND_SETTING'
          '\ne: RECV_UPDATE_SETTING \tq: QUIT_CLIENT')
    #read the user's choice 
    selection = input('\n ENTER COMMAND: ')
    ser.write((selection + '\n').encode())
    # selection_endline = selection + '\n'
    if(selection == 'a'):
        g.sendInstructionByte(instructionBytes["STATE_WAIT_READ_BYTE"] << 5)
    elif(selection == 'b'):
        g.sendInstructionByte(instructionBytes["STATE_PREVIEW_SCAN"] << 5)
    elif(selection == 'c'):
        g.sendInstructionByte(instructionBytes["STATE_ACQUISITION_SCAN"] << 5)
    elif(selection == 'd'):
        g.sendInstructionByte(instructionBytes["STATE_SEND_SETTING"] << 6)
    elif(selection == 'e'):
        g.sendInstructionByte(instructionBytes["STATE_RECV_UPDATE_SETTING"] << 6)
    elif(selection == 'q'):
        print('Exiting client')
        has_quit = True #exit the client 
        #be sure to close the port 
        g.serial.close()




has_quit = False
#menu loop 
while not has_quit:
    print('DAQ SCANNING MODE INTERFACE')
    #display the menu options; this list will grow 
    print('a: Perform Raster Acquisition \tb: Perform Raster Interleaved'
          '\nc: Perform Raster Speckle Rdn \td: Perform Circular Acquisition'
          '\ne: Perform Circular Interleaved \tf: Perform Circular Speckle Rdn'
          '\nq: Quit Client')
    #read the user's choice 
    selection = input('\nENTER COMMAND: ')

    selection_endline = selection+'\n'
    ser.write(selection_endline.encode()) #.encode turns the string into a char array 

    #take the apporiate action 
    #PYTHON HAS NO switch() so use if elif statements instead 
    if(selection == 'a'):
        pass
    elif(selection == 'b'):
        pass
    elif(selection == 'c'):
        pass
    elif(selection == 'd'):
        pass
    elif(selection == 'f'):
        pass 
    elif(selection == 'q'):
        print('Exiting client')
        has_quit = True #exit the client 
        #be sure to close the port 
        g.serial.close()