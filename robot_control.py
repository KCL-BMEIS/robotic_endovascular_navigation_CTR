from urllib import response
import numpy as np 
import matplotlib.pyplot as plt
import serial
from datetime import datetime
# import matlab.engine
import re

from time import perf_counter, sleep
import os
import math

# import keyboard
# from inputs import get_key
import msvcrt


# colors
WHITE = '\033[37m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'
RED = '\033[31m'
CYAN = '\033[36m'
PURPLE = '\033[35m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

## parameters
# catheter parameters
CATH_NUMBERS = 1 # the number of catheters present in the video
l0_cath = [0, 0, 0] # inserted cath points

# controller parameters
image_frequency = 7.5 # controller frequency
# control2cnc = [[2, 5], [1, 4], [0, 3]] # [[translation1, rotation1]_innermost, ..., [translationN, rotationN]_outermost], trans and rot correspond to cnc board actuator number [XYZ_outer2inner rot_outer2inner]
control2cnc = [[1, 4], [0, 3], [2, 5]] # Lennart's single device tests
target_converg_range = 0.1 # target convergance radius to stop the controller

# cnc controller parameters:
CNC_CONNECT = True # should connect to the board or not
board_com = [ 'COM3' ] # port addresses for layer (top) 1 - 4 (bottom), my laptop
motor_code = [ 'X' , 'Y' , 'Z' , 'A' , 'B' , 'C' ] # motor gcodes, XYZ_outer2inner: translation, ABC_outer2inner: rotation, XA: for outermost
feedrate = 15000 # axis feedrate in mm/min (the commands are in mm)
dist_scale = [10, 10, 10, 1/9, 1/9, 1/9] # motor step scale factor to get mm and deg units
travel_step = 5 # mm, travel distance stepkk
rot_step = 30 # deg, rotation angle step


## functions
# keyboard input
def keyboard_input():
    if msvcrt.kbhit():
        keyboard_input = ord( msvcrt.getch() )
        while(msvcrt.kbhit() ): # empty the keyboard buffer due to rapid key press
            keyboard_input = ord( msvcrt.getch() )
    else:
        keyboard_input = -1
    # print(keyboard_input) # test
    return keyboard_input
 
# send/receive gcode commands
def command(board_serials, board_no, command):
    global CNC_CONNECT        
    response = '' # default pos response
    
    if CNC_CONNECT:
        # print( YELLOW + 'board ' + str( board_no) + ' - ' + command )        
        board_serials[board_no].write(str.encode(command + '\r\n')) 
        # sleep(0.1)

        while True:
            line = board_serials[board_no].readline()
            if line == b'ok\n':
                break
            # print( BLUE + 'Board response: ' )
            # print( line )
            response = line

    return response

# get current position
def get_pos(board_serials):
    global CNC_CONNECT
    pos = [ [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] ] # position vector for all motors on 1st cnc board
    if CNC_CONNECT:
        for i_board in range(0,1):
            response = command(board_serials, i_board, 'M114 R')
            # command(board_serials, i_board, 'M400') # wait until the motion is finished
            pos_list = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", response.decode('utf-8')) # find float numnbers in the line
            # print(pos_list)
            for i in range(0,6):
                pos[i_board][i] = float( pos_list[i] ) / dist_scale[i] # scale to mm and deg
    # print(pos) # test
    return pos

# pass line length values
def dist2gcode( board_serials , dist ):
    global feedrate
    command_str = [ 'G1' ]
    for i in range(0,6) : # itterate tendon#
        # command_str[ 0 ] += ' ' + motor_code[ i ] + str( int( dist[i]*dist_scale[i] ) ) # scale to mm and deg
        command_str[ 0 ] += ' ' + motor_code[ i ] + str( ( dist[i]*dist_scale[i] ) ) # scale to mm and deg
        
    for i in range(0,1) : # call the cnc drivers
        command_str[i] += ( ' F' + str(feedrate) ) # add velocity info
        print( BLUE + board_com[i] + ': ' + command_str[i] )
        command( board_serials, i , "G91" ) # set to relative positioning
        command( board_serials, i , command_str[i] )
    
# check if the motion is finished
def motion_completed(board_serials,abs_encoder):
    pos = get_pos(board_serials) # get current pos
    # print(WHITE + str(pos)) # test
    # print(WHITE + str(abs_encoder)) # test
    
    for i in range(0,6):
        if abs( pos[0][i] - abs_encoder[i] ) < 0.1 :
            MOTOIN_STATE = True # remain True only if all axes are reached
        else:
            MOTOIN_STATE = False # returns false and terminate the code with first not reached axis
            break    
    return MOTOIN_STATE

# stop the controller if the target is reached
def target_reached(target_coords,tracking):
    TARGET_REACHED = False
    error = np.sqrt( np.power(target_coords[0][0]-tracking[0][0],2) + np.power(target_coords[0][1]-tracking[0][1],2) )
    if error < target_converg_range:
        TARGET_REACHED = True
    return TARGET_REACHED

## initialize cnc board
board_serials = [0]
if CNC_CONNECT:
    for i in range( 0 , 1 ) : # start the boards
        print(WHITE + 'Starting board on port ' + board_com[i] + '...')
        board_serials[i] = serial.Serial(board_com[i], 250000)
        sleep(2)
        command(board_serials, i, "G92 X0 Y0 Z0 A0 B0 C0") # set current (home) position


# ## initialize matlab
# eng = matlab.engine.start_matlab()
# eng.addpath(r'C:\Hadi\Postdoc\2_KCL\5. Spinoff\1. STREAM - SCS + Kaspar\3. Model\0. CC',nargout=0)


## program main loop
dist = [0, 0, 0, 0, 0, 0] # travel distance/rotation reset
abs_encoder = [0, 0, 0, 0, 0, 0]

UPDATED_INPUT = 0 # is input updated? 1 to calculate the default pose
t_prev_cmd = 0 # time previous command sent

key = 1 # initialize key value to show the instructions in the first run
key_cash = 1 # cashed key value to execute the latest input when the system is still executing the latest ommand
while(True):            

    if key_cash != -1: # only shows this after a key is pressed to avoid repeatadly printing this
        # Instructions:      
        print(GREEN + 'Press:')
        # print(CYAN + '- ''h or main-right or start'' for homing at the system start state,')
        # print(CYAN + '- ''f or main left'' for homing at the fixe end or limit switch locations,')
        # print(CYAN + '- ''c or L2'' for manual gcode command,')
        print(CYAN + '- ''[w,s,a,d] or left axis, [i,k,l,j] or right axis, [t,g,h,f] or main directions'' for moving the outermost to innermost tube (up-down for translation, left-right for trotation),')
        print(CYAN + '- ''[c,v] or [LT, RT]'' for translating all tubes,')
        print(CYAN + '- ''[e,r] or [LB, RB]'' for rotating all tubes,')
        print(CYAN + '- ''q or Back'' to quit the program!')

    # keyboard inputs    
    # key_cash = cv2.waitKey(33) # does not run in parallel, misses inputs
    # key_cash = ord( keyboard.read_key() ) # parallel run but blocking the code for input
    key_cash = keyboard_input() # non blocking keyboard input
    if key_cash != -1: # new cashed command is arrived
        key = key_cash

    if key == ord('q'): # quit the program
        break
                
    # wait until previous input motion is finished
    if motion_completed(board_serials,abs_encoder) or not CNC_CONNECT: # Previous motion is completed or cnc board is disconnected
        sleep(0.1)

        dist = [0, 0, 0, 0, 0, 0] # travel distance/rotation reset
        UPDATED_INPUT = 0

        if key != -1:
            # print(WHITE + 'key pressed: ') # keys: w119 s115 d100 a97 i105 k107 l108 j106 o111 u117
            # print(key) # test keys: w119 s115 d100 a97 i105 k107 l108 j106 o111 u117
            
            if key in { ord('n') }: # homing at pre-tensioned locations
                print(WHITE + 'homing at pre-tensioned position in progress...')  
                abs_encoder = [0, 0, 0, 0, 0, 0] # reset the absolute encoder         
                for i in range(0,1) : # move all axis to the home position
                    print(WHITE +  'Homing motors on board ' + board_com[i] )
                    command( board_serials, i , "G90") # set to absolute positioning
                    command( board_serials, i , 'G1 X0 Y0 Z0 A0 B0 C0' ) # home at the pre-tensioned state
        
            elif key == ord('w'):
                dist[0] += travel_step
                UPDATED_INPUT = 1
            elif key == ord('s'):
                dist[0] -= travel_step
                UPDATED_INPUT = 1
                
            elif key == ord('t'):
                dist[1] += travel_step
                UPDATED_INPUT = 1
            elif key == ord('g'):
                dist[1] -= travel_step
                UPDATED_INPUT = 1

            elif key == ord('i'):
                dist[2] += travel_step
                UPDATED_INPUT = 1
            elif key == ord('k'):
                dist[2] -= travel_step
                UPDATED_INPUT = 1

            elif key == ord('a'):
                dist[3] += rot_step
                UPDATED_INPUT = 1
            elif key == ord('d'):
                dist[3] -= rot_step
                UPDATED_INPUT = 1

            elif key == ord('f'):
                dist[4] += rot_step
                UPDATED_INPUT = 1
            elif key == ord('h'):
                dist[4] -= rot_step
                UPDATED_INPUT = 1
                
            elif key == ord('j'):
                dist[5] += rot_step
                UPDATED_INPUT = 1
            elif key == ord('l'):
                dist[5] -= rot_step
                UPDATED_INPUT = 1
                
            elif key == ord('v'):
                dist[0] += travel_step
                dist[1] += travel_step
                dist[2] += travel_step
                UPDATED_INPUT = 1
            elif key == ord('c'):
                dist[0] -= travel_step
                dist[1] -= travel_step
                dist[2] -= travel_step
                UPDATED_INPUT = 1
                
            elif key == ord('e'):
                dist[3] += rot_step
                dist[4] += rot_step
                dist[5] += rot_step
                UPDATED_INPUT = 1
            elif key == ord('r'):
                dist[3] -= rot_step
                dist[4] -= rot_step
                dist[5] -= rot_step
                UPDATED_INPUT = 1
            
        key = -1 # acknowledge that the last input is executed
        if UPDATED_INPUT:            
            print(WHITE +  'dist = ')
            print(dist)
            dist2gcode( board_serials , dist ) # relative motion
            for i in range(0,6): # record motors' absolute position
                abs_encoder[i] += dist[i]
            

## treminate cnc board
for i in range(0,1) : # move all axis to the home position
    # print(WHITE +  'Homing motors on board ' + board_com[i] )
    # command( board_serials, i , "G90") # set to absolute positioning
    # command( board_serials, i , 'G1 X0 Y0 Z0 A0 B0 C0' ) home system upon exit
    sleep(2)
    print(WHITE +  'Closing down serial port ' + board_com[i])
if CNC_CONNECT:
        board_serials[i].close()