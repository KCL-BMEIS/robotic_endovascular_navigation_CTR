"""
--------------------------------------------------------------------------------------------------------------------------------
Description: This module is responsible for communicating with the CNC board (Octopus, BIGTREETECH)
and move motors in safe and robust manner for robotic manipulation of endovascular instruments.
    
Authors: Seyedmohammadhadi Sadati (smh_sadati@kcl.ac.uk), Nikola Fischer (nikola.fischer@kcl.ac.uk)
Updated: 2025-01-17

License: MIT License (https://opensource.org/licenses/MIT)

Note: Safe for the use of two axes, namely X and Z in the given configuration.
--------------------------------------------------------------------------------------------------------------------------------
"""
import serial
import re
from time import sleep
import os
import time
import sys 
import os
import numpy as np
# Dynamic introduction of global variables used in this script. Defined in config_json5.py!
board_COM = dist_init = CATH_NUMBERS =  travel_step = feedrate_quick = rot_step = dist_scale = None
x_max_dist_abs_after_reset = motor_code = dist_abs_init = feedrate_default = dist_abs_centered_configuration = None 
input_frequency = translation_limit = feedrate_max = rotation_limit = None
GREEN = CYAN = WHITE = YELLOW = BLUE = RED = PURPLE = ENDC = BOLD = UNDERLINE = CNC_CALIBRATION = None
# Append the parent directory of the current script to sys.path (conig iles are stored here)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from config_json5 import * # config parameters as set in config.json and provided by config_json5.py
import Utilities.input_device_keyboard as keyboard

# FUNCTIONS ---------------------------------------------

# Move directly to home position 0 0 0 0 0 0
def handle_homing(cnc_board_serials, abs_encoder):
    """Handle homing operation."""
    #print(WHITE + "Homing at pre-tensioned position in progress...")
    abs_encoder[:] = dist_init
    for i in range(1):
        #print(WHITE + f"Homing motors on board {cnc_board_serials[i]}")
        command(cnc_board_serials, i, "G90")  # Set to absolute positioning
        command(cnc_board_serials, i, "G1 X0 Y0 Z0 A0 B0 C0 F{}".format(feedrate_quick))
# Defines dist for translation of 2 or 3 instruments
def handle_translate_all_pos(dist):
    """Handle translation of all tubes based on CATH_NUMBERS."""
    if CATH_NUMBERS >= 3:
        dist[0] += travel_step
        dist[1] += travel_step
        dist[2] += travel_step
    else:
        dist[0] += travel_step
        dist[2] += travel_step
def handle_translate_all_neg(dist):
    """Handle translation of all tubes based on CATH_NUMBERS."""
    if CATH_NUMBERS >= 3:
        dist[0] -= travel_step
        dist[1] -= travel_step
        dist[2] -= travel_step
    else:
        dist[0] -= travel_step
        dist[2] -= travel_step
def handle_absolute_translation_xz():
        global flag_absolute_movement, dist_abs
        flag_absolute_movement = True
        dist_abs[0], dist_abs[2]  = keyboard.get_user_input_for_dist_x_z() # returns dist_x and dist_z        
        dist_abs[1] = abs_encoder[1] # conirm existing coordiates for all other variables
        dist_abs[3] = abs_encoder[3]
        dist_abs[4] = abs_encoder[4]
        dist_abs[5] = abs_encoder[5]
        #print(PURPLE + "abs_encoder (in handle_absolute_translation_xz) = " + str(abs_encoder))  
# Defines dist for rotation of 2 or 3 instruments
def handle_rotate_all(dist):
    """Handle rotation of all tubes."""
    dist[3] += rot_step
    dist[4] += rot_step
    dist[5] += rot_step

def key_action_dispatch(key, dist, cnc_board_serials, abs_encoder):
    """Dispatch actions based on key input."""
    global UPDATED_INPUT, flag_absolute_movement, flag_resetted
    if key == ord('n'):
        handle_homing(cnc_board_serials, abs_encoder)
    elif key == ord('0'):
        flag_resetted = False
        resetting_home_x_z(cnc_board_serials)
        UPDATED_INPUT = True
    elif key == ord('1'):
        handle_absolute_translation_xz()
        UPDATED_INPUT = True
    elif key in [ord('w'), ord('s'), ord('t'), ord('g'), ord('i'), ord('k')]:
        keyboard.process_translation(key, dist)
        UPDATED_INPUT = True
    elif key in [ord('a'), ord('d'), ord('f'), ord('h'), ord('j'), ord('l')]:
        keyboard.process_rotation(key, dist)
        UPDATED_INPUT = True
    elif key == ord('v'):
        handle_translate_all_pos(dist)
        UPDATED_INPUT = True
    elif key == ord('c'):
        handle_translate_all_neg(dist)  # Handle reverse translation if needed
        UPDATED_INPUT = True
    elif key == ord('e'):
        handle_rotate_all(dist)
        UPDATED_INPUT = True
    elif key == ord('r'):
        handle_rotate_all(dist)  # Handle reverse rotation if needed
        UPDATED_INPUT = True

# Provides endstop states
def get_endstop_states(ser):
    try:
        # Check if the serial connection is open
        if not ser.is_open:
            print(RED + "Serial port is not open.")
            return [99, 99, 99, 99, 99, 99]  # Return default array if not open

        # Send the M119 command to get endstop status
        ser.write(b'M119\n')
        
        endstop_states = []
        # Wait and read responses
        start_time = time.time()
        while time.time() - start_time < .05:  # Wait up to 1 second for data
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                #print(response)  # For debugging

                # Parse the response for each endstop
                if "x_min:" in response:
                    endstop_states.append(0 if "TRIGGERED" in response else 1)
                elif "y_min:" in response:
                    endstop_states.append(0 if "TRIGGERED" in response else 1)
                elif "z_min:" in response:
                    endstop_states.append(0 if "TRIGGERED" in response else 1)
                elif "a_min:" in response:
                    endstop_states.append(0 if "TRIGGERED" in response else 1)
                elif "b_min:" in response:
                    endstop_states.append(0 if "TRIGGERED" in response else 1)
                elif "c_min:" in response:
                    endstop_states.append(0 if "TRIGGERED" in response else 1)

        # Ensure the array has the right size
        while len(endstop_states) < 6:
            endstop_states.append(99)  # Default to 99 if not present
        return endstop_states

    except Exception as e:
        print(RED + f"Error: {e}")
        return [99, 99, 99, 99, 99, 99]  # Return a default array in case of an error

# calibration of endstops and sotware encoders, defining maximum and minimum distances to travel
def resetting_home_x_z(cnc_board_serials):
    global abs_encoder, flag_resetted, dist
    dist = [0, 0, 0, 0, 0, 0] # local variable in resetting home function
    # references defined relative to absolute encoders
        #reference_coordinates=[ x_ref_max, x_ref_min, y_ref_max, y_ref_min, z_ref_max, z_ref_min ]
        #reference_coordinates: ref_0, ref_max, ref_min, each [x,y,z]
    endstop_states = get_endstop_states(cnc_board_serials[0])
    print(RED + "RESETTING HOME")
    print("Driving X- and Z-axis to Endstop of Z for resetting home ...")
    while endstop_states[2] == 0:
        #if motion_completed(cnc_board_serials,abs_encoder) == True: # endstop z reached and previous motion ended
        if CATH_NUMBERS == 3:
                #dist[0] -= travel_step
                #dist[1] -= travel_step
                #dist[2] -= travel_step
            print(RED + "No resetting function implemented for 3 or more instruments.")
        elif CATH_NUMBERS == 2 or CATH_NUMBERS == 1: # two instruments only, meaning y motor remains untouched by c, v
            dist[0] = -travel_step
            dist[2] = -travel_step
            move_to_relative_position( cnc_board_serials, feedrate_quick, dist)
        while motion_completed(cnc_board_serials) == False: 
            sleep(.1)
            print("Driving Z-axis (and X-axis if applicable) to Endstop of Z. Waiting for motion to be completed ...")
        endstop_states = get_endstop_states(cnc_board_serials[0])
    print("Endstop Z reached.")
    print("Driving X-axis to Endstop of X for resetting home ...")

    while endstop_states[0] == 0:
        dist[0] = -travel_step
        move_to_relative_position( cnc_board_serials, feedrate_quick, dist)
        #print(RED + "Endstopstate [0] (x) = " + str(endstop_states[0]))
        while motion_completed(cnc_board_serials) == False: 
            sleep(.1)
            print("Driving X-axis to Endstop of X. Waiting for motion to be completed ...")
        endstop_states = get_endstop_states(cnc_board_serials[0])

    #sleep(1)
    # Create one travel step safety margin as virtual endstop before mechanical one is activated
    print("Implementing VIRTUAL endstop")
    dist[0] = 2*travel_step # safety margin 2x (effective 1x in relation to z)
    move_to_relative_position( cnc_board_serials, feedrate_quick, dist)
    while motion_completed(cnc_board_serials) == False: 
            sleep(.1)
            #print("Driving X-axis to VIRTUAL endstop. Waiting for motion to be completed ...")
    
    dist[0] = 0 # setting X movement to 0
    dist[2] = travel_step # safety margin 1x or Z movement
    move_to_relative_position( cnc_board_serials, feedrate_quick, dist)
    while motion_completed(cnc_board_serials) == False: 
            sleep(.5)
            print("Driving Z-axis to VIRTUAL endstop. Waiting for motion to be completed ...")
    abs_encoder = [0, 0, 0, 0, 0, 0] # set abs_encoder values to home position
    command(cnc_board_serials, 0, "G92 X0 Y0 Z0 A0 B0 C0") # set current (home) position
    flag_resetted = True
    print(WHITE + "---------------------- Home resetted, New Home = " + str(abs_encoder) + " ----------------------------")

# send/receive gcode commands
def command(cnc_board_serials, board_no, command):
    global CNC_CONNECT        
    response = '' # default pos response
    
    if CNC_CONNECT:
        # check endstop flags
        #endstop_states = get_endstop_states(cnc_board_serials[board_no])
        #print("Endstop-states: " + str(endstop_states))
        #print("boardnumber: " + str(board_no))
        # print( YELLOW + 'board ' + str( board_no) + ' - ' + command ) 
        #  # Flush any remaining data in the buffers
        cnc_board_serials[board_no].flushInput()
        cnc_board_serials[board_no].flushOutput()       
        cnc_board_serials[board_no].write(str.encode(command + '\r\n')) 
        sleep(0.1)

        while True:
            line = cnc_board_serials[board_no].readline()
            if line == b'ok\n':
                break
            # print( BLUE + 'Board response: ' )
            # print( line )
            response = line

    return response

# get current position
def get_pos(cnc_board_serials):
    global CNC_CONNECT
    pos = [ [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ] ] # position vector for all motors on 1st cnc board
    if CNC_CONNECT:
        for i_board in range(0,1):
            response = command(cnc_board_serials, i_board, 'M114 R')
            #print(WHITE + "response in get_pos(cnc_board_serials): " + str(response))
            if isinstance(response, bytes):
                response = response.decode('utf-8')
            pos_list = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", response)
            for i in range(0,6):
                pos[i_board][i] = float( pos_list[i] ) / dist_scale[i] # scale to mm and deg
    #print(pos) # test
    return pos

# pass relative travel destination in reference to the current position - into G-code (former dist2gcode)
def move_to_relative_position( cnc_board_serials, feedrate, dist): # not global dist but local dist!
    global abs_encoder

    command_str = [ 'G1' ]
    endstop_states = get_endstop_states(cnc_board_serials[0]) # hard coded [0]
    #print(WHITE + "Distance to travel (dist, relative movement) = " + str(dist))
    print(WHITE + "Absolute encoder values - START (dist, virtually counted) = " + str(abs_encoder))

    # Endstop of motor_code[i] is deactivated, commands can go through for both adjoining motor axes
    stop_x = endstop_states[0]
    stop_y = endstop_states[1]
    stop_z = endstop_states[2]
   
    if flag_resetted == False: # Resetting not performed yet, motor endstops are master
        for i in range(0,3) : # itterate first 3 linear axes
            add_on_val = 0 # local variable in this function only to allow for motor end switch integration 
            if i == 0:  # X direction logic
                if abs_encoder[0] >= x_max_dist_abs_after_reset:  # Maximum X displacement reached
                    print(RED + "|-------!!!--------- Motion not allowed. Reason: Max displacement in X reached.")
                    if stop_x == 1:  # Endstop X activated
                        add_on_val = 0  # No movement allowed
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: Max displacement in X reached + Endstop X active.")
                    else:  # stop_x == 0, Endstop X not activated
                        if dist[0] > 0:  # Only negative displacement allowed
                            add_on_val = 0  # Forbidden (no further displacement allowed)
                            print(RED + "|-------!!!--------- Motion not allowed. Reason: Max displacement in X reached + Endstop X NOT active. Only negative motion for X allowed.")
                        else:  # dist[0] <= 0, Negative displacement allowed
                            add_on_val = dist[0] * dist_scale[0]
                            abs_encoder[0] += dist[0]
                
                else:  # No maximum displacement 
                    if stop_x == 0:  # Endstop X not activated
                        add_on_val = dist[0] * dist_scale[0]
                        abs_encoder[0] += dist[0]
                    elif stop_x == 1:  # Endstop X activated
                        #print(RED + "abs_encocer[0] = " + str(abs_encoder[0]))
                        #print(RED + "-------------- stop_x is 1 or abs_encoder[0] < x_mindist (..) = " + str(x_min_dist_abs_after_reset))
                        if dist[0] >= 0:  # Positive displacement allowed
                            add_on_val = dist[0] * dist_scale[0]
                            abs_encoder[0] += dist[0]
                        else:  # Negative displacement forbidden
                            add_on_val = 0
                            print(RED + "|-------!!!--------- Motion not allowed. Reason: Endstop X active - only positive motion in X allowed.")
            if i == 2: # Z
                if stop_x == 0 and stop_z == 0: # endstop X and Z not activated, free to roam
                    #print("stop_x == 0 and stop_z == 0")
                    add_on_val = dist[2]*dist_scale[2] 
                    abs_encoder[2] += dist[2] # update abs encoder
                elif stop_x == 1 and stop_z == 1: # endstop X and Z both activated, no movement allowed
                    add_on_val = 0
                    print(RED + "|-------!!!--------- Motion not allowed. Reason: Endstop X and Z are active - No motion in Z allowed.")
                elif stop_x == 1 and stop_z == 0: # only endstop X activated
                    #print("stop_x == 1 and stop_z == 0")
                    if dist[2] > 0: # forbidden 
                        add_on_val = 0 
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: Endstop X is active - only negative motion in Z allowed.")
                    elif dist[2] <= 0: # allowed
                        add_on_val = dist[2]*dist_scale[2] 
                        abs_encoder[2] += dist[2] # update abs encoder
                elif stop_x == 0 and stop_z == 1: # only endstop z activated
                #print("stop_x == 0 and stop_z == 1")
                    if dist[2] < 0: # forbidden 
                        add_on_val = 0 
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: Endstop Z is active - only positive motion in Z allowed.")
                    elif dist[2] >= 0: # allowed
                        add_on_val = dist[2]*dist_scale[2] 
                        abs_encoder[2] += dist[2] # update abs encoder
            # -----------------------------------------------------------------------------------------------------------------   
            # # -----------------------------------------------------------------------------------------------------------------
            # # -----------------------------------------------------------------------------------------------------------------
            # # -----------------------------------------------------------------------------------------------------------------       
            if i == 1: # Y ---------------------------------------------------------------- Experimental MODE - Not tested yet!
                if stop_y == 0 and stop_z == 0: # endstop Z and Y not activated
                    add_on_val = dist[1]*dist_scale[1]
                    abs_encoder[1] += dist[1] # update abs encoder
                elif stop_y == 1 and stop_z == 1: # endstop Z and Y both activated, no movement allowed
                    add_on_val = 0
                    print(RED + "|-------!!!--------- Motion not allowed. Reason: Endstop X and Z are active - No motion in Y allowed.")
                elif stop_y == 1 and stop_z == 0: # only endstop X activated
                    if dist[1] < 0: # forbidden 
                        add_on_val = 0 
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: Endstop Y is active - No negative motion in Y allowed.")
                    elif dist[1] >= 0: # allowed
                        add_on_val = dist[2]*dist_scale[2] 
                        abs_encoder[2] += dist[2] # update abs encoder
                elif stop_y == 0 and stop_z == 1: # only endstop z activated
                    if dist[1] > 0: # forbidden 
                        add_on_val = 0 
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: Endstop Z is active - No positive motion in Y allowed.")
                    elif dist[1] <= 0: # allowed
                        #print(RED + "dist <=0")
                        add_on_val = dist[1]*dist_scale[1] 
                        abs_encoder[1] += dist[1] # update abs encoder
            # Form command line    
            command_str[ 0 ] += ' ' + motor_code[ i ] + str( add_on_val ) # scale to mm and deg 
   
    elif flag_resetted == True: # Resetting has been performed, virtual endstops instad of motor endstops
        for i in range(0,3) : # itterate first 3 linear axes
            add_on_val = 0 # local variable in this function only
            if i == 0:  # X direction logic
                if abs_encoder[0] >= x_max_dist_abs_after_reset:  # Maximum X displacement reached
                    print(RED + "|-------!!!--------- Motion not allowed. Reason: Max displacement in X reached.")
                    if abs_encoder[0] <= abs_encoder[2]:  # Virtual Endstop X activated, minimum distance between motors X and Z
                        add_on_val = 0  # No movement allowed
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: Max displacement in X reached + (Virtual) Endstop X active.")
                    else:  # stop_x == 0, Virtual Endstop X not activated
                        if dist[0] > 0:  # Only negative displacement allowed
                            add_on_val = 0  # Forbidden (no further displacement allowed)
                            print(RED + "|-------!!!--------- Motion not allowed. Reason: Max displacement in X reached + (Virtual) Endstop X NOT active. Only negative motion for X allowed.")
                        else:  # dist[0] <= 0, Negative displacement allowed
                            add_on_val = dist[0] * dist_scale[0]
                            abs_encoder[0] += dist[0]
                
                else:  # No maximum displacement 
                    if abs_encoder[0] > abs_encoder[2] and abs_encoder[0] > 0:  # Endstop X not activated
                        add_on_val = dist[0] * dist_scale[0]
                        abs_encoder[0] += dist[0]

                    elif abs_encoder[0] < abs_encoder[2] or abs_encoder[0] == 0:  # Endstop X activated
                        #print(RED + "abs_encocer[0] = " + str(abs_encoder[0]))
                        #print(RED + "-------------- stop_x is 1 or abs_encoder[0] < x_mindist (..) = " + str(x_min_dist_abs_after_reset))
                        if dist[0] >= 0:  # Positive displacement allowed
                            add_on_val = dist[0] * dist_scale[0]
                            abs_encoder[0] += dist[0]
                        else:  # Negative displacement forbidden
                            add_on_val = 0
                            print(RED + "|-------!!!--------- Motion not allowed. Reason: (Virtual) Endstop X active - only positive motion in X allowed.")                            

                    elif abs_encoder[0] == abs_encoder[2]: # minimum distance between both X and Z reached
                        if abs_encoder[0] >= 0:
                            if dist[0] >= 0:  # Positive displacement allowed
                                add_on_val = dist[0] * dist_scale[0]
                                abs_encoder[0] += dist[0]
                            else:  # Negative displacement forbidden
                                add_on_val = 0
                                print(RED + "|-------!!!--------- Motion not allowed. Reason: X and Z are too close. Negative motion of X forbidden.")                                                   

                        else: # abs_encoders <0
                            add_on_val = 0    
                            print(RED + "|-------!!!--------- Motion not allowed. Reason: X and Z are in the negative field, which is forbidden. Remark: This case should not be happening when caliration has been performed properly.")                                                   

            if i == 2: # Z
                if abs_encoder[0] > abs_encoder[2] and abs_encoder[2] > 0: # sae distance X-Z, Z>0
                    #stop_x == 0 and stop_z == 0: # endstop X and Z not activated, free to roam
                    #print("stop_x == 0 and stop_z == 0")
                    add_on_val = dist[2]*dist_scale[2] 
                    abs_encoder[2] += dist[2] # update abs encoder
                elif abs_encoder[0] <= abs_encoder[2] and abs_encoder[2] <= 0: #stop_x == 1 and stop_z == 1: # endstop X and Z both activated, no movement allowed
                    #print("stop_x == 1 and stop_z == 1")
                    add_on_val = 0
                    print(RED + "|-------!!!--------- Motion not allowed. Reason: (Virtual) Endstop X and Z are active - No motion in Z allowed.")                
                elif abs_encoder[0] <= abs_encoder[2] and stop_z > 0: # only endstop X activated
                    #print("stop_x == 1 and stop_z == 0")
                    if dist[2] > 0: # forbidden 
                        add_on_val = 0 
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: (Virtual) Endstop X is active - only negative motion in Z allowed.")
                    elif dist[2] <= 0: # allowed
                        add_on_val = dist[2]*dist_scale[2] 
                        abs_encoder[2] += dist[2] # update abs encoder
                elif abs_encoder[0] > abs_encoder[2] and abs_encoder[2] <= 0: #stop_x == 0 and stop_z == 1: # only endstop z activated
                #print("stop_x == 0 and stop_z == 1")
                    if dist[2] < 0: # forbidden 
                        add_on_val = 0 
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: (Virtual) Endstop Z is active - only positive motion in Z allowed.")
                    elif dist[2] >= 0: # allowed
                        add_on_val = dist[2]*dist_scale[2] 
                        abs_encoder[2] += dist[2] # update abs encoder

                elif abs_encoder[0] == abs_encoder[2]: # minimum distance between both X and Z reached
                    if abs_encoder[2] > 0:
                        if dist[2] <= 0 or dist[0] > 0:  # Negative displacement allowed if negative buffer present OR X moves in parallel in pos direction
                            add_on_val = dist[2] * dist_scale[2]
                            abs_encoder[2] += dist[2]
                        else:  # Positive displacement forbidden
                            add_on_val = 0
                            print(RED + "|-------!!!--------- Motion not allowed. Reason: X and Z are too close. Positive motion of Z forbidden.")                                                   

                    else: # abs_encoders <0
                        add_on_val = 0
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: X and Z are in the negative field, which is forbidden. Remark: This case should not be happening when caliration has been performed properly.")                                                   
                           
            # -----------------------------------------------------------------------------------------------------------------   
            # # -----------------------------------------------------------------------------------------------------------------
            # # -----------------------------------------------------------------------------------------------------------------
            # # -----------------------------------------------------------------------------------------------------------------       
            if i == 1: # Y -- only implemented with hardware motor stops. Virtual Endstops not implmented yet.
                if stop_y == 0 and stop_z == 0: # endstop Z and Y not activated
                    add_on_val = dist[1]*dist_scale[1]
                    abs_encoder[1] += dist[1] # update abs encoder
                elif stop_y == 1 and stop_z == 1: # endstop Z and Y both activated, no movement allowed
                    add_on_val = 0
                    print(RED + "|-------!!!--------- Motion not allowed. Reason: (Virtual) Endstop X and Z are active - No motion in Y allowed.")
                elif stop_y == 1 and stop_z == 0: # only endstop X activated
                    if dist[1] < 0: # forbidden 
                        add_on_val = 0 
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: (Virtual) Endstop Y is active - No negative motion in Y allowed.")
                    elif dist[1] >= 0: # allowed
                        add_on_val = dist[2]*dist_scale[2] 
                        abs_encoder[2] += dist[2] # update abs encoder
                elif stop_y == 0 and stop_z == 1: # only endstop z activated
                    if dist[1] > 0: # forbidden 
                        add_on_val = 0 
                        print(RED + "|-------!!!--------- Motion not allowed. Reason: (Virtual) Endstop Z is active - No positive motion in Y allowed.")

                    elif dist[1] <= 0: # allowed
                        #print(RED + "dist <=0")
                        add_on_val = dist[1]*dist_scale[1] 
                        abs_encoder[1] += dist[1] # update abs encoder
            # Form command line    
            command_str[ 0 ] += ' ' + motor_code[ i ] + str( add_on_val ) # scale to mm and deg 

    for i in range(3,6) : # itterate tendon#
        #print(RED + BOLD + "i in range 3-6 ---> " + str(i))
        # command_str[ 0 ] += ' ' + motor_code[ i ] + str( int( dist[i]*dist_scale[i] ) ) # scale to mm and deg
        command_str[ 0 ] += ' ' + motor_code[ i ] + str( ( dist[i]*dist_scale[i] ) ) # scale to mm and deg
        abs_encoder[i] += dist[i] # update remaining values for abs encoder  
    #print(GREEN + "Updated all absolute encoder values - Now starting to execute command.")

    for i in range(0,1) : # call the cnc drivers
        command_str[i] += ( ' F' + str(feedrate) ) # add velocity info
        print( BLUE + board_COM + ': ' + command_str[i] )
        command( cnc_board_serials, i , "G91" ) # set to relative positioning
        command( cnc_board_serials, i , command_str[i] )
        #print( RED + "CNC drivers called" )
    #print(GREEN + "Absolute encoder values - END (dist, virtually counted) = " + str(abs_encoder))


# pass absolute travel destination in reference to the CNC origin - into G-code
def move_to_abs_position(cnc_board_serials, dist_abs_local, feedrate):
    board_no = 0
    global abs_encoder, flag_absolute_movement
    # Initialize the G-code command string with G90 (absolute positioning mode)
    #gcode_command = "G91\n"  # Absolute positioning
    gcode_command = "G90\n"  # Absolute positioning
    gcode_command += "G1"    # Controlled move command

    # Axis labels (X, Y, Z, A, B, C)
    #axis_labels = ['X', 'Y', 'Z', 'A', 'B', 'C']

    # Build the G-code command based on the dist list
    gcode_command += " X" + str( dist_abs_local[0] * dist_scale[0] ) + " Z" + str(( dist_abs_local[2] * dist_scale[2] ) ) # scale to mm and deg # G1 X50 Z20 F100

    # Add the feedrate to the command
    gcode_command += f" F{feedrate}\n"

    # Send the G-code command to the CNC board via the existing command function
    response = command(cnc_board_serials, board_no, gcode_command)
    print(f"Sent G-code: {gcode_command.strip()}")
    while motion_completed(cnc_board_serials) == False: 
            sleep(.5)
            print("Driving X-axis  ...")

    abs_encoder = [float(dist_abs_local[0]), 0.0, float(dist_abs_local[2]), 0.0, 0.0, 0.0] 
    flag_absolute_movement = False # Reset flag
    #print(RED + "abs_encoder updated at the end of def move_to_abs_position to: " + str(abs_encoder))

# Trigger Movement based on keyboard inputs
def execute_movement(cnc_board_serials):
    global flag_absolute_movement, dist_abs, dist, abs_encoder
    """Send gcode commands to execute relative movement."""
    if flag_absolute_movement:
        move_to_abs_position(cnc_board_serials, dist_abs, feedrate_quick) # input argument dist abs
        #abs_encoder = [float(dist_abs[0]), 0.0, float(dist_abs[2]), 0.0, 0.0, 0.0] 
        dist_abs[:] = dist_abs_init  # Reset distance array after movement
    else: # relative movement
        move_to_relative_position(cnc_board_serials, feedrate_default, dist)
        dist[:] = dist_init  # Reset distance array after movement

# check if the motion is finished
def motion_completed(cnc_board_serials):
    pos = get_pos(cnc_board_serials) # get current pos
    for i in range(0,6):
        if abs( pos[0][i] - abs_encoder[i] ) < 0.1 :
            MOTOIN_STATE = True # remain True only if all axes are reached
        else:
            MOTOIN_STATE = False # returns false and terminate the code with first not reached axis
            break    
    return MOTOIN_STATE

# terminate cnc board
def terminate_cnc_board(cnc_board_serials):
    for i in range(0,1) : # move all axis to the home position
        sleep(2)
        print(WHITE +  'Closing down serial port ' + board_COM)
    if CNC_CONNECT:
            cnc_board_serials[i].close()

# initialize cnc board
def initialize_CNC ():
    global key, key_cash, UPDATED_INPUT, dist, dist_abs
    dist = dist_init[:] 
    dist_abs = dist_abs_init[:] # absolute distance
    cnc_board_serials = [0]
    if CNC_CONNECT:
        for i in range( 0 , 1 ) : # start the boards
            print(WHITE + 'Starting board on port ' + board_COM + '...')
            cnc_board_serials[i] = serial.Serial(board_COM, 250000)
            #sleep(2)
            command(cnc_board_serials, i, "G92 X0 Y0 Z0 A0 B0 C0") # set current (home) position
            command(cnc_board_serials, i, "M18 S5") # disable motors after 5 sec of inactivity
            print("CNC connected to board.------------------------------------------")
        ## program main loop (starting with hardware reset)
    if CNC_CALIBRATION:
        resetting_home_x_z(cnc_board_serials) # calibrating home position
        print(GREEN + "Intitialization finished.")

        print(WHITE + "Move towards centered starting configuration ... ")
        move_to_abs_position(cnc_board_serials, dist_abs_centered_configuration, feedrate_quick ) # Move to a balanced middle position to start from
        dist_abs[:] = dist_abs_init  # Reset distance array after movement

    key = 1 # initialize key value to show the instructions in the first run
    key_cash = 1 # cashed key value to execute the latest input when the system is still executing the latest ommandccw
    UPDATED_INPUT = 0 

    return cnc_board_serials 

# a little simulation to create Harrys agent output as only "1"
def agent_output_simulation():
    #sleep(1)
    output_vector = [1,1,1,1] 
    return output_vector

def agent_controlstep_with_const_max_feedrate(agent_input, cnc_board_serials): # further parameters are set in config
   # Define the time interval for each control step
    control_step_time = 1 / input_frequency
    
    # Start timing the control step
    start_time = time.time()
    
    # Calculate distance based on max feedrate and frequency
    distance_input_mm_deg = np.array([0, 0, 0, 0])
    distance_input_mm_deg[0] = (((agent_input[0] * translation_limit)**2 * 60) / feedrate_max) / input_frequency
    distance_input_mm_deg[1] = (((agent_input[1] * rotation_limit)**2 * 60) / feedrate_max) / input_frequency
    distance_input_mm_deg[2] = (((agent_input[2] * translation_limit)**2 * 60) / feedrate_max) / input_frequency
    distance_input_mm_deg[3] = (((agent_input[3] * rotation_limit)**2 * 60) / feedrate_max) / input_frequency
    
    # Prepare movement command
    dist_with_const_max_feedrate = [
        distance_input_mm_deg[0], 0,
        distance_input_mm_deg[2], distance_input_mm_deg[1],
        0, distance_input_mm_deg[3]
    ]
    
    # Execute the movement
    move_to_relative_position(cnc_board_serials, feedrate_max, dist_with_const_max_feedrate)
    
    # Wait for the motion to complete
    while not motion_completed(cnc_board_serials):
        time.sleep(0.05)
    
    # Calculate elapsed time
    elapsed_time = time.time() - start_time
    
    # Print a warning if the function takes too long
    if elapsed_time > control_step_time:
        print(RED + f"Warning: Control step exceeded allocated time. Took {elapsed_time:.3f}s, expected {control_step_time:.3f}s")
    
    # Ensure the total control step duration matches control_step_time
    remaining_time = control_step_time - elapsed_time
    if remaining_time > 0:
        time.sleep(remaining_time)


# MAIN --------------------------------------------------------------------
def robot_control_main():
    """Main control loop."""
    global key_cash, key, UPDATED_INPUT
    
    cnc_board_serials = initialize_CNC()

    while True:
        #print_instructions()
        if key_cash != -1:
            keyboard.print_instructions()
        key_cash = keyboard.keyboard_input()  # Non-blocking keyboard input

        if key_cash != -1:
            key = key_cash

        if key == ord('q'):
            break  # Quit the loop

        if motion_completed(cnc_board_serials) or not CNC_CONNECT:
            UPDATED_INPUT = 0
            dist[:] = dist_init
            key_action_dispatch(key, dist, cnc_board_serials, abs_encoder)
            key = -1  # Reset key

            if UPDATED_INPUT:
                execute_movement(cnc_board_serials)

# Run the main loop only if this script is executed directly
if __name__ == "__main__":
    robot_control_main()