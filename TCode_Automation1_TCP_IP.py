import math
import os
import numpy as np
from math import sqrt
import time
import socket
from threading import Thread
from sympy import symbols, solve

from datetime import datetime
from datetime import date
import serial
from pymodbus.client import ModbusTcpClient


'''Functions used with auxiliary connections'''
def openport(port):
    ## opens serial ports (typically used with Nordson pressure box communication)
    # IMPORTS
    baudrate = 115200
    bytesize = 8
    timeout = 2

    return serial.Serial("COM" + str(port), baudrate=baudrate, bytesize=bytesize, timeout=timeout,stopbits=serial.STOPBITS_ONE)
def WAGO_ValveCommands(valve_number, action):
    ## Communicates to wago valves
    client = ModbusTcpClient('192.168.1.1')
    client.write_coil(valve_number, action)  # 1 = port number: 0...7
    client.close()

now = datetime.now()  # datetime object containing current date and time
current_date = date.today()
dt_string = now.strftime("%y/%m/%d")  # y/m/d

'''
Example code for "Time Code for Multifunctional 3D Printhead Controls"
This code is formatted to work with Automation1 (Aerotech) through a TCP/IP network connection
Input G-Code should be in relative (G91)
Current aux compatibility: Nordson Ultimus V and WAGO solenoid valve control
'''

############################ INPUTS #############################################
feed = 10# feedrate mm/s
accel = 1000  # mm/s^2
decel = -accel  # mm/s^2

offset = 1.5 # (units in mm). Amount of time needed to account for delay of aux eecution. Use a negative number to increase length of time/material being on
offset_initial = offset  # ignore
start_delay = 0  # ignore

path = '' #path to file/folder
# file_or_folder this can be a single file or a folder (if performing parallel printing)
file_or_folder =  'Example_30mm_5x5Checkerboard_GCODE.txt' #'Example_30mm_3x3Checkerboard_GCODE.txt' # example for single file
# file_or_folder = 'ExampleGCodes_ParallelPrinting'  # example for parallel printing using a folder

intro_gcode = 'SPropst_automation1_DGCintro.txt'
Z_var = ["D"]
z_height = 0.8 # layer height
Z_start = -100 + z_height # absolution location of z-zero + z_height

home = False  # do you want to home it? (True = yes)

# Open the ports for the pressure box
press_com1 = 8
press_com2 = 9
press_com3 = 10

resync_type ='direction-based'  # OPTIONS: 'direction-based', False

resync_test_time_adjust = True
resync_number = 1  # number of directions changes between resyncing (1 = resync at every direction change; other resync values have not been tested)

PING_delay = 0.0019946302958353313  # ping delay between the G-Code motion controller and python

# Opens appropriate serial ports used for pressure box connectivity
# serialPort1 = openport(press_com1)
# serialPort2 = openport(press_com2)
# serialPort3 = openport(press_com3)


############################ Print feed and accel used #############################################
print("Feedrate = ", feed, "mm/s")
print("Acceleration = ", accel, "mm/s^2", "\nDeceleration = ", decel, "mm/s^2")

start_time = time.time()

################################G-code to time functions#########################################
'''Supplemental functions'''
def find_G(gcode_dict):  # s = string to search, ch = character to find
    for elem in enumerate(gcode_dict):
        if "G" in elem[1]:
            return elem[1]  # finds location of character, strips it, and outputs numerical number
def find_distances(gcode_dict, ch):  # s = string to search, ch = character to find
    # ch = character to find
    result = 0, 0
    for elem in enumerate(gcode_dict):
        if ch in elem[1]:
            result = float(
                elem[1].strip(ch)), ch  # finds location of character, strips it, and outputs numerical number
            break

    return result
## pythag thm
def pythag(x, y, z):
    return (sqrt(abs(float(x)) ** 2 + abs(float(y)) ** 2 + abs(float(z)) ** 2))

## finds arc_angle
def find_theta(X, Y, I, J):  # finds angle between intersecting lines
    from math import atan2
    a = math.atan2(-J, -I)
    b = math.atan2(Y - J, X - I)
    theta = b - a
    return theta

# finds accleration length (distance and time)
def accel_length(v_0, v_f, accel):
    from sympy import symbols, solve
    # v_0 = starting velocity
    # max_v = desired feedrate #mm/s
    # accel = acceleration/ramprate
    x = symbols('x')
    t = symbols('t')
    v = symbols('v')
    find_x = v_0 ** 2 + 2 * accel * x - v_f ** 2  # finds distance to steady state velocity
    sol_x = solve(find_x)

    for i in range(len(sol_x)):
        try:
            sol_x[i] >= 0
            x_steady = sol_x[i]
        except TypeError:
            print("error in value of x; it may be imaginary")

    find_t = v_0 + accel * t - v_f  # finds time to steady state velocity
    sol_t = solve(find_t)
    t_steady = sol_t[0]

    return x_steady, t_steady

# finds time to travel each distance
def findt(v_0, accel, x):
    # v_0 = starting velocity
    # accel = acceleration/ramprate
    # x = distance traveled
    t = symbols('t')
    find_t = v_0 * t + 0.5 * (accel) * t ** 2 - x  # finds time
    sol_t = solve(find_t)
    for i in range(len(sol_t)):
        try:
            sol_t[i] >= 0
            t = sol_t[i]
        except TypeError:
            check = 0
            print("error when calcuating time; might be getting an imaginary number.")
    return t

# finds the final velocity for each distance travelled
def findv(v_0, accel, x):
    from sympy import symbols, solve
    # v_0 = starting velocity
    # accel = acceleration/ramprate
    # x = distance traveled

    try:
        v = sqrt(v_0 ** 2 + 2 * accel * x)
    except TypeError:
        print("error when calcuating velocity; might be getting an imaginary number.")

    return v

# finds the time it takes to go from an initial velocity to a final velocity
def findt_using_v(v_0, v_f, accel):
    t = (v_f - v_0) / accel
    return float(t)

'''Main functions'''
## open and read in gcode txt file into a list - remove comments, spaces, random characters
def open_gcode(gcode_txt):
    gcode_list = []
    with open(gcode_txt, "r") as gcode:
        for myline in gcode:  # For each elem in the file,
            gcode_list.append(myline.strip('\n'))
        gcode_list = [x for x in gcode_list if x != ""]  # removes spaces

        gcode_list = [x for x in gcode_list if ";--" not in x]  # removes comments
        gcode_list = [x for x in gcode_list if "---" not in x]  # removes comments
        gcode.close()
        return gcode_list

## splits up gcode into directions, distances, G command type (G1, G2, G3, etc)
## create a gcode,  distance, and direction dictionary
### Parses gcode
def parse_gcode(gcode_list):
    ## Finds G-command (i.e., G1, G2, G3)
    G_command_dict = {}
    gcode_dict = {}

    X_dist_dict = {}
    Y_dist_dict = {}
    Z_dist_dict = {}
    I_dist_dict = {}
    J_dist_dict = {}
    All_dist_dict = {}
    All_var_dict = {}
    distance_commands_dict = {}
    slope_dict = {}
    count = 0

    for i in range(len(gcode_list)):
        if '{aux_command}' in gcode_list[i] or 'serialPort' in gcode_list[i]:
            command = gcode_list[i].replace('{aux_command}', '')
            distance_commands_dict[i] = str(command)
        else:
            gcode_dict[i] = gcode_list[i].split(" ")

            find_G_result = find_G(gcode_dict[i])

            ## Stores distance values for each command
            find_X = find_distances(gcode_dict[i], "X")
            find_Y = find_distances(gcode_dict[i], "Y")
            find_Z = find_distances(gcode_dict[i], "Z")
            find_I = find_distances(gcode_dict[i], "I")
            find_J = find_distances(gcode_dict[i], "J")

            # if find_G_result[0] == True:
            G_command_dict[count] = find_G_result
            X_dist_dict[count] = find_X[0]
            Y_dist_dict[count] = find_Y[0]
            Z_dist_dict[count] = find_Z[0]
            I_dist_dict[count] = find_I[0]
            J_dist_dict[count] = find_J[0]

            All_dist_dict[count] = [X_dist_dict[count], Y_dist_dict[count], Z_dist_dict[count], I_dist_dict[count],
                                    J_dist_dict[count]]
            All_var_dict[count] = [find_X[1], find_Y[1], find_Z[1], find_I[1], find_J[1]]

            ### Linear Commands
            direction_check = []
            if G_command_dict[count] == 'G1':
                distance = pythag(X_dist_dict[count], Y_dist_dict[count], Z_dist_dict[count])
                for elem in All_var_dict[count]:
                    if elem != 0:
                        direction_check.append(elem)

                if "X" in direction_check and "Y" in direction_check and "Z" not in direction_check:
                    slope = None
                    if X_dist_dict[count] != 0:
                        slope = Y_dist_dict[count] / X_dist_dict[count]
                else:
                    slope = None

                slope_dict[count] = slope

            ### Circular commands
            elif G_command_dict[count] == 'G3' or 'G03':
                theta = find_theta(X_dist_dict[count], Y_dist_dict[count], I_dist_dict[count], J_dist_dict[count])
                if theta <= 0:
                    theta = 2 * np.pi - abs(theta)
                R = pythag(I_dist_dict[count], J_dist_dict[count], 0)
                arc_length = R * theta
                distance = arc_length

                slope_dict[count] = round(R, 9)


            elif G_command_dict[count] == 'G2' or 'G02':
                theta = find_theta(X_dist_dict[count], Y_dist_dict[count], I_dist_dict[count], J_dist_dict[count])
                if theta < 0:
                    theta = abs(theta)
                else:
                    theta = 2 * np.pi - theta
                R = pythag(I_dist_dict[count], J_dist_dict[count], 0)
                arc_length = R * theta
                distance = arc_length

                slope_dict[count] = round(R, 9)

            count += 1
            distance_commands_dict[i] = distance

    # print(gcode_dict)
    # print(G_command_dict)
    # print(X_dist_dict)
    # print(Y_dist_dict)
    # print(Z_dist_dict)
    # print(I_dist_dict)
    # print(J_dist_dict)
    # print(All_dist_dict)
    # print(All_var_dict)
    # print(slope_dict)
    # print(distance_commands_dict)

    return gcode_dict, G_command_dict, X_dist_dict, Y_dist_dict, Z_dist_dict, I_dist_dict, J_dist_dict, All_dist_dict, All_var_dict, slope_dict, distance_commands_dict

## Combines "like" gcode lines into a continuous path (used to create final gcode and acceleration path)
def condense_gcode(resync_number, resync_type, distance_commands_dict, G_command_dict, All_var_dict, X_dist_dict,Y_dist_dict, Z_dist_dict, I_dist_dict, J_dist_dict):
    dir_change_count = 0
    X_sum = X_dist_dict[0]
    Y_sum = Y_dist_dict[0]
    Z_sum = Z_dist_dict[0]
    I_0 = I_dist_dict[0]
    J_0 = J_dist_dict[0]

    Sum_G_command_dict = {0: G_command_dict[0]}
    Sum_var_dict = {0: All_var_dict[0]}
    Sum_coord_dict = {0: [X_dist_dict[0], Y_dist_dict[0], I_dist_dict[0], J_dist_dict[0]]}

    #### Finds initial distances....

    ### For linear moves
    if G_command_dict[0] == "G1":
        sum_distance = pythag(X_sum, Y_sum, Z_sum)

    ### For circles
    else:
        theta = find_theta(X_sum, Y_sum, I_0, J_0)
        if G_command_dict[0] == "G3":
            if theta <= 0:
                theta = 2 * np.pi - abs(theta)
        else:
            theta = find_theta(X_sum, Y_sum, I_0, J_0)
            if theta < 0:
                theta = abs(theta)
            else:
                theta = 2 * np.pi - theta

        R = pythag(I_0, J_0, 0)
        sum_distance = R * theta

    Sum_distance_dict = {0: sum_distance}

    resync_trigger_distance_dict = {}  # dictionary that keeps track of where direction changes occur (use to trigger a resync after N number of direction changes). ex: if key = 12, the direction changes between keys 12 and 13 in distance_commands_dict
    direction_change_dict = {}  # keeps track of direction changes

    command_count = 0
    i = 0  # count for number of times a distance command is used
    for j in range(len(distance_commands_dict)):
        type_check = type(distance_commands_dict[j])
        if type_check == str:
            command_count += 1

        if type_check != str and i < len(G_command_dict) - 1:
            i += 1
            current_G_command = G_command_dict[i]
            current_var = All_var_dict[i]
            ## Combines "like" gcode lines into a continuous path
            # for circular moves, slope = radius
            if G_command_dict[i] == G_command_dict[i - 1] and All_var_dict[i] == All_var_dict[i - 1] and slope_dict[i] == slope_dict[i - 1]:
                # '''G1 Commands'''
                if current_G_command == "G1":
                    X_sum += X_dist_dict[i]
                    Y_sum += Y_dist_dict[i]
                    Z_sum += Z_dist_dict[i]

                # '''G2/G3 Commands'''
                elif (round(I_dist_dict[i], 9) == round(I_dist_dict[i - 1] - X_dist_dict[i - 1]), 9) and (
                        round(J_dist_dict[i], 9) == round(J_dist_dict[i - 1] - Y_dist_dict[i - 1]), 9):

                    X_sum += X_dist_dict[i]
                    Y_sum += Y_dist_dict[i]

                    if round(X_sum, 9) == 0:
                        X_sum = 0
                    if round(Y_sum, 9) == 0:
                        Y_sum = 0


                else:  # if it is not a continuous circle

                    dir_change_count += 1

                    X_sum = X_dist_dict[i]
                    Y_sum = Y_dist_dict[i]
                    Z_sum = Z_dist_dict[i]
                    I_0 = I_dist_dict[i]  # I value is always referenced from starting point
                    J_0 = J_dist_dict[i]  # J value is always referenced from starting point

                    if resync_type == 'direction-based' and dir_change_count % resync_number == 0:
                        resync_trigger_distance_dict[dir_change_count - 1] = j

            else:
                dir_change_count += 1
                X_sum = X_dist_dict[i]
                Y_sum = Y_dist_dict[i]
                Z_sum = Z_dist_dict[i]
                I_0 = I_dist_dict[i]  # I value is always referenced from starting point
                J_0 = J_dist_dict[i]  # J value is always referenced from starting point

                if resync_type == 'direction-based' and dir_change_count % resync_number == 0:
                    resync_trigger_distance_dict[dir_change_count - 1] = j

            ### Finds total distances that will be used in acceleration profile
            if current_G_command == "G1":
                sum_distance = pythag(X_sum, Y_sum, Z_sum)

            #### for circles
            else:
                theta = find_theta(X_sum, Y_sum, I_0, J_0)
                if current_G_command == "G3":
                    if theta <= 0:
                        theta = 2 * np.pi - abs(theta)
                else:
                    theta = find_theta(X_sum, Y_sum, I_0, J_0)
                    if theta < 0:
                        theta = abs(theta)
                    else:
                        theta = 2 * np.pi - theta

                R = pythag(I_0, J_0, 0)
                sum_distance = R * theta

            Sum_G_command_dict[dir_change_count] = current_G_command  # for writing condensed gcode
            Sum_var_dict[dir_change_count] = current_var  # for writing condensed gcode
            Sum_coord_dict[dir_change_count] = [X_sum, Y_sum, Z_sum, I_0, J_0]  # for writing condensed gcode
            Sum_distance_dict[dir_change_count] = sum_distance  # for writing acceleration profile

    # print(Sum_G_command_dict) # G-commands
    # print(Sum_var_dict) # variables, X, Y, Z....
    # print(Sum_coord_dict) # coordinate values
    # print(Sum_distance_dict)
    # print("reset based on number of commands: ", resync_trigger_numCommand_dict)

    # print("reset based on distance: ", resync_trigger_distance_dict)
    return Sum_G_command_dict, Sum_var_dict, Sum_coord_dict, Sum_distance_dict, resync_trigger_distance_dict

## Writes uninterrupted g-code to file for use by 3D printer
def generate_gcode(final_gcode_txt_export, accel, Z_var, z_o, feed, Sum_G_command_dict, Sum_var_dict, Sum_coord_dict,resync_trigger_distance_dict):
    # create txt for gcode used in 3d printer
    resync_PING = '\nSocketWriteString($clientSocket, "PING")'  # '\nSocketWriteInt32($clientSocket, 100)'#'\nSocketWriteString($clientSocket, "PING")' #('\n\rFILEWRITE $hFile, TIMER(0, PERFORMANCE)')

    with open(final_gcode_txt_export, "w") as f:
        with open(intro_gcode, "r") as g:
            for line in g:
                Z_var_string = Z_var[0]
                Z_var_string_0 = Z_var[0] + str(0)
                Z_var_string_start = Z_var[0] + str(Z_start)
                for elem in Z_var[1:]:
                    Z_var_string = Z_var_string + ', '+ elem
                    Z_var_string_0 = Z_var_string_0 + ' ' + elem + str(0)
                    Z_var_string_start = Z_var_string_start + ' ' + elem + str(Z_start)

                line = line.replace('{Z_var_list}', Z_var_string).replace('{ramprate}', str(accel)).replace('{feed}',str(feed))
                line = line.replace('{Z_var_0}', Z_var_string_0)
                line = line.replace('{Z_var_Z_start}', Z_var_string_start)


                f.write(line)
            g.close()

        f.write('\nDwell(3)\n\r')

        for i in range(len(Sum_coord_dict)):
            G_command = Sum_G_command_dict[i]
            # if G_command == "G3":
            #     G_command = "CCW"
            # if G_command == "G2":
            #     G_command = "CW"
            coordinates = str(G_command) + " "
            for j in range(len(Sum_coord_dict[i])):
                dist = Sum_coord_dict[i][j]
                variable = Sum_var_dict[i][j]
                if variable == 'Z':
                    variable = Z_var[0]
                if variable != 0:
                    if variable == Z_var[0]:
                        for elem in Z_var:
                            coordinates += str(elem) + str(dist) + " "
                    else:
                        coordinates += str(variable) + str(dist) + " "


            f.write("\n\r" + coordinates)

            if i != 0 and i < len(Sum_coord_dict) - 1 and i in resync_trigger_distance_dict:
            # if i in resync_trigger_distance_dict:
                f.write(resync_PING)

        f.write('\n\rDwell(5)')
        f.write('\n\rHome(['+Z_var_string+ '])')

        f.close()

    return f

## creates acceleration profile
def accel_profile(Sum_distance_dict):
    import time
    accel_dist_dict = {}
    accel_time_dict = {}
    accel_dist_abs_dict = {}
    accel_time_abs_dict = {}
    flag_accel_result = {}  # flags are used to see if the feed and accel has been used to calculate accel and decel values yet
    flag_decel_result = {}
    flag_short_move = {}
    decel_abs_dist = 0
    decel_abs_time = 0


    for i in range(len(Sum_distance_dict)):
        if accel == 0:
            steady_state_dist = abs(Sum_distance_dict[i])
            accel_dist = 0
            decel_dist = 0
            accel_time = 0
            decel_time = 0

        else:
            key = str(feed) + ', ' + str(accel)
            try:  # attempts to save run time be searching if this value has already been calculated
                accel_dist = flag_accel_result[key][0]
                accel_time = flag_accel_result[key][1]
                decel_dist = flag_decel_result[key][0]
                decel_time = flag_decel_result[key][1]

            except:
                flag_accel_result[key] = accel_length(0, feed, accel)  # adds calc to flag list
                flag_decel_result[key] = accel_length(feed, 0, decel)  # adds calc to flag list

                accel_dist = flag_accel_result[key][0]
                accel_time = flag_accel_result[key][1]
                decel_dist = flag_decel_result[key][0]
                decel_time = flag_decel_result[key][1]

            steady_state_dist = abs(Sum_distance_dict[i]) - accel_dist - decel_dist

        if steady_state_dist <= 0:
            steady_state_dist = 0
            accel_dist = abs(Sum_distance_dict[i]) * 0.5
            decel_dist = accel_dist
            key = str(feed) + ', ' + str(accel) + ', ' + str(accel_dist)

            try:
                accel_time = flag_short_move[key][0]
                decel_time = flag_short_move[key][1]
            except:
                flag_short_move_list = []
                accel_time = findt(0, accel, accel_dist)
                v_current = findv(0, accel, accel_dist)
                decel_time = findt_using_v(v_current, 0, decel)

                flag_short_move_list.append(accel_time)
                flag_short_move_list.append(decel_time)
                flag_short_move[key] = flag_short_move_list

        steady_state_time = steady_state_dist / feed
        accel_dist_dict[i] = [accel_dist, steady_state_dist, decel_dist]
        accel_time_dict[i] = [accel_time, steady_state_time, decel_time]

        key = i

        accel_abs_dist = decel_abs_dist + accel_dist
        steady_abs_dist = accel_abs_dist + steady_state_dist
        decel_abs_dist = steady_abs_dist + decel_dist

        accel_dist_abs_dict[key] = [accel_abs_dist, steady_abs_dist, decel_abs_dist]

        accel_abs_time = decel_abs_time + accel_time
        steady_abs_time = accel_abs_time + steady_state_time
        decel_abs_time = steady_abs_time + decel_time

        accel_time_abs_dict[key] = [accel_abs_time, steady_abs_time, decel_abs_time]

    # print("accel_dist_dict = ", accel_dist_dict)
    # print("accel_time_dict = ", accel_time_dict) # never used
    # print("accel_dist_abs_dict = ", accel_dist_abs_dict) # used in time-based function
    # print("accel_time_abs_dict = ", accel_time_abs_dict) # used in time-based function

    return accel_dist_abs_dict, accel_time_abs_dict

## creates dictionary of time and commands
def distance2time(accel_profile_distance, accel_profile_time, feed, accel, decel, distance_commands_dict, locate_resync):
    time_list = []
    time_dict = {}
    x_current = 0
    t = 0
    distance = 0
    index_start = 0
    flag_rel_dist_accel = {}
    flag_rel_dist_max_vel = {}
    flag_rel_dist_decel = {}
    flag_count = 0
    count = 0
    time_resync = 0
    command_list = []
    times_of_resync = []
    for j in range(len(distance_commands_dict)):
        if type(distance_commands_dict[j]) == str:
            #time_dict[j] = distance_commands_dict[j]
            command_list.append(distance_commands_dict[j])
            time_dict[t] = command_list
        else:
            command_list = []
            distance += abs(distance_commands_dict[j])
            for i in range(index_start, len(accel_profile_distance)):
                accel_region = accel_profile_distance[i][0]
                max_velocity_region = accel_profile_distance[i][1]
                decel_region = accel_profile_distance[i][2]

                if distance >= decel_region:
                    t_current = accel_profile_time[i][2] - time_resync
                    x_current = decel_region

                    t = t_current
                    index_start += 1

                elif distance >= max_velocity_region:
                    t_current = accel_profile_time[i][1] - time_resync
                    x_current = max_velocity_region
                    t = t_current

                elif distance >= accel_region:
                    t_current = accel_profile_time[i][0] - time_resync
                    x_current = accel_region
                    t = t_current

                if distance < accel_region:
                    relative_distance = distance - x_current
                    try:
                        relative_distance = round(relative_distance, 7)
                        t_current = flag_rel_dist_accel[relative_distance]
                    except:
                        t_current = findt(0, accel, relative_distance)
                        relative_distance = round(relative_distance, 7)

                    t += t_current
                    flag_rel_dist_accel[relative_distance] = t_current
                    break

                elif distance < max_velocity_region:
                    count += 1
                    relative_distance = distance - x_current
                    try:
                        relative_distance = round(relative_distance, 7)
                        t_current = flag_rel_dist_max_vel[relative_distance]
                        flag_count += 1
                    except:
                        t_current = relative_distance / feed  # findt(feed, 0, relative_distance)
                        relative_distance = round(relative_distance, 7)
                    t += t_current
                    flag_rel_dist_max_vel[relative_distance] = t_current
                    break

                elif distance < decel_region:
                    relative_distance = distance - x_current

                    try:
                        relative_distance = round(relative_distance, 7)
                        t_current = flag_rel_dist_decel[relative_distance]
                    except:
                        t_current = findt(feed, decel, relative_distance)
                        relative_distance = round(relative_distance, 7)

                    t += t_current
                    flag_rel_dist_decel[relative_distance] = t_current
                    break

                if distance == decel_region or distance == max_velocity_region or distance == accel_region:
                    break

            time_output = t
            time_list.append(time_output)
            if j in locate_resync:
                times_of_resync.append(t)

            #time_dict[j] = time_output

    return time_dict, t, times_of_resync

################################Execute DGC Functions#########################################
path = path + file_or_folder

try:
    f_list = os.listdir(path)
    f_list_final = []
    for file in f_list:
        f_list_final.append(file)
except:
    f_list_final = [path]

print('imported g-codes: ', f_list_final)


compiled_initial_list = []
compiled_command_dict = {}
for file in f_list_final:
    if len(f_list_final) > 1:
        gcode_txt_imported = path+ '\\' + file
        final_gcode_txt_export = file_or_folder + '_TCODE.txt'  # '230831_1mm_32_70_Gradient_diamond_lattice_aerotech.txt'

    else:
        gcode_txt_imported = file
        final_gcode_txt_export = gcode_txt_imported.replace('GCODE','TCODE')  # '230831_1mm_32_70_Gradient_diamond_lattice_aerotech.txt'

    print("\r\nImporting Gcode....", file)
    ## open and read in gcode txt file into a list - remove comments, spaces, random characters
    gcode_list = open_gcode(gcode_txt_imported)

    ## splits up gcode into directions, distances, G command type (G1, G2, G3, etc)
    ## create a gcode,  distance, and direction dictionary
    print("Translating Gcode to Time....")
    parse_output = parse_gcode(gcode_list)
    gcode_dict = parse_output[0]
    G_command_dict = parse_output[1]
    X_dist_dict = parse_output[2]
    Y_dist_dict = parse_output[3]
    Z_dist_dict = parse_output[4]
    I_dist_dict = parse_output[5]
    J_dist_dict = parse_output[6]
    All_dist_dict = parse_output[7]
    All_var_dict = parse_output[8]
    slope_dict = parse_output[9]
    distance_commands_dict = parse_output[10]

    ## Combines "like" gcode lines into a continuous path (used to create final gcode and acceleration path)
    condense_results = condense_gcode(resync_number, resync_type, distance_commands_dict, G_command_dict, All_var_dict,X_dist_dict, Y_dist_dict, Z_dist_dict, I_dist_dict, J_dist_dict)
    Sum_G_command_dict = condense_results[0]
    Sum_var_dict = condense_results[1]
    Sum_coord_dict = condense_results[2]
    Sum_distance_dict = condense_results[3]
    resync_trigger_distance_dict = condense_results[4]
    locate_resync = list(resync_trigger_distance_dict.values())

    # Writes uninterrupted g-code to file for use by 3D printer - Use this if you need to double check paths are the same
    # if len(f_list_final) > 1:
    #     final_gcode_txt_export = 'VerifyParallelPrint_' + file  # '230831_1mm_32_70_Gradient_diamond_lattice_aerotech.txt'

    generate_gcode(final_gcode_txt_export, accel, Z_var, Z_start, feed, Sum_G_command_dict, Sum_var_dict, Sum_coord_dict, resync_trigger_distance_dict)
    print("\n", final_gcode_txt_export, "has been created\n\r")

    ## creates acceleration profile
    accel_profile_output = accel_profile(Sum_distance_dict)
    accel_profile_distance = accel_profile_output[0]
    accel_profile_time = accel_profile_output[1]


    ## creates dictionary of time and commands
    time_dict_output = distance2time(accel_profile_distance, accel_profile_time, feed, accel, decel, distance_commands_dict, locate_resync)
    time_dict = time_dict_output[0]
    times_of_resync = time_dict_output[2]
    start_delay_time = float(distance2time(accel_profile_distance, accel_profile_time, feed, accel, decel, [start_delay], locate_resync)[1])
    offset_time = float(distance2time(accel_profile_distance, accel_profile_time, feed, accel, decel, [abs(offset)], locate_resync)[1])
    offset_time_initial = float(distance2time(accel_profile_distance, accel_profile_time, feed, accel, decel, [abs(offset_initial)], locate_resync)[1])


    if offset < 0:
        offset_time = -offset_time
    if offset_initial < 0:
        offset_time_initial = -offset_time_initial

    ## creates final list of commands and times to use
    time_list_final_current = list(time_dict.keys())[1:]
    command_list_final_current = list(time_dict.values())[1:]
    initial_commands_list_current = list(time_dict.values())[0]

    for i in range(len(time_list_final_current)):
        command = command_list_final_current[i]
        key = time_list_final_current[i]
        if key not in compiled_command_dict:
            compiled_command_dict[key] = command
        else:
            compiled_command_dict[key].extend(command)


    #if file == f_list_final[0]:
    print('current gcode number time-stamps/commands', len(time_list_final_current), len(command_list_final_current))

    compiled_initial_list.extend(initial_commands_list_current)


    if resync_number == 1:
        times_of_resync = times_of_resync[1:]

compiled_command_dict = dict(sorted(compiled_command_dict.items())) # sorts the compiled dictionary using the time-stamps
#print(compiled_command_dict)
time_list_final = list(compiled_command_dict.keys())
command_list_final = list(compiled_command_dict.values())
initial_command_list_final = list(compiled_initial_list)

print('number of compiled time-stamps/commands', len(time_list_final), len(command_list_final))

preset_list = []
initial_toggle_list = []
for command in initial_command_list_final:
    if '{preset}' in command:
        command = command.replace('{preset}', '')
        preset_list.append(command)
        #print(True)
    else:
        initial_toggle_list.append(command)

preset_list = '[%s]' % ', '.join(map(str, preset_list))
initial_toggle_list = '[%s]' % ', '.join(map(str, initial_toggle_list))



# for i in range(len(command_list_final)):
#     command_list = command_list_final[i]
#     command_list_final[i] = '[%s]' % ', '.join(map(str, command_list))

end_time = time.time()
total_time = end_time - start_time
print("\nTotal time to translate distance to time: ", total_time)

print("preset_list: ", preset_list)
print("initial_toggle_list: ", initial_toggle_list)
print("time_list_final = ", time_list_final)
print("command_list_final = ", command_list_final)

print("\nWaiting for ping to start....")
##### WAITING FOR PING ##################################
## function to be used as thread to listen for incoming resync pings in the background
count = 0
add_time = 0
def thread_listen():
    global got_a_ping
    global conn
    while True:
        #print('waiting for ping')
        data = conn.recv(4)
        decoded_data = data.decode("utf-8")
        if decoded_data == 'PING':
            got_a_ping = True

got_a_ping = False
if __name__ == '__main__':

    HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
    PORT = 65432  # Port to listen on (non-privileged ports are > 1023)
    list_data = []

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:

            print(f"Connected by {addr}")
            data = conn.recv(5)
            decoded_data = data.decode("utf-8")

            if decoded_data == 'START':
                exec(preset_list)
                print('Received start PING')

                print("Setting the pressures....")

            #### Executes Absolute timing ####
            list_data = []
            time.sleep(3 - PING_delay - start_delay_time)

            time_stamps_from_aerotech = []
            python_received_time_stamps = []
            add_time_list = []
            i = 0
            # count = 0
            # add_time = 0

            start_time = time.time()
            print("Initial toggle....")
            exec(initial_toggle_list)

            Thread(target=thread_listen).start()
            while i < len(command_list_final):
                if resync_type is not False and got_a_ping == True:
                    got_a_ping = False

                    python_recieved_time = real_time  # to match print time rather than when the aux turn on or off
                    #print('-------------Received command sync number ', count)
                    #print('python time stamp t = ', python_recieved_time)

                    add_time = (python_recieved_time - PING_delay) - times_of_resync[count]

                    add_time_list.append(add_time)
                    #print('calculated resync time stamp t = ', times_of_resync[count])
                    print('add_time = ', add_time)
                    count += 1

                if i == 0:
                    offset = offset_time_initial
                else:
                    offset = offset_time

                real_time = time.time() - start_time
                current_time_stamp_execute = (time_list_final[i] - offset) + add_time
                if real_time >= current_time_stamp_execute:
                    print('----executed command number '+ str(i+1) + ' of '+str(len(command_list_final)) )
                    print(command_list_final[i])

                    for elem in command_list_final[i]:
                        exec(elem)
                    #exec(command_list_final[i])
                    i += 1

            serialPort1.close()
            serialPort2.close()
            serialPort3.close()

            print("DONE!")