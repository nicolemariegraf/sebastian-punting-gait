### Author: John Grezmak
### Python library of gaits for the 18-dof Sebastian robot

import numpy as np
import time

def blind_grip(robot, a_mov, sensing, tripod="both"):
    ### This function makes the robot perform a blind grip from its current stance by moving ankle servos inwards
    
    # robot:        Instance of robot class from Sebastian_library_Maeastro.py
    # a_mov:        Change in ankle servo position (quarter-microseconds)
    # sensing:      
    # tripod:       Which tripod to move (str): "left", "right", or "both"
    
    print('Starting blind grip...')
    
    ### Pause by set amount to allow data collection to start
    if sensing == True:
        time.sleep(4.5)
        
    
    grip_mode = 1
    if grip_mode == 1:
        ### Perform blind grip mode 1 (all ankle servos move simulatenously)
        speed = 20
        if tripod == "both":
            robot.move_by([robot.leg_L1.ankle_servo, robot.leg_L2.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R2.ankle_servo, robot.leg_R3.ankle_servo], \
                            [a_mov, a_mov, a_mov, a_mov, a_mov, a_mov], [speed, speed, speed, speed, speed, speed], [0, 0, 0, 0, 0, 0])
        elif tripod == "left":
            robot.move_by([robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [a_mov, a_mov, a_mov], [speed, speed, speed], [0, 0, 0])
        elif tripod == "right":
            robot.move_by([robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [a_mov, a_mov, a_mov], [speed, speed, speed], [0, 0, 0])
        
    elif grip_mode == 2:
        ### Perform blind grip mode 2 (all ankle and knee servos move simulatenously)
        k_mov = 450
        robot.move_by([robot.leg_L1.ankle_servo, robot.leg_L2.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R2.ankle_servo, robot.leg_R3.ankle_servo], \
                        [a_mov, a_mov, a_mov, a_mov, a_mov, a_mov], [30, 30, 30, 30, 30, 30], [0, 0, 0, 0, 0, 0])
        time.sleep(2)
        robot.move_by([robot.leg_L1.knee_servo, robot.leg_L2.knee_servo, robot.leg_L3.knee_servo, robot.leg_R1.knee_servo, robot.leg_R2.knee_servo, robot.leg_R3.knee_servo], \
                        [-1*k_mov, -1*k_mov, -1*k_mov, -1*k_mov, -1*k_mov, -1*k_mov], [30, 30, 30, 30, 30, 30], [0, 0, 0, 0, 0, 0])
    elif grip_mode == 3:
        ### Perform blind grip mode 3 (left then right tripod move)
        robot.move_by([robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [a_mov, a_mov, a_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(0.75)
        robot.move_by([robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [a_mov, a_mov, a_mov], [0, 0, 0], [0, 0, 0])
    elif grip_mode == 4:
        ###
        k_mov = 200
        delay = 0.5
        robot.move_by([robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [a_mov, a_mov, a_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(delay)
        robot.move_by([robot.leg_L1.knee_servo, robot.leg_L3.knee_servo, robot.leg_R2.knee_servo], [-1*k_mov, -1*k_mov, -1*k_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(delay)
        robot.move_by([robot.leg_L1.knee_servo, robot.leg_L3.knee_servo, robot.leg_R2.knee_servo], [k_mov, k_mov, k_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(delay)
        robot.move_by([robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(delay)
        
        robot.move_by([robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [a_mov, a_mov, a_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(delay)
        robot.move_by([robot.leg_R1.knee_servo, robot.leg_R3.knee_servo, robot.leg_L2.knee_servo], [-1*k_mov, -1*k_mov, -1*k_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(delay)
        robot.move_by([robot.leg_R1.knee_servo, robot.leg_R3.knee_servo, robot.leg_L2.knee_servo], [k_mov, k_mov, k_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(delay)
        robot.move_by([robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [0, 0, 0], [0, 0, 0])
        time.sleep(delay)
    elif grip_mode == 5:
        ###
        for k in range(3):
            robot.move_by([robot.leg_L1.knee_servo, robot.leg_L3.knee_servo, robot.leg_R2.knee_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [450, 450, 450, 350, 350, 350], [30, 30, 30, 30, 30, 30], [0, 0, 0, 0, 0, 0])
            time.sleep(0.75)
            robot.move_by([robot.leg_L1.knee_servo, robot.leg_L3.knee_servo, robot.leg_R2.knee_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [-450, -450, -450, -700, -700, -700], [30, 30, 30, 60, 60, 60], [0, 0, 0, 0, 0, 0])
            time.sleep(1)
            robot.move_by([robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [350, 350, 350], [100, 100, 100], [0, 0, 0])
            time.sleep(1)
            
        robot.move_by([robot.leg_L1.knee_servo, robot.leg_L3.knee_servo, robot.leg_R2.knee_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [350, 350, 350, 350, 350, 350], [30, 30, 30, 30, 30, 30], [0, 0, 0, 0, 0, 0])
        time.sleep(0.5)
        robot.move_by([robot.leg_L1.knee_servo, robot.leg_L3.knee_servo, robot.leg_R2.knee_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.ankle_servo], [-1*350, -1*350, -1*350, -1*350, -1*350, -1*350], [80, 80, 80, 80, 80, 80], [0, 0, 0, 0, 0, 0])
        time.sleep(1)

        
        for k in range(3):
            robot.move_by([robot.leg_R1.knee_servo, robot.leg_R3.knee_servo, robot.leg_L2.knee_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [450, 450, 450, 350, 350, 350], [30, 30, 30, 30, 30, 30], [0, 0, 0, 0, 0, 0])
            time.sleep(0.75)
            robot.move_by([robot.leg_R1.knee_servo, robot.leg_R3.knee_servo, robot.leg_L2.knee_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [-450, -450, -450, -700, -700, -700], [30, 30, 30, 60, 60, 60], [0, 0, 0, 0, 0, 0])
            time.sleep(1)
            robot.move_by([robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [350, 350, 350], [100, 100, 100], [0, 0, 0])
            time.sleep(1)
            
        robot.move_by([robot.leg_R1.knee_servo, robot.leg_R3.knee_servo, robot.leg_L2.knee_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [350, 350, 350, 350, 350, 350], [30, 30, 30, 30, 30, 30], [0, 0, 0, 0, 0, 0])
        time.sleep(0.5)
        robot.move_by([robot.leg_R1.knee_servo, robot.leg_R3.knee_servo, robot.leg_L2.knee_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.ankle_servo], [-1*350, -1*350, -1*350, -1*350, -1*350, -1*350], [80, 80, 80, 80, 80, 80], [0, 0, 0, 0, 0, 0])
        time.sleep(1)
            
        
                    

def forward_gait_V1(robot, al_speed, ap_speed, a_mov, delta_sl, max_dl, base_speed, base_accel, delay_gait, delay_trans, steps):
    ### This is a 6-phase forwards gait for the 18-dof robot, in which the ankle joints move up into the body during swing
    
    # robot:        Instance of robot class from Sebastian_library_Maeastro.py
    # al_speed:     Speed of ankle servos when lifting
    # ap_speed:     Speed of ankle servos when planting
    # a_mov:        Ankle movement length (in quarter-microseconds)
    # delta_sl:     Half of angle change of single-leg side of tripod base servos (degrees)
    # max_dl:       Maximum angle of double-leg side of tripod base servos from horizontal
    # base_speed:   Speed of all base servos
    # base_accel:   Acceleration of all base servos
    # delay_gait:   Delay between phases of gait
    # delay_trans:  Delay between transition from starting position to gait
    # steps:        Number of steps to take
    
    ### Calculate minimum angle of double-leg side of tripod base servos from horizontal
    min_dl = round(np.arcsin(np.sin(max_dl*np.pi/180) - 2*np.sin(delta_sl*np.pi/180))*180/np.pi, 1)
    
    ### Check if collision between single-side leg and double-side leg of opposite tripod
    if min_dl - delta_sl < 10:
        print('Potential collision detected; adjust stride parameters')
        return
    
    ### Calculate ratio of single-leg to double-leg side base servo movements
    sl_to_dl = (2*delta_sl)/(max_dl - min_dl)
    
    ### Convert degree change to microseconds
    L2_delta_start = -1*int(delta_sl*10)
    R1_delta_start = int((max_dl - 45)*10)
    R3_delta_start = int((45 - min_dl)*10)
    R2_delta_start = -1*int(delta_sl*10)
    L1_delta_start = int((45 - min_dl)*10)
    L3_delta_start = int((max_dl - 45)*10)
    
    L2_delta = int(2*delta_sl*10)
    R1_delta = -1*int((max_dl - min_dl)*10)
    R3_delta = -1*int((max_dl - min_dl)*10)
    R2_delta = int(2*delta_sl*10)
    L1_delta = -1*int((max_dl - min_dl)*10)
    L3_delta = -1*int((max_dl - min_dl)*10)
    
    ### Calculate speeds for base servos
    base_speed2_start = int(base_speed/(R1_delta_start/R3_delta_start))
    base_speed2 = int(base_speed/sl_to_dl)
    
    ### Move legs to starting position
    print('Moving legs to starting position...')
    
    # Lift ankle servos of right tripod
    robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo], [a_mov, a_mov, a_mov], [al_speed, al_speed, al_speed], [0, 0, 0])
    time.sleep(delay_gait)
    
    # Move base servos of right tripod
    robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo], [L2_delta_start, R1_delta_start, R3_delta_start], [base_speed, base_speed, base_speed2_start], [base_accel, base_accel, base_accel])
    time.sleep(delay_gait)
    
    # Move ankle servos of right tripod
    robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [ap_speed, ap_speed, ap_speed], [0, 0, 0])
    time.sleep(delay_gait)
    
    # Move ankle servos of left tripod
    robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo], [a_mov, a_mov, a_mov], [al_speed, al_speed, al_speed], [0, 0, 0])
    time.sleep(delay_gait)
    
    # Move base servos of left tripod
    robot.move_by([robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], [R2_delta_start, L1_delta_start, L3_delta_start], [base_speed, base_speed2_start, base_speed], [base_accel, base_accel, base_accel])
    time.sleep(delay_gait)
    
    # Move ankle servos of left tripod
    robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [ap_speed, ap_speed, ap_speed], [0, 0, 0])
    
    time.sleep(delay_trans)
    
    ### Start gait
    print('Starting gait...')
    
    for k in range(steps):
        # Lift ankle servos of left tripod
        robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo], [a_mov, a_mov, a_mov], [al_speed, al_speed, al_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Move bases
        robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, \
                        robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                        [L2_delta, R1_delta, R3_delta, R2_delta, L1_delta, L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Plant ankle servos of left tripod
        robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [ap_speed, ap_speed, ap_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Lift ankle servos of right tripod
        robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo], [a_mov, a_mov, a_mov], [al_speed, al_speed, al_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Move bases
        robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, \
                        robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                        [-1*L2_delta, -1*R1_delta, -1*R3_delta, -1*R2_delta, -1*L1_delta, -1*L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Plant ankle servos of right tripod
        robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [ap_speed, ap_speed, ap_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
    

def forward_gait_V1_2(robot, al_speed, ap_speed, a_mov, delta_sl, max_dl, base_speed, base_accel, delay_gait, delay_trans, steps):
    ### This is a 4-phase forwards gait for the 18-dof robot, in which the ankle joints move up into the body during swing
    
    # robot:        Instance of robot class from Sebastian_library_Maeastro.py
    # al_speed:     Speed of ankle servos when lifting
    # ap_speed:     Speed of ankle servos when planting
    # a_mov:        Ankle movement length (in quarter-microseconds)
    # delta_sl:     Half of angle change of single-leg side of tripod base servos (degrees)
    # max_dl:       Maximum angle of double-leg side of tripod base servos from horizontal
    # base_speed:   Speed of all base servos
    # base_accel:   Acceleration of all base servos
    # delay_gait:   Delay between phases of gait
    # delay_trans:  Delay between transition from starting position to gait
    # steps:        Number of steps to take
    
    ### Calculate minimum angle of double-leg side of tripod base servos from horizontal
    min_dl = round(np.arcsin(np.sin(max_dl*np.pi/180) - 2*np.sin(delta_sl*np.pi/180))*180/np.pi, 1)
    
    ### Check if collision between single-side leg and double-side leg of opposite tripod
    if min_dl - delta_sl < 10:
        print('Potential collision detected; adjust stride parameters')
        return
    
    ### Calculate ratio of single-leg to double-leg side base servo movements
    sl_to_dl = (2*delta_sl)/(max_dl - min_dl)
    
    ### Convert degree change to microseconds
    L2_delta_start = -1*int(delta_sl*10)
    R1_delta_start = int((max_dl - 45)*10)
    R3_delta_start = int((45 - min_dl)*10)
    R2_delta_start = -1*int(delta_sl*10)
    L1_delta_start = int((45 - min_dl)*10)
    L3_delta_start = int((max_dl - 45)*10)
    
    L2_delta = int(2*delta_sl*10)
    R1_delta = -1*int((max_dl - min_dl)*10)
    R3_delta = -1*int((max_dl - min_dl)*10)
    R2_delta = int(2*delta_sl*10)
    L1_delta = -1*int((max_dl - min_dl)*10)
    L3_delta = -1*int((max_dl - min_dl)*10)
    
    ### Calculate speeds for base servos
    base_speed2_start = int(base_speed/(R1_delta_start/R3_delta_start))
    base_speed2 = int(base_speed/sl_to_dl)
    
    ### Move legs to starting position
    print('Moving legs to starting position...')
    
    # Lift ankle servos and move base servos of right tripod
    robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo], [a_mov, a_mov, a_mov, L2_delta_start, R1_delta_start, R3_delta_start], [al_speed, al_speed, al_speed, base_speed, base_speed, base_speed2_start], [0, 0, 0, base_accel, base_accel, base_accel])
    time.sleep(delay_gait)
    
    # Move ankle servos of right tripod
    robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [ap_speed, ap_speed, ap_speed], [0, 0, 0])
    time.sleep(delay_gait)
    
    # Move ankle servos of left tripod
    robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo], [a_mov, a_mov, a_mov], [al_speed, al_speed, al_speed], [0, 0, 0])
    time.sleep(delay_gait)
    
    # Move base servos of left tripod
    robot.move_by([robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], [R2_delta_start, L1_delta_start, L3_delta_start], [base_speed, base_speed2_start, base_speed], [base_accel, base_accel, base_accel])
    time.sleep(delay_gait)
    
    # Move ankle servos of left tripod
    robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [ap_speed, ap_speed, ap_speed], [0, 0, 0])
    
    time.sleep(delay_trans)
    
    ### Start gait
    print('Starting gait...')
    
    for k in range(steps):
        # Lift ankle servos of left tripod
        robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo], [a_mov, a_mov, a_mov], [al_speed, al_speed, al_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Move bases
        robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, \
                        robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                        [L2_delta, R1_delta, R3_delta, R2_delta, L1_delta, L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Plant ankle servos of left tripod
        robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [ap_speed, ap_speed, ap_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Lift ankle servos of right tripod
        robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo], [a_mov, a_mov, a_mov], [al_speed, al_speed, al_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Move bases
        robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, \
                        robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                        [-1*L2_delta, -1*R1_delta, -1*R3_delta, -1*R2_delta, -1*L1_delta, -1*L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Plant ankle servos of right tripod
        robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo], [-1*a_mov, -1*a_mov, -1*a_mov], [ap_speed, ap_speed, ap_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        
        
def forward_gait_V2(robot, kl_speed, kp_speed, k_mov, delta_sl, max_dl, base_speed, base_accel, delay_gait, delay_trans, steps):
    ### This is a 6-phase forwards gait for the 18-dof robot, in which the ankle joints move up into the body during swing
    
    # robot:        Instance of robot class from Sebastian_library_Maeastro.py
    # kl_speed:     Speed of knee servos when lifting
    # kp_speed:     Speed of knee servos when planting
    # k_mov:        Knee movement length (in quarter-microseconds)
    # delta_sl:     Half of angle change of single-leg side of tripod base servos (degrees)
    # max_dl:       Maximum angle of double-leg side of tripod base servos from horizontal
    # base_speed:   Speed of all base servos
    # base_accel:   Acceleration of all base servos
    # delay_gait:   Delay between phases of gait
    # delay_trans:  Delay between transition from starting position to gait
    # steps:        Number of steps to take
    
    ### Calculate minimum angle of double-leg side of tripod base servos from horizontal
    min_dl = round(np.arcsin(np.sin(max_dl*np.pi/180) - 2*np.sin(delta_sl*np.pi/180))*180/np.pi, 1)
    
    ### Check if collision between single-side leg and double-side leg of opposite tripod
    if min_dl - delta_sl < 10:
        print('Potential collision detected; adjust stride parameters')
        return
    
    ### Calculate ratio of single-leg to double-leg side base servo movements
    sl_to_dl = (2*delta_sl)/(max_dl - min_dl)
    
    ### Convert degree change to microseconds
    L2_delta_start = -1*int(delta_sl*10)
    R1_delta_start = int((max_dl - 45)*10)
    R3_delta_start = int((45 - min_dl)*10)
    R2_delta_start = -1*int(delta_sl*10)
    L1_delta_start = int((45 - min_dl)*10)
    L3_delta_start = int((max_dl - 45)*10)
    
    L2_delta = int(2*delta_sl*10)
    R1_delta = -1*int((max_dl - min_dl)*10)
    R3_delta = -1*int((max_dl - min_dl)*10)
    R2_delta = int(2*delta_sl*10)
    L1_delta = -1*int((max_dl - min_dl)*10)
    L3_delta = -1*int((max_dl - min_dl)*10)
    
    ### Calculate speeds for base servos
    base_speed2_start = int(base_speed/(R1_delta_start/R3_delta_start))
    base_speed2 = int(base_speed/sl_to_dl)
    
    ### Move legs to starting position
    print('Moving legs to starting position...')
    
    # Lift knee servos of right tripod
    robot.move_by([robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], [k_mov, k_mov, k_mov], [kl_speed, kl_speed, kl_speed], [0, 0, 0])
    time.sleep(delay_gait)
    
    # Move base servos of right tripod
    robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo], [L2_delta_start, R1_delta_start, R3_delta_start], [base_speed, base_speed, base_speed2_start], [base_accel, base_accel, base_accel])
    time.sleep(delay_gait)
    
    # Move ankle servos of right tripod
    robot.move_by([robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], [-1*k_mov, -1*k_mov, -1*k_mov], [kp_speed, kp_speed, kp_speed], [0, 0, 0])
    time.sleep(delay_gait)
    
    # Move ankle servos of left tripod
    robot.move_by([robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], [k_mov, k_mov, k_mov], [kl_speed, kl_speed, kl_speed], [0, 0, 0])
    time.sleep(delay_gait)
    
    # Move base servos of left tripod
    robot.move_by([robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], [R2_delta_start, L1_delta_start, L3_delta_start], [base_speed, base_speed2_start, base_speed], [base_accel, base_accel, base_accel])
    time.sleep(delay_gait)
    
    # Move ankle servos of left tripod
    robot.move_by([robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], [-1*k_mov, -1*k_mov, -1*k_mov], [kp_speed, kp_speed, kp_speed], [0, 0, 0])
    
    time.sleep(delay_trans)
    
    ### Start gait
    print('Starting gait...')
    
    for k in range(steps):
        # Lift ankle servos of left tripod
        robot.move_by([robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], [k_mov, k_mov, k_mov], [kl_speed, kl_speed, kl_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Move bases
        robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, \
                        robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                        [L2_delta, R1_delta, R3_delta, R2_delta, L1_delta, L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Plant ankle servos of left tripod
        robot.move_by([robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], [-1*k_mov, -1*k_mov, -1*k_mov], [kp_speed, kp_speed, kp_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Lift ankle servos of right tripod
        robot.move_by([robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], [k_mov, k_mov, k_mov], [kl_speed, kl_speed, kl_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        # Move bases
        robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, \
                        robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                        [-1*L2_delta, -1*R1_delta, -1*R3_delta, -1*R2_delta, -1*L1_delta, -1*L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Plant ankle servos of right tripod
        robot.move_by([robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], [-1*k_mov, -1*k_mov, -1*k_mov], [kp_speed, kp_speed, kp_speed], [0, 0, 0])
        time.sleep(delay_gait)
        
        

def forward_gait_V3(robot, al_speed, ap_speed, a_mov, kl_speed, kp_speed, k_mov, delta_sl, max_dl, base_speed, base_accel, delay_gait, delay_trans, steps, init_only=False):
    ### This is a 6-phase forwards gait for the 18-dof robot, in which  both the knee and ankle joints move up into the body during swing
    
    # robot:        Instance of robot class from Sebastian_library_Maeastro.py
    # al_speed:     Speed of ankle servos when lifting
    # ap_speed:     Speed of ankle servos when planting
    # a_mov:        Ankle movement length (in quarter-microseconds)
    # kl_speed:     Speed of knee servos when lifting
    # kp_speed:     Speed of knee servos when planting
    # k_mov:        Knee movement length (in quarter-microseconds)
    # delta_sl:     Half of angle change of single-leg side of tripod base servos (degrees)
    # max_dl:       Maximum angle of double-leg side of tripod base servos from horizontal
    # base_speed:   Speed of all base servos
    # base_accel:   Acceleration of all base servos
    # delay_gait:   Delay between phases of gait
    # delay_trans:  Delay between transition from starting position to gait
    # steps:        Number of steps to take
    
    ### Calculate minimum angle of double-leg side of tripod base servos from horizontal
    min_dl = round(np.arcsin(np.sin(max_dl*np.pi/180) - 2*np.sin(delta_sl*np.pi/180))*180/np.pi, 1)
    
    ### Check if collision between single-side leg and double-side leg of opposite tripod
    if min_dl - delta_sl < 10:
        print('Potential collision detected; adjust stride parameters')
        return
    
    ### Calculate ratio of single-leg to double-leg side base servo movements
    sl_to_dl = (2*delta_sl)/(max_dl - min_dl)
    
    ### Convert degree change to microseconds
    L2_delta_start = -1*int(delta_sl*10)
    R1_delta_start = int((max_dl - 45)*10)
    R3_delta_start = int((45 - min_dl)*10)
    R2_delta_start = -1*int(delta_sl*10)
    L1_delta_start = int((45 - min_dl)*10)
    L3_delta_start = int((max_dl - 45)*10)
    
    L2_delta = int(2*delta_sl*10)
    R1_delta = -1*int((max_dl - min_dl)*10)
    R3_delta = -1*int((max_dl - min_dl)*10)
    R2_delta = int(2*delta_sl*10)
    L1_delta = -1*int((max_dl - min_dl)*10)
    L3_delta = -1*int((max_dl - min_dl)*10)
    
    ### Calculate speeds for base servos
    base_speed2_start = int(base_speed/(R1_delta_start/R3_delta_start))
    base_speed2 = int(base_speed/sl_to_dl)
    
    if init_only == True:
        ### Move legs to starting position
        print('Moving legs to starting position...')
        
        
        # Lift knee servos of right tripod
        robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], \
                        [a_mov, a_mov, a_mov, k_mov, k_mov, k_mov], [al_speed, al_speed, al_speed, kl_speed, kl_speed, kl_speed], [0, 0, 0, 0, 0, 0])
        time.sleep(delay_gait)
        
        # Move base servos of right tripod
        robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo], [L2_delta_start, R1_delta_start, R3_delta_start], [base_speed, base_speed, base_speed2_start], [base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Move ankle servos of right tripod
        robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], \
                        [-1*a_mov, -1*a_mov, -1*a_mov, -1*k_mov, -1*k_mov, -1*k_mov], [ap_speed, ap_speed, ap_speed, kp_speed, kp_speed, kp_speed], [0, 0, 0, 0, 0, 0])
        time.sleep(delay_gait)
        
        # Move ankle servos of left tripod
        robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], \
                        [a_mov, a_mov, a_mov, k_mov, k_mov, k_mov], [al_speed, al_speed, al_speed, kl_speed, kl_speed, kl_speed], [0, 0, 0, 0, 0, 0])
        time.sleep(delay_gait)
        
        # Move base servos of left tripod
        robot.move_by([robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], [R2_delta_start, L1_delta_start, L3_delta_start], [base_speed, base_speed2_start, base_speed], [base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Move ankle servos of left tripod
        robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], \
                        [-1*a_mov, -1*a_mov, -1*a_mov, -1*k_mov, -1*k_mov, -1*k_mov], [ap_speed, ap_speed, ap_speed, kp_speed, kp_speed, kp_speed], [0, 0, 0, 0, 0, 0])
        
        time.sleep(delay_trans)
    else:
        ### Start gait
        
        for k in range(steps):
            # Lift ankle and knee servos of left tripod
            robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], \
                            [a_mov, a_mov, a_mov, k_mov, k_mov, k_mov], [al_speed, al_speed, al_speed, kl_speed, kl_speed, kl_speed], [0, 0, 0, 0, 0, 0])
            time.sleep(delay_gait)
            
            # Move bases
            robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                            [L2_delta, R1_delta, R3_delta, R2_delta, L1_delta, L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
            time.sleep(delay_gait)
            
            # Plant ankle and knee servos of left tripod
            robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], \
                            [-1*a_mov, -1*a_mov, -1*a_mov, -1*k_mov, -1*k_mov, -1*k_mov], [ap_speed, ap_speed, ap_speed, kp_speed, kp_speed, kp_speed], [0, 0, 0, 0, 0, 0])
            time.sleep(delay_gait)
            
            # Lift ankle and knee servos of right tripod
            robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], \
                            [a_mov, a_mov, a_mov, k_mov, k_mov, k_mov], [al_speed, al_speed, al_speed, kl_speed, kl_speed, kl_speed], [0, 0, 0, 0, 0, 0])
            time.sleep(delay_gait)
            
            # Move bases
            robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                            [-1*L2_delta, -1*R1_delta, -1*R3_delta, -1*R2_delta, -1*L1_delta, -1*L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
            time.sleep(delay_gait)
            
            # Plant ankle and knee servos of right tripod
            robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], \
                            [-1*a_mov, -1*a_mov, -1*a_mov, -1*k_mov, -1*k_mov, -1*k_mov], [ap_speed, ap_speed, ap_speed, kp_speed, kp_speed, kp_speed], [0, 0, 0, 0, 0, 0])
            time.sleep(delay_gait)
        
        

def forward_gait_V3_turning(robot, al_speed, ap_speed, a_mov, kl_speed, kp_speed, k_mov, delta_sl_long, delta_sl_short, max_dl_long, max_dl_short, long_ID, base_speed, base_accel, delay_gait, delay_trans, steps, init_only=False):
    ### This is a 6-phase forwards gait for the 18-dof robot, in which  both the knee and ankle joints move up into the body during swing
    
    # robot:            Instance of robot class from Sebastian_library_Maeastro.py
    # al_speed:         Speed of ankle servos when lifting
    # ap_speed:         Speed of ankle servos when planting
    # a_mov:            Ankle movement length (in quarter-microseconds)
    # kl_speed:         Speed of knee servos when lifting
    # kp_speed:         Speed of knee servos when planting
    # k_mov:            Knee movement length (in quarter-microseconds)
    # delta_sl_long:    Half of angle change of single-leg side of tripod base servos (degrees), longer moving tripod
    # delta_sl_short:   Half of angle change of single-leg side of tripod base servos (degrees), shorter moving tripod
    # max_dl_long:      Maximum angle of double-leg side of tripod base servos from horizontal, longer moving tripod
    # max_dl_short      Maximum angle of double-leg side of tripod base servos from horizontal, shorter moving tripod
    # long_ID:          ID for which tripod is longer moving; 0 == left, 1 == right
    # base_speed:       Speed of all base servos for longer moving tripod
    # base_accel:       Acceleration of all base servos
    # delay_gait:       Delay between phases of gait
    # delay_trans:      Delay between transition from starting position to gait
    # steps:            Number of steps to take
    
    ### Calculate minimum angle of double-leg side of tripod base servos from horizontal
    min_dl_short = round(np.arcsin(np.sin(max_dl_short*np.pi/180) - 2*np.sin(delta_sl_short*np.pi/180))*180/np.pi, 1)
    min_dl_long = round(np.arcsin(np.sin(max_dl_long*np.pi/180) - 2*np.sin(delta_sl_long*np.pi/180))*180/np.pi, 1)
    
    ### Check if collision between single-side leg and double-side leg of opposite tripod
    if min_dl_short - delta_sl_short < 5:
        print('Potential collision detected; adjust stride parameters')
        return
    
    if min_dl_long - delta_sl_long < 5:
        print('Potential collision detected; adjust stride parameters')
        return
        
        
    ### Calculate ratio of single-leg to double-leg side base servo movements
    #sl_to_dl_short = (2*delta_sl_short)/(max_dl_short - min_dl_short)
    sl_to_dl = (2*delta_sl_long)/(max_dl_long - min_dl_long)
    
    ### Convert degree change to microseconds
    if long_ID == "left":
        L2_delta_start = -1*int(delta_sl_short*10)
        R1_delta_start = int((max_dl_short - 45)*10)
        R3_delta_start = int((45 - min_dl_short)*10)
        R2_delta_start = -1*int(delta_sl_long*10)
        L1_delta_start = int((45 - min_dl_long)*10)
        L3_delta_start = int((max_dl_long - 45)*10)
        
        L2_delta = int(2*delta_sl_short*10)
        R1_delta = -1*int((max_dl_short - min_dl_short)*10)
        R3_delta = -1*int((max_dl_short - min_dl_short)*10)
        R2_delta = int(2*delta_sl_long*10)
        L1_delta = -1*int((max_dl_long - min_dl_long)*10)
        L3_delta = -1*int((max_dl_long - min_dl_long)*10)
    elif long_ID == "right":
        L2_delta_start = -1*int(delta_sl_long*10)
        R1_delta_start = int((max_dl_long - 45)*10)
        R3_delta_start = int((45 - min_dl_long)*10)
        R2_delta_start = -1*int(delta_sl_short*10)
        L1_delta_start = int((45 - min_dl_short)*10)
        L3_delta_start = int((max_dl_short - 45)*10)
        
        L2_delta = int(2*delta_sl_long*10)
        R1_delta = -1*int((max_dl_long - min_dl_long)*10)
        R3_delta = -1*int((max_dl_long - min_dl_long)*10)
        R2_delta = int(2*delta_sl_short*10)
        L1_delta = -1*int((max_dl_short - min_dl_short)*10)
        L3_delta = -1*int((max_dl_short - min_dl_short)*10)
        
    
    ### Calculate speeds for base servos
    if long_ID == "left":
        base_speed2_start = int(base_speed/(L1_delta_start/L3_delta_start))
    elif long_ID == "right":
        base_speed2_start = int(base_speed/(R1_delta_start/R3_delta_start))
        
    base_speed2 = int(base_speed/sl_to_dl)
    
    if init_only == True:
        ### Move legs to starting position
        print('Moving legs to starting position...')
        
        
        # Lift knee servos of right tripod
        robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], \
                        [a_mov, a_mov, a_mov, k_mov, k_mov, k_mov], [al_speed, al_speed, al_speed, kl_speed, kl_speed, kl_speed], [0, 0, 0, 0, 0, 0])
        time.sleep(delay_gait)
        
        # Move base servos of right tripod
        robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo], [L2_delta_start, R1_delta_start, R3_delta_start], [base_speed, base_speed, base_speed2_start], [base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Move ankle servos of right tripod
        robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], \
                        [-1*a_mov, -1*a_mov, -1*a_mov, -1*k_mov, -1*k_mov, -1*k_mov], [ap_speed, ap_speed, ap_speed, kp_speed, kp_speed, kp_speed], [0, 0, 0, 0, 0, 0])
        time.sleep(delay_gait)
        
        # Move ankle servos of left tripod
        robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], \
                        [a_mov, a_mov, a_mov, k_mov, k_mov, k_mov], [al_speed, al_speed, al_speed, kl_speed, kl_speed, kl_speed], [0, 0, 0, 0, 0, 0])
        time.sleep(delay_gait)
        
        # Move base servos of left tripod
        robot.move_by([robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], [R2_delta_start, L1_delta_start, L3_delta_start], [base_speed, base_speed2_start, base_speed], [base_accel, base_accel, base_accel])
        time.sleep(delay_gait)
        
        # Move ankle servos of left tripod
        robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], \
                        [-1*a_mov, -1*a_mov, -1*a_mov, -1*k_mov, -1*k_mov, -1*k_mov], [ap_speed, ap_speed, ap_speed, kp_speed, kp_speed, kp_speed], [0, 0, 0, 0, 0, 0])
        
        time.sleep(delay_trans)
    else:
        ### Start gait
        
        for k in range(steps):
            # Lift ankle and knee servos of left tripod
            robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], \
                            [a_mov, a_mov, a_mov, k_mov, k_mov, k_mov], [al_speed, al_speed, al_speed, kl_speed, kl_speed, kl_speed], [0, 0, 0, 0, 0, 0])
            time.sleep(delay_gait)
            
            # Move bases
            robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                            [L2_delta, R1_delta, R3_delta, R2_delta, L1_delta, L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
            time.sleep(delay_gait)
            
            # Plant ankle and knee servos of left tripod
            robot.move_by([robot.leg_R2.ankle_servo, robot.leg_L1.ankle_servo, robot.leg_L3.ankle_servo, robot.leg_R2.knee_servo, robot.leg_L1.knee_servo, robot.leg_L3.knee_servo], \
                            [-1*a_mov, -1*a_mov, -1*a_mov, -1*k_mov, -1*k_mov, -1*k_mov], [ap_speed, ap_speed, ap_speed, kp_speed, kp_speed, kp_speed], [0, 0, 0, 0, 0, 0])
            time.sleep(delay_gait)
            
            # Lift ankle and knee servos of right tripod
            robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], \
                            [a_mov, a_mov, a_mov, k_mov, k_mov, k_mov], [al_speed, al_speed, al_speed, kl_speed, kl_speed, kl_speed], [0, 0, 0, 0, 0, 0])
            time.sleep(delay_gait)
            
            # Move bases
            robot.move_by([robot.leg_L2.base_servo, robot.leg_R1.base_servo, robot.leg_R3.base_servo, robot.leg_R2.base_servo, robot.leg_L1.base_servo, robot.leg_L3.base_servo], \
                            [-1*L2_delta, -1*R1_delta, -1*R3_delta, -1*R2_delta, -1*L1_delta, -1*L3_delta], [base_speed, base_speed2, base_speed2, base_speed, base_speed2, base_speed2], [base_accel, base_accel, base_accel, base_accel, base_accel, base_accel])
            time.sleep(delay_gait)
            
            # Plant ankle and knee servos of right tripod
            robot.move_by([robot.leg_L2.ankle_servo, robot.leg_R1.ankle_servo, robot.leg_R3.ankle_servo, robot.leg_L2.knee_servo, robot.leg_R1.knee_servo, robot.leg_R3.knee_servo], \
                            [-1*a_mov, -1*a_mov, -1*a_mov, -1*k_mov, -1*k_mov, -1*k_mov], [ap_speed, ap_speed, ap_speed, kp_speed, kp_speed, kp_speed], [0, 0, 0, 0, 0, 0])
            time.sleep(delay_gait)

def get_gait_params(params_string):
    ### This function gets the gait paramters from gait_parameters.py based on the string input
    
    # params_string:    String used to identify set of gait parameters to load and use
    
    if params_string == 'forward_gait_V1_validation':
        from med_Sebastian_info import reset3_pos_info
        from gait_parameters import forward_gait_V1_validation
        return reset3_pos_info, forward_gait_V1_validation
    else:
        return 0, 0
