# Nicole Graf

### Author: Nicole Graf
### Python library of grip-related functions for the 18 DOF Sebastian robot

def small_grip(robot_dims, thetas0, points, dt):
    # This function computes leg angle vs. time relations resulting in linear
    # footpaths for a 3-DOF leg, given initial leg angles and desired foot path 
    # velocities. 
    #
    # robot_dims:       Robot leg dimensions (Python dictionary)
    # thetas0:          Initial leg angles of leg (Python list, units = deg, format: [hip, knee, ankle])
    # vx:               Lateral speed of foot tip path (float, units = mm/s)
    # vy:               Forward speed of foot tip path (float, units = mm/s)
    # vh:               Vertical speed of foot tip path (float, units = mm/s)
    # points:           Number of time points to solve foot path equations (int)
    # dt:               Step size for solving foot path equations (float, units = s)
       
    # Define leg dimensions
        L0 = robot_dims['L0']   # L0 dimension
        L1 = robot_dims['L1']   # L1 dimension
        L2 = robot_dims['L2']   # L2 dimension
    
    # Define constant for converting degrees to radians
        T = np.pi/180
    
    # Define initial conditions
        theta1_0 = thetas0[0] # Hip angle (ϴ1)
        theta2_0 = thetas0[1] # Knee angle (ϴ2)
        theta3_0 = thetas0[2] # Ankle angle (ϴ3)
        
    # Initialize vectors to store values of time, theta1, theta2, and theta3 
        times = np.linspace(0, dt*points, points) # Time
        theta1_vec = np.zeros([points, ])         # Theta1
        theta2_vec = np.zeros([points, ])         # Theta2
        theta3_vec = np.zeros([points, ])         # Theta3
    
    # Initialize theta1, theta2, and theta3 
        theta1 = theta1_0*T
        theta2 = theta2_0*T   
        theta3 = theta3_0*T

        dtheta1 = 1*15*T
        dtheta2 = 1*1*T
        dtheta3 = -1*1*T
   
   ## NG: should not need lines 46-75. only need a line that changes starting theta to desired theta and update thetas
        ##John line 62 on
    for k in range(1, points): # NG: start edits here on
        # Update theta1 according to Euler method
            dtheta1_dt = (-1*vx + np.cos(theta1_vec[k - 1])*(-L1*np.sin(theta2_vec[k - 1])*dtheta2_vec[k - 1] - \
                        L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1])*(dtheta2_vec[k - 1] + \
                        dtheta3_vec[k - 1])))/(np.sin(theta1_vec[k - 1])*(L0 + L1*np.cos(theta2_vec[k - 1]) + \
                        L2*np.cos(theta2_vec[k - 1] + theta3_vec[k - 1])))
            theta1 = theta1 + dt*dtheta1_dt
            theta1_vec[k] = theta1
            dtheta1_vec[k] = dtheta1_dt
        
        # Update theta2 according to Euler method
            dtheta2_dt = ((vy + vx*(1/np.tan(theta1_vec[k - 1])))*np.sin(theta1_vec[k - 1]) + \
                          L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1])*dtheta3_vec[k - 1])/\
                        (-L1*np.sin(theta2_vec[k - 1]) - L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1]))
            theta2 = theta2 + dt*dtheta2_dt
            theta2_vec[k] = theta2
            dtheta2_vec[k] = dtheta2_dt
        
        # Update theta3 according to Euler method
            dtheta3_dt = (vh - dtheta2_vec[k - 1]*(L1*np.cos(theta2_vec[k - 1]) + L2*np.cos(theta2_vec[k - 1] + \
                        theta3_vec[k - 1])))/(L2*np.cos(theta2_vec[k - 1] + theta3_vec[k - 1]))
            theta3 = theta3 + dt*dtheta3_dt
            theta3_vec[k] = theta3
            dtheta3_vec[k] = dtheta3_dt
        
    
        # Convert thetas back to degrees
            theta1_vec = theta1_vec/T
            theta2_vec = theta2_vec/T
            theta3_vec = theta3_vec/T

            #####################################

            ### Author: Nicole Graf

def linear_foot_path_solver(robot_dims, thetas0, vx, vy, vh, points, dt):
    # This function computes leg angle vs. time relations resulting in linear
    # footpaths for a 3-DOF leg, given initial leg angles and desired foot path 
    # velocities. 
    #
    # robot_dims:       Robot leg dimensions (Python dictionary)
    # thetas0:          Initial leg angles of leg (Python list, units = deg, format: [hip, knee, ankle])
    # vx:               Lateral speed of foot tip path (float, units = mm/s)
    # vy:               Forward speed of foot tip path (float, units = mm/s)
    # vh:               Vertical speed of foot tip path (float, units = mm/s)
    # points:           Number of time points to solve foot path equations (int)
    # dt:               Step size for solving foot path equations (float, units = s)
    
    # Define leg dimensions
    L0 = robot_dims['L0']   # L0 dimension
    L1 = robot_dims['L1']   # L1 dimension
    L2 = robot_dims['L2']   # L2 dimension
    
    # Define constant for converting degrees to radians
    T = np.pi/180
    
    # Define initial conditions
    theta1_0 = thetas0[0] # Hip angle (ϴ1)
    theta2_0 = thetas0[1] # Knee angle (ϴ2)
    theta3_0 = thetas0[2] # Ankle angle (ϴ3)
    
    # Initialize vectors to store values of time, theta1, theta2, theta3 and derivatives
    times = np.linspace(0, dt*points, points) # Time
    theta1_vec = np.zeros([points, ])         # Theta1
    theta2_vec = np.zeros([points, ])         # Theta2
    theta3_vec = np.zeros([points, ])         # Theta3
    dtheta1_vec = np.zeros([points, ])        # dTheta1_dt
    dtheta2_vec = np.zeros([points, ])        # dTheta2_dt
    dtheta3_vec = np.zeros([points, ])        # dTheta3_dt
    
    # Initialize theta1, theta2, theta3 and derivatives
    theta1 = theta1_0*T
    theta1_vec[0] = theta1
    theta2 = theta2_0*T
    theta2_vec[0] = theta2
    theta3 = theta3_0*T
    theta3_vec[0] = theta3

    dtheta1 = 1*15*T
    dtheta1_vec[0] = dtheta1
    dtheta2 = 1*1*T
    dtheta2_vec[0] = dtheta2
    dtheta3 = -1*1*T
    dtheta3_vec[0] = dtheta3
    
    for k in range(1, points):
        # Update theta1 according to Euler method
        dtheta1_dt = (-1*vx + np.cos(theta1_vec[k - 1])*(-L1*np.sin(theta2_vec[k - 1])*dtheta2_vec[k - 1] - \
                    L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1])*(dtheta2_vec[k - 1] + \
                    dtheta3_vec[k - 1])))/(np.sin(theta1_vec[k - 1])*(L0 + L1*np.cos(theta2_vec[k - 1]) + \
                    L2*np.cos(theta2_vec[k - 1] + theta3_vec[k - 1])))
        theta1 = theta1 + dt*dtheta1_dt
        theta1_vec[k] = theta1
        dtheta1_vec[k] = dtheta1_dt
        
        # Update theta2 according to Euler method
        dtheta2_dt = ((vy + vx*(1/np.tan(theta1_vec[k - 1])))*np.sin(theta1_vec[k - 1]) + \
                      L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1])*dtheta3_vec[k - 1])/\
                    (-L1*np.sin(theta2_vec[k - 1]) - L2*np.sin(theta2_vec[k - 1] + theta3_vec[k - 1]))
        theta2 = theta2 + dt*dtheta2_dt
        theta2_vec[k] = theta2
        dtheta2_vec[k] = dtheta2_dt
        
        # Update theta3 according to Euler method
        dtheta3_dt = (vh - dtheta2_vec[k - 1]*(L1*np.cos(theta2_vec[k - 1]) + L2*np.cos(theta2_vec[k - 1] + \
                    theta3_vec[k - 1])))/(L2*np.cos(theta2_vec[k - 1] + theta3_vec[k - 1]))
        theta3 = theta3 + dt*dtheta3_dt
        theta3_vec[k] = theta3
        dtheta3_vec[k] = dtheta3_dt
        
    
    # Convert thetas back to degrees
    theta1_vec = theta1_vec/T
    theta2_vec = theta2_vec/T
    theta3_vec = theta3_vec/T
    
    return theta1_vec, theta2_vec, theta3_vec

    def thetas_plotter(theta_vecs):
    # This functions takes a dictionary of theta values for all legs of
    # the robot and plots the theta vs. time relations.
    #
    # theta_vecs:   Python dictionary of theta values for all legs
    
    # Define time vector
    points = theta_vecs['points']
    dt = theta_vecs['dt']
    times = np.linspace(0, dt*points, points)
    
    # Plot theta vs. time relations
    fig, ((ax1, ax2, ax3), (ax4, ax5, ax6), (ax7, ax8, ax9)) = plt.subplots(3, 3, figsize=(20, 20))
    plt.subplots_adjust(hspace=0.3)
    
    keys = ['theta1_R1', 'theta2_R1', 'theta3_R1', 'theta1_L2', 'theta2_L2', 'theta3_L2', \
                'theta1_R3', 'theta2_R3', 'theta3_R3']
    colors = ['blue', 'darkorange', 'green']
                
    for m in range(3):
        eval('ax'+str(3*m + 1)).plot(times, theta_vecs[keys[3*m]], linewidth=2.5, color=colors[m])
        eval('ax'+str(3*m + 1)).set_ylabel('Angle (degrees)', fontsize=14)
        eval('ax'+str(3*m + 1)).set_title(keys[3*m], fontsize=16)
        eval('ax'+str(3*m + 1)).tick_params(axis='x', labelsize=13)
        eval('ax'+str(3*m + 1)).tick_params(axis='y', labelsize=13)
        eval('ax'+str(3*m + 1)).set_xlim([0, dt*points])
        eval('ax'+str(3*m + 1)).set_ylim([np.min(theta_vecs[keys[3*m]][20:]) - 1, np.max(theta_vecs[keys[3*m]][20:]) + 1])
        
        eval('ax'+str(3*m + 2)).plot(times, theta_vecs[keys[3*m + 1]], linewidth=2.5, color=colors[m])
        eval('ax'+str(3*m + 2)).set_title(keys[3*m + 1], fontsize=16)
        eval('ax'+str(3*m + 2)).tick_params(axis='x', labelsize=13)
        eval('ax'+str(3*m + 2)).tick_params(axis='y', labelsize=13)
        eval('ax'+str(3*m + 2)).set_xlim([0, dt*points])
        eval('ax'+str(3*m + 2)).set_ylim([np.min(theta_vecs[keys[3*m + 1]][20:]) - 1, np.max(theta_vecs[keys[3*m + 1]][20:]) + 1])
        
        eval('ax'+str(3*m + 3)).plot(times, theta_vecs[keys[3*m + 2]], linewidth=2.5, color=colors[m])
        eval('ax'+str(3*m + 3)).set_title(keys[3*m + 2], fontsize=16)
        eval('ax'+str(3*m + 3)).tick_params(axis='x', labelsize=13)
        eval('ax'+str(3*m + 3)).tick_params(axis='y', labelsize=13)
        eval('ax'+str(3*m + 3)).set_xlim([0, dt*points])
        eval('ax'+str(3*m + 3)).set_ylim([np.min(theta_vecs[keys[3*m + 2]][20:]) - 1, np.max(theta_vecs[keys[3*m + 2]][20:]) + 1])
        
        if m == 2:
            eval('ax'+str(3*m + 1)).set_xlabel('Time (s)', fontsize=14)
            eval('ax'+str(3*m + 2)).set_xlabel('Time (s)', fontsize=14)
            eval('ax'+str(3*m + 3)).set_xlabel('Time (s)', fontsize=14)
            
        
        
    plt.show()

# Resuming on line