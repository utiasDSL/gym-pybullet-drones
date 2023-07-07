import numpy as np

def relative_movement(x0: float, y0: float, z0: float, dx: float, dy: float, dz: float):
    """
    Use: TARGET_POS[i, :] = relative_movement(TARGET_POS[i-1, 0], TARGET_POS[i-1, 1],TARGET_POS[i-1, 2], -0.002, 0.003, 0.004)
    """
    return [x0+dx, y0+dy, z0+dz]

def circle(Radius: float, NUM_WP: int, init_const:float): 
    """
    Radius -- 0.5 of (The Width of a Circle); look at :: https://youtu.be/pnRNAIQAc50 ; 
    init_const -- const value > 0    
    Use:    INIT_XYZ, TARGET_POS = circle(Radius, NUM_WP, init_const)
    """
    INIT_XYZ = np.array([ init_const+ Radius*np.cos(0), init_const + np.sin(0), init_const]).reshape(1,3)
    TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + Radius*np.cos(t), INIT_XYZ[0, 1] + Radius*np.sin(t), INIT_XYZ[0, 2] #+ i/NUM_WP
    return INIT_XYZ, TARGET_POS

def circle_many_drones(num_drones: int, Radius: float, NUM_WP: int, init_const:float): 
    """
    Radius -- 0.5 of (The Width of a Circle); look at :: https://youtu.be/pnRNAIQAc50 ; 
    init_const -- const value > 0 
    num_drones > 1
    Use:    INIT_XYZS, TARGET_POS = circle_many_drones(num_drones, R, NUM_WP, 0.1)
    """
    init_step = 0.1       
    INIT_XYZ = np.array([[init_const+ Radius*np.cos(0), init_const + np.sin(0), init_const+i*init_step] for i in range(num_drones)])
    TARGET_POS = np.zeros((NUM_WP,3))    
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + Radius*np.cos(t), INIT_XYZ[0, 1] + Radius*np.sin(t), INIT_XYZ[0, 2] #+ i/NUM_WP
    return INIT_XYZ, TARGET_POS

def cylinder(Radius: float, NUM_WP: int, init_const:float, height: float): 
    """
    Radius -- 0.5 of (The Width of a Circle); look at :: https://youtu.be/pnRNAIQAc50 ; 
    init_const -- const value > 0    
    height - height of cylinder
    Use:    INIT_XYZ, TARGET_POS = cylinder(Radius, NUM_WP, init_const, height)
    """       
    INIT_XYZ = np.array([ init_const+ Radius*np.cos(0), init_const + np.sin(0), init_const]).reshape(1,3)
    TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + Radius*np.cos(t), INIT_XYZ[0, 1] + Radius*np.sin(t), INIT_XYZ[0, 2] + height*i/NUM_WP
    return INIT_XYZ, TARGET_POS

def cylinder_many_drones(num_drones: int, Radius: float, NUM_WP: int, init_const:float, height: float): 
    """
    Radius -- 0.5 of (The Width of a Circle); look at :: https://youtu.be/pnRNAIQAc50 ; 
    init_const -- const value > 0
    height - height of cylinder
    Use:     INIT_XYZS, TARGET_POS = cylinder_many_drones(num_drones, R, NUM_WP, init_const, height)
    """
    init_step = 0.1       
    INIT_XYZ = np.array([[init_const+ Radius*np.cos(0), init_const + np.sin(0), init_const+i*init_step] for i in range(num_drones)])
    TARGET_POS = np.zeros((NUM_WP,3))   
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + Radius*np.cos(t), INIT_XYZ[0, 1] +  Radius*np.sin(t), INIT_XYZ[0, 2] + height*i/NUM_WP
    return INIT_XYZ, TARGET_POS

def ellipse(Radius_x: float, Radius_y: float, NUM_WP: int, init_const:float): 
    """
    Radius_x -- "A" value in the analytically the equation of a standard ellipse ; look at :: https://en.wikipedia.org/wiki/Ellipse
    Radius_y -- "B" value in the analytically the equation of a standard ellipse ; look at :: https://en.wikipedia.org/wiki/Ellipse
    init_const -- const value > 0    
    Use:   INIT_XYZ, TARGET_POS = ellipse(Radius_x, Radius_y, NUM_WP, init_const)
    """
    INIT_XYZ = np.array([ Radius_x*np.cos(0) - Radius_x, np.sin(0), init_const]).reshape(1,3)
    TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        if i == 0:
            TARGET_POS[i :] = INIT_XYZ[0, 0], INIT_XYZ[0, 1], INIT_XYZ[0, 2]
            continue        
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + Radius_x*np.cos(t), INIT_XYZ[0, 1] + Radius_y*np.sin(t), init_const #+ i/NUM_WP
    return INIT_XYZ, TARGET_POS

def ellipse_many_drones(num_drones: int, Radius_x: float, Radius_y: float, NUM_WP: int, init_const:float): 

    """
    Radius_x -- "A" value in the analytically the equation of a standard ellipse ; look at :: https://en.wikipedia.org/wiki/Ellipse
    Radius_y -- "B" value in the analytically the equation of a standard ellipse ; look at :: https://en.wikipedia.org/wiki/Ellipse
    init_const -- const value > 0   
    Use: INIT_XYZS, TARGET_POS = ps.spiral_many_drones(num_drones, R, NUM_WP, init_const)
    """
    init_step = 0.1       
    INIT_XYZ = np.array([[init_const+ Radius_x*np.cos(0), init_const + np.sin(0), init_const+i*init_step] for i in range(num_drones)])
    TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + Radius_x*np.cos(t), init_const + Radius_y*np.sin(t), init_const #+ i/NUM_WP
    return INIT_XYZ, TARGET_POS

def spiral(Radius: float, NUM_WP: int, init_const:float): 
    """
    Radius -- 0.5 of (The Width of a Circle); look at :: https://youtu.be/pnRNAIQAc50 ; 
    init_const -- const value > 0    
    Use:    INIT_XYZ, TARGET_POS = spiral(Radius, NUM_WP, init_const)
    """
    INIT_XYZ = np.array([ init_const+ Radius*np.cos(0), init_const + np.sin(0), init_const]).reshape(1,3)
    TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + (1-i/NUM_WP) * Radius*np.cos(t), INIT_XYZ[0, 1] + (1-i/NUM_WP) * Radius*np.sin(t), INIT_XYZ[0, 2] 
    return INIT_XYZ, TARGET_POS
   
def spiral_many_drones(num_drones: int, Radius: float, NUM_WP: int, init_const:float): 
    """
    Radius -- 0.5 of (The Width of a Circle); look at :: https://youtu.be/pnRNAIQAc50 ; 
    init_const -- const value > 0    
    Use:    INIT_XYZ, TARGET_POS = spiral_many_drones(num_drones,Radius, NUM_WP, init_const)
    """
    init_step = 0.1       
    INIT_XYZ = np.array([[init_const+ Radius*np.cos(0), init_const + np.sin(0), init_const+i*init_step] for i in range(num_drones)])
    TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + (-i/NUM_WP) * Radius*np.cos(t), INIT_XYZ[0, 1] + (-i/NUM_WP) * Radius*np.sin(t), INIT_XYZ[0, 2] #+ i/NUM_WP
    return INIT_XYZ, TARGET_POS 

def cone(Radius: float, NUM_WP: int, init_const:float, height: float): 
    """
    Radius -- 0.5 of (The Width of a Circle); look at :: https://youtu.be/pnRNAIQAc50 ; 
    init_const -- const value > 0    
    height - height of cone
    Use:    INIT_XYZ, TARGET_POS = cone(Radius, NUM_WP, init_const, height)
    """       
    INIT_XYZ = np.array([ init_const+ Radius*np.cos(0), init_const + np.sin(0), init_const]).reshape(1,3)
    TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + (1-i/NUM_WP) * Radius*np.cos(t), INIT_XYZ[0, 1] + (1-i/NUM_WP) * Radius*np.sin(t), INIT_XYZ[0, 2] + height*i/NUM_WP
    return INIT_XYZ, TARGET_POS

def cone_many_drones(num_drones: int, Radius: float, NUM_WP: int, init_const:float, height: float): 
    """
    Radius -- 0.5 of (The Width of a Circle); look at :: https://youtu.be/pnRNAIQAc50 ; 
    init_const -- const value > 0    
    Use:    INIT_XYZS, TARGET_POS = ps.cone_many_drones(num_drones, R, NUM_WP, init_const, height)
    """
    init_step = 0.1       
    INIT_XYZ = np.array([[init_const+ Radius*np.cos(0), init_const + np.sin(0), init_const+i*init_step] for i in range(num_drones)])
    TARGET_POS = np.zeros((NUM_WP,3))
    for i in range(NUM_WP):
        t = (2*np.pi*i)/NUM_WP
        TARGET_POS[i, :] = INIT_XYZ[0, 0] + (-i/NUM_WP) * Radius*np.cos(t), INIT_XYZ[0, 1] + (-i/NUM_WP) * Radius*np.sin(t), INIT_XYZ[0, 2] + height*i/NUM_WP
    return INIT_XYZ, TARGET_POS

def spherical_to_cartesian(R:float, phi:float, teta:float): # angles in radians
    """
    conversion from Cartesian to spherical coordinate system
    https://en.wikipedia.org/wiki/Spherical_coordinate_system
    """
    x = R*np.cos(teta)*np.sin(phi)
    y = R*np.sin(teta)*np.cos(phi)
    z = R*np.cos(teta)
    return [x, y, z]

def cylindrical_to_cartesian(ro: float, phi: float, z: float): # angles in radians
    """
    conversion from Cartesian to cylindrical coordinates
    https://en.wikipedia.org/wiki/Cylindrical_coordinate_system
    """
    x = ro*np.cos(phi)
    y = ro*np.sin(phi)
    z = z
    return [x, y, z] 
