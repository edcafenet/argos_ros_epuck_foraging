import math

def quaternion_to_euler_angle(Pose):
    t0 = +2.0 * (Pose.orientation.w * Pose.orientation.x + Pose.orientation.y * Pose.orientation.z)
    t1 = +1.0 - 2.0 * (Pose.orientation.x * Pose.orientation.x + Pose.orientation.y * Pose.orientation.y)
    X = math.degrees(math.atan2(t0, t1))
    
    t2 = +2.0 * (Pose.orientation.w * Pose.orientation.y - Pose.orientation.z * Pose.orientation.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
	
    t3 = +2.0 * (Pose.orientation.w * Pose.orientation.z + Pose.orientation.x * Pose.orientation.y)
    t4 = +1.0 - 2.0 * (Pose.orientation.y * Pose.orientation.y + Pose.orientation.z * Pose.orientation.z)
    Z = math.degrees(math.atan2(t3, t4))
        
    return X, Y, Z

def GetPuckColorBasedOnType(Puck):

    puck_color = ''
    
    if(Puck.type == 1):
        puck_color = 'green'
    elif(Puck.type == 2):
        puck_color = 'red'
    elif(Puck.type == 3):
        puck_color = 'blue'
    elif(Puck.type == 4):
        puck_color = 'yellow'
    elif(Puck.type == 5):
        puck_color = 'magenta'
    elif(Puck.type == 6):
        puck_color = 'cyan'
    elif(Puck.type == 7):
        puck_color = 'white'
    elif(Puck.type == 8):
        puck_color = 'orange'
    
    return puck_color    
       

def GetPuckTypeBasedOnColor(Color):

    if(Color == 'green'):
        puck_type = 1
    elif(Color == 'red'):
        puck_type = 2
    elif(Color == 'blue'):
        puck_type = 3
    elif(Color == 'yellow'):
        puck_type = 4
    elif(Color == 'magenta'):
        puck_type = 5
    elif(Color == 'cyan'):
        puck_type = 6
    elif(Color == 'white'):
        puck_type = 7
    elif(Color == 'orange'):
        puck_type = 8
        
    return puck_type    
