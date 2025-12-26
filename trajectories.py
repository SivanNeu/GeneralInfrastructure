import time
import numpy as np
from common import YAW_COMMAND, unitVec, lissajous_func

def horz_circle(center=np.array([0,0,0]), radius=3, missionAttitudeDirection=None, Vel=1, start_time=None):
    if missionAttitudeDirection is None:
        missionAttitudeDirection = np.array([-1, 0, 0])
    if start_time is None:
        t = time.monotonic()
    else:
        t = time.monotonic() - start_time
    
    A = radius
    B = radius
    C = 0  #0.2

    R=(A+B)/2
    L=2*np.pi*R
    w=Vel/L

    d = np.pi / 2 * 0

    a = 2*np.pi*w  #.1
    b = 2*np.pi*w  #.2
    c = 0*2*np.pi*w*2  #.2

    # % t = linspace(0, 2*pi, 2*pi*100+1);
    # % x = A * sin(a * t + d);
    # % y = B * sin(b * t);
    # % z = alt + C * cos(2 * t);
    # % plot3(x, y, z);

    xd =      np.array([A *         np.sin(a * t + d), B *         np.cos(b * t), C * np.cos(c * t)])+center
    x_dot =  np.array([A * a *      np.cos(a * t + d), B * b *    -np.sin(b * t), C * c * -np.sin(c * t)])
    x_2dot =  np.array([A * a**2 * -np.sin(a * t + d), B * b**2 * -np.cos(b * t), C * c**2 * -np.cos(c * t)])
    x_3dot =  np.array([A * a**3 * -np.cos(a * t + d), B * b**3 *  np.sin(b * t), C * c**3 * np.sin(c * t)])
    x_4dot =  np.array([A * a**4 *  np.sin(a * t + d), B * b**4 *  np.cos(b * t), C * c**4 * np.cos(c * t)])

    b1 = missionAttitudeDirection
    b1_dot = np.array([0,0,0])  #
    b1_2dot = np.array([0,0,0])        # w = 2 * pi / 10
    b1 = np.array([np.cos(w * t), np.sin(w * t), 0])
    # b1d_dot = w * np.array([-sin(w * t), cos(w * t), 0])
    # b1d_2dot = w**2 * np.array([-cos(w * t), -sin(w * t), 0])

    pos_control = [False, False, True]
    vel_control = [True, True, False]
    yaw_control = YAW_COMMAND.DEFINED_DIR
    return (xd, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)

def pos_point(missionPoint=[np.array([0, 0, -30])], missionAttitudeDirection=None, start_time=None):
    if missionAttitudeDirection is None:
        missionAttitudeDirection = np.array([-1, 0, 0])
    if start_time is None:
        t = time.monotonic()
    else:
        t = time.monotonic() - start_time
    
    x = missionPoint[0]
    if len(missionPoint) > 1:
        T = 10
        w = 2 * np.pi / T
        state = np.sin(w*t)
        x = missionPoint[0] + np.sign(state) * (missionPoint[1] - missionPoint[0])

    x_dot = np.array([0,0,0])
    x_2dot = np.array([0,0,0])
    x_3dot = np.array([0,0,0])
    x_4dot = np.array([0,0,0])

    b1 = missionAttitudeDirection
    b1_dot = np.array([0, 0, 0])
    b1_2dot = np.array([0, 0, 0])
    
    pos_control = [True, True, True]
    vel_control = [False, False, False]
    yaw_control = YAW_COMMAND.DEFINED_DIR
    return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)

def vel_point(missionVelocity=np.array([1, 0, 0]), missionAttitudeDirection=None, start_time=None):
    if missionAttitudeDirection is None:
        missionAttitudeDirection = np.array([-1, 0, 0])
    if start_time is None:
        t = time.monotonic()
    else:
        t = time.monotonic() - start_time
    
    x = np.array([0, 0, 0])
    x_dot = missionVelocity
    x_2dot = np.array([0,0,0])
    x_3dot = np.array([0,0,0])
    x_4dot = np.array([0,0,0])

    b1 = missionAttitudeDirection
    b1_dot = np.array([0, 0, 0])
    b1_2dot = np.array([0, 0, 0])
    
    pos_control = [False, False, False]
    vel_control = [True, True, True]
    yaw_control = YAW_COMMAND.DEFINED_DIR
    return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)

def lineConstVel(startPoint, endPoint, speed, startTime, missionAttitudeDirection=None):
    if missionAttitudeDirection is None:
        missionAttitudeDirection = np.array([-1, 0, 0])
        
    finished = False
    t=time.monotonic()-startTime
    
    deltaDistance = endPoint-startPoint
    deltaDir = unitVec(deltaDistance)
    deltaDistance = np.linalg.norm(deltaDistance)
    velocity = deltaDir * speed
    x = startPoint + t*velocity
    x_dot = velocity
    if t*speed > deltaDistance:
        x = endPoint
        x_dot = np.array([0,0,0])
        finished = True
    x_2dot = np.array([0,0,0])
    x_3dot = np.array([0,0,0])
    x_4dot = np.array([0,0,0])

    b1 = missionAttitudeDirection
    b1_dot = np.array([0, 0, 0])
    b1_2dot = np.array([0, 0, 0])
    
    pos_control = [True, True, True]
    vel_control = [t*speed < deltaDistance, t*speed < deltaDistance, t*speed < deltaDistance]
    yaw_control = YAW_COMMAND.DEFINED_DIR
    return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control), finished

def vert_circle(start_time=None):
    if start_time is None:
        t = time.monotonic()
    else:
        t = time.monotonic() - start_time
    
    Vel=3

    A = 5
    B = 5
    C = 0  #0.2

    R=(A+B)/2
    L=2*np.pi*R
    w=Vel/L

    d = np.pi / 2 * 0

    a = 2*np.pi*w  #.1
    b = 2*np.pi*w  #.2
    c = 2*np.pi*w*2  #.2
    alt = -20

    # % t = linspace(0, 2*pi, 2*pi*100+1);
    # % x = A * sin(a * t + d);
    # % y = B * sin(b * t);
    # % z = alt + C * cos(2 * t);
    # % plot3(x, y, z);

    x =      np.array([C * np.cos(c * t),         A *         np.sin(a * t + d), B *         np.cos(b * t), alt + C * np.cos(c * t)])
    x_dot =  np.array([C * c * -np.sin(c * t),    A * a *     np.cos(a * t + d), B * b *    -np.sin(b * t)])
    x_2dot =  np.array([C * c**2 * -np.cos(c * t), A * a**2 * -np.sin(a * t + d), B * b**2 * -np.cos(b * t)])
    x_3dot =  np.array([C * c**3 *  np.sin(c * t), A * a**3 * -np.cos(a * t + d), B * b**3 *  np.sin(b * t)])
    x_4dot =  np.array([C * c**4 *  np.cos(c * t), A * a**4 *  np.sin(a * t + d), B * b**4 *  np.cos(b * t)])

    b1 = np.array([1,0,0])
    b1_dot = np.array([0,0,0])  #
    b1_2dot = np.array([0,0,0])        # w = 2 * pi / 10
    # self.b1d = np.array([cos(w * t), sin(w * t), 0])
    # self.b1d_dot = w * np.array([-sin(w * t), cos(w * t), 0])
    # self.b1d_2dot = w**2 * np.array([-cos(w * t), -sin(w * t), 0])

    pos_control = [False, False, False]
    vel_control = [True, True, True]
    yaw_control = YAW_COMMAND.DEFINED_DIR
    return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)

def command_Lissajous(t=None, start_time=None):
    if t is None:
        if start_time is None:
            t = time.monotonic()
        else:
            t = time.monotonic() - start_time
    
    # Lissajous curve
    # x=A*sin(a*t+d), y=B*sin(b*t), z=alt+C*cos(c*t)
    # several common parameters:
    # a:b   d    0       pi/4      pi/2      3/4*pi    pi
    # 1:1        /      ellipse   circle    ellipce    \
    # 1:2        )      8-figure  8-figure  8-figure   (
    # 1:3        S      
    
    A = 15   # X amplitude
    B = 15   # Y amplitude
    C = 5 # Z amplitude
    alt = -1  # Z offset

    d = np.pi / 2 * 0

    a = .2*1.5  # X frequency
    b = .3*1.5  # Y frequency
    c = .2    # Z frequency
    w = 2 * np.pi / 10  # attitude rotation frequency
    (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot) = lissajous_func(t, A=A, B=B, C=C, a=a, b=b, c=c, alt=alt, w = w)
    
    pos_control = [False, False, False]
    vel_control = [True, True, True]
    yaw_control = YAW_COMMAND.DEFINED_DIR
    return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)

