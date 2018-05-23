'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import *

import time

#calculates spline curve using times(argument x) and keys(argument y) in our dataset, with help of pseudo code from wikipedia
#and returns approximated output value of the function with a given time
def cubic_spline(x, y, time):
    if (time == 0.0):
        return 0.0
    #returning the last angle if the time is past keyframe
    if (time > x[len(x)-1]):
        #print("Done")
        return y[len(y)-1][0]
    #coefficients(a,b,c,d) and helper variables(u,h,l,z)
    a = []
    b,d,u = [],[],[]
    h,c,l,z = [],[],[],[]
    for i in range(len(x)):
        a.append(y[i][0])
        #print(y[i][0])
        c.append(0)
        l.append(0)
        z.append(0)
    for i in range(len(x)-1):
        h.append(x[i+1]-x[i])
        b.append(0)
        d.append(0)
        u.append(0)
    alpha = []
    alpha.append(0)
    for i in range(1,len(x)-1):
        alpha_i = (3/h[i])*(a[i+1]-a[i]) - (3/h[i-1])*(a[i]-a[i-1])
        alpha.append(alpha_i)
    l[0] = 1
    for i in range(1,len(x)-1):
        l[i] = (2*(x[i+1]-x[i-1])-h[i-1]*u[i-1])
        u[i] = (h[i]/l[i])
        z[i] = ((a[i]-h[i-1]*z[i-1])/l[i])
    l[len(x)-1] = 1
    for j in range(len(x)-2,-1, -1):
        c[j] = z[j]- u[j]*c[j+1]
        b[j] = ((a[j+1]-a[j])/h[j]) - ((h[j]*(c[j+1]+2*c[j]))/3)
        d[j] = (c[j+1] - c[j]) / (3 * h[j])
    #p(x) = ai + bi(x-xi) + ci(x-xi)cubed + di(x-xi)tripled
    for i in range(len(x)-1):
        if( time < x[i+1]):
            xi = x[i]
            
            #returning the approximated angle
            return a[i] + b[i]*(time-xi) + c[i]*(time-xi)*(time-xi) + d[i]*(time-xi)*(time-xi)*(time-xi)

    return y[len(y)-1][0]

start = -1


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)


    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        #setting motion start time
        global start
        if( start == -1):
            start = perception.time
        time = perception.time - start 
        names, times, keys = keyframes
        #iterating name list
        for i in range(len(names)):
            #cubic_spline() function calculates  
            spline = cubic_spline(times[i], keys[i], time)
            target_joints[names[i]] = spline
            #print(spline)

        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    # hello and wipe_forehead seems to be working,
    # at standing up the movement seems ok, but cant quite stand up, 
    # the functions might be too slow for advanced movements
    agent.keyframes = hello()
    #agent.keyframes = leftBackToStand()
    #agent.keyframes = leftBellyToStand()
    #agent.keyframes = rightBackToStand()
    #agent.keyframes = wipe_forehead(1)
    agent.run()
