############################
# Imports
############################

USE_TEX = False

# For numerical maths and robotics functions
import numpy as np
from math import *
import modern_robotics as mr

# For plotting and animation
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.animation import FuncAnimation
# matplotlib.rcParams['text.usetex'] = USE_TEX

# Shorthands for functions
from numpy.linalg import inv

#######################################
# Pre-defined constants
#######################################

# Define constants
L1 = 1.0        # m
L2 = 1.0        # m
r1 = L1 / 2.0   # m
r2 = L2 / 2.0   # m
m1 = 3.0        # kg
m2 = 2.0        # kg
I1 = 2.0        # kg m^2
I2 = 1.0        # kg m^2

# End-effector inertias
G1 = np.diag([0, 0, I1, m1, m1, m1])
G2 = np.diag([0, 0, I2, m2, m2, m2])

# Zero pose of frame at space frames
C0 = np.array([
    [ 1, 0, 0, 0 ],
    [ 0, 1, 0, 0 ],
    [ 0, 0, 1, 0 ],
    [ 0, 0, 0, 1 ]
])

# Zero pose of frame at joint 1
C1 = np.array([
    [ 1, 0, 0, L1 ],
    [ 0, 1, 0,  0 ],
    [ 0, 0, 1,  0 ],
    [ 0, 0, 0,  1 ]
])

# Zero pose of frame at joint 2
C2 = np.array([
    [ 1, 0, 0, L1 + L2 ],
    [ 0, 1, 0,       0 ],
    [ 0, 0, 1,       0 ],
    [ 0, 0, 0,       1 ]
])

# Zero pose of space frame
M0 = np.array([
    [ 1, 0, 0, 0 ],
    [ 0, 1, 0, 0 ],
    [ 0, 0, 1, 0 ],
    [ 0, 0, 0, 1 ]
])

# Zero pose of centre of mass 1
M1 = np.array([
    [ 1, 0, 0, L1 / 2 ],
    [ 0, 1, 0,      0 ],
    [ 0, 0, 1,      0 ],
    [ 0, 0, 0,      1 ]
])

# Zero pose of centre of mass 2
M2 = np.array([
    [ 1, 0, 0, L1 + L2 / 2 ],
    [ 0, 1, 0,           0 ],
    [ 0, 0, 1,           0 ],
    [ 0, 0, 0,           1 ]
])

# Zero pose of body frame
M3 = np.array([
    [ 1, 0, 0, L1 + L2 ],
    [ 0, 1, 0,       0 ],
    [ 0, 0, 1,       0 ],
    [ 0, 0, 0,       1 ]
])

# Generate frame-to-frame transformations
M01 = mr.TransInv(M0) @ M1
M12 = mr.TransInv(M1) @ M2
M23 = mr.TransInv(M2) @ M3

steps = 900
dt = 1 / 30.0
T = (steps - 1) * dt

# Screw Axes
S1 = np.array([ 0, 0, 1,   0, 0, 0 ])
S2 = np.array([ 0, 0, 1, 0, -L1, 0 ])

# Screw Axes relative to frame Mi
A1 = mr.Adjoint(mr.TransInv(M1)) @ S1
A2 = mr.Adjoint(mr.TransInv(M2)) @ S2

# Gravity
# g = np.array([0, -9.8, 0]) # TODO: CHANGE TO REMOVE GRAVITY
g = np.array([0, 0, 0])

# List of constants to pass into algorithm
Mlist = np.array([M01, M12, M23])
Glist = np.array([G1, G2])
Slist = np.array([S1, S2]).T


def MassMatrix(thetalist, Mlist, Glist, S):
    """
    Construct the mass matrix D column-by-column
    """
    n = len(thetalist)
    # Set joint velocity, gravity and end-effector wrench to zero
    dthetalist = [0] * n
    g = [0] * 3
    Ftip = [0] * 6

    # Initialise D matrix with zeros
    D = np.zeros((n, n))
    for i in range(n):
        # Populate each column of D
        # bysetting ddtheta to the basis vectors,
        # and setting other terms to zero.
        ddthetalist = [0] * n
        ddthetalist[i] = 1
        D[:, i] = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
    return D    

def VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist):
    """
    Calculate the velocity quadratic forces
    """
    n = len(thetalist)
    # Set joint acceleration, end-effector wrench and gravity to zero
    ddthetalist = [0] * n
    Ftip = [0] * 6
    g = [0] * 3
    # Calculate velocity quadratic joint torques
    hlist = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
    return hlist

def GravityForces(thetalist, g, Mlist, Glist, Slist):
    """
    Calculate the gravity forces
    """
    n = len(thetalist)
    # Set joint velocity, joint acceleration, and end-effector wrench to zero.
    dthetalist = [0] * n
    ddthetalist = [0] * n
    Ftip = [0] * 6
    # Calculate gravity joint torques
    glist = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
    return glist

def EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist):
    """
    Calculate end-effector forces
    """
    n = len(thetalist)
    # Set joint velocity, joint acceleration and gravity to zero
    dthetalist = [0] * n
    ddthetalist = [0] * n
    g = [0] * 3
    # Calculate joint torques generated by end-effector
    tautiplist = mr.InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist)
    return tautiplist

def ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist):
    """
    Calculate joint acceleration
    """
    Dmat =  MassMatrix(thetalist, Mlist, Glist, Slist)
    clist = VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
    glist = GravityForces(thetalist, g, Mlist, Glist, Slist)
    tautiplist = EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist)
    ddthetalist = inv(Dmat) @ (taulist - clist - glist - tautiplist)
    return ddthetalist

def EulerStep(thetalist, dthetalist, ddthetalist, dt):
    return (thetalist + dt * np.array(dthetalist), dthetalist + dt * np.array(ddthetalist))

def GetPID(Kp, Ki, Kd, e, eint, de, dt):
    taulist = Kp @ e + Ki @ eint * dt + Kd @ (de / dt)
    return taulist

def GetAdjustment(prefp, dpactp, pact, k=5):
    # print(np.linalg.norm(dpactp))
    return k/max(1, np.linalg.norm(dpactp))*(pact-prefp)

def GetpRefPD(prefp, adjustment):
    return prefp +  adjustment# TODO: Pole from k/dpactp for very small dpactp, need a mapping of velocity to force

# def SimulatePID(theta0list, dtheta0list, thetalistd, dthetalistd, g, Ftipmat, Mlist, Glist, Slist, dt):
#     n = len(theta0list)
#     N = Ftipmat.shape[0]
#     Ftipmat = np.array(Ftipmat).T
#     thetamat = np.zeros((N, n))
#     dthetamat = np.zeros((N, n))
#     dthetamat = np.zeros((N, n))
#     thetamat[0, :] = theta0list
#     dthetamat[0, :] = dtheta0list
#     eint = np.array([0.0, 0.0])
#     for i in range(N - 1):
#         e = thetalistd - thetamat[i, :]
#         eint += e
#         de = dthetalistd - dthetamat[i, :]
#         taulist = GetPID(Kp, Ki, Kd, e, eint, de, dt) # TODO: CHANGE TAULIST TO BE CONTANTS
#         ddthetalist = ForwardDynamics(thetamat[i, :], dthetamat[i, :], taulist, g, Ftipmat[:, i], Mlist, Glist, Slist) # TODO: CHANGE FTIPMAT TO BE PD CONTROLLED
#         # One euler step
#         thetamat[i + 1, :], dthetamat[i + 1, :] = EulerStep(thetamat[i, :], dthetamat[i, :], ddthetalist, dt)
#         # Change angles to lie between -pi and pi
#         thetamat[i + 1, :] = -((pi - thetamat[i + 1, :]) % (2.0 * pi) - pi)
        
#     return thetamat, dthetamat


N = steps
prefmat = np.zeros((N, 2))
dprefmat = np.zeros((N, 2))
pactmat = np.zeros((N, 2))
dpactmat = np.zeros((N, 2))
dprefmat = np.zeros((N, 2))

def SimulatePID(theta0list, dtheta0list, thetalistd, dthetalistd, g, Ftipmat, Mlist, Glist, Slist, dt):
    n = len(theta0list)
    N = Ftipmat.shape[0]
    Ftipmat = np.array(Ftipmat).T
    thetamat = np.zeros((N, n))
    dthetamat = np.zeros((N, n))
    # prefmat = np.zeros((N, 2))
    # dprefmat = np.zeros((N, 2))
    # pactmat = np.zeros((N, 2))
    # dpactmat = np.zeros((N, 2))
    # dprefmat = np.zeros((N, 2))
    thetamat[0, :] = theta0list
    dthetamat[0, :] = dtheta0list
    eint = np.array([0.0, 0.0])
    for i in range(N - 1):
        T2ref = mr.FKinSpace(C2, Slist[:, :2], thetamat[i, :2])
        if i == 0:
            dprefmat[i, :] = [0, 0]
            dpactmat[i, :] = [0, 0]
            pactmat[i, :] = np.array([T2ref[0, 3], T2ref[1, 3]])
            prefmat[i, :] = np.array([T2ref[0, 3], T2ref[1, 3]])
        else:
            dprefmat[i, :] = (prefmat[i, :] - prefmat[i-1, :])/dt# [x/dt for x in (prefmat[i] - prefmat[i-1])]
            dpactmat[i, :] = (pactmat[i, :] - pactmat[i-1, :])/dt
        pactmat[i, :] = np.array([T2ref[0, 3], T2ref[1, 3]])
        prefmat[i+1, :] = GetpRefPD(prefmat[i, :], GetAdjustment(prefmat[i, :], dpactmat[i, :], pactmat[i, :]))
        e = thetalistd - thetamat[i, :]
        eint += e
        de = dthetalistd - dthetamat[i, :]
        taulist = [1, 1] #GetPID(Kp, Ki, Kd, e, eint, de, dt) # TODO: CHANGE TAULIST TO BE CONTANTS
        # ddthetalist = ForwardDynamics(thetamat[i, :], dthetamat[i, :], taulist, g, Ftipmat[:, i], Mlist, Glist, Slist) # TODO: CHANGE FTIPMAT TO BE PD CONTROLLED
        ftip = [0 , 0, 0, -15*(pactmat[i, 1] - prefmat[i, 1]), -15*(pactmat[i, 0] - prefmat[i, 0]), 0]
        print(ftip)
        ddthetalist = ForwardDynamics(thetamat[i, :], dthetamat[i, :], taulist, g, ftip, Mlist, Glist, Slist) # TODO: CHANGE FTIPMAT TO BE PD CONTROLLED
        # One euler step
        thetamat[i + 1, :], dthetamat[i + 1, :] = EulerStep(thetamat[i, :], dthetamat[i, :], ddthetalist, dt)
        # Change angles to lie between -pi and pi
        thetamat[i + 1, :] = -((pi - thetamat[i + 1, :]) % (2.0 * pi) - pi)
    print(prefmat[0:50, :])
    print(dpactmat[0:50, :])
    print(pactmat[0:50, :])
    return thetamat, dthetamat


##################################
# Initialise simulation variables
################################

# Desired States
thetalistd = np.array([pi / 3, -pi / 6])
dthetalistd = np.array([0, 0])

# External forces
Ftipmat = np.zeros((steps, 6))

# PID Gain Matrices (Diagonal)
Kp = np.diag([100.0, 0.3])
Ki = np.diag([180.0, 1.5])
Kd = np.diag([0.5, 0.25])

# Initial States
theta0list = np.array([-pi / 4, pi / 4])
dtheta0list = np.array([1.0, -0.5])

# Simulation (returns series of state vectors)
thetamat, dthetamat = SimulatePID(theta0list, dtheta0list, thetalistd, dthetalistd, g, Ftipmat, Mlist, Glist, Slist, dt)

# matplotlib animation function
def animate(i):
    # Actual pose
    T1 = mr.FKinSpace(C1, Slist[:, :1], thetamat[i, :1])
    T2 = mr.FKinSpace(C2, Slist[:, :2], thetamat[i, :2])
    # Desired pose
    T1d = mr.FKinSpace(C1, Slist[:, :1], thetalistd[:1])
    T2d = mr.FKinSpace(C2, Slist[:, :2], thetalistd[:2])
    
    # Origin
    x0 = 0.0
    y0 = 0.0

    x2r = prefmat[i, 0]
    y2r = prefmat[i, 1]

    # Actual positions
    x1 = T1[0, 3]
    y1 = T1[1, 3]
    x2 = T2[0, 3]
    y2 = T2[1, 3]
    # Desired positions
    x1d = T1d[0, 3]
    y1d = T1d[1, 3]
    x2d = T2d[0, 3]
    y2d = T2d[1, 3]
    # Clear the figure
    plt.clf()
    # Set axes to 4:3 ratio
    plt.axes(xlim=(-4, 4), ylim=(-3, 3))
    # Plot desired and actual points
    plt.plot([x2r, x2r+0.0001], [y2r, y2r+0.0001], 'b')
    plt.plot([x0, x1d, x2d], [y0, y1d, y2d], 'r--')
    return plt.plot([x0, x1, x2], [y0, y1, y2], 'k')

fig = plt.figure()
FPS = int(1 / dt)
# Might need to change these depending on supported codecs
anim = FuncAnimation(fig, animate, frames=steps, interval=FPS, blit=True)
FFwriter = animation.FFMpegWriter(FPS, extra_args=['-vcodec', 'h264'])
anim.save(str(FPS) + '_fps.mp4', writer=FFwriter)

# Setup data for plotting
theta1 = thetamat[:, 0]
theta2 = thetamat[:, 1]
dtheta1 = dthetamat[:, 0]
dtheta2 = dthetamat[:, 1]
timestamp = np.linspace(0, T, steps)

plt.close('all')

plt.subplot(2, 1, 1)
if USE_TEX:
    plt.plot(timestamp, theta1, label = r"$\theta_{1}$")
    plt.plot(timestamp, theta2, label = r"$\theta_{2}$")
else:
    plt.plot(timestamp, theta1, label = r"theta1")
    plt.plot(timestamp, theta2, label = r"theta2")

plt.ylim (-pi, pi)
plt.legend(loc = 'lower right')
plt.xlabel("Time")
plt.ylabel("Joint Angles")

plt.subplot(2, 1, 2)
if USE_TEX:
    plt.plot(timestamp, dtheta1, label = r"$\dot{\theta}_{1}$")
    plt.plot(timestamp, dtheta2, label = r"$\dot{\theta}_{2}$")
else:
    plt.plot(timestamp, dtheta1, label = r"dtheta1")
    plt.plot(timestamp, dtheta2, label = r"dtheta2")
plt.ylim (-pi, pi)
plt.legend(loc = 'lower right')
plt.xlabel("Time")
plt.ylabel("Joint Velocities")

plt.show()