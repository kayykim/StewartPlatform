print("Hello World!")
import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.special import sph_harm
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D


#test_sph3(6)
l=10;l_11=8;l_12=8;h=10;w=5
a = np.arctan2(h,w)
swh = np.sqrt(w**2+h**2)

def IKin(t_11,t):
    swh = np.sqrt(w**2+h**2)
    LHS = (l_12**2 - l_11**2 -h**2 - w**2 - l**2)/swh
    RHS = -2*l*np.cos(t+a)+2*l_11*np.cos(t_11+a)-2*l*l_11*np.cos(t-t_11)/swh
    return LHS-RHS
def FKin(t,t_11):
    swh = np.sqrt(w**2+h**2)
    LHS = (l_12**2 - l_11**2 -h**2 - w**2 - l**2)/swh
    RHS = -2*l*np.cos(t+a)+2*l_11*np.cos(t_11+a)-2*l*l_11*np.cos(t-t_11)/swh
    return LHS-RHS

print("========================Begin For loop==========================")
LHS = (l_12**2 - l_11**2 -h**2 - w**2 - l**2)/swh
for ang1 in range(-90,90,3):
    alpha=ang1*np.pi/180
    initial_guess = (alpha)*0.8
    t=alpha
    root = fsolve(IKin, initial_guess, t)
    t_11 = root[0]
    numer = (l*np.sin(t)-l_11*np.sin(t_11)+h)
    if numer > l_12:
        #print("Numerator is greater than l_12, skipping this angle numerator: {:.2f} l_12: {:.2f}".format(numer, l_12))
        print("1 = t {:.5f} t_11 {:.5f}".format(np.rad2deg(t),np.rad2deg(t_11)))
    else:
        theta2 = np.pi - np.arctan2(numer,l_12)
        print("1 = t {:.5f} t_11 {:.5f}".format(np.rad2deg(t),np.rad2deg(t_11)),"theta2 {:.5f}".format(np.rad2deg(theta2)))
    root = fsolve(FKin, t, t_11)
    t = root[0]
    RHS = -2*l*np.cos(t+a)+2*l_11*np.cos(t_11+a)-2*l*l_11*np.cos(t-t_11)/swh
    print("2 = t {:.5f} t_11 {:.5f}".format(np.rad2deg(t),np.rad2deg(t_11)))

# Remove z coordinate from all pline definitions
pline = np.array([
    [-l, 0.1],
    [l, 0.1],
    [l, -0.1],
    [-l, -0.1],
    [-l, 0.1]
])

# Update TransformLink for 2D
def TransformLink(pl, alpha, translation):
    Rz = np.array([
        [np.cos(alpha), -np.sin(alpha)],
        [np.sin(alpha),  np.cos(alpha)]
    ])
    pl_transformed = (Rz @ pl.T).T + translation
    return pl_transformed

# Create 2D plot
fig, ax = plt.subplots()
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)

theta = np.deg2rad(22)
pline_transformed = TransformLink(pline, theta, np.array([0, 0]))
line1, = ax.plot(pline_transformed[:, 0], pline_transformed[:, 1], color='blue')
initial_guess = 0.5

root = fsolve(IKin, initial_guess, theta)
theta1 = root[0]
pline1 = np.array([
    [0, 0.1],
    [l_11, 0.1],
    [l_11, -0.1],
    [0, -0.1],
    [0, 0.1]
])
pline1_transformed = TransformLink(pline1, theta1, np.array([w, -h]))
line2, = ax.plot(pline1_transformed[:, 0], pline1_transformed[:, 1], color='red')

pline2 = np.array([
    [0, 0.1],
    [l_12, 0.1],
    [l_12, -0.1],
    [0, -0.1],
    [0, 0.1]
])
numer = (l*np.sin(theta)-l_11*np.sin(theta1)+h)
theta2 = np.pi-(np.arcsin(numer/l_12))
pline2_transformed = TransformLink(pline2, theta2, np.array([
    (pline1_transformed[1,0]+pline1_transformed[2,0])/2,
    (pline1_transformed[1,1]+pline1_transformed[2,1])/2
]))
line3, = ax.plot(pline2_transformed[:, 0], pline2_transformed[:, 1], color='green')

def update(frame):
    alpha = np.deg2rad(((frame % 45)-22))
    initial_guess = 0.5
    root = fsolve(IKin, initial_guess, alpha)
    theta = alpha
    theta1 = root[0]
    theta2 = (np.pi - np.arcsin((l*np.sin(theta)-l_11*np.sin(theta1)+h)/l_12))
    pline_transformed = TransformLink(pline, theta, np.array([0, 0]))
    line1.set_data(pline_transformed[:, 0], pline_transformed[:, 1])
    pline1_transformed = TransformLink(pline1, theta1, np.array([w, -h]))
    line2.set_data(pline1_transformed[:, 0], pline1_transformed[:, 1])
    trans = np.array([
        (pline1_transformed[1,0]+pline1_transformed[2,0])/2,
        (pline1_transformed[1,1]+pline1_transformed[2,1])/2
    ])
    pline2_transformed = TransformLink(pline2, theta2, trans)
    line3.set_data(pline2_transformed[:, 0], pline2_transformed[:, 1])

ani = FuncAnimation(fig, update, frames=1000, interval=100, blit=False)
plt.show()