# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# mass, spring constant, initial position and velocity
m = 6.42e23
G0 = 6.674e-11
G=m*G0
x = np.array([4000000,0,0])
v = np.array([0,4500,0])

# simulation time, timestep and time
t_max = 1000
dt = 1

t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []
###
# Euler integration
for t in t_array:

    # append current state to trajectories
    x_list.append(x)
    v_list.append(v)

    # calculate new position and velocity
    a = -G * x /(np.linalg.norm(x))**3
    x = x + dt * v
    v = v + dt * a

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(x_list)
v_array = np.array(v_list)

# plot the position-time graph
#plt.figure(1)
#plt.clf()
#plt.xlabel('time (s)')
#plt.grid()
#plt.plot(t_array, x_array, label='x (m)')
#plt.plot(t_array, v_array, label='v (m/s)')
#plt.legend()
#plt.show()

x_array1 = np.array(x_list)[:,0]
x_array2 = np.array(x_list)[:,1]
# plot the position-time graph
#fig = plt.figure()
#ax = plt.axes(projection="3d")
#ax.plot3D(x_array1, x_array2, x_array3)
#plt.show()
#ax=Axes3D(fig)
#ax.plot(xs=x_array1,ys=x_array2,zs=x_array3)


plt.figure(1)
plt.clf()
plt.grid()
plt.gca().set_aspect('equal', adjustable='box')#change to auto to show descend's graph
plt.plot(x_array1, x_array2)
#plt.plot(t_array,x_array1) #use this if simulating descending
plt.legend()
plt.show()
