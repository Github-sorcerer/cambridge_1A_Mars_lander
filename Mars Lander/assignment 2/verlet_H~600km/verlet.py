# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# mass, spring constant, initial position and velocity
# simulation time, timestep and time
t_max = 1000
dt = 1

t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []

#Verlet
m = 6.42e23
G0 = 6.674e-11
G=m*G0
x = np.array([4000000,0,0])
v = np.array([0,0,0])
x_list1=[]
v_list1=[]
for t in t_array:

    # append current state to trajectories
    x_list1.append(x)
    v_list1.append(v)

    # calculate new position and velocity
    a = -G * x /(np.linalg.norm(x))**3
    if len(x_list1)==1:
        x = x + dt * v#euler
    else:
        x = 2*x -x_list1[-2] + dt*dt* a
    v = (x-x_list1[-1])/dt

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array1 = np.array(x_list1)[:,0]
x_array2 = np.array(x_list1)[:,1]
# plot the position-time graph

plt.figure(1)
plt.clf()
plt.grid()
plt.gca().set_aspect('auto', adjustable='box')#change to auto to show descend's graph
#plt.plot(x_array1, x_array2)
plt.plot(t_array,x_array1) #use this if simulating descending
plt.legend()
plt.show()