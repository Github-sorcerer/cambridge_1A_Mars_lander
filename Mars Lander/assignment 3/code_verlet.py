# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import time
start_time = time.time()

# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 10000
dt = 1

t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []

#Verlet
m = 1
k = 1
x = 0
v = 1
x_list1=[]
v_list1=[]
for t in t_array:

    # append current state to trajectories
    x_list1.append(x)
    v_list1.append(v)

    # calculate new position and velocity
    a = -k * x / m
    if len(x_list1)==1:
        x = x + dt * v#euler
    else:
        x = 2*x -x_list1[-2] + dt*dt* a
    v = (x-x_list1[-1])/dt

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array1 = np.array(x_list1)
v_array1 = np.array(v_list1)

print("--- %s seconds ---" % (time.time() - start_time))