# -*- coding: utf-8 -*-

from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt 
import csv

number_of_points=100
v_max = 12#speed in meters per second
floor_friction_coeff =0.4#Should range from 0 to 1 (0.7 dry road, 0.4 wet road)
max_fx_acceleration = 15
max_fx_decceleration = 10
mass=4#Mass of car guessed at 4kg
k=1 #Curvature Scaling Factor
csv_path = "sim_points.csv"




#LOAD CSV
rows=[]
with open(csv_path, 'r') as file: 
            csvreader = csv.reader(file)
            header = next(csvreader)
            for row in csvreader:
                rows.append(row[0:2])
points=np.array(rows)
 
x =points[:,0].astype("float")
y =points[:,1].astype("float")        

#CREATE SPLINE
xbefore = np.flip(np.flip(x)[0:int(len(x)/2)])
xafter = np.flip(np.flip(x)[int(len(x)/2):])
x_extended = np.append(np.append(xbefore, x),xafter)

ybefore = np.flip(np.flip(y)[0:int(len(y)/2)])
yafter = np.flip(np.flip(y)[int(len(y)/2):])
y_extended = np.append(np.append(ybefore, y),yafter)

spline_data, m = interpolate.splprep([x, y], s=0, per=True)
x_spline, y_spline = interpolate.splev(np.linspace(0, 1, number_of_points), spline_data)

plt.figure(2)
plt.clf()
plt.plot(x_spline,y_spline)
plt.scatter(x,y, color='orange', marker='o')


#CALCULATE DERIVATIVES FOR CURVATURE
xder1 = np.gradient(x_spline,edge_order=2)
yder1 = np.gradient(y_spline,edge_order=2)

xder2 = np.gradient(xder1)
yder2 = np.gradient(yder1)

curvature = np.abs((xder1 * yder2) - (yder1 * xder2)) / np.power(np.power(xder1,2) + np.power(yder1,2),3/2)


xy_points = np.array((np.flip(x_spline), np.flip(y_spline)))
xy_offset = np.roll(xy_points,1)
distances = np.linalg.norm(xy_points -xy_offset, axis=0)
distances[0]=0

#pass 1
ux =np.sqrt( (floor_friction_coeff * 9.81) / np.abs(curvature * k))#Curvature should really be curve radius
ux = np.minimum(ux, v_max)

#Forward pass
v_forward=[]

for i in range(len(ux)):
    li = distances[i-1]
    v_forward.append(np.sqrt(ux[i-1]**2+(2*(max_fx_acceleration/mass)*li)))
    
v_forward = np.array(v_forward)

#Backward Pass
v_backward=[]
for i in range(len(distances)):
    i_index = i+1
    if i_index == len(distances):
        i_index = 0
        
    li = distances[i_index]
    v_backward.append(np.sqrt(v_forward[i_index]**2-(2*(max_fx_decceleration/mass)*li)))

v_backward = np.array(v_backward)
v_backward = np.minimum(v_backward, v_max)

final_velocity_profile = v_backward

plt.figure(1)
plt.clf()
plt.plot(ux, label = "original")
plt.plot(v_forward,label = "Forward Pass")
plt.plot(v_backward, label = "Backward Pass")
plt.legend()
plt.show()


plt.figure(3)
plt.clf()
plt.scatter(x_spline,y_spline, c = (v_backward/v_max), cmap='RdYlGn')

















