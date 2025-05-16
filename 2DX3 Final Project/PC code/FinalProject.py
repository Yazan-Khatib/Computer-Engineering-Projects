import serial
from math import sin, cos
import numpy as np
import open3d as o3d
 
POINTS_PER_SCAN = 32      
X_INCREMENT = 300         

f = open("Scan.xyz", "w")   

s = serial.Serial('COM4')
s.baudrate = 115200
s.bytesize = 8
s.parity = 'N'
s.stopbits = 1


print("Opening: " + s.name)

 
input("Press Enter to start")

 
s.reset_output_buffer()
s.reset_input_buffer()

print("Sending 's' to MCU")
s.write(b's')
 

 
print("Receiving measurements from MCU")

 
points = 0
colours = []
x = 0
measuring = True

 
while measuring == True:
    for i in range(POINTS_PER_SCAN):
        
        data = s.readline()
         
        data = data.decode()
       
        print("Data received: " + data)

        if data == "Measurements done\n":
            measuring = False
            break




         
        data = data.split(",")
        distance = float(data[0])
        rangeStatus = int(data[1])

         
        y = distance * sin(-11.25*(i*(3.14/180.0)))
        z = distance * cos(-11.25*(i*(3.14/180.0)))

        
        f.write('{0:.2f} {1:.2f} {2:.2f}\n'.format(x, y, z))


      
        if rangeStatus == 0: 
            colours.append([0, 1, 0])
        elif rangeStatus == 1 or rangeStatus == 2:  
            colours.append([1, 1, 0])
        else:  
            colours.append([1, 0, 0])

        points += 1

    
    x += X_INCREMENT

  
print("Closing: " + s.name)
s.close()
 
f.close()

 
pcd = o3d.io.read_point_cloud("Scan.xyz", format="xyz")

 
pcd.colors = o3d.utility.Vector3dVector(colours) 

 
yz_slice_vertex = []
for i in range(points):
    yz_slice_vertex.append([i])

     
lines = []  
 
for i in range(points - POINTS_PER_SCAN):
    lines.append([yz_slice_vertex[i], yz_slice_vertex[i+32]])

 
for i in range(points):
    if (i+1) % 32 == 0:  
        lines.append([yz_slice_vertex[i], yz_slice_vertex[i-31]])
    elif i < points - 1:
        lines.append([yz_slice_vertex[i], yz_slice_vertex[i+1]])

 
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
 
o3d.visualization.draw_geometries([pcd, line_set])
