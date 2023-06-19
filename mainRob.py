
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import numpy as np
import random
import itertools
import networkx as nx
import matplotlib.pyplot as plt
import argparse

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host,file):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.filename=file
        self.ignore_pose=True
        self.connectionns=[]
        self.outOfLine = 0
        self.oldline=None
        self.final_connections=[]
        self.grace_period_counter=0 
        self.looking=None
        self.sensor_history = [[0,0,0,0,0,0,0], [0,0,0,0,0,0,0]]
        self.x = 1
        self.y = 1
        self.seenBeacons = {0: (10,24)}
        self.compassdiff=[]
        self.oldrow=1
        self.oldcol=1
        self.theta = 0
        self.outl_prev = 0
        self.outr_prev = 0
        self.diameter = 1  # Th
        self.position = (self.x, self.y)  # Robot's current position (row, column)
        self.path = [(1,1)]  # List to store the robot's path
        self.G=nx.Graph()
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
    def fakewonder(self):
        wheel_speed = 0.15
        if self.measures.time == 16:
            self.driveMotors(0,0)
            exit()
        else:
            print("------------------   ", self.measures.time, "   ------------------")
            print("Line sensors: ", self.measures.lineSensor)
            print("Compass: ", self.measures.compass)
            print("Line sensors: ", self.measures.lineSensor)
            threshold=0.1
            line = [x == '1' for x in self.measures.lineSensor]
            self.position=(self.x ,self.y)
            
            
            if self.oldline != line: 
                #If looking right and line sensor says there is a curve to the right point is row -1
                if self.looking == "r" and line[-1] and line[-2]:
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)

                #If looking right and line sensor says there is a curve to the left point is row +1
                elif self.looking == "r" and line[0] and line[1]:
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)
                #If looking right and line sensor says there is a curve to the front point is col +1
                elif self.looking == "r" and all(line):
                    self.driveMotors(wheel_speed,wheel_speed)
                    self.update_pose(wheel_speed,wheel_speed)
                #If looking down and line sensor says there is a curve to the right point is col -1
                elif self.looking == "d" and line[-1] and line[-2]:
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)
                #If looking down and line sensor says there is a curve to the left point is col +1
                elif self.looking == "d" and line[0] and line[1]:
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)
                #If looking down and line sensor says there is a curve to the front point is row +1
                elif self.looking == "d" and all(line):
                    self.driveMotors(wheel_speed,wheel_speed)
                    self.update_pose(wheel_speed,wheel_speed)

                #If looking up and line sensor says there is a curve to the right point is col +1
                elif self.looking == "u" and line[-1] and line[-2]:
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)
                #If looking up and line sensor says there is a curve to the left point is col -1
                elif self.looking == "u" and line[0] and line[1]:   
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)
                #If looking up and line sensor says there is a curve to the front point is row -1
                elif self.looking == "u" and all(line):
                    self.driveMotors(wheel_speed,wheel_speed)
                    self.update_pose(wheel_speed,wheel_speed)

                #If looking left and line sensor says there is a curve to the right point is row +1
                elif self.looking == "l" and line[-1] and line[-2]:
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)

                #If looking left and line sensor says there is a curve to the left point is row -1
                elif self.looking == "l" and line[0] and line[1]:
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)
                #If looking left and line sensor says there is a curve to the front point is col -1
                elif self.looking == "l" and all(line):
                    self.driveMotors(wheel_speed,wheel_speed)
                    self.update_pose(wheel_speed,wheel_speed)


                print("I'm at a corner")
            if -15 < self.measures.compass <15:

                print("Looking right")
                self.looking="r"
                self.driveMotors(wheel_speed,+wheel_speed)
                self.update_pose(wheel_speed,+wheel_speed)
            elif -105 < self.measures.compass < -75:
                print("Looking down")
                self.looking="d"
                self.driveMotors(wheel_speed,+wheel_speed)
                self.update_pose(wheel_speed,+wheel_speed)
            elif 75 < self.measures.compass < 105  :
                print("Looking Up")
                self.looking="u"
                self.driveMotors(wheel_speed,+wheel_speed)
                self.update_pose(wheel_speed,+wheel_speed)
            elif -195 < self.measures.compass < -165 or 165 < self.measures.compass < 195:
                print("Looking left")
                self.looking="l"
                self.driveMotors(wheel_speed,+wheel_speed)
                self.update_pose(wheel_speed,+wheel_speed)
            else:
                print("I'm lost")
                self.driveMotors(wheel_speed,+wheel_speed)
                self.update_pose(wheel_speed,+wheel_speed)    
        self.oldline=line

    def wander(self):
        print("----------------------------------  Starting wander ----------------------------------")
        print("Time: ", self.measures.time)
        print("Compass: ", self.measures.compass)
        print("Line sensors: ", self.measures.lineSensor)
        print("Beacon: ", self.measures.beacon)
        print("Ground: ", self.measures.ground)
        print("Obstacle: ", self.measures.collisions)
        test=[]
        test.append(self.measures.compass)
        wheel_speed = 0.15
        if self.measures.time % 100 == 0:
            self.print_map_and_path()
        threshold=0.1
        line = [x == '1' for x in self.measures.lineSensor]
        self.sensor_history.append(line)
        self.sensor_history = self.sensor_history[-5:]
        print("History: ", self.sensor_history)
        print("Current line: ", line)
       
        #Calculate position

        self.position=(self.x ,self.y)
                
        print("Position: ", self.position)
        row, column = self.position  
        row = np.ceil(row) if 1- row % 1 < threshold else np.floor(row)
        column = np.ceil(column) if 1- column % 1 < threshold else np.floor(column)


        if sum(line[0:3]) > sum(line[-3:]):
            line[-3:] = [0,0,0]
        elif sum(line[0:3]) < sum(line[-3:]):
            line[0:3] = [0,0,0]
        elif sum(line[0:3]) == sum(line[-3:]):
            line[0:3] = [0,0,0]
            line[-3:] = [0,0,0]


        print("Processed line: ", line)
        #left
        #stuck on a corner
        if self.oldcol==column and self.oldrow==row:
            self.grace_period_counter+=1
            if self.grace_period_counter>10:
                self.grace_period_counter=0
                self.driveMotors(-wheel_speed,-wheel_speed)
                self.update_pose(-wheel_speed,-wheel_speed)
                print("Decision -> Corner")
                return

        
        if line[0] and line[1]:
            print("Decision -> Sharp Left")
            self.driveMotors(-wheel_speed,+wheel_speed)
            self.update_pose(-wheel_speed,+wheel_speed)
        elif line[1]:
            
            print("Decision -> Slow Left")
            self.driveMotors(0,+wheel_speed)
            self.update_pose(0,+wheel_speed)
        #right
        elif line[-1] and line[-2]:
            
            print("Decision -> Sharp Right")
            self.driveMotors(+wheel_speed,-wheel_speed)
            self.update_pose(+wheel_speed,-wheel_speed)
        elif line[-2]:
            
            print("Decision -> Slow Right")
            self.driveMotors(+wheel_speed,0)
            self.update_pose(+wheel_speed,0)
        #forward
        elif line[2] or line[3] or line[4]:
            
            print("Decision -> Forward")
            self.driveMotors(wheel_speed,wheel_speed)
            self.update_pose(wheel_speed,wheel_speed)
        # if the robot isn't in any of the above conditions, it might have lost the line, so we stop it
        else:
            
            print("Decision -> Stop. Lost the line.")
            print("Maybe at T-junction?")
                        # Randomly decide to turn left or right
            #if compass is 0 360 -360looking right
            #if compass is 180 -180 looking left
            # if compass is 90 -270 looking up
            # if compass is -90 270 looking down
            thold =10
            if 360 -thold <self.measures.compass < 360+thold or -thold < self.measures.compass < thold:
                
                #up
                #TODO MAYBE MUDAR O PATH
                if (row+1, column) not in self.path:
                    print("Decision -> Turn Right")
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)
                else:
                    print("Decision -> Turn Left")
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)
                    
                    
            elif 180 -thold <self.measures.compass < 180+thold or -180 -thold < self.measures.compass < -180+thold:
                #left
                if (row, column+1) not in self.path:
                    print("Decision -> Turn Right")
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)
                else:
                    print("Decision -> Turn Left")
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)
            elif 90 -thold <self.measures.compass < 90+thold or -270 -thold < self.measures.compass < -270+thold:
                #right
                if (row, column-1) not in self.path:
                    print("Decision -> Turn Right")
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)
                else:
                    print("Decision -> Turn Left")
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)
            elif -90 -thold <self.measures.compass < -90+thold or 270 -thold < self.measures.compass < 270+thold:
                #down
                if (row-1, column) not in self.path:
                    print("Decision -> Turn Right")
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)
                else:
                    print("Decision -> Turn Left")
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)

            else:
                #both not visited

                turn_decision = random.choice(['left', 'right'])
                
                if turn_decision == 'left':
                    print("Decision -> Turn Left")
                    self.driveMotors(-wheel_speed,+wheel_speed)
                    self.update_pose(-wheel_speed,+wheel_speed)
                else:
                    print("Decision -> Turn Right")
                    self.driveMotors(+wheel_speed,-wheel_speed)
                    self.update_pose(+wheel_speed,-wheel_speed)


        # Update map
        #only count in cell if close to a threshold
        threshold = 0.1


        
        print("row",row)
        print("column" ,column)
        #check if position is valid
        validrow= 1 if (self.oldrow+1 == row or self.oldrow-1 == row) and column==self.oldcol  else 0
        validcolumn= 1 if (self.oldcol+1 == column or self.oldcol-1 == column) and row== self.oldrow else 0 
        print("Valid row: ", validrow)
        print("Valid column: ", validcolumn)
        mapper={
            2:0,
            1:1,
            0:2,
            -1:3,
            -2:4,
            -3:5,
            -4:6,

        }
        self.compassdiff.append(self.measures.compass)
        
        self.compassdiff= self.compassdiff[-2:]

        if (validrow and row<13) or (validcolumn and column<6) and abs(self.compassdiff[-1])-abs(self.compassdiff[0])<12:
            self.connectionns.append([ (int(mapper[self.oldcol]),int(self.oldrow)), (int(mapper[column]),int(row))])
            self.oldrow = row
            self.oldcol = column
            print("#######################################################################################################################################")
            # Update path
            self.path.append((int(row), mapper[column]))
            
    
        if self.measures.ground!= -1 and self.measures.ground!= 0:
            #Convert to future Beacon position
            brow=mapper[column]
            bcol=row
            #first convert all points to reference 0 0 as start
            bcol-=1
            brow-=1
            #then convert all of them to be in diameters 
            bcol*=2
            brow*=2
            bcol+=24
            brow+=10
            
            self.seenBeacons[self.measures.ground] = (brow,bcol)

            
        print("Seen beacons: ", self.seenBeacons)
        
        if self.measures.time==5000 :
            print("Path: ", self.path)


            print("Seen beacons: ", self.seenBeacons)
            print("Connections: ", self.connectionns)
            self.drawmap()
            self.createSmartPath()

            exit()

    def drawmap(self):
        final_path=[]
        final_connections=[]
        # Initialize the 2D array that represents the map. 
        # 21 rows and 49 columns, filled with ' ' (space) initially
        map = [[' ' for _ in range(49)] for _ in range(21)] 
        #since my position is colujmn row and not row column
        for i in self.path:
            col, row =i
            #first convert all points to reference 0 0 as start
            col-=1
            row-=1
            #then convert all of them to be in diameters 
            col*=2
            row*=2
            col+=24
            row+=10
            final_path.append((row,col))
        for i in self.connectionns:
            row1,col1 =i[0]
            row2, col2 =i[1]
            #first convert all points to reference 0 0 as start
            col1-=1
            row1-=1
            col2-=1
            row2-=1
            #then convert all of them to be in diameters 
            col1*=2
            row1*=2
            col2*=2
            row2*=2
            col1+=24
            row1+=10
            col2+=24
            row2+=10
            final_connections.append([(row1,col1),(row2,col2)])
        #Compare both
        print("Original path: ", self.path)
        print("Final path: ", final_path)
        # Mark the starting position as 0. According to your description, 
        # it is always at the center of the map.

        map[10][24] = '0'  

        #Connections are fine
        #just need to account for the offset
        
        print("Final connections: ", final_connections)
        self.final_connections=final_connections
    
        for k in final_connections:
                i,j = k
                #if same row
                print("i: ", i)
                print("j: ", j)
                if i[0]==j[0]:
                    if i[1]<j[1]:
                        map[i[0]][i[1]+1]='-'
                    else:
                        map[i[0]][i[1]-1]='-'
                elif i[1]==j[1]:
                    if i[0]<j[0]:
                        map[i[0]+1][i[1]]='|'
                    else:
                        map[i[0]-1][i[1]]='|'



        
        # Mark the targets on the map.
        # Assuming targets is a list of tuples where each tuple is the coordinate of a target.
        for id in self.seenBeacons.keys():
            row,col = self.seenBeacons[id]
            map[int(row)][int(col)] = str(id)
        print("Map: ", map)

        # Write the map to the file
        name=self.filename+'.map'
        with open(name, 'w') as f:
            for row in map:
                f.write(''.join(row))
                f.write('\n')

    def grid_positions(self):
        return {node: (node[1], -node[0]) for node in self.G.nodes}


    def createSmartPath(self):

    # Extract unique nodes from the connections
        connections= self.final_connections
        beacons={}
        counter=0
        for i in self.seenBeacons:
            beacons[counter]=self.seenBeacons[i]
            counter+=1

        for c in connections:
            self.G.add_edge(*c)
        pos = self.grid_positions()
        nx.draw(self.G, pos, with_labels=True)
        plt.show()

        # Compute the shortest path distance between each pair of beacons
        beacon_distances = {}
        start_node = self.seenBeacons[0]
        print("Beacons:", beacons)

        for i in range(len(beacons)):
            for j in range(i+1, len(beacons)):
                path = nx.shortest_path(self.G, beacons[i], beacons[j])
                print("Path: ", path)
                distance = len(path) - 1  # each edge has the same weight
                beacon_distances[(beacons[i], beacons[j])] = distance
                beacon_distances[(beacons[j], beacons[i])] = distance

        # Solve TSP among the beacons
        shortest_distance = float('inf')
        shortest_path = None
        
        print("Beacon distances",beacon_distances)
        for path in itertools.permutations(beacons):
            print("Path: ", path)
            path = [start_node, *[beacons[beacon] for beacon in path if beacons[beacon]!=start_node], start_node]
            print("Path: ", path)
            distance = sum(beacon_distances[(path[i], path[i + 1])] for i in range(len(path) - 1))

            if distance < shortest_distance:
                shortest_distance = distance
                shortest_path = path

        print(f"The shortest path is {shortest_path} with a total distance of {shortest_distance}")
        path_tofile=[]
        for i in range(len(shortest_path)-1):
            print("i: ", i)
            print("shortest path i: ", shortest_path[i])
            print("shortest path i+1: ", shortest_path[i+1])
            path = nx.shortest_path(self.G, shortest_path[i], shortest_path[i+1])
            print("this path is: ", path)
            for i in path:
                a,c=i
                a-=10
                c-=24
                k=(int(c),int(a))
                path_tofile.append(k)

        print("Path to file: ", path_tofile)
        name= self.filename+'.path'
        # Write the shortest route to a text file
        with open(name, 'w') as f:
            for node in path_tofile:
                f.write(' '.join(map(str, node)) + '\n')
    def print_map_and_path(self):

        # Print path
        for position in self.path:
            print(position)

    def update_pose(self, inl, inr, sigma=0.05):
        # Apply the IIR filter to the motor powers
        outl = ((inl + self.outl_prev) / 2 )* np.random.normal(1, sigma)
        outr = ((inr + self.outr_prev) / 2 )* np.random.normal(1, sigma)

        # Save current powers for the next step
        self.outl_prev = outl
        self.outr_prev = outr

        # Calculate the linear and rotational movement
        lin = (outl + outr) / 4
        rot = (outr - outl) / self.diameter

        # Calculate new pose
        self.x = self.x + lin * np.cos(np.deg2rad(self.measures.compass))
        self.y = self.y + lin * np.sin(np.deg2rad(self.measures.compass))
        self.theta = self.theta + rot

        #IF COM        if busslla
        # Update pose pra atualizar a orientação


        print("Pose: ", np.cos(np.deg2rad(self.measures.compass)))
        print("lin",lin)
            


rob_name = "pClient"
host = "localhost"
pos = 1
filename = 'output'

for i in range(1, len(sys.argv),2):
    print(sys.argv[i])
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--challenge" or sys.argv[i] == "-c") and i != len(sys.argv) - 1:
        challenge = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--filename" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        filename = sys.argv[i + 1]
        print("dentro",filename)
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        maps = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()
print(filename)
# If filename is provided, read the map

if __name__ == '__main__':
    rob=MyRob(rob_name, pos, [0.0,60.0,-60.0,180.0], host,filename)

    
    rob.run()
