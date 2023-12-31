
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
import time
CELLROWS=7
CELLCOLS=14
TRESHOLD_ROUNDING=0.3

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host,file):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.compass_readings=[]
        self.window_size=5
        self.filename=file
        self.rotationBuffer=0
        self.pose = [0, 0, 0]
        self.ignore_pose=True
        self.connectionns=[]
        self.outOfLine = 0
        self.oldline=None
        self.final_connections=[]
        self.grace_period_counter=0 
        self.looking="r"
        self.sensor_history = [[0,0,0,0,0,0,0], [0,0,0,0,0,0,0]]
        self.x = 1
        self.flag=True
        self.y = 1
        self.mklist=[]
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
        self.sharpturn=False

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
                #if self.measures.time==1000:
                #    if self.looking != None:
                #        self.returnToStart()

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

#REAL cODE
    def returnToStart(self):
        """Returns to the starting point"""
        print("Returning to start")
        path = nx.shortest_path(self.G, source=(self.x,self.y), target=(1,1))
        currx=self.x
        curry=self.y
        print("Path is", path)
        while path:  # while there are still nodes in the path
            next_node = path[0]  # get the next node from the path

            # calculate the direction of the next node from the current position
            dx = next_node.x - currx
            dy = next_node.y - curry
            target_angle = np.arctan2(dy, dx)
            target_angle = np.degrees(target_angle)
            # calculate the angle you need to rotate to face the next node
            #self.looing r=0 d=-90 u=90 l=-180
            rotate_angle = target_angle - self.measures.compass
            print("Rotating", rotate_angle)
            # rotate to that direction (you would replace this with your actual method to rotate the robot)
            #self.rotate(rotate_angle)

            # move to the next node (replace this with your actual method to move the robot)
            #self.move_to_position(next_node.x, next_node.y)

            #update the current position
            currx = next_node.x
            curry = next_node.y
            # remove the first node from the path
            path.pop(0)
    def whereLooking(self):
            """Checks the compass to see which direction the robot is looking"""
            if -15 < self.measures.compass <15:
                self.looking="r"

            elif -105 < self.measures.compass < -75:

                self.looking="d"

            elif 75 < self.measures.compass < 105  :

                self.looking="u"
             
            elif -195 < self.measures.compass < -165 or 165 < self.measures.compass < 195:

                self.looking="l"
               
            else:
                self.looking=None

    def rotateMyself(self,pos=(1,1)):
        """Rotates the robot on the spot and registers the lines it sees"""
        i=6
        rotate=1
        count_new=0
        self.whereLooking()
        currentlook=self.looking
        print("I'm looking at ",self.looking ,"and I'm at ",pos)
        while self.looking!=currentlook or i>3:
            i-=1
            self.driveMotors(rotate,-rotate)
            self.update_pose(rotate,-rotate)
            self.whereLooking()

            
        
            line=sum(int(i) for i in self.measures.lineSensor)

            if line >= 2 and self.looking!=None:
                if (pos,self.looking) not in self.mklist:
                    print("I see the line", self.measures.lineSensor, "at", self.looking)
                    count_new+=1
                    self.mklist.append((pos,self.looking))
            self.readSensors()
        self.driveMotors(-rotate,rotate)
        self.update_pose(-rotate,rotate)
        self.readSensors()
        return count_new


    def corner(self,pos=(1,1)):
        """Defines what is seen when there is a corner (gets fed every line) to do when it reaches a corner"""
        self.rotationBuffer+=1
        line = self.measures.lineSensor
        line = [int(x) for x in line]
        if self.rotationBuffer>5:
            print("---------------------------Rotating myself---------------------------")
            if all(line):
                """I'm at a T or CrossRoad junction"""
                #If a at a T junction rotate all around 
                count =self.rotateMyself(pos)
                if count==4:
                    """I'm at a crossroad"""
                else:
                    """I'm at a T junction"""


            elif not all(line):
                """I'm out of line"""
            elif line[:6]:
                """I'm at a left turn"""
            elif line[-6:]:
                """I'm at a right turn"""
                self.rotateMyself(pos)
        elif self.measures.time==0:
            self.rotateMyself(pos)

    def wander(self):
        """Wanders around the environment"""

        print("----------------------------------  Starting wander ----------------------------------")
        print("Time: ", self.measures.time)
        print("Compass: ", self.measures.compass)
        print("Line sensors: ", self.measures.lineSensor)
        print("Beacon: ", self.measures.beacon)
        print("Ground: ", self.measures.ground)
        print("Obstacle: ", self.measures.collisions)

        wheel_speed = 0.15
        if self.measures.time % 100 == 0:
            self.print_map_and_path()
        line = [x == '1' for x in self.measures.lineSensor]

        #Calculate position
        self.position=(self.x ,self.y)
        print("Position: ", self.position)
                
        row, column = self.position  
        row = np.ceil(row) if 1- row % 1 < TRESHOLD_ROUNDING else np.floor(row)
        column = np.ceil(column) if 1- column % 1 < TRESHOLD_ROUNDING else np.floor(column)

        print("Rounded position: ", (row, column))


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
            # 0 0 1 1 1 0 0
            #   1 1 1 1 1 1 1
            # 0 1 0
            # 0 0 0 00 0 

            for i in range(6):

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
                    #both visited

                    turn_decision = random.choice(['left', 'right'])
                    
                    if turn_decision == 'left':
                        print("Decision -> Turn Left")
                        self.driveMotors(-wheel_speed,+wheel_speed)
                        self.update_pose(-wheel_speed,+wheel_speed)
                    else:
                        print("Decision -> Turn Right")
                        self.driveMotors(+wheel_speed,-wheel_speed)
                        self.update_pose(+wheel_speed,-wheel_speed)



        self.whereLooking()
        print("Looking at: ", self.looking)
        print("Unprocessed row/column: ",(row,column))
        print("Old row/column: ",(self.oldrow,self.oldcol))

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

        try:
            mapper[column]
        except:
            column=6
    
        if (validrow and row<13) or (validcolumn and column<6) and abs(self.compassdiff[-1])-abs(self.compassdiff[0])<12:
            print("#######################################################################################################################################")
            self.whereLooking()
            print("#######################################################################################################################################")
            if [ (int(mapper[self.oldcol]),int(self.oldrow)), (int(mapper[column]),int(row))] not in self.connectionns:
                print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Inserting connection between: ", (int(mapper[self.oldcol]),int(self.oldrow)), (int(mapper[column]),int(row)))
                self.connectionns.append([ (int(mapper[self.oldcol]),int(self.oldrow)), (int(mapper[column]),int(row))])
            self.oldrow = row
            self.oldcol = column
            # Update path
            self.path.append((int(row), mapper[column]))
        
        """        self.rotationBuffer += 1
        if self.sharpturn :
            if self.rotationBuffer >= 20:
                pos=(int(mapper[column]),int(row))
                self.rotateMyself(pos)
                self.rotationBuffer = 0  # reset counter after rotation
                self.sharpturn = False
        elif self.measures.time == 0:
            self.rotateMyself()
            self.sharpturn = False"""
        #Check if I'm at a corner
        #pos=(int(mapper[column]),int(row))
        #self.corner(pos)
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
            name=self.filename+'.png'

            plt.savefig(name, dpi=200)
            #input("Press Enter to finish...")
            print("Possibe turns list",self.mklist)
            exit()

    def drawmap(self):
        """Draws the map in the terminal and saves it to a file"""
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
        for i in map:
            print(i[20:])

        # Write the map to the file
        name=self.filename+'.map'
        with open(name, 'w') as f:
            for row in map:
                f.write(''.join(row))
                f.write('\n')

    def grid_positions(self):
        """Returns a dictionary with the positions of the nodes in the grid"""
        return {node: (node[1], -node[0]) for node in self.G.nodes}


    def createSmartPath(self):
        """Creates the shortest path from the map"""

    # Extract unique nodes from the connections
        connections= self.final_connections
        beacons={}
        counter=0
        for i in self.seenBeacons:
            beacons[counter]=self.seenBeacons[i]
            counter+=1
        #Build the graph
        for c in connections:
            self.G.add_edge(*c)

        pos = self.grid_positions()
        special_nodes=[]
        for i in self.seenBeacons:
            if i!=0:
                brow,bcol=self.seenBeacons[i]
                special_node=(brow,bcol)
                special_nodes.append(special_node)
            else:
                start_node=self.seenBeacons[i]
                
        node_colors=[]
        print("special nodes: ", special_nodes)
        print("start node: ", start_node)
        for node in self.G.nodes():
            print("node: ", node)
            if node in special_nodes:
                node_colors.append('red')  # special nodes
            elif node == start_node:
                node_colors.append('green')  # start node
            else:
                node_colors.append('blue')  # other nodes
        plt.clf()
        plt.ion()
        nx.draw(self.G, pos, with_labels=False,node_color=node_colors)
        plt.show()
        plt.plot([], [], color='blue', label='Regular nodes')  # regular nodes dummy plot
        plt.plot([], [], color='red', label='Beacons')  # special nodes dummy plot
        plt.plot([], [], color='green', label='Start node')  # start node dummy plot
        plt.legend()

        # Compute the shortest path distance between each pair of beacons
        beacon_distances = {}
        start_node = self.seenBeacons[0]
        print("Beacons:", beacons)
        if len(self.seenBeacons)<2:
            print("Not enough beacons")
            a,c=self.seenBeacons[0]
            
            a-=10
            c-=24
            k=(int(c),-int(a))
            

            print("Only start node: ", k)
            name= self.filename+'.path'
            # Write the shortest route to a text file
            with open(name, 'w') as f:
                
                f.write(' '.join(map(str, k)) + '\n')
            return

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
                k=(int(c),-int(a))
                path_tofile.append(k)

        print("Path to file: ", path_tofile)
        name= self.filename+'.path'
        # Write the shortest route to a text file
        with open(name, 'w') as f:
            for node in path_tofile:
                f.write(' '.join(map(str, node)) + '\n')
    def print_map_and_path(self):
        """Prints the map and the path until this point"""

        # Print path
        for position in self.path:
            print(position)

    def update_pose(self, inl, inr, sigma=0.05):
        """Updates the pose of the robot given the motor inputs"""
        # Apply the IIR filter to the motor powers
        outl = ((inl + self.outl_prev) / 2 )* np.random.normal(1, sigma)
        outr = ((inr + self.outr_prev) / 2 )* np.random.normal(1, sigma)

        # Save current powers for the next step
        self.outl_prev = outl
        self.outr_prev = outr

        # Calculate the linear and rotational movement
        lin = (outl + outr) / 4
        rot = (outr - outl) / self.diameter

        # Use the moving average filter for the compass reading
        if len(self.compass_readings) >= self.window_size:
            self.compass_readings.pop(0)
        self.compass_readings.append(self.measures.compass)
        filtered_compass = np.mean(self.compass_readings)

        # Calculate new pose
        self.x = self.x + lin * np.cos(np.deg2rad(self.measures.compass))
        self.y = self.y + lin * np.sin(np.deg2rad(self.measures.compass))
        self.theta = self.theta + rot

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
