
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

"""
JS IMPORTS
"""
import numpy as np
from numpy.linalg import norm 

#TODO remove
import matplotlib.pyplot as plt

CELLROWS=7
CELLCOLS=14

"""
CLASS MyRob
"""
class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

        # data
        self.ir_fov = pi / 3 # 60 deg
        self.wall_thick = 0.2 # 0.1 each side
        self.cell_size = 2

        # store the expected measurements from each sensor
        self.ground_truth = {'center': None,
            'left': None,
            'right': None,
            'back': None}









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

            if self.measures.collision:
                print(self.robName + " collided")
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
                    self.setVisitingLed(True);
                self.wander()

            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)

            elif state == 'return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()
            

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        # if self.measures.irSensor[center_id] > 5.0\
        #    or self.measures.irSensor[left_id]   > 5.0\
        #    or self.measures.irSensor[right_id]  > 5.0\
        #    or self.measures.irSensor[back_id]   > 5.0:
        #     print('Rotate left')
        #     self.driveMotors(-0.1,+0.1)
        # elif self.measures.irSensor[left_id]> 2.7:
        #     print('Rotate slowly right')
        #     self.driveMotors(0.1,0.0)
        # elif self.measures.irSensor[right_id]> 2.7:
        #     print('Rotate slowly left')
        #     self.driveMotors(0.0,0.1)
        # else:
        # print('Go')
        self.driveMotors(0.1,0.1)
        



    """
    JS METHODS
    """
    def unitVector(self, vector):
        """ Returns the unit vector of the vector.  
        
        Based on https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
        """
        
        return vector / norm(vector)

    def angleBetween(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'.
        """
        # return np.arctan2(norm(np.cross(v1,v2)), np.dot(v1,v2))
        return np.arctan2(np.cross(v1,v2), np.dot(v1,v2))

    def lineIntersection(self, line1, line2):
        """
        
        Based on https://stackoverflow.com/questions/3252194/numpy-and-line-intersections
        """

        # s for stacked
        s = np.vstack([line1[0], line1[1], line2[0], line2[1]])        
        
        # h for homogeneous
        h = np.hstack((s, np.ones((4, 1)))) 
        
        # get first line
        l1 = np.cross(h[0], h[1])           
        
        # get second line
        l2 = np.cross(h[2], h[3])           
        
        # point of intersection
        x, y, z = np.cross(l1, l2)          
        
        # lines are parallel
        if z == 0:                          
            return None

        return (x/z, y/z)

    def rotateVector(self, vector, angle):
        """ Rotate a vector
        """
        rotation = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
        return np.dot(rotation, vector)

    def plotMapAndRobot(self, point, versor, walls):

        # walls
        for wall in walls:
            plt.plot([wall[0][0], wall[1][0]] , [wall[0][1], wall[1][1]])

        plt.xlim([0, CELLCOLS * 2])
        plt.xticks(range(0, CELLCOLS * 2 + 1, 2))

        plt.ylim([0, CELLROWS * 2])
        plt.yticks(range(0, CELLROWS * 2 + 1, 2))

        # fov
        fov0 = self.rotateVector(versor, -1 * self.ir_fov / 2)
        fov0 = np.multiply(fov0, 100)

        fov1 = self.rotateVector(versor, self.ir_fov / 2)
        fov1 = np.multiply(fov1, 100)

        for end_point in [np.add(point, fov0), np.add(point, fov1)]:
            plt.plot([point[0], end_point[0]], [point[1], end_point[1]])

        plt.grid(True)
        plt.show()


    def minDistanceToWall(self, point, wall_points):
        """ Compute the minimum distance from a point to a wall,
        defined by two points (line segment).

        Based on https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/
        """

        # initialize
        min_dist = None

        # get wall corners
        corner0 = wall_points[0]
        corner1 = wall_points[1]

        # build vectors from wall corners to point
        wall = np.subtract(corner1, corner0)
        point_to_corner0 = np.subtract(point, corner0)
        point_to_corner1 = np.subtract(point, corner1)

        # compute dot products
        dot_corner0 = np.dot(wall, point_to_corner0)
        dot_corner1 = np.dot(wall, point_to_corner1)

        # case 1
        if (dot_corner1 > 0):         
            # find magnitude
            min_dist = norm(point_to_corner1)
        
        # case 2
        elif (dot_corner0 < 0):
            # find magnitude
            min_dist = norm(point_to_corner0)

        # case 3
        else:        
            # find magnitude
            min_dist = norm(np.cross(wall, point_to_corner1)) / norm(wall)

        return min_dist 


    def getWallsCorners(self):
        """ Compute the corners coordinates (X and Y) of all walls in the map.
        Also, it accounts for thickness. 
        """

        walls = []
        wall_indices = []

        for row in range(CELLROWS * 2 - 1):
            for col in range(CELLCOLS * 2 - 1):                
                if self.labMap[row][col] in ['-', '|']:
                    wall_indices.append((row, col))


        for wall in wall_indices:

            row = wall[0]
            col = wall[1]

            wall_direction = self.labMap[row][col]

            if wall_direction == '|': # vertical wall
                
                side_1 = [((col-1)+2-self.wall_thick/2, row), ((col-1)+2-self.wall_thick/2, row+2)]
                side_2 = [((col-1)+2+self.wall_thick/2, row), ((col-1)+2+self.wall_thick/2, row+2)]

                walls.append(side_1)
                walls.append(side_2)

            elif wall_direction == '-': # horizontal wall

                side_1 = [(col, (row-1)+2-self.wall_thick/2), (col+2, (row-1)+2-self.wall_thick/2)]
                side_2 = [(col,(row-1)+2+self.wall_thick/2), (col+2, (row-1)+2+self.wall_thick/2)]

                walls.append(side_1)
                walls.append(side_2) 

        return walls
        
        
    def computeSensorMeasure(self, point, versor, wall):
        
        # compute angle to wall extremities
        sensor_to_corner0 = np.subtract(wall[0], point)
        sensor_to_corner1 = np.subtract(wall[1], point)

        angle0 = self.angleBetween(versor, sensor_to_corner0)
        angle1 = self.angleBetween(versor, sensor_to_corner1)

        # check if inside FOV (divide by two to use only the one part of it)
        angle0_inside = np.abs(angle0) <= self.ir_fov / 2 
        angle1_inside = np.abs(angle1) <= self.ir_fov / 2 

        # case 1: both angles inside fov
        if angle0_inside and angle1_inside:

            self.minDistanceToWall(point, wall)

        # case 2: only corner 0 is inside
        elif angle0_inside != angle1_inside:
            
            # rotate versor to the direction of the FOV bounds
            theta = np.sign(angle1) * self.ir_fov / 2
            fov_versor = self.rotateVector(versor, theta)









            # line from IR sensor
            line_ir = [point, np.add(point, fov_versor)]
            intersection = self.lineIntersection(line_ir, wall)
            
            print(angle0, angle1)
            print(wall)
            print(intersection)

            print(2)
            
            
            
            
            # self.minDistanceToWall([ ,intersection, ], point)
            

            

        # case 3: only corner 1 is inside
        # elif angle0_inside != angle1_inside and angle1_inside:
        #     print(3)    

        # case 4: both angles outside fov
        else:
            print(4)
            pass # do nothing











    def computeGroundTruth(self):  

        # iterate over each cell                
        for row in range(CELLROWS):
            for col in range(CELLCOLS):
                
                # compute coordinates of the center of this cell
                center_cell = (col + self.cell_size / 2, row + self.cell_size / 2)
                # print(center_cell)











"""
CLASS MAP
"""
class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


"""
MAIN
"""
rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)

    if mapc != None:

        rob.setMap(mapc.labMap)
        rob.printMap()

        
        # TODO remove
        print()
        ir_point = [1+0.5, 13]
        ir_versor = [1, 0]

        walls = rob.getWallsCorners()

        for wall in walls:
            rob.computeSensorMeasure(ir_point, ir_versor, wall)

        print('Done boss')

        rob.plotMapAndRobot(ir_point, ir_versor, walls)
        




        # rob.computeGroundTruth() 
        # rob.wallMinDistance([])
    
    rob.run()
