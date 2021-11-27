
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

"""
JS IMPORTS
"""
import numpy as np
from numpy.linalg import norm, inv, det

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
        self.diameter = 1

        # walls description
        self.walls = None

        # positional stuff
        self.current_position = None
        self.last_position = None
        self.last_velocity = None
        
        self.right_speed = 0
        self.left_speed = 0

        # store the expected measurements from each sensor
        self.ground_truth = None
        self.cells_probability = None

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
            print(state)
            
            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if self.measures.collision:
                print(self.robName + " collided")
                quit()  

            # compute traveled distance
            self.current_position = [round(i, 3) for i in self.computeTraveledDistance(self.left_speed, self.right_speed)]
              
            # behaviors
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

            if norm(self.current_position[:2]) % self.cell_size == 0:

                print(f'                          {norm(self.current_position[:2]) / self.cell_size}')


    def wander(self):
        # center_id = 0
        # left_id = 1
        # right_id = 2
        # back_id = 3        

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
        #     print('Go')

        self.left_speed = 0.1
        self.right_speed = 0.1

        self.driveMotors(self.left_speed, self.right_speed)

    """
    JS METHODS
    """
    def _unitVector(self, vector):
        """ Returns the unit vector of the vector.  
        
        Based on https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
        """
        
        return vector / norm(vector)

    def _angleBetween(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'.
        """
        # return np.arctan2(np.cross(v1,v2), np.dot(v1,v2))

        sign = np.sign(np.cross(v1,v2))
        dot_product = np.clip(np.dot(self._unitVector(v1), self._unitVector(v2)), -1.0, 1.0)
        return sign * np.arccos(dot_product)

    def _lineIntersection(self, line1, line2):
        """ Compute the point where two lines intersect, or None if they not.
        
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

    def _rotateVector(self, vector, angle):
        """ Rotate a vector
        """
        rotation = np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])
        return np.dot(rotation, vector)

    def plotMapAndRobot(self, point, versor, walls):

        # walls
        for wall in walls:
            plt.plot([wall[0][0], wall[1][0]] , [wall[0][1], wall[1][1]])

        plt.xlim([-1, CELLCOLS * 2 + 1])
        plt.xticks(range(0, CELLCOLS * 2 + 1, 2))

        plt.ylim([-1, CELLROWS * 2 + 1])
        plt.yticks(range(0, CELLROWS * 2 + 1, 2))

        # fov
        fov0 = self._rotateVector(versor, -1 * self.ir_fov / 2)
        fov0 = np.multiply(fov0, 100)

        fov1 = self._rotateVector(versor, self.ir_fov / 2)
        fov1 = np.multiply(fov1, 100)

        for end_point in [np.add(point, fov0), np.add(point, fov1)]:
            plt.plot([point[0], end_point[0]], [point[1], end_point[1]])

        plt.grid(True)
        plt.show()

    def plotProbabilitiesMap(self, wait=0.1):

        plt.imshow(self.cells_probability, cmap='gray', interpolation='nearest', vmin=0, vmax=1)

        plt.ion()
        plt.show()
        plt.pause(wait)

    def _getCellSensorsPoses(self, row, col):
        """ Computes all the sensor poses for a given cell
        """

        cell_center = (self.cell_size * (col + 0.5), self.cell_size * (row + 0.5))

        versors = [self._rotateVector((1,0), np.deg2rad(x)) for x in self.angs]

        positions = [np.add(cell_center, np.multiply(versor, 0.5)) for versor in versors]

        return (positions, versors)

    def _minDistanceToWall(self, point, wall_points):
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

    def _getWallsCorners(self):
        """ Compute the corners coordinates (X and Y) of all walls in the map.
        Also, it accounts for thickness. 
        """

        self.walls = []
        wall_indices = []

        # inner walls indices
        for row in range(CELLROWS * self.cell_size - 1):
            for col in range(CELLCOLS * self.cell_size - 1):                
                if self.labMap[row][col] in ['-', '|']:
                    wall_indices.append((row, col))

        # inner walls corners
        for wall in wall_indices:

            row = wall[0]
            col = wall[1]

            wall_direction = self.labMap[row][col]

            if wall_direction == '|': # vertical wall
                
                side_1 = [((col-1) + self.cell_size -self.wall_thick/2, row), ((col-1)+ self.cell_size -self.wall_thick/2, row+2)]
                side_2 = [((col-1) + self.cell_size + self.wall_thick/2, row), ((col-1) + self.cell_size + self.wall_thick/2, row + self.cell_size)]

                self.walls.append(side_1)
                self.walls.append(side_2)

            elif wall_direction == '-': # horizontal wall

                side_1 = [(col, (row-1) + self.cell_size -self.wall_thick/2), (col+2, (row-1) + self.cell_size -self.wall_thick/2)]
                side_2 = [(col,(row-1) + self.cell_size +self.wall_thick/2), (col+2, (row-1) + self.cell_size + self.wall_thick/2)]

                self.walls.append(side_1)
                self.walls.append(side_2) 

        # outter walls corners
        corner0 = (0,0)
        corner1 = (0, CELLROWS * self.cell_size)
        corner2 = (CELLCOLS * self.cell_size, CELLROWS * self.cell_size)
        corner3 = (CELLCOLS * self.cell_size, 0)

        self.walls.append([corner0, corner1])
        self.walls.append([corner0, corner3])
        self.walls.append([corner2, corner1])
        self.walls.append([corner2, corner3])
        
        
    def _computeClosestDistanceToWall(self, point, versor, wall):
        """ Computes the closest distance to a given wall. If not in FOV, 
        returns None
        """

        distance = None
        
        # compute angle to wall extremities
        sensor_to_corner0 = np.subtract(wall[0], point)
        sensor_to_corner1 = np.subtract(wall[1], point)

        angle0 = self._angleBetween(versor, sensor_to_corner0)
        angle1 = self._angleBetween(versor, sensor_to_corner1)

        # check if inside FOV (divide by two to use only the one part of it)
        angle0_inside = np.abs(angle0) <= self.ir_fov / 2 
        angle1_inside = np.abs(angle1) <= self.ir_fov / 2 

        # case both angles inside fov
        if angle0_inside and angle1_inside:
            
            distance = self._minDistanceToWall(point, wall)

        # case wall is so close that both angles outside
        elif not (angle0_inside or angle1_inside) and (np.sign(angle0) != np.sign(angle1)):
            
            distance = self._minDistanceToWall(point, wall)

        # case only one corner is inside
        elif angle0_inside != angle1_inside:
            
            # rotate versor to the direction of the FOV bounds
            theta = np.sign(angle0) * self.ir_fov / 2
            fov_versor = self._rotateVector(versor, theta)

            # line from IR sensor
            line_ir = [point, np.add(point, fov_versor)]
            intersection = self._lineIntersection(line_ir, wall)

            subs_index = [angle0_inside, angle1_inside].index(False)
            wall_in_fov = wall.copy() 
            wall_in_fov[subs_index] = intersection
            
            distance = self._minDistanceToWall(point, wall_in_fov)           

        return distance

    def _computeCellMeasures(self, row, col):
        """ Compute the measure done for all sensores in a given cell.
        """

        # compute sensors poses
        sensors = self._getCellSensorsPoses(row, col)
        measures = []

        for dir in range(4):

            distances = []

            ir_point = sensors[0][dir]
            ir_versor = sensors[1][dir]

            for wall in self.walls:

                dist = self._computeClosestDistanceToWall(ir_point, ir_versor, wall)
                if dist is not None:
                    distances.append(dist)

            measures.append(1/min(distances))

        return measures

    def initProbabilities(self):

        probability = 1 / (CELLCOLS * CELLROWS)
        self.cells_probability = [[probability] * CELLCOLS for i in range(CELLROWS)]

    def computeGroundTruth(self):  
        """ Compute ground truth measures for all sensors and all cells.
        """

        self.ground_truth = []

        # get corners of all walls
        self._getWallsCorners()

        # iterate over each cell                
        for row in range(CELLROWS):
            self.ground_truth.append([self._computeCellMeasures(row, col) for col in range(CELLCOLS)])

    def computeTraveledDistance(self, in_left, in_right):
        """ Compute the traveled distance since the motion started.
        """

        noise = 1 # it is deterministic motion

        if self.last_position is None:
            self.last_position = [0] * 3

        if self.last_velocity is None:
            self.last_velocity = [0] * 2

        out_right = (in_right * 0.5 + self.last_velocity[0] * 0.5) * noise
        out_left = (in_left * 0.5 + self.last_velocity[1] * 0.5) * noise
        
        vel_linear = (out_left + out_right) / 2
        rotation = (out_left - out_right) / self.diameter

        x = self.last_position[0] + vel_linear * np.cos(self.last_position[2])
        y = self.last_position[1] + vel_linear * np.sin(self.last_position[2])
        theta = self.last_position[2] + rotation

        self.last_position = [x, y, theta]
        self.last_velocity = [out_right, out_left]

        return (x, y, theta)


    # def kf_predict(self, X, P, A, Q, B, U):
    #     X = np.dot(A, X) + np.dot(B, U)
    #     P = np.dot(A, np.dot(P, A.T)) + Q
    #     return(X,P) 

    # def kf_update(self, X, P, Y, H, R):
    #     IM = np.dot(H, X)
    #     IS = R + np.dot(H, np.dot(P, H.T))
    #     K = np.dot(P, np.dot(H.T, inv(IS)))
    #     X = X + np.dot(K, (Y-IM))
    #     P = P - np.dot(K, np.dot(IS, K.T))
    #     LH = self._gauss_pdf(Y, IM, IS)
    #     return (X,P,K,IM,IS,LH)

    # def _gauss_pdf(self, X, M, S):
    #     if M.shape()[1] == 1:
    #         DX = X - np.tile(M, X.shape()[1])
    #         E = 0.5 * sum(DX * (np.dot(inv(S), DX)), axis=0)
    #         E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
    #         P = exp(-E)
    #     elif X.shape()[1] == 1:
    #         DX = np.tile(X, M.shape()[1])- M
    #         E = 0.5 * sum(DX * (np.dot(inv(S), DX)), axis=0)
    #         E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
    #         P = exp(-E)
    #     else:
    #         DX = X-M
    #         E = 0.5 * np.dot(DX.T, np.dot(inv(S), DX))
    #         E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
    #         P = exp(-E)
    #     return (P[0],E[0]) 

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

        #######
        rob.initProbabilities()
        # rob.computeGroundTruth()

        # row = input('Row')
        # col = input('Col')

        # print(rob.ground_truth[int(row)][int(col)])
        # rob.readSensors()
        # print(rob.measures.irSensor)

        # rob.plotProbabilitiesMap(1)

    # rob.run()
