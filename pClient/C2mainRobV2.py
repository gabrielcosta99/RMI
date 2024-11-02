
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import heapq

CELLROWS=7
CELLCOLS=14
drawnMap = [[' ', '0'] * 28 for _ in range(27)]  # Correct initialization
for i in range(1,27,2):
    drawnMap[i] = ['0'] * 55
drawnMap[13][27] = 'I'
initialX, initialY = 13.0, 27.0
posX, posY = 13.0, 27.0
turning = (-1,-1)
toRotate = 0
positions_to_visit = []
search = False
e_m1, e_m2, u_m1 = 0,0,0
path = []

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))
            
    def addToMap(self,x,y,char):
        if (x % 2 == 1) or (y % 2 == 1):
            drawnMap[x][y] = char
        

    def run(self):
        global initialX, initialY
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        self.readSensors()
        initialX = self.measures.x - 27.0
        initialY = self.measures.y - 13.0
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
                    self.setVisitingLed(True);
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


    def wander(self):
        global toRotate, turning, positions_to_visit,search
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        # Kp = 0.015
        # e = (self.measures.irSensor[left_id]-self.measures.irSensor[right_id])
        posX = self.measures.x - initialX
        posY = self.measures.y - initialY
        mapX = round(posX)
        mapY = round( 26 - posY)
        rotation = self.measures.compass
        # print("center: ",self.measures.irSensor[center_id])
        # print("left: ",self.measures.irSensor[left_id])
        # print("right: ",self.measures.irSensor[right_id])
        # print("back: ",self.measures.irSensor[back_id])
        self.printDrawnMap()
        print("mapX:", mapX)
        print("mapY: ",mapY)
        print("rotation: ",rotation)
        print("center: ",self.measures.irSensor[center_id])
        print("left: ",self.measures.irSensor[left_id])
        print("right: ",self.measures.irSensor[right_id])
        print("back: ",self.measures.irSensor[back_id])
        print("posX: ",posX)
        print("posY (parsed): ",26 - posY)
        print("compass: ",self.measures.compass)
        print("toRotate: ",toRotate)

        

# Movement decision
        if toRotate > 0:
            if toRotate<0.3:
                u = toRotate/4
                self.driveMotors(-u,u)
                toRotate=0
            else:
                toRotate -=0.3
                self.driveMotors(-0.15,0.15)
        elif toRotate < 0 :
            if toRotate >-0.3:
                u = toRotate/4
                self.driveMotors(-u,u)
                toRotate=0
            else:
                toRotate +=0.3
                self.driveMotors(0.15,-0.15)
        else:
            if drawnMap[mapY][mapX] == '0' and (mapY % 2 == 1 or mapX % 2 == 1):
                drawnMap[mapY][mapX] = 'X'
                # print(drawnMap[mapY][mapX])
                
    # Wall draw

            if abs(rotation) <=2:
                if mapX % 2 == 1:
                    
                    if self.measures.irSensor[right_id] > 1.5:
                        # drawnMap[mapY+1][mapX] = '-'
                        self.addToMap(mapY+1,mapX,'-')
                        
                    if self.measures.irSensor[center_id] > 1.5:
                        # drawnMap[mapY][mapX+1] = '|'
                        self.addToMap(mapY,mapX+1,'|')
                        
                    if self.measures.irSensor[left_id] > 1.5:
                        # drawnMap[mapY-1][mapX] = '-'
                        self.addToMap(mapY-1,mapX,'-')
                        
            elif abs(rotation) >= 178:
                if mapX % 2 == 1:
                    

                    if self.measures.irSensor[right_id] > 1.5:
                        # drawnMap[mapY-1][mapX] = '-'
                        self.addToMap(mapY-1,mapX,'-')
                        
                    if self.measures.irSensor[center_id] > 1.5:
                        # drawnMap[mapY][mapX-1] = '|'
                        self.addToMap(mapY,mapX-1,'|')
                        
                    if self.measures.irSensor[left_id] > 1.5:
                        # drawnMap[mapY+1][mapX] = '-'
                        self.addToMap(mapY+1,mapX,'-')
                        
            elif rotation > 88 and rotation<92:
                if mapY % 2 == 1:
                    if self.measures.irSensor[right_id] > 1.5:
                        # drawnMap[mapY][mapX+1] = '|'
                        self.addToMap(mapY,mapX+1,'|')
                        
                    if self.measures.irSensor[center_id] > 1.5:
                        # drawnMap[mapY-1][mapX] = '-'
                        self.addToMap(mapY-1,mapX,'-')
                        
                    if self.measures.irSensor[left_id] > 1.5:
                        # drawnMap[mapY][mapX-1] = '|'
                        self.addToMap(mapY,mapX-1,'|')
                        
            elif rotation > -92 and rotation < -88:
                if mapY % 2 == 1:
                    if self.measures.irSensor[right_id] > 1.5:
                        # drawnMap[mapY][mapX-1] = '|'
                        self.addToMap(mapY,mapX-1,'|')
                        
                    if self.measures.irSensor[center_id] > 1.5:
                        # drawnMap[mapY+1][mapX] = '-'
                        self.addToMap(mapY+1,mapX,'-')
                        
                    if self.measures.irSensor[left_id] > 1.5:
                        # drawnMap[mapY][mapX+1] = '|'
                        self.addToMap(mapY,mapX+1,'|')

            if drawnMap[mapY][mapX+1] == "0" and (mapX+1,mapY) not in positions_to_visit:
                pos = (mapX+1,mapY)
                positions_to_visit.append(pos)
            if drawnMap[mapY][mapX-1] == "0" and (mapX-1,mapY) not in positions_to_visit:
                pos = (mapX-1,mapY)
                positions_to_visit.append(pos)
            if drawnMap[mapY+1][mapX] == "0" and (mapX,mapY+1) not in positions_to_visit:
                pos = (mapX,mapY+1)
                positions_to_visit.append(pos)
            if drawnMap[mapY-1][mapX] == "0" and (mapX,mapY-1) not in positions_to_visit:
                pos = (mapX,mapY-1)
                positions_to_visit.append(pos)

            pos = (mapX,mapY)
            if pos in positions_to_visit:
                positions_to_visit.remove(pos)
            if search:
                self.searchAlgorithm(mapX,mapY)
                print(positions_to_visit)
            else:
                print("Roaming")
                self.roam(mapX,mapY)            
        print()

    def getClosestPosIdx(self,positions_to_visit,mapX,mapY):
        minDist = 1000
        closestIdx = 0
        i = 0
        while i < len(positions_to_visit):
            pos = positions_to_visit[i]
            if drawnMap[pos[1]][pos[0]] != "0":
                positions_to_visit.pop(i)
                continue
            path = self.a_star(drawnMap,(mapX,mapY),pos)
            # print("testing: ",pos," generated path: ",path)
            dist = len(path)
            if dist < minDist:
                minDist = dist
                closestIdx = i
            i+=1
        return closestIdx

    def roam(self,mapX,mapY):
        global toRotate, turning, positions_to_visit,search,path
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        #check rotation first
        movingHorizontaly = abs(self.measures.compass) < 60.0 or abs(self.measures.compass) > 120.0
        print("movingHorizontaly: ",movingHorizontaly)

        # compass = self.measures.compass
        if drawnMap[mapY][mapX+1] != "0" and drawnMap[mapY][mapX-1] != "0" and drawnMap[mapY+1][mapX] !="0" and drawnMap[mapY-1][mapX] != "0":
            search = True
            self.driveMotors(0.0,0.0)
            try:
                pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                # while drawnMap[pos_to_reach[1]][pos_to_reach[0]] != "0":
                #     pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                print("pos_to_reach: ",pos_to_reach, "value: ",drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                path = self.a_star(drawnMap,(mapX,mapY),pos_to_reach)
                print("path: ",path)
                self.writeDrawnMap()
            except:
                self.printDrawnMap()
                self.writeDrawnMap()
                self.finish()
                return
            return
        
        if movingHorizontaly:
            # if (mapX)%2==0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
            #     print("Keep going forward")
            #     self.driveMotors(0.1,0.1)
            # else:   # We are at the center of the cell, we may go to the right
            
            if self.measures.irSensor[right_id] < 1.2\
            and ((abs(self.measures.compass) < 60 and drawnMap[mapY+1][mapX] == '0') \
            or (abs(self.measures.compass)>120 and drawnMap[mapY-1][mapX] == '0')):
                if (mapX)%2==0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
                    print("Keep going forward")
                    self.driveMotors(0.1,0.1)
                else:
                    print('Rotate riiiiiiiiiiight')
                    self.driveMotors(0.15,0.15)
                    toRotate = -pi/2
            # if there is an open space to the left that we havent visited and we have already visited the front cell, turn left
            elif self.measures.irSensor[left_id] <1.0 \
                and (((abs(self.measures.compass) < 60 and drawnMap[mapY][mapX+1] != '0') \
                and (abs(self.measures.compass) < 60 and drawnMap[mapY-1][mapX] == '0')) \
                or ((abs(self.measures.compass) > 120 and drawnMap[mapY][mapX-1] != '0' ) \
                and (abs(self.measures.compass)>120 and drawnMap[mapY+1][mapX] == '0'))):
                if (mapX)%2==0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
                    print("Keep going forward")
                    self.driveMotors(0.1,0.1)
                else:
                    print('Rotate leeeeeft')
                    # print(drawnMap[mapY][mapX+1]) if abs(self.measures.compass) < 60 else print(drawnMap[26 - (int(posY+0.5))][int(posX+0.5)-1])
                    self.driveMotors(0.15,0.15)
                    toRotate = pi/2
                
            elif (self.measures.compass>1 and self.measures.compass <59) or (self.measures.compass>-179 and self.measures.compass<-121):
                print("Rotate slightly right")
                if self.measures.compass > 0:
                    toRotate = (0-self.measures.compass)*pi/180
                else:
                    toRotate = (-180-self.measures.compass)*pi/180
            elif (self.measures.compass<179 and self.measures.compass >121) or (self.measures.compass>-59 and self.measures.compass<-1):
                print("Rotate slightly left")
                if self.measures.compass > 0:
                    toRotate = (180 - self.measures.compass)*pi/180
                else:
                    toRotate = (0 - self.measures.compass)* pi/180
            elif self.measures.irSensor[center_id] > 1.7 and self.measures.irSensor[left_id] >1.3 and self.measures.irSensor[right_id]>1.3:
                print('------> TURN AROUND <-------- ')
                self.driveMotors(0.0,0.0)
                toRotate = pi
                search = True
                try:
                    pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                    # while drawnMap[pos_to_reach[1]][pos_to_reach[0]] != "0":
                    #     pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                    #     print("checking: ",pos_to_reach, "value: ",drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                    print("pos_to_reach: ",pos_to_reach, "value: ",drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                    path = self.a_star(drawnMap,(mapX,mapY),pos_to_reach)
                    print("path: ",path)
                    self.writeDrawnMap()
                except:
                    self.printDrawnMap()
                    self.finish()                       
                    return
            else:
                print("Go")
                self.driveMotors(0.15,0.15)
        else:
            # if (mapY)%2 == 0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:
            #     print("Keep going forward")
            #     self.driveMotors(0.1,0.1)
            # else:
            if self.measures.irSensor[right_id] < 1.2 \
            and ((self.measures.compass > 0 and drawnMap[mapY][mapX+1] == '0') \
            or (self.measures.compass<0 and drawnMap[mapY][mapX-1] == '0')):
                if (mapY)%2 == 0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:
                    print("Keep going forward")
                    self.driveMotors(0.1,0.1)
                else:
                    print('Rotate riiiiiiiiiiight')
                    self.driveMotors(0.15,0.15)
                    toRotate = -pi/2
            elif self.measures.irSensor[left_id] <1.0\
            and (((self.measures.compass > 0 and drawnMap[mapY-1][mapX] != '0') \
            and (self.measures.compass > 0 and drawnMap[mapY][mapX-1] == '0')) \
            or ((self.measures.compass < 0 and drawnMap[mapY+1][mapX] != '0') \
            and (self.measures.compass < 0 and drawnMap[mapY][mapX+1] == '0'))):
                if (mapY)%2 == 0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:
                    print("Keep going forward")
                    self.driveMotors(0.1,0.1)
                else:
                    print('Rotate leeeeeft')
                    # print(drawnMap[26 -( int(posY+0.5)+2)][int(posX+0.5)]) if self.measures.compass >0 else print(drawnMap[26 -( int(posY+0.5)-2)][int(posX+0.5)])
                    self.driveMotors(0.15,0.15)
                    toRotate = pi/2
            elif (self.measures.compass>91 and self.measures.compass <119) or (self.measures.compass>-89 and self.measures.compass<-61):
                print("Rotate slightly right")
                if self.measures.compass > 0:
                    toRotate = (90-self.measures.compass)*pi/180
                else:
                    toRotate = (-90-self.measures.compass)*pi/180
            elif (self.measures.compass<89 and self.measures.compass >61) or (self.measures.compass>-119 and self.measures.compass<-91):
                print("Rotate slightly left")
                if self.measures.compass > 0:
                    toRotate = (90-self.measures.compass)*pi/180
                else:
                    toRotate = (-90-self.measures.compass)*pi/180
            elif self.measures.irSensor[center_id] > 1.7 and self.measures.irSensor[left_id] >1.7 and self.measures.irSensor[right_id]>1.7:
                print('------> TURN AROUND <-------- ')
                self.driveMotors(0.15,0.15)
                toRotate = pi
                try:
                    search = True
                    pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                    # while drawnMap[pos_to_reach[1]][pos_to_reach[0]] != "0":
                    #     pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                    #     print("checking: ",pos_to_reach, "value: ",drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                    print("pos_to_reach: ",pos_to_reach, "value: ",drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                    path = self.a_star(drawnMap,(mapX,mapY),pos_to_reach)
                    print("path: ",path)
                    self.writeDrawnMap()
                except:
                    self.printDrawnMap()
                    self.finish()
                    return
            else:
                print("Go")
                self.driveMotors(0.15,0.15)

        
            
    def a_star(self,grid, start, goal): # returns a path from "start" to "goal" using A*
        def heuristic(a, b):
            # Manhattan distance on a square grid
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        # Directions (down, up, right, left) for movement
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        open_list = []
        heapq.heappush(open_list, (0, start))  # (cost, cell)
        came_from = {start: None}
        cost_so_far = {start: 0}

        while open_list:
            current_cost, current = heapq.heappop(open_list)

            # If the goal is reached, reconstruct path
            if (abs(current[0]- goal[0]) == 1  and abs(current[1]-goal[1])==0)or( abs(current[1]-goal[1])==1 and abs(current[0]-goal[0])==0):
                path = []
                while current:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]  # Return reversed path

            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                # Check bounds and that neighbor is walkable
                if grid[neighbor[1]][neighbor[0]] == 'X' or grid[neighbor[1]][neighbor[0]] == 'I':
                #and 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0])  :
                    new_cost = cost_so_far[current] + 1  # Each step costs 1

                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (priority, neighbor))
                        came_from[neighbor] = current
        return None  # Return None if no path is found


    def searchAlgorithm(self,mapX,mapY):
        global toRotate, turning, positions_to_visit,search,path
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        #check rotation first
        print("SEARCH")
        print("path: ",path)
        # exit()
        #plan the path
        if path[0] == (mapX,mapY):
            path.pop(0)
            if path == []:
                search = False
                return
        next_position = path[0]
        diff = (next_position[0]-mapX,next_position[1]-mapY)
        print("diff: ",diff)
        movingHorizontaly = False
        # compass = self.measures.compass
        if diff[0]!=0:  # move horizontaly
            if mapX%2==0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
                print("Keep going forward")
                self.driveMotors(0.15,0.15)
            if diff[0]==1: # if we have to move to the right
                if abs(self.measures.compass)>2:
                    toRotate = (0-self.measures.compass)*pi/180
                else:
                    print("Go")
                    self.driveMotors(0.15,0.15)

            else: # if we have to move to the left
                if abs(self.measures.compass)<178:
                    if self.measures.compass > 0:
                        toRotate = (180-self.measures.compass)*pi/180
                    else:
                        toRotate = (-180-self.measures.compass)*pi/180
                else:
                    print("Go")
                    self.driveMotors(0.15,0.15)

        else:
            if mapY%2 == 0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:
                print("Keep going forward")
                self.driveMotors(0.15,0.15)
            if diff[1]==1: # if we have to go down
                if self.measures.compass>-88 or self.measures.compass<-92:
                    toRotate = (-90-self.measures.compass)*pi/180
                else:
                    print("Go")
                    self.driveMotors(0.15,0.15)
            else: # if we have to go up
                if self.measures.compass<88 or self.measures.compass>92:
                    toRotate = (90-self.measures.compass)*pi/180
                else:
                    print("Go")
                    self.driveMotors(0.15,0.15)
        
        print()
        return 


    def printDrawnMap(self):
        for i in drawnMap:
            print("".join(i[:55]).replace('0',' '))
    
    def writeDrawnMap(self):
        mapFile = open("outMap.out", "w")
        mapFile.write("")
        mapFile.close()
        mapOut = open("outMap.out", "a")
        for i in drawnMap:
            mapOut.write("".join(i[:55]).replace('0',' '))
            mapOut.write('\n')
        mapOut.close()
        

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


rob_name = "pClient2"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    rob.printDrawnMap()
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
