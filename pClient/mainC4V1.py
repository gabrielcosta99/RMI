
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import heapq
import math
import itertools

CELLROWS=7
CELLCOLS=14
# initialX, initialY = 13.0, 27.0
posX, posY = 27.0, 13.0
outL,outR = 0.0,0.0
turning = (-1,-1)
toRotate = 0
positions_to_visit = []
search = False
e_m1, e_m2, u_m1 = 0,0,0
path = []

obj_pos = [(27,13)]
prev_beacons = [300]
detected = False

def nearestOdd(num):
    rounded = int(num)
    if rounded % 2 == 0:
        return rounded + 1
    return rounded

def nearest_multiple_of_90(value):
    return round(value / 90) * 90

def redirect(mapX,mapY,path,compass):
    toRotate = 0
    next_position = path[0] if path[0] != (mapX,mapY) else path[1]
    diff = (next_position[0]-mapX,next_position[1]-mapY)
    print("redirect_diff: ",diff)
    if diff[0]!=0:  # move horizontaly
        if diff[0]==1: # if we have to move to the right
            if abs(compass)>5:
                toRotate = (0-compass)*pi/180
        else: # if we have to move to the left
            if abs(compass)<175:
                if compass > 0:
                    toRotate = (180-compass)*pi/180
                else:
                    toRotate = (-180-compass)*pi/180
    else:
        if diff[1]==1: # if we have to go down
            if compass>-85 or compass<-95:
                toRotate = (-90-compass)*pi/180
        else: # if we have to go up
            if compass<85 or compass>95:
                toRotate = (90-compass)*pi/180
    return toRotate


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.drawnMap = [[' ', '0'] * 28 for _ in range(27)]  # Correct initialization
        for i in range(1,27,2):
            self.drawnMap[i] = ['0'] * 55
        self.drawnMap[13][27] = 'I'

        #variables to adjust the wheels speed if in the last iteration, the robot moved a bit to the left or to the right
        self.oldCompass = 200
        self.offsetLeft = 0
        self.offsetRight = 0
        
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))
            
    def addToMap(self,x,y,char):
        if (x % 2 == 1) or (y % 2 == 1):
            self.drawnMap[x][y] = char

    def calculateOut(self,inL,inR):
        global outL,outR
        #outx = (min_max(inx, −0.15, 0.15) + outx−1)/2
        # guarantee that inL and inR are between -0.15 and 0.15
        inL = min(max(inL,-0.15),0.15)
        inR = min(max(inR,-0.15),0.15)
        # outL = (inL+outL)/2
        # outR = (inR+outR)/2
        print("inL and inR: ",inL,inR)
        outL = 0.90*inL +outL*0.10
        outR = 0.90*inR +outR*0.10
    
    def calculateXandY(self,outL,outR):
        global posX,posY
        # lin = (outL+outR)/2
        # posX = posX(t-1) + lin * cos(rot(t-1))
        lin = (outL+outR)/2
        # rot = self.measures.compass-toRotate
        rot = self.measures.compass #- toRotate/(pi/180)  # rotation in the previous instant
        print("(calcXandY) rot:",rot)
        posX += lin * cos(rot*pi/180)
        posY -= lin * sin(rot*pi/180)
    
    def rotToAxis(self):
        if abs(self.measures.compass) <= 10:
            return 'E'                                   # right
        elif self.measures.compass >= 80 and  self.measures.compass <= 100:
            return 'N'                                   # up
        elif abs(self.measures.compass) >= 170:
            return 'W'                                    # left
        elif self.measures.compass >= -100 and  self.measures.compass <= -80:
            return 'S'                                    # down
        
    def calibratePos(self):
        global posX, posY
        center_id = 0
        left_id = 1
        right_id = 2
        rotAx = self.rotToAxis()
        auxX = nearestOdd(posX)
        auxY = nearestOdd(posY)
        print("(auxX,auxY,rotAx): ({},{},{})".format(auxX,auxY,rotAx))
        if self.measures.irSensor[center_id] > 1.5:
            if rotAx == 'E':
                posX = auxX + 1 - 1/(self.measures.irSensor[center_id]) - 0.5
            elif rotAx == 'N':
                posY = auxY - 1 + 1/(self.measures.irSensor[center_id]) + 0.5
            elif rotAx == 'W':
                posX = auxX - 1 + 1/(self.measures.irSensor[center_id]) + 0.5
            elif rotAx == 'S':
                posY = auxY + 1 - 1/(self.measures.irSensor[center_id]) - 0.5
                
        
        if self.measures.irSensor[right_id] > 1.5 and self.measures.irSensor[right_id] > self.measures.irSensor[left_id]:
            if rotAx == 'E':
                posY = auxY + 1 - 1/(self.measures.irSensor[right_id]) - 0.5
            elif rotAx == 'N':
                posX = auxX + 1 - 1/(self.measures.irSensor[right_id]) - 0.5
            elif rotAx == 'W':
                posY = auxY - 1 + 1/(self.measures.irSensor[right_id]) + 0.5
            elif rotAx == 'S':
                posX = auxX - 1 + 1/(self.measures.irSensor[right_id]) + 0.5
                
        elif self.measures.irSensor[left_id] > 1.5:
            if rotAx == 'E':
                posY = auxY - 1 + 1/(self.measures.irSensor[left_id]) + 0.5
            elif rotAx == 'N':
                posX = auxX - 1 + 1/(self.measures.irSensor[left_id]) + 0.5
            elif rotAx == 'W':
                posY = auxY + 1 - 1/(self.measures.irSensor[left_id]) - 0.5
            elif rotAx == 'S':
                posX = auxX + 1 - 1/(self.measures.irSensor[left_id]) - 0.5
                

    def run(self):
        global initialX, initialY, prev_beacons, obj_pos
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        self.readSensors()
        initialX = self.measures.x - 27.0
        initialY = self.measures.y - 13.0
        prev_beacons += (len(self.measures.beacon)-1) * [200]
        obj_pos += [(0,0)] * (len(self.measures.beacon)-1)
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
        global toRotate, turning, positions_to_visit,search, detected,posX,posY
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        inL,inR = 0.0,0.0
        gpsPosX = self.measures.x - initialX
        gpsPosY = 26 - (self.measures.y - initialY)
        rotation = self.measures.compass
        # mapX = round(posX)
        # mapY = round(posY)
        self.printDrawnMap()
        
        print("rotation: ",rotation)
        print("center: ",self.measures.irSensor[center_id])
        print("left: ",self.measures.irSensor[left_id])
        print("right: ",self.measures.irSensor[right_id])
        # print("back: ",self.measures.irSensor[back_id])
        print("compass: ",self.measures.compass)
        print("beacon:", self.measures.beacon)
        print("objectives:", obj_pos)

        self.calibratePos()
        mapX = round(posX)
        mapY = round(posY)
        print("mapX:", mapX)
        print("mapY: ",mapY)
        print("posX: ",posX)
        print("posY: ",posY)
        print("gpsPosX and Y: ",gpsPosX,gpsPosY)
        print("diff between gps and pos: ",gpsPosX-posX,gpsPosY-posY)
        print("marked positions: ",positions_to_visit)

        if self.drawnMap[mapY][mapX] == '0' and (mapY % 2 == 1 or mapX % 2 == 1):
            self.drawnMap[mapY][mapX] = 'X'
            # print(self.drawnMap[mapY][mapX])

# Map draw -----------------------------------------------------------------------
        upPosNotMapped = self.drawnMap[mapY-1][mapX] == "0"
        downPosNotMapped = self.drawnMap[mapY+1][mapX] == "0"
        leftPosNotMapped = self.drawnMap[mapY][mapX-1] == "0"
        rightPosNotMapped = self.drawnMap[mapY][mapX+1] == "0"
        decimalX = round(posX*10)%10
        decimalY = round(posY*10)%10
        if abs(rotation) <=5:
            if mapX % 2 == 1 and (decimalX >= 8 or decimalX <=2):  # only count as an odd number if the number is between .8 and .3
                
                if self.measures.irSensor[right_id] > 1.7 and downPosNotMapped:
                    # self.drawnMap[mapY+1][mapX] = '-'
                    self.addToMap(mapY+1,mapX,'-')
                    
                if self.measures.irSensor[center_id] > 1.7 and rightPosNotMapped:
                    # self.drawnMap[mapY][mapX+1] = '|'
                    self.addToMap(mapY,mapX+1,'|')
                    
                if self.measures.irSensor[left_id] > 1.7 and upPosNotMapped:
                    # self.drawnMap[mapY-1][mapX] = '-'
                    self.addToMap(mapY-1,mapX,'-')
                    
        elif abs(rotation) >= 175:
            if mapX % 2 == 1 and (decimalX >= 8 or decimalX <=2): # only count as an odd number if the number is between .8 and .3
                

                if self.measures.irSensor[right_id] > 1.7 and upPosNotMapped:
                    # self.drawnMap[mapY-1][mapX] = '-'
                    self.addToMap(mapY-1,mapX,'-')
                    
                if self.measures.irSensor[center_id] > 1.7 and leftPosNotMapped:
                    # self.drawnMap[mapY][mapX-1] = '|'
                    self.addToMap(mapY,mapX-1,'|')
                    
                if self.measures.irSensor[left_id] > 1.7 and downPosNotMapped:
                    # self.drawnMap[mapY+1][mapX] = '-'
                    self.addToMap(mapY+1,mapX,'-')
                    
        elif rotation >= 85 and rotation<=95:
            if mapY % 2 == 1 and (decimalY >= 8 or decimalY <=2): # only count as an odd number if the number is between .7 and .3
                if self.measures.irSensor[right_id] > 1.7 and rightPosNotMapped:
                    # self.drawnMap[mapY][mapX+1] = '|'
                    self.addToMap(mapY,mapX+1,'|')
                    
                if self.measures.irSensor[center_id] > 1.7 and upPosNotMapped:
                    # self.drawnMap[mapY-1][mapX] = '-'
                    self.addToMap(mapY-1,mapX,'-')
                    
                if self.measures.irSensor[left_id] > 1.7 and leftPosNotMapped:
                    # self.drawnMap[mapY][mapX-1] = '|'
                    self.addToMap(mapY,mapX-1,'|')
                    
        elif rotation >= -95 and rotation <= -85:
            if mapY % 2 == 1 and (decimalY >= 8 or decimalY <=2): # only count as an odd number if the number is between .7 and .3
                if self.measures.irSensor[right_id] > 1.7 and leftPosNotMapped:
                    # self.drawnMap[mapY][mapX-1] = '|'
                    self.addToMap(mapY,mapX-1,'|')
                    
                if self.measures.irSensor[center_id] > 1.7 and downPosNotMapped:
                    # self.drawnMap[mapY+1][mapX] = '-'
                    self.addToMap(mapY+1,mapX,'-')
                    
                if self.measures.irSensor[left_id] > 1.7 and rightPosNotMapped:
                    # self.drawnMap[mapY][mapX+1] = '|'
                    self.addToMap(mapY,mapX+1,'|')

        if self.drawnMap[mapY][mapX+1] == "0" and (mapX+1,mapY) not in positions_to_visit:
            pos = (mapX+1,mapY)
            positions_to_visit.append(pos)
        if self.drawnMap[mapY][mapX-1] == "0" and (mapX-1,mapY) not in positions_to_visit:
            pos = (mapX-1,mapY)
            positions_to_visit.append(pos)
        if self.drawnMap[mapY+1][mapX] == "0" and (mapX,mapY+1) not in positions_to_visit:
            pos = (mapX,mapY+1)
            positions_to_visit.append(pos)
        if self.drawnMap[mapY-1][mapX] == "0" and (mapX,mapY-1) not in positions_to_visit:
            pos = (mapX,mapY-1)
            positions_to_visit.append(pos)

        pos = (mapX,mapY)
        if pos in positions_to_visit:
            positions_to_visit.remove(pos)
            
# Objective detection -----------------------------------------------------------------------------

        if (self.measures.ground >= 1  and self.measures.ground <= len(self.measures.beacon)) and prev_beacons != ([300] * len(self.measures.beacon)):
            obj_pos[self.measures.ground] = (mapX,mapY)
            if mapX % 2 == 1 and mapY % 2 == 1:
                self.addToMap(mapY,mapX,str(self.measures.ground))
            prev_beacons[self.measures.ground] = 300
            if prev_beacons == ([300] * len(self.measures.beacon)):
                detected = True
                path = self.shortestPath()
                self.savePath(path)
            

# Movement decision -------------------------------------------------------------------------------
        if toRotate != 0 and abs(toRotate) <0.3:
            target_compass = nearest_multiple_of_90(self.measures.compass)
            print("target_compass: ",target_compass)
            if (abs(target_compass- (360-self.measures.compass)) > 3 and abs(self.measures.compass) >= 177) \
            or (abs(target_compass-self.measures.compass) > 3 and abs(self.measures.compass) < 177):
                toRotate = math.radians(target_compass-self.measures.compass)
            else:
                toRotate = 0
            print("toRotate: ",toRotate)
        if toRotate > 0:
            if toRotate<0.3:
                # u = toRotate/4
                # self.driveMotors(-u,u)
                inL = -toRotate/2
                inR = toRotate/2
                # toRotate=0
                self.driveMotors(inL,inR)
            else:
                # toRotate -=0.3
                inL = -0.15
                inR = 0.15
                self.driveMotors(inL,inR)

                self.calculateOut(inL,inR)
                toRotate -= (outR-outL) 
        elif toRotate < 0 :
            if toRotate >-0.3:
                # u = toRotate/4
                # self.driveMotors(-u,u)
                inL = -toRotate/2
                inR = toRotate/2
                self.driveMotors(inL,inR)
                # toRotate=0
               
            else:
                # toRotate +=0.3
                inL = 0.15
                inR = -0.15
                # self.driveMotors(0.15,-0.15)
                self.driveMotors(inL,inR)
                self.calculateOut(inL,inR)
                toRotate -= (outR-outL) 
        else:
            self.offsetLeft = 0
            self.offsetRight = 0
            if self.oldCompass < 200:
                self.oldCompass = 360+self.oldCompass if self.oldCompass < 0 and abs(self.oldCompass)>170 else self.oldCompass
                compass =  360+self.measures.compass if self.measures.compass <0 and abs(self.oldCompass)>170 else self.measures.compass
                compassDiff = compass - self.oldCompass
                if abs(compassDiff) > 4:
                    if compassDiff > 0:
                        self.offsetLeft = math.radians(compassDiff)/2
                    else:
                        self.offsetRight = math.radians(compassDiff)/2
                print("adjustment offsets: ",self.offsetLeft,self.offsetRight)
                self.oldCompass = 300
            elif self.oldCompass == 300:
                self.oldCompass = 200

            if search:
                # print("positions_to_visit: ",positions_to_visit)
                inL,inR = self.searchAlgorithm(mapX,mapY)
            else:
                print("Roaming")
                inL,inR = self.roam(mapX,mapY) 
            self.calculateOut(inL,inR)
            self.calculateXandY(outL,outR)
            # posX = posX + outL
            # posY = posY + outR
        
        print("outL,outR: ",outL,outL)
        print()

    def getClosestPosIdx(self,positions_to_visit,mapX,mapY):
        minDist = 1000
        closestIdx = 0
        for i in range(len(positions_to_visit)):
            pos = positions_to_visit[i]
            if self.drawnMap[pos[1]][pos[0]] != "0":
                continue
            path = self.a_star(self.drawnMap,(mapX,mapY),pos)
            # print("testing: ",pos," generated path: ",path)
            if path == None:
                continue
            dist = len(path)
            if dist < minDist:
                minDist = dist
                closestIdx = i
        return closestIdx
    



    def roam(self,mapX,mapY):
        global toRotate, turning, positions_to_visit,search,path
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        #check rotation first
        movingHorizontaly = False
        inL,inR = 0.0,0.0
        decimalX = round(posX*10)%10
        decimalY = round(posY*10)%10
        # compass = self.measures.compass
        if self.drawnMap[mapY][mapX+1] != "0" and self.drawnMap[mapY][mapX-1] != "0" and self.drawnMap[mapY+1][mapX] !="0" and self.drawnMap[mapY-1][mapX] != "0":
            search = True
            inL,inR = 0.0,0.0
            self.driveMotors(inL,inR)
            try:
                pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                while self.drawnMap[pos_to_reach[1]][pos_to_reach[0]] != "0":
                    pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                print("pos_to_reach: ",pos_to_reach, "value: ",self.drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                path = self.a_star(self.drawnMap,(mapX,mapY),pos_to_reach)
                print("path: ",path)
                toRotate = redirect(mapX,mapY,path,self.measures.compass)
                self.writeDrawnMap()
            except:
                self.printDrawnMap()
                self.writeDrawnMap()
                path = self.shortestPath()
                self.savePath(path)

                # self.finish()
                path = self.a_star(self.drawnMap,(mapX,mapY),obj_pos[0]) + [(27,13),-1]
                search = True
                
                return inL,inR
            return inL,inR
        
        if abs(self.measures.compass) < 60.0 or abs(self.measures.compass) > 120.0:
            movingHorizontaly = True
        if movingHorizontaly:
            # if (mapX)%2==0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
            #     print("Keep going forward")
            #     self.driveMotors(0.1,0.1)
            # else:   # We are at the center of the cell, we may go to the right
            
            if self.measures.irSensor[right_id] < 1.5\
            and ((abs(self.measures.compass) < 60 and self.drawnMap[mapY+1][mapX] == '0') \
            or (abs(self.measures.compass)>120 and self.drawnMap[mapY-1][mapX] == '0')):
                if ((mapX)%2==0 or (mapX%2==1 and (decimalX<=8 and decimalX >=2))) and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
                    print("Keep going forward")
                    inL,inR = 0.1,0.1
                    # self.driveMotors(inL,inR)
                else:
                    print('Rotate riiiiiiiiiiight')
                    inL,inR = 0.0,0.0
                    # self.driveMotors(inL,inR)
                    # toRotate = -pi/2
                    angleObjective = nearest_multiple_of_90(self.measures.compass-90)
                    toRotate = angleObjective*pi/180 - self.measures.compass*pi/180
            # if there is an open space to the left that we havent visited and we have already visited the front cell, turn left
            elif self.measures.irSensor[left_id] <1.5 \
                and (((abs(self.measures.compass) < 60 and self.drawnMap[mapY][mapX+1] != '0') \
                and (abs(self.measures.compass) < 60 and self.drawnMap[mapY-1][mapX] == '0')) \
                or ((abs(self.measures.compass) > 120 and self.drawnMap[mapY][mapX-1] != '0' ) \
                and (abs(self.measures.compass)>120 and self.drawnMap[mapY+1][mapX] == '0'))):
                if ((mapX)%2==0 or (mapX%2==1 and (decimalX<=8 and decimalX >=2))) and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
                    print("Keep going forward")
                    inL,inR = 0.1,0.1
                    # self.driveMotors(inL,inR)
                else:
                    print('Rotate leeeeeft')
                    inL,inR = 0.0,0.0
                    # self.driveMotors(inL,inR)
                    # toRotate = pi/2
                    angleObjective = nearest_multiple_of_90(self.measures.compass+90)
                    toRotate = angleObjective*pi/180 - self.measures.compass*pi/180
                
            elif (self.measures.compass>3 and self.measures.compass <57) or (self.measures.compass>-177 and self.measures.compass<-123):
                print("Rotate slightly right")
                if self.measures.compass > 0:
                    toRotate = (0-self.measures.compass)*pi/180
                else:
                    toRotate = (-180-self.measures.compass)*pi/180
            elif (self.measures.compass<177 and self.measures.compass >123) or (self.measures.compass>-57 and self.measures.compass<-3):
                print("Rotate slightly left")
                if self.measures.compass > 0:
                    toRotate = (180 - self.measures.compass)*pi/180
                else:
                    toRotate = (0 - self.measures.compass)* pi/180
            elif self.measures.irSensor[center_id] > 1.7 and self.measures.irSensor[left_id] >1.3 and self.measures.irSensor[right_id]>1.3:
                print('------> TURN AROUND <-------- ')
                inL,inR = 0.0,0.0
                # self.driveMotors(inL,inR)
                # toRotate = pi
                angleObjective = nearest_multiple_of_90(self.measures.compass+180)
                toRotate = angleObjective*pi/180 - self.measures.compass*pi/180
                search = True
                try:
                    pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                    while self.drawnMap[pos_to_reach[1]][pos_to_reach[0]] != "0":
                        pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                        print("checking: ",pos_to_reach, "value: ",self.drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                    print("pos_to_reach: ",pos_to_reach, "value: ",self.drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                    path = self.a_star(self.drawnMap,(mapX,mapY),pos_to_reach)
                    print("path: ",path)
                except:
                    self.printDrawnMap()
                    path = self.shortestPath()
                    self.savePath(path)
                    # self.finish()
                    path = self.a_star(self.drawnMap,(mapX,mapY),obj_pos[0]) + [(27,13),-1]
                    search = True
                    return
            else:
                print("Go")
                inL,inR = 0.15-self.offsetLeft,0.15-self.offsetRight
                self.oldCompass = self.measures.compass
                # self.driveMotors(inL,inR)
        else:
            # if (mapY)%2 == 0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:
            #     print("Keep going forward")
            #     self.driveMotors(0.1,0.1)
            # else:
            if self.measures.irSensor[right_id] < 1.5 \
            and ((self.measures.compass > 0 and self.drawnMap[mapY][mapX+1] == '0') \
            or (self.measures.compass<0 and self.drawnMap[mapY][mapX-1] == '0')):
                if ((mapY)%2 == 0 or (mapY%2==1 and (decimalY<=8 and decimalY >=2))) and self.measures.irSensor[center_id]<4.0:
                    print("Keep going forward")
                    inL,inR = 0.1,0.1
                    # self.driveMotors(inL,inR)
                else:
                    print('Rotate riiiiiiiiiiight')
                    inL,inR = 0.0,0.0
                    # self.driveMotors(inL,inR)
                    # toRotate = -pi/2
                    angleObjective = nearest_multiple_of_90(self.measures.compass-90)
                    toRotate = angleObjective*pi/180 - self.measures.compass*pi/180
            elif self.measures.irSensor[left_id] <1.5\
            and (((self.measures.compass > 0 and self.drawnMap[mapY-1][mapX] != '0') \
            and (self.measures.compass > 0 and self.drawnMap[mapY][mapX-1] == '0')) \
            or ((self.measures.compass < 0 and self.drawnMap[mapY+1][mapX] != '0') \
            and (self.measures.compass < 0 and self.drawnMap[mapY][mapX+1] == '0'))):
                if ((mapY)%2 == 0 or (decimalY<=8 and decimalY >=2))  and self.measures.irSensor[center_id]<4.0:
                    print("Keep going forward")
                    inL,inR = 0.1,0.1
                    # self.driveMotors(inL,inR)
                else:
                    print('Rotate leeeeeft')
                    inL,inR = 0.0,0.0
                    # self.driveMotors(inL,inR)
                    # toRotate = pi/2
                    angleObjective = nearest_multiple_of_90(self.measures.compass+90)
                    toRotate = angleObjective*pi/180 - self.measures.compass*pi/180
            elif (self.measures.compass>93 and self.measures.compass <117) or (self.measures.compass>-87 and self.measures.compass<-63):
                print("Rotate slightly right")
                if self.measures.compass > 0:
                    toRotate = (90-self.measures.compass)*pi/180
                else:
                    toRotate = (-90-self.measures.compass)*pi/180
            elif (self.measures.compass<87 and self.measures.compass >63) or (self.measures.compass>-117 and self.measures.compass<-93):
                print("Rotate slightly left")
                if self.measures.compass > 0:
                    toRotate = (90-self.measures.compass)*pi/180
                else:
                    toRotate = (-90-self.measures.compass)*pi/180
            elif (self.measures.irSensor[center_id] > 1.7 and self.measures.irSensor[left_id] >1.7 and self.measures.irSensor[right_id]>1.7):
                print('------> TURN AROUND <-------- ')
                inL,inR = 0.0,0.0
                self.driveMotors(inL,inR)
                # toRotate = pi
                angleObjective = nearest_multiple_of_90(self.measures.compass+180)
                toRotate = angleObjective*pi/180 - self.measures.compass*pi/180
                try:
                    search = True
                    pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                    while self.drawnMap[pos_to_reach[1]][pos_to_reach[0]] != "0":
                        pos_to_reach = positions_to_visit.pop(self.getClosestPosIdx(positions_to_visit,mapX,mapY))
                        print("checking: ",pos_to_reach, "value: ",self.drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                    print("pos_to_reach: ",pos_to_reach, "value: ",self.drawnMap[pos_to_reach[1]][pos_to_reach[0]])
                    path = self.a_star(self.drawnMap,(mapX,mapY),pos_to_reach)
                    print("path: ",path)
                    self.writeDrawnMap()
                except:
                    self.printDrawnMap()
                    self.writeDrawnMap()
                    path = self.shortestPath()
                    self.savePath(path)
                    # self.finish()
                    path = self.a_star(self.drawnMap,(mapX,mapY),obj_pos[0]) + [(27,13),-1]
                    search = True
                    return
            else:
                print("Go")
                inL,inR = 0.15-self.offsetLeft,0.15-self.offsetRight
                # self.driveMotors(inL,inR)

        self.driveMotors(inL,inR)
        print("movingHorizontaly: ",movingHorizontaly)
        return inL,inR

            
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
        inL,inR = 0.0, 0.0
        decimalX = round(posX*10)%10
        decimalY = round(posY*10)%10
        print("decimalX, decimalY: ",decimalX,decimalY)
        #check rotation first
        print("SEARCH")
        print("path: ",path)
        # exit()
        #plan the path
        if path[0] == (mapX,mapY):
            path.pop(0)
            if path == []:
                search = False
                print("-----------> SEARCH DONE <-----------------")
                return inL,inR
            elif path == [-1]:
                self.finish()
                return inL,inR
            if (decimalX<=2 or decimalX>=8) or (decimalY<= 2 or decimalY>=8):
                print("Keep going forward")
                inL,inR = 0.1,0.1
                self.driveMotors(inL,inR)
                return inL,inR
        next_position = path[0]
        diff = (next_position[0]-mapX,next_position[1]-mapY)
        print("diff: ",diff)
        movingHorizontaly = abs(self.measures.compass) < 60.0 or abs(self.measures.compass) > 120.0
        # compass = self.measures.compass
        if diff[0]!=0:  # move horizontaly
            # if mapX%2==0 or (mapX%2==1 and (decimalX<=8 and decimalX >=2)) and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
            #     print("Keep going forward")
            #     inL,inR = 0.1,0.1
                # self.driveMotors(inL,inR)
            if diff[0]==1: # if we have to move to the right
                if abs(self.measures.compass)>5:
                    if (not movingHorizontaly):
                        if (mapY%2 == 0 or (mapY%2==1 and (decimalY<=8 and decimalY >=2)) and self.measures.irSensor[center_id]<4.0):
                            print("Keep going forward")
                            inL,inR = 0.1,0.1
                        else:
                            toRotate = (0-self.measures.compass)*pi/180
                    elif movingHorizontaly :
                        toRotate = (0-self.measures.compass)*pi/180
                else:
                    print("Go")
                    inL,inR = 0.15-self.offsetLeft,0.15-self.offsetRight
                    self.oldCompass = self.measures.compass
                    # self.driveMotors(inL,inR)

            else: # if we have to move to the left
                if abs(self.measures.compass)<175:
                    if (not movingHorizontaly):
                        if (mapY%2 == 0 or (mapY%2==1 and (decimalY<=8 and decimalY >=2)) and self.measures.irSensor[center_id]<4.0):
                            print("Keep going forward")
                            inL,inR = 0.1,0.1
                        else:
                            if self.measures.compass > 0:
                                toRotate = (180-self.measures.compass)*pi/180
                            else:
                                toRotate = (-180-self.measures.compass)*pi/180
                    elif movingHorizontaly :
                        if self.measures.compass > 0:
                            toRotate = (180-self.measures.compass)*pi/180
                        else:
                            toRotate = (-180-self.measures.compass)*pi/180
                else:
                    print("Go")
                    inL,inR = 0.15-self.offsetLeft,0.15-self.offsetRight
                    self.oldCompass = self.measures.compass
                    # self.driveMotors(inL,inR)
        
        else:
            # if mapY%2 == 0 or (mapY%2==1 and (decimalY<=8 and decimalY >=2)) and self.measures.irSensor[center_id]<4.0:
            #     print("Keep going forward")
            #     inL,inR = 0.1,0.1
            # self.driveMotors(inL,inR)
            if diff[1]==1: # if we have to go down
                if self.measures.compass>-85 or self.measures.compass<-95:
                    if movingHorizontaly:
                        if (mapX%2 == 0 or (mapX%2==1 and (decimalX<=8 and decimalX >=2)) and self.measures.irSensor[center_id]<4.0):
                            print("Keep going forward")
                            inL,inR = 0.1,0.1
                        else:
                            toRotate = (-90-self.measures.compass)*pi/180
                    elif not movingHorizontaly :
                        toRotate = (-90-self.measures.compass)*pi/180
                else:
                    print("Go")
                    inL,inR = 0.15-self.offsetLeft,0.15-self.offsetRight
                    self.oldCompass = self.measures.compass
                    # self.driveMotors(inL,inR)
            else: # if we have to go up
                if self.measures.compass<85 or self.measures.compass>95:
                    if movingHorizontaly:
                        if (mapX%2 == 0 or (mapX%2==1 and (decimalX<=8 and decimalX >=2)) and self.measures.irSensor[center_id]<4.0):
                            print("Keep going forward")
                            inL,inR = 0.1,0.1
                        else:
                            toRotate = (90-self.measures.compass)*pi/180
                    elif not movingHorizontaly:
                        toRotate = (90-self.measures.compass)*pi/180
                else:
                    print("Go")
                    inL,inR = 0.15-self.offsetLeft,0.15-self.offsetRight
                    self.oldCompass = self.measures.compass
                    # self.driveMotors(inL,inR)
        
        self.driveMotors(inL,inR)
        print("movingHorizontaly: ",movingHorizontaly)
        
        print()
        return inL,inR

                    
    def printDrawnMap(self):
        for i in self.drawnMap:
            print("".join(i[:55]).replace('0',' '))
    
    def writeDrawnMap(self):
        mapFile = open("output.map", "w")
        mapFile.write("")
        mapFile.close()
        mapOut = open("output.map", "a")
        for i in self.drawnMap:
            mapOut.write("".join(i[:55]).replace('0',' '))
            mapOut.write('\n')
        mapOut.close()
            
    def printPath(self,path):
        for i in path:
            print(f"{i[0]} {i[1]}")
            
    def findPath(self,obj1,obj2,obj3):
        path = []
        path += self.a_star(self.drawnMap,obj1,obj2)
        path += self.a_star(self.drawnMap,obj2,obj3)
        path += self.a_star(self.drawnMap,obj3,obj1)
        path.append(obj1)
        return path
        
    def savePath(self, path):
        pathFile = open("output.path", "w")
        pathFile.write("")
        pathFile.close()
        pathOut = open("output.path", "a")
        for i in path:
            if i[0]%2 == 1 and i[1]%2 == 1:
                pathOut.write(f"{i[0]-27} {13-i[1]}\n")
        pathOut.write("0 0\n")
        pathOut.close()
        
    def shortestPath(self):
        shortLength = 200
        shortPath = None
        shortOrder = None
        
        # calculate every permutation of closed paths beginning at 27,13
        orders = [order for order in itertools.permutations(obj_pos) if order[0] == (27, 13)]
     
        for order in orders:
            auxPath = []
            auxOrder = list(order)
            auxOrder.append((27,13))  # Create closed path
            
            print("Beacon Order {}".format(order))
            for i in range(len(auxOrder) - 1):
                # print(auxOrder[i])
                # print(auxOrder[i + 1])
                segment = self.a_star(self.drawnMap, auxOrder[i], auxOrder[i + 1])
                if segment is None:
                    print (auxOrder[i])
                    print (auxOrder[i+1])
                auxPath.extend(segment)
            pathLength = len(auxPath)
            if pathLength < shortLength:
                shortLength = pathLength
                shortPath = auxPath
                shortOrder = auxOrder  


        print("Shortest path with known map {} with len {} and permutation {}".format(shortPath, shortLength, shortOrder))
        shortPath+=[(0,0)]
        print(shortPath)
        return shortPath

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
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
