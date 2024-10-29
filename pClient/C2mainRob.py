
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

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
backtrack = False
e_m1, e_m2, u_m1 = 0,0,0

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
        global toRotate, turning, positions_to_visit,backtrack
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        Kp = 0.015
        e = (self.measures.irSensor[left_id]-self.measures.irSensor[right_id])
        posX = self.measures.x - initialX
        posY = self.measures.y - initialY
        mapX = int(posX + 0.5)
        mapY = int( 26 - posY + 0.5)
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
        print("posY: ",posY)
        

        if drawnMap[mapY][mapX] == '0' and (mapY % 2 == 1 or mapX % 2 == 1):
            drawnMap[mapY][mapX] = 'X'
            # print(drawnMap[mapY][mapX])
            
# Wall draw

        if abs(rotation) <=20:
            if mapX % 2 == 1:
                if self.measures.irSensor[right_id] > 1.5:
                    drawnMap[mapY+1][mapX] = '-'
                    
                if self.measures.irSensor[center_id] > 1.5:
                    drawnMap[mapY][mapX+1] = '|'
                    
                if self.measures.irSensor[left_id] > 1.5:
                    drawnMap[mapY-1][mapX] = '-'
                
        elif abs(rotation) >= 160:
            if mapX % 2 == 1:
                if self.measures.irSensor[right_id] > 1.5:
                    drawnMap[mapY-1][mapX] = '-'
                    
                if self.measures.irSensor[center_id] > 1.5:
                    drawnMap[mapY][mapX-1] = '|'
                    
                if self.measures.irSensor[left_id] > 1.5:
                    drawnMap[mapY+1][mapX] = '-'
                    
        elif rotation > 80 and rotation<100:
            if mapY % 2 == 1:
                if self.measures.irSensor[right_id] > 1.5:
                    drawnMap[mapY][mapX+1] = '|'
                    
                if self.measures.irSensor[center_id] > 1.5:
                    drawnMap[mapY-1][mapX] = '-'
                
                    
                if self.measures.irSensor[left_id] > 1.5:
                    drawnMap[mapY][mapX-1] = '|'
                    
        elif rotation > -100 and rotation < -80:
            if mapY % 2 == 1:
                if self.measures.irSensor[right_id] > 1.5:
                    drawnMap[mapY][mapX-1] = '|'
               
                    
                if self.measures.irSensor[center_id] > 1.5:
                    drawnMap[mapY+1][mapX] = '-'
                    
                if self.measures.irSensor[left_id] > 1.5:
                    drawnMap[mapY][mapX+1] = '|'
               
            
# Movement decision
        if drawnMap[mapY][mapX+1] == "0" or drawnMap[mapY][mapX-1] == "0" or drawnMap[mapY+1][mapX]=="0" or drawnMap[mapY-1][mapX] == "0":
            print("CANCELING BACKTRACK")
            backtrack = False
        elif drawnMap[mapY][mapX+1] != "0" or drawnMap[mapY][mapX-1] != "0" or drawnMap[mapY+1][mapX]!="0" or drawnMap[mapY-1][mapX] != "0":
            print("BACKTRACKING")
            backtrack = True

        if backtrack:
            self.wanderBacktrack(mapX,mapY)
        else:
            self.wanderDepth(mapX,mapY)            
        print()


    def wanderDepth(self,mapX,mapY):
        global toRotate, turning, positions_to_visit,backtrack
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        #check rotation first
        movingHorizontaly = False
        # compass = self.measures.compass
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
            if abs(self.measures.compass) < 60.0 or abs(self.measures.compass) > 120.0:
                movingHorizontaly = True
            if movingHorizontaly:
                if (round(posX))%2==0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
                    print("Keep going forward")
                    self.driveMotors(0.15,0.15)
                else:   # We are at the center of the cell, we may go to the right
                    if turning != (-1,-1):
                        print("MAKING A TURN")
                        self.driveMotors(0.15,0.15)
                        if mapX-turning[0] != 0 or mapY-turning[1]!=0:
                            turning = (-1,-1)
                    
                    elif self.measures.irSensor[right_id] < 1.3\
                    and ((abs(self.measures.compass) < 60 and drawnMap[mapY+1][mapX] == '0') \
                    or (abs(self.measures.compass)>120 and drawnMap[mapY-1][mapX] == '0')):
                        print('Rotate riiiiiiiiiiight')
                        self.driveMotors(0.15,0.15)
                        toRotate = -pi/2
                        turning = (mapX,mapY)
                        positions_to_visit.append(turning)
                    elif self.measures.irSensor[left_id] <1.0 \
                    and ((abs(self.measures.compass) < 60 and drawnMap[mapY-1][mapX] == '0') \
                    or (abs(self.measures.compass)>120 and drawnMap[mapY+1][mapX] == '0')):
                        print('Rotate leeeeeft')
                        # print(drawnMap[26 - (int(posY+0.5))][int(posX+0.5)+1]) if abs(self.measures.compass) < 60 else print(drawnMap[26 - (int(posY+0.5))][int(posX+0.5)-1])
                        self.driveMotors(0.15,0.15)
                        toRotate = pi/2
                        turning = (mapX,mapY)
                        positions_to_visit.append(turning)
                    elif (self.measures.compass>2 and self.measures.compass <58) or (self.measures.compass>-178 and self.measures.compass<-122):
                        print("Rotate slightly right")
                        if self.measures.compass > 0:
                            toRotate = (0-self.measures.compass)*pi/180
                        else:
                            toRotate = (-180-self.measures.compass)*pi/180
                    elif (self.measures.compass<178 and self.measures.compass >122) or (self.measures.compass>-58 and self.measures.compass<-2):
                        print("Rotate slightly left")
                        if self.measures.compass > 0:
                            toRotate = (180 - self.measures.compass)*pi/180
                        else:
                            toRotate = (0 - self.measures.compass)* pi/180
                    elif self.measures.irSensor[center_id] > 1.7 and self.measures.irSensor[left_id] >1.3 and self.measures.irSensor[right_id]>1.3:
                        print('------> TURN AROUND <-------- ')
                        self.driveMotors(0.15,0.15)
                        toRotate = pi
                        backtrack = True
                    else:
                        print("Go")
                        self.driveMotors(0.15,0.15)
            else:
                if (round(posY))%2 == 0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:
                    print("Keep going forward")
                    self.driveMotors(0.15,0.15)
                else:
                    if turning != (-1,-1):
                        print("MAKING A TURN")
                        self.driveMotors(0.15,0.15)
                        if mapX-turning[0] != 0 or mapY-turning[1]!=0:
                            turning = (-1,-1)
                    elif self.measures.irSensor[right_id] < 1.0 \
                    and ((self.measures.compass > 0 and drawnMap[mapY][mapX+1] == '0') \
                    or (self.measures.compass<0 and drawnMap[mapY][mapX-1] == '0')):
                        print('Rotate riiiiiiiiiiight')
                        self.driveMotors(0.15,0.15)
                        toRotate = -pi/2
                        turning = (mapX,mapY)
                        positions_to_visit.append(turning)
                    elif self.measures.irSensor[left_id] <1.0\
                    and ((self.measures.compass > 0 and drawnMap[mapY][mapX-1] == '0') \
                    or (self.measures.compass < 0 and drawnMap[mapY][mapX+1] == '0')):
                        print('Rotate leeeeeft')
                        # print(drawnMap[26 -( int(posY+0.5)+2)][int(posX+0.5)]) if self.measures.compass >0 else print(drawnMap[26 -( int(posY+0.5)-2)][int(posX+0.5)])
                        self.driveMotors(0.15,0.15)
                        toRotate = pi/2
                        turning = (mapX,mapY)
                        positions_to_visit.append(turning)
                    elif (self.measures.compass>92 and self.measures.compass <118) or (self.measures.compass>-88 and self.measures.compass<-62):
                        print("Rotate slightly right")
                        if self.measures.compass > 0:
                            toRotate = (90-self.measures.compass)*pi/180
                        else:
                            toRotate = (-90-self.measures.compass)*pi/180
                    elif (self.measures.compass<88 and self.measures.compass >62) or (self.measures.compass>-118 and self.measures.compass<-92):
                        print("Rotate slightly left")
                        if self.measures.compass > 0:
                            toRotate = (90-self.measures.compass)*pi/180
                        else:
                            toRotate = (-90-self.measures.compass)*pi/180
                    elif self.measures.irSensor[center_id] > 1.7 and self.measures.irSensor[left_id] >1.7 and self.measures.irSensor[right_id]>1.7:
                        print('------> TURN AROUND <-------- ')
                        self.driveMotors(0.15,0.15)
                        toRotate = pi
                    else:
                        print("Go")
                        self.driveMotors(0.15,0.15)

            print("movingHorizontaly: ",movingHorizontaly)
            print("compass: ",self.measures.compass)
            print("toRotate: ",toRotate)


    def wanderBacktrack(self,mapX,mapY):
        print("BACKTRACKING")
        global toRotate, turning, positions_to_visit, backtrack
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        #check rotation first
        movingHorizontaly = False
        # compass = self.measures.compass
        if toRotate > 0:
            if toRotate<0.3:
                u = toRotate/4
                self.driveMotors(-u,u)
                toRotate=0
            else:
                toRotate -= 0.3
                self.driveMotors(-0.15,0.15)
        elif toRotate < 0 :
            if toRotate >-0.3:
                u = toRotate/4
                self.driveMotors(-u,u)
                toRotate=0
            else:
                toRotate += 0.3
                self.driveMotors(0.15,-0.15)
        else:
            if abs(self.measures.compass) < 60.0 or abs(self.measures.compass) > 120.0:
                movingHorizontaly = True
            if movingHorizontaly:
                if (round(posX))%2==0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
                    print("Keep going forward")
                    self.driveMotors(0.15,0.15)
                else:   # We are at the center of the cell, we may go to the right
                    if turning != (-1,-1):
                        print("MAKING A TURN")
                        self.driveMotors(0.15,0.15)
                        if mapX-turning[0] != 0 or mapY-turning[1]!=0:
                            turning = (-1,-1)
                    elif self.measures.irSensor[center_id] < 5.0\
                    and ((abs(self.measures.compass) < 2 and drawnMap[mapY][mapX+1] == 'X') \
                    or (abs(self.measures.compass)>178 and drawnMap[mapY][mapX-1] == 'X')):
                        print("GO FORWARD")
                        self.driveMotors(0.15,0.15)
                    elif self.measures.irSensor[right_id] < 1.0\
                    and ((abs(self.measures.compass) < 60 and drawnMap[mapY+1][mapX] == 'X') \
                    or (abs(self.measures.compass)>120 and drawnMap[mapY-1][mapX] == 'X')):
                        print('Rotate riiiiiiiiiiight')
                        self.driveMotors(0.15,0.15)
                        toRotate = -pi/2
                        turning = (mapX,mapY)
                        positions_to_visit.append(turning)
                    elif self.measures.irSensor[left_id] <1.0 \
                    and ((abs(self.measures.compass) < 60 and drawnMap[mapY-1][mapX] == 'X') \
                    or (abs(self.measures.compass)>120 and drawnMap[mapY+1][mapX] == 'X')):
                        print('Rotate leeeeeft')
                        # print(drawnMap[26 - (int(posY+0.5))][int(posX+0.5)+1]) if abs(self.measures.compass) < 60 else print(drawnMap[26 - (int(posY+0.5))][int(posX+0.5)-1])
                        self.driveMotors(0.15,0.15)
                        toRotate = pi/2
                        turning = (mapX,mapY)
                        positions_to_visit.append(turning)
                    elif (self.measures.compass>2 and self.measures.compass <58) or (self.measures.compass>-178 and self.measures.compass<-122):
                        print("Rotate slightly right")
                        if self.measures.compass > 0:
                            toRotate = (0-self.measures.compass)*pi/180
                        else:
                            toRotate = (-180-self.measures.compass)*pi/180
                    elif (self.measures.compass<178 and self.measures.compass >122) or (self.measures.compass>-58 and self.measures.compass<-2):
                        print("Rotate slightly left")
                        if self.measures.compass > 0:
                            toRotate = (180 - self.measures.compass)*pi/180
                        else:
                            toRotate = (0 - self.measures.compass)* pi/180
                    elif self.measures.irSensor[center_id] > 1.7 and self.measures.irSensor[left_id] >1.3 and self.measures.irSensor[right_id]>1.3:
                        print('------> TURN AROUND <-------- ')
                        self.driveMotors(0.15,0.15)
                        toRotate = pi
                    else:
                        print("Go")
                        self.driveMotors(0.15,0.15)
            else:
                if (round(posY))%2 == 0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:
                    print("Keep going forward")
                    self.driveMotors(0.15,0.15)
                else:
                    if turning != (-1,-1):
                        print("MAKING A TURN")
                        self.driveMotors(0.15,0.15)
                        if mapX-turning[0] != 0 or mapY-turning[1]!=0:
                            turning = (-1,-1)
                    elif self.measures.irSensor[center_id] < 5.0 \
                    and ((self.measures.compass > 60 and self.measures.compass < 120 and drawnMap[mapY-1][mapX] == 'X')
                    or (self.measures.compass > -60 and self.measures.compass < -120 and drawnMap[mapY+1][mapX] == 'X')):
                        print("GO FORWARD")
                        self.driveMotors(0.15,0.15)
                    elif self.measures.irSensor[right_id] < 1.0 \
                    and ((self.measures.compass > 0 and drawnMap[mapY][mapX+1] == 'X') \
                    or (self.measures.compass<0 and drawnMap[mapY][mapX-1] == 'X')):
                        print('Rotate riiiiiiiiiiight')
                        self.driveMotors(0.15,0.15)
                        toRotate = -pi/2
                        turning = (mapX,mapY)
                        positions_to_visit.append(turning)
                    elif self.measures.irSensor[left_id] <1.0\
                    and ((self.measures.compass > 0 and drawnMap[mapY][mapX-1] == 'X') \
                    or (self.measures.compass < 0 and drawnMap[mapY][mapX+1] == 'X')):
                        print('Rotate leeeeeft')
                        # print(drawnMap[26 -( int(posY+0.5)+2)][int(posX+0.5)]) if self.measures.compass >0 else print(drawnMap[26 -( int(posY+0.5)-2)][int(posX+0.5)])
                        self.driveMotors(0.15,0.15)
                        toRotate = pi/2
                        turning = (mapX,mapY)
                        positions_to_visit.append(turning)
                    elif (self.measures.compass>92 and self.measures.compass <118) or (self.measures.compass>-88 and self.measures.compass<-62):
                        print("Rotate slightly right")
                        if self.measures.compass > 0:
                            toRotate = (90-self.measures.compass)*pi/180
                        else:
                            toRotate = (-90-self.measures.compass)*pi/180
                    elif (self.measures.compass<88 and self.measures.compass >62) or (self.measures.compass>-118 and self.measures.compass<-92):
                        print("Rotate slightly left")
                        if self.measures.compass > 0:
                            toRotate = (90-self.measures.compass)*pi/180
                        else:
                            toRotate = (-90-self.measures.compass)*pi/180
                    elif self.measures.irSensor[center_id] > 1.7 and self.measures.irSensor[left_id] >1.7 and self.measures.irSensor[right_id]>1.7:
                        print('------> TURN AROUND <-------- ')
                        self.driveMotors(0.15,0.15)
                        toRotate = pi
                    else:
                        print("Go")
                        self.driveMotors(0.15,0.15)

            print("movingHorizontaly: ",movingHorizontaly)
            print("compass: ",self.measures.compass)
            print("toRotate: ",toRotate)


    def printDrawnMap(self):
        for i in drawnMap:
            print("".join(i[:55]).replace('0',' '))
    
    def writeDrawnMap(self):
        mapFile = open("outMap.map", "w")
        mapFile.write("")
        mapFile.close()
        mapOut = open("outMap.map", "a")
        for i in drawnMap:
            map.write("".join(i[:55]).replace('0',' '))
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
