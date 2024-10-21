
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

toRotate = 0
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
        global toRotate
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        Kp = 0.1
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
        print(mapX)
        print(mapY)
        print(rotation)
        # self.printDrawnMap()
        print("center: ",self.measures.irSensor[center_id])
        print("left: ",self.measures.irSensor[left_id])
        print("right: ",self.measures.irSensor[right_id])
        print("back: ",self.measures.irSensor[back_id])
        print("posX: ",posX)
        print("posY: ",posY)
        print("compass: ",self.measures.compass)
        

        if drawnMap[mapY][mapX] == '0' and (mapY % 2 == 1 or mapX % 2 == 1):
            drawnMap[mapY][mapX] = 'X'
            # print(drawnMap[mapY][mapX])
            
# Wall draw

        if abs(rotation) <=60:
            if posX % 2 == 1:
                if self.measures.irSensor[right_id] < 2.0:
                    drawnMap[mapY+1][mapX] = '-'
                else:
                    drawnMap[mapY+1][mapX] = 'X'
                    
                if self.measures.irSensor[center_id] < 2.0:
                    drawnMap[mapY][mapX+1] = '|'
                else:
                    drawnMap[mapY][mapX+1] = 'X'
                    
                if self.measures.irSensor[left_id] < 2.0:
                    drawnMap[mapY-1][mapX] = '-'
                else:
                    drawnMap[mapY-1][mapX] = 'X'
                    
        elif abs(rotation) >= 120:
            if posX % 2 == 1:
                if self.measures.irSensor[right_id] < 2.0:
                    drawnMap[mapY-1][mapX] = '-'
                else:
                    drawnMap[mapY-1][mapX] = 'X'
                    
                if self.measures.irSensor[center_id] < 2.0:
                    drawnMap[mapY][mapX-1] = '|'
                else:
                    drawnMap[mapY][mapX-1] = 'X'
                    
                if self.measures.irSensor[left_id] < 2.0:
                    drawnMap[mapY+1][mapX] = '-'
                else:
                    drawnMap[mapY+1][mapX] = 'X'
                    
        elif rotation > 0:
            if posY % 2 == 1:
                if self.measures.irSensor[right_id] < 2.0:
                    drawnMap[mapY][mapX+1] = '|'
                else:
                    drawnMap[mapY][mapX+1] = 'X'
                    
                if self.measures.irSensor[center_id] < 2.0:
                    drawnMap[mapY-1][mapX] = '-'
                else:
                    drawnMap[mapY-1][mapX] = 'X'
                    
                if self.measures.irSensor[left_id] < 2.0:
                    drawnMap[mapY][mapX-1] = '|'
                else:
                    drawnMap[mapY][mapX-1] = 'X'
                    
        elif rotation < 0:
            if posY % 2 == 1:
                if self.measures.irSensor[right_id] < 2.0:
                    drawnMap[mapY][mapX-1] = '|'
                else:
                    drawnMap[mapY][mapX-1] = 'X'
                    
                if self.measures.irSensor[center_id] < 2.0:
                    drawnMap[mapY+1][mapX] = '-'
                else:
                    drawnMap[mapY+1][mapX] = 'X'
                    
                if self.measures.irSensor[left_id] < 2.0:
                    drawnMap[mapY][mapX+1] = '|'
                else:
                    drawnMap[mapY][mapX+1] = 'X'
            
            
# Movement decision
        #check rotation first
       
        movingHorizontaly = False
        compass = self.measures.compass
        if toRotate > 0:
            if toRotate<0.3:
                u = toRotate/2
                self.driveMotors(-u,u)
                toRotate=0
            else:
                toRotate -=0.3
                self.driveMotors(-0.15,0.15)
        elif toRotate < 0:
            if toRotate >-0.3:
                u = toRotate/2
                self.driveMotors(-u,u)
                toRotate=0
            else:
                toRotate +=0.3
                self.driveMotors(0.15,-0.15)
        else:
            if abs(compass) < 60.0 or abs(compass) > 120.0:
                movingHorizontaly = True
            if movingHorizontaly:
                if (round(posX))%2==0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:   # We are not at the center of the cell, keep moving forward
                    print("Keep going forward")
                    self.driveMotors(0.15,0.15)
                else:   # We are at the center of the cell, we may go to the right
                    if self.measures.irSensor[right_id] < 1.0\
                        and ((abs(compass) < 60 and drawnMap[26 - (int(posY+0.5-1))][int(posX+0.5)] == '0') \
                        or (abs(compass)>120 and drawnMap[26 - (int(posY+0.5+1))][int(posX+0.5)] == '0')):
                        print('Rotate riiiiiiiiiiight')
                        self.driveMotors(0.15,0.15)
                        toRotate = -pi/2
                    # if there is an open space to the left that we havent visited and we have already visited the front cell, turn left
                    elif self.measures.irSensor[left_id] <1.0 \
                        and (((abs(compass) < 60 and drawnMap[26 - (int(posY+0.5))][int(posX+0.5)+1] != '0') \
                        and (abs(compass) < 60 and drawnMap[26 - (int(posY+0.5-1))][int(posX+0.5)] == '0')) \
                        or ((abs(compass) > 120 and drawnMap[26 - (int(posY+0.5))][int(posX+0.5)-1] != '0' ) \
                        and (abs(compass)>120 and drawnMap[26 - (int(posY+0.5+1))][int(posX+0.5)] == '0'))):
                        print('Rotate leeeeeft')
                        print(drawnMap[26 - (int(posY+0.5))][int(posX+0.5)+1]) if abs(compass) < 60 else print(drawnMap[26 - (int(posY+0.5))][int(posX+0.5)-1])
                        self.driveMotors(0.15,0.15)
                        toRotate = pi/2
                    elif self.measures.irSensor[center_id] > 1.7 \
                        and ((self.measures.irSensor[right_id]<2.0 and self.measures.irSensor[left_id]>1.5)\
                        or self.measures.irSensor[right_id]<self.measures.irSensor[left_id]):
                        print('Dodge riiiiiiiiiiight')
                        self.driveMotors(0.15,0.15)
                        toRotate = -pi/2
                    elif self.measures.irSensor[center_id] > 1.7 \
                        and ((self.measures.irSensor[left_id]<2.0 and self.measures.irSensor[right_id]>1.5)\
                        or self.measures.irSensor[left_id]<self.measures.irSensor[right_id]):
                        print('Dodge leeeeft')
                        self.driveMotors(0.15,0.15)
                        toRotate = pi/2
                    elif (compass>2 and compass <58) or (compass>-180 and compass<-122):
                        print("Rotate slightly right")
                        u = (abs(compass)%60/60)*Kp
                        print("u: ",u)
                        self.driveMotors(0.1+u,0.1-u)
                    elif (compass<178 and compass >122) or (compass>-58 and compass<-2):
                        print("Rotate slightly left")
                        u = (abs(compass)%60/60)*Kp
                        print("u: ",u)
                        self.driveMotors(0.1-u,0.1+u)
                    elif self.measures.irSensor[right_id]>5.0 or self.measures.irSensor[left_id]>5.0:
                        print("Rotate slightly: ",e)
                        self.driveMotors(0.1+e,0.1-e)
                    else:
                        print("Go")
                        self.driveMotors(0.15,0.15)
            else:
                if (round(posY))%2 == 0 and self.measures.irSensor[left_id]<4.0 and self.measures.irSensor[right_id]<4.0 and self.measures.irSensor[center_id]<4.0:
                    print("Keep going forward")
                    self.driveMotors(0.15,0.15)
                else:
                    if self.measures.irSensor[right_id] < 1.0 \
                        and ((compass > 0 and drawnMap[26 -( int(posY+0.5))][int(posX+0.5)+1] == '0') \
                        or (compass<0 and drawnMap[26 - (int(posY+0.5))][int(posX+0.5)-1] == '0')):
                        print('Rotate riiiiiiiiiiight')
                        self.driveMotors(0.15,0.15)
                        toRotate = -pi/2
                    elif self.measures.irSensor[left_id] <1.0\
                        and (((compass > 0 and drawnMap[26 -( int(posY+0.5)+1)][int(posX+0.5)] != '0') \
                        and (compass > 0 and drawnMap[26 -( int(posY+0.5))][int(posX+0.5)-1] == '0')) \
                        or ((compass < 0 and drawnMap[26 -( int(posY+0.5)-1)][int(posX+0.5)] != '0') \
                        and (compass < 0 and drawnMap[26 - (int(posY+0.5))][int(posX+0.5)+1] == '0'))):
                        print('Rotate leeeeeft')
                        print(drawnMap[26 -( int(posY+0.5)+1)][int(posX+0.5)]) if compass >0 else print(drawnMap[26 -( int(posY+0.5)-1)][int(posX+0.5)])
                        self.driveMotors(0.15,0.15)
                        toRotate = pi/2
                    elif self.measures.irSensor[center_id] > 1.7 \
                        and ((self.measures.irSensor[right_id]<2.0 and self.measures.irSensor[left_id]>1.5)\
                        or self.measures.irSensor[right_id]<self.measures.irSensor[left_id]):
                        print('Dodge riiiiiiiiiiight')
                        self.driveMotors(0.15,0.15)
                        toRotate = -pi/2
                    elif self.measures.irSensor[center_id] > 1.7 \
                        and ((self.measures.irSensor[left_id]<2.0 and self.measures.irSensor[right_id]>1.5)\
                        or self.measures.irSensor[left_id]<self.measures.irSensor[right_id]):
                        print('Dodge leeeeeft')
                        self.driveMotors(0.15,0.15)
                        toRotate = pi/2
                    elif (compass>92 and compass <118) or (compass>-88 and compass<-62):
                        print("Rotate slightly right")
                        u = (abs(compass)%90/90)*Kp
                        print("u: ",u)
                        self.driveMotors(0.1+u,0.1-u)
                    elif (compass<88 and compass >62) or (compass>-118 and compass<-92):
                        print("Rotate slightly left")
                        u = (abs(compass)%90/90)*Kp
                        print("u: ",u)
                        self.driveMotors(0.1-u,0.1+u)
                    elif self.measures.irSensor[right_id]>5.0 or self.measures.irSensor[left_id]>5.0:
                        print("Rotate slightly: ",e)
                        self.driveMotors(0.1+e,0.1-e)
                    else:
                        print("Go")
                        self.driveMotors(0.15,0.15)
        print("movingHorizontaly: ",movingHorizontaly)
        print("toRotate: ",toRotate)
                

        
        # if self.measures.irSensor[right_id] < 1.8:
        #     print('Rotate riiiiiiiiiiight')
        #     self.driveMotors(0.15,-0.10)
        # elif   self.measures.irSensor[center_id] > 1.7 \
        #     and ((self.measures.irSensor[right_id]<2.7 and self.measures.irSensor[left_id]>2.0)\
        #     or self.measures.irSensor[right_id]<self.measures.irSensor[left_id]):
        #     print('Rotate riiiiiiiiiight')
        #     self.driveMotors(-0.15,+0.15)
        # elif   self.measures.irSensor[center_id] > 1.7 \
        #     and ((self.measures.irSensor[left_id]<2.7 and self.measures.irSensor[right_id]>2.0)\
        #     or self.measures.irSensor[left_id]<self.measures.irSensor[right_id]):
        #     print('Rotate leeeeeeeeeeft')
        #     self.driveMotors(-0.15,+0.15)
        # elif self.measures.irSensor[left_id]> 2.5 :
        #     print('Rotate slowly right')
        #     deltav = e*Kp/2
        #     print("deltav: ",deltav)
        #     self.driveMotors(0.12+deltav,0.12-deltav)
        # elif (self.measures.irSensor[right_id]> 2.5 ) :
        #     print('Rotate slowly left')
        #     deltav = e*Kp
        #     print("deltav: ",deltav)
        #     self.driveMotors(0.12+deltav,0.12-deltav)
        # else:
        #     print('Go')
        #     self.driveMotors(0.15,0.15)
        print()

    def controller(self):
        global e_m1, e_m2, u_m1
        
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        Kp = 0.015    
        h = 1.5
        u=0
        max_u = 0.05

        Td = 0*h     # Td - differential time
        K0 = Kp*(1+Td/h)
        K1 = -Kp*(1+2*Td/h)
        K2 = Kp*Td/h
        e = self.measures.irSensor[left_id]-self.measures.irSensor[right_id]
        u = u_m1 + K0*e + K1*e_m1 + K2*e_m2
        print(self.measures.beacon)

        print("center: ",self.measures.irSensor[center_id])
        print("left: ",self.measures.irSensor[left_id])
        print("right: ",self.measures.irSensor[right_id])
        print("back: ",self.measures.irSensor[back_id])
        if self.measures.irSensor[center_id] > 1.0 \
            and ((self.measures.irSensor[right_id]<2.0 and self.measures.irSensor[left_id]>1.5)\
            or self.measures.irSensor[right_id]<self.measures.irSensor[left_id]):
            print('Rotate riiiiiiiiiiight')
            u=0.25
        elif   self.measures.irSensor[center_id] > 1.0 \
            and ((self.measures.irSensor[left_id]<2.0 and self.measures.irSensor[right_id]>1.5)\
            or self.measures.irSensor[left_id]<self.measures.irSensor[right_id]):
            print('Rotate leeeeeeeeeeft')
            u=-0.25
        elif e > 1:#self.measures.irSensor[left_id]> 2.5 :
            print('Rotate slowly right')
            e_m2 = e_m1
            e_m1 = e
            u_m1 = u

            #Clip the control signal to avoid saturation
            if(u > max_u):
                u = max_u
            
            if (u < -max_u):
                u = -max_u
            print("u before: ",u)
        elif e<-1: #(self.measures.irSensor[right_id]> 2.5 ) :
            print('Rotate slowly left')
            e_m2 = e_m1
            e_m1 = e
            u_m1 = u

            #Clip the control signal to avoid saturation
            if(u > max_u):
                u = max_u
            
            if (u < -max_u):
                u = -max_u
            print("u before: ",u)

           
        else:
            u=None

        return u

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
