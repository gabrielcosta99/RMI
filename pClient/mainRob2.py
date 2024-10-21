
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
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        Kp = 0.03
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


        if drawnMap[mapY][mapX] == '0' and (mapY % 2 == 1 or mapX % 2 == 1):
            drawnMap[mapY][mapX] = 'X'
            # print(drawnMap[mapY][mapX])
            
# Wall draw

        if abs(rotation) <=60:
            if posX % 2 == 1:
                if self.measures.irSensor[right_id] > 2.0:
                    drawnMap[mapY+1][mapX] = '-'
                else:
                    drawnMap[mapY+1][mapX] = 'X'
                    
                if self.measures.irSensor[center_id] > 2.0:
                    drawnMap[mapY][mapX+1] = '|'
                else:
                    drawnMap[mapY][mapX+1] = 'X'
                    
                if self.measures.irSensor[left_id] > 2.0:
                    drawnMap[mapY-1][mapX] = '-'
                else:
                    drawnMap[mapY-1][mapX] = 'X'
                    
        elif abs(rotation) >= 120:
            if posX % 2 == 1:
                if self.measures.irSensor[right_id] > 2.0:
                    drawnMap[mapY-1][mapX] = '-'
                else:
                    drawnMap[mapY-1][mapX] = 'X'
                    
                if self.measures.irSensor[center_id] > 2.0:
                    drawnMap[mapY][mapX-1] = '|'
                else:
                    drawnMap[mapY][mapX-1] = 'X'
                    
                if self.measures.irSensor[left_id] > 2.0:
                    drawnMap[mapY+1][mapX] = '-'
                else:
                    drawnMap[mapY+1][mapX] = 'X'
                    
        elif rotation > 0:
            if posY % 2 == 1:
                if self.measures.irSensor[right_id] > 2.0:
                    drawnMap[mapY][mapX+1] = '|'
                else:
                    drawnMap[mapY][mapX+1] = 'X'
                    
                if self.measures.irSensor[center_id] > 2.0:
                    drawnMap[mapY-1][mapX] = '-'
                else:
                    drawnMap[mapY-1][mapX] = 'X'
                    
                if self.measures.irSensor[left_id] > 2.0:
                    drawnMap[mapY][mapX-1] = '|'
                else:
                    drawnMap[mapY][mapX-1] = 'X'
                    
        elif rotation < 0:
            if posY % 2 == 1:
                if self.measures.irSensor[right_id] > 2.0:
                    drawnMap[mapY][mapX-1] = '|'
                else:
                    drawnMap[mapY][mapX-1] = 'X'
                    
                if self.measures.irSensor[center_id] > 2.0:
                    drawnMap[mapY+1][mapX] = '-'
                else:
                    drawnMap[mapY+1][mapX] = 'X'
                    
                if self.measures.irSensor[left_id] > 2.0:
                    drawnMap[mapY][mapX+1] = '|'
                else:
                    drawnMap[mapY][mapX+1] = 'X'
            
            
# Movement decision

    
        print()

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
