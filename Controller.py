#   Created by Łukasz Granat at 2019
#   This project shows comparison of classic PID controller and Fuzzed PI controller.
#   Both controllers are designed to controll height of the liquid in tank where there is contolable inflow and constant outflow
#   Both controllers are designed to reach and hold height at 10 m.
#   Fuzzed regulator is based in scikit-fuzzy library
#   All license statements of library work also in this project
#   https://github.com/scikit-fuzzy/scikit-fuzzy/blob/master/LICENSE.txt

import math
import matplotlib.pyplot as diagram
import Fuzzy as fz
import csv

#   Definition of class which allows to make a tank with every parameter the user wants
class TankFilling:
    def __init__(self, tankBaseField, samplingTime, beta, symulationDuration, startHeight, aimHeight, pidTd, pidTi, pidTp, pidKp):
                #(-, m^2, s, m^2, m^3/s, s, m, m, -, -, -, -) - units of input variables
        self.tankBaseField = float(tankBaseField)
        self.samplingTime = float(samplingTime)
        self.beta = float(beta)
        self.flowIn = 0
        self.samplesCount = float(symulationDuration) / float(samplingTime)
        self.startHeight = float(startHeight)
        self.aimHeight = float(aimHeight)
        self.pidTd = float(pidTd)
        self.pidTi = float(pidTi)
        self.pidTp = float(pidTp)
        self.pidKp = float(pidKp)        
        self.heightSamplesList = []
        self.heightErrorList = []
        self.uRegList = []   
        self.extortion = []
        self.uFuzzRegList = []
        self.heightFuzzSamplesList = []
        self.heightFuzzErrorList = []
        self.heightFuzzErrorChangeList = []
        self.extortionFuzz = []

    #This function represents classic PID controller and steering of solenoid valve
    #All history of signals is written to collections on output
    #In this function integrating is realised by Euler's method
    #I take that the control signal is proportional to the flow rate of the solenoid valve in 1:1 proportion and is in the range (0;10)       
    def HeightDuringTheFillingProcessUsualPID(self):        
        eOld = 0
        tempHeightNextResult = self.startHeight
        self.heightErrorList.append(eOld)
        self.extortion.append(0)
        for i in range(int(self.samplesCount)):            
            tempHeightNextResult = self.samplingTime * ((1 / self.tankBaseField) * self.flowIn - (self.beta / self.tankBaseField) * math.sqrt(tempHeightNextResult)) + tempHeightNextResult
            self.heightSamplesList.append(tempHeightNextResult)
            eNow = self.aimHeight - tempHeightNextResult
            self.heightErrorList.append(eNow)
            uReg = self.pidKp * (eNow + (self.pidTp / self.pidTi) * sum(self.heightErrorList) + (self.pidTd / self.pidTp) * (eNow - eOld))           
            self.uRegList.append(uReg)                 
            eOld = eNow                                                                             
            self.flowIn = uReg 
            if(self.flowIn > 10): self.flowIn = 10
            if(self.flowIn < 0): self.flowIn = 0
            self.extortion.append(1)
        return self.heightSamplesList, self.heightErrorList, self.uRegList, self.extortion

    #This function represents fuzzed PI controller and steering of solenoid valve
    #All history of signals is written to collections on output
    #I take that the control signal is proportional to the flow rate of the solenoid valve in 1:1 proportion                  
    def HeightDuringTheFillingProcessFuzzedPI(self):
       eOld = 0
       uReg = 0
       tempFuzzHeightNextResult = self.startHeight
       self.heightFuzzErrorList.append(eOld)
       self.extortionFuzz.append(0)
       for i in range(int(self.samplesCount)):            
           tempFuzzHeightNextResult = self.samplingTime * ((1 / self.tankBaseField) * self.flowIn - (self.beta / self.tankBaseField) * math.sqrt(tempFuzzHeightNextResult)) + tempFuzzHeightNextResult
           self.heightFuzzSamplesList.append(tempFuzzHeightNextResult)
           eNow = self.aimHeight - tempFuzzHeightNextResult
           self.heightFuzzErrorList.append(eNow)
           errorChange = eNow - eOld
           self.heightFuzzErrorChangeList.append(errorChange)
           uRegOld = uReg
           uReg = fz.FuzzControlSignal(eNow, errorChange, 10, 1, 10)     
           self.uFuzzRegList.append(uReg)           
           eOld = eNow                                                        
           self.flowIn = uReg + uRegOld
           if(self.flowIn > 10): self.flowIn = 10
           if(self.flowIn < 0): self.flowIn = 0
           self.extortionFuzz.append(1)           
       return self.heightFuzzSamplesList, self.heightFuzzErrorList, self.uFuzzRegList, self.extortionFuzz, self.heightFuzzErrorChangeList
        
#   Settings of a tank
#   I recommend to ser symulationDuration and aimHeight on 10
print("Podaj czas symulacji [s]")
symulationDuration = input()
print("Podaj docelową wysokość cieczy")
aimHeight = input()
#print("Podaj pole powierzchni zbiornika [m^2]")
tankBaseField = 4
#print("Podaj czas probkowania [s]")
samplingTime = 0.1
#print("Podaj parametr Beta [m^2]")
beta = 0.25
#print("Podaj początkowy poziom cieczy w zbiorniku [m]")
startHeight = 0
#print("Podaj wartość Td regulatora")
pidTd = 0.1
#print("Podaj wartość Ti regulatora")
pidTi = 50
#print("Podaj wartość Tp regulatora")
pidTp = 0.01
#print("Podaj wzmocnienie Kp regulatra")
pidKp = 3

heightLevelControll = TankFilling(tankBaseField, samplingTime, beta, symulationDuration, startHeight, aimHeight, pidTd, pidTi, pidTp, pidKp)

actualHeight, actualError, actualControlParam, actualExtortion= heightLevelControll.HeightDuringTheFillingProcessUsualPID()
actualFuzzHeight, actualFuzzError, actualFuzzControlParam, actualFuzzExtortion, actualFuzzErrorChange= heightLevelControll.HeightDuringTheFillingProcessFuzzedPI()

#   Here useful signals from both controllers are written to csv files
with open('NormalPIDMeasurements.csv','w', newline = '') as out:
        csv.writer(out, delimiter='\t', quoting=csv.QUOTE_MINIMAL).writerows(zip(actualHeight, actualError, actualControlParam))

with open('FuzzyMeasurements.csv','w', newline = '') as out:
        csv.writer(out, delimiter='\t', quoting=csv.QUOTE_MINIMAL).writerows(zip(actualFuzzHeight, actualFuzzError, actualFuzzControlParam, actualFuzzErrorChange))

#   Creating chatrs with signals
diagram.figure(1)
diagram.suptitle("Zwykły PID", fontsize = 16)
diagram.subplots_adjust(hspace=0)
diagram.subplot(4,1,1)   
diagram.plot(actualHeight)   
diagram.ylabel("Poziom cieczy [m]")
diagram.grid(True)
diagram.subplot(4,1,2) 
diagram.plot(actualError)  
diagram.ylabel("Uchyb")
diagram.grid(True)
diagram.subplot(4,1,3)  
diagram.plot(actualControlParam)  
diagram.ylabel("Sygnał sterujący u")
diagram.grid(True)
diagram.subplot(4,1,4) 
diagram.plot(actualExtortion)
diagram.ylabel("Wymuszenie")
diagram.xlabel("Liczba próbek")
diagram.grid(True)

diagram.figure(2)
diagram.suptitle("Rozmyty PID", fontsize = 16)
diagram.subplots_adjust(hspace=0)
diagram.subplot(5,1,1)   
diagram.plot(actualFuzzHeight)   
diagram.ylabel("Poziom cieczy [m]")
diagram.grid(True)
diagram.subplot(5,1,2) 
diagram.plot(actualFuzzError)  
diagram.ylabel("Uchyb")
diagram.grid(True)
diagram.subplot(5,1,3)  
diagram.plot(actualFuzzControlParam)  
diagram.ylabel("Sygnał sterujący u")
diagram.grid(True)
diagram.subplot(5,1,4) 
diagram.plot(actualFuzzExtortion)
diagram.ylabel("Wymuszenie")
diagram.grid(True)
diagram.subplot(5,1,5) 
diagram.plot(actualFuzzErrorChange)
diagram.ylabel("Zmiana uchybu")
diagram.xlabel("Liczba próbek")
diagram.grid(True)
diagram.show()

#   Creating chatrs with triangles which tak part in fuzzy steering
fz.error.view()
fz.errorChange.view()
fz.u.view()
diagram.show()

stop = input()