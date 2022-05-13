# -*- coding: utf-8 -*-
"""
Created on Thu May 12 10:06:41 2022

@author: dowdt
"""

import os
import math
import re
import math

### file operations
# File Names
fileLocal = "G:\My Drive\Documents\Purdue\GraduateSchool\Robotic Extrusion Overprinting (REO)\RoboSimulation"
fileName = "testProgram.LS"
openName = os.path.join(fileLocal, fileName)

# Read file
with open(openName) as file:
    rawFile = file.read().splitlines()

### Extract and organize information
# Extract main program between where it starts and position locations start
for i, line in enumerate(rawFile):
    if "  1:" in line:
        startIdx = i
    if "/POS" in line:
        endIdx = i

# Extract sections of program
startBits = rawFile[0:startIdx-1]
commands = rawFile[startIdx-1:endIdx] # Index lines up with line number
positions = rawFile[endIdx:len(rawFile)]

# Extract position points, where indices line up with position numbers
x = [0]; y = [0]; z = [0]; i = 0
while i < len(positions):
    
    line = positions[i]
    
    if "GP1" in line:
        i += 2
        xyz = positions[i].split()
        if "X" in xyz[0]:
            x.append(float(xyz[2]))
            y.append(float(xyz[6]))
            z.append(float(xyz[10]))    
        else:
            x.append(0)
            y.append(0)
            z.append(0)    
            
    i+= 1
    
### Useful functions
# Find total move distance between x,y,z points i and j
def calcDist(i, j):
    global x; global y; global z
    
    if i >= len(x) or j >= len(x):
        dist = 0
        return dist
        
    delX = x[i] - x[j]
    delY = y[i] - y[j]
    delZ = z[i] - z[j]
    dist = round(math.sqrt(delX**2 + delY**2 + delZ**2),3)
    return dist

# Find total extruded material, since sometimes 2+ calls
def extrudePos(i, newProg):
    reset = False
    length = 0
    
    # Move backware until first "Extrud" called
    while "Extrud" not in newProg[i]:
        i -= 1
    
    # Iterates backward, until the next command is not an extrude command
    iterations = 0
    while "Extrud" in newProg[i]:
        l = float(newProg[i].split()[2].split("(")[1].split(")")[0])

        # Checks if extrusion length was reset, if so, don't set extrusion length to zero, but change flag
        if l != 0:
            length = l
        else:
            reset = True
        i-=1
        iterations += 1
        
        # Infinite loop safety
        if iterations > 10:
            raise Exception("totalExtrusionCalcFailed")
            return
            
    return length, reset

# Maps the extrusion rate to the output bits
def erateMap(eRate, lineNum):
      
    totRangeSteps = 2**6 - 1
    # Accurate zone, accounting for ~80% of values
    #maxMin = np.percentile(eRates, [80, 1]).tolist()
    maxMin = [10, 1]
    mRateMax = maxMin[0]
    mRateMin = maxMin[1]
    mainRangeSteps = int(math.floor(0.8*totRangeSteps))
    lowRangeSteps = int((totRangeSteps - mainRangeSteps)/3)
    upperRangeSteps = int(totRangeSteps - mainRangeSteps - lowRangeSteps)
    
    # Lower Range Mapping:
    lRateMax = mRateMin
    lRateMin = -15
    
    # Upper Range Mapping
    hRateMin = mRateMax
    hRateMax = 30
    
    # Ranges: -20mm/s -10mm/s -5mm/s
    
    # Main Range
    if eRate >= mRateMin and eRate <= mRateMax:
        eRateMapped = lowRangeSteps + int((eRate - mRateMin)/(mRateMax - mRateMin)*mainRangeSteps)
        
    # Low Range
    elif eRate < mRateMin:
        if eRate < lRateMin:
            eRateMapped = 1
        else:
            eRateMapped = 1 + int(round((eRate - lRateMin)/(lRateMax - lRateMin)*lowRangeSteps))
        
    # High Range
     # Low Range
    elif eRate > mRateMax:
        if eRate > hRateMax: 
            eRateMapped = totRangeSteps
        else:
            eRateMapped = lowRangeSteps +  mainRangeSteps + int((eRate - hRateMin)/(hRateMax - hRateMin)*upperRangeSteps)
        
    groupIO = f"{lineNum}:G[3] = {eRateMapped};"
    # print(groupIO)
    
    return groupIO
        
### Determine extrusion rates by iterating through program
eRates = []; ePoses = [0]; eDists = []; moveDists = []; moveRates = []
moveRate = 0
newProg = commands.copy()
prevEPos = 0
reset = False; lastReset = False

i = 1
iMax = float("inf") # i + 400
while i < len(newProg):
    
    # Find instance of movement with an extrusion
    if "P[" in newProg[i] and "Extrud" in newProg[i+1]:
        
        # Position number of current line
        posNum = int(newProg[i].split()[1][2:][:-1])
        
        # Distance between this position and the previous position
        moveDist = calcDist(posNum, posNum-1); moveDists.append(moveDist)
        
        # Movement rate of current line
        moveRate = float(newProg[i].split()[2].split("mm")[0]); moveRates.append(moveRate)
        
        # Position of extruder after running this set of movements (all extrudes after this position until next position called)
        ePos, reset = extrudePos(i-1, newProg)
        ePoses.append(ePos)
        
        # If extruder position was reset last time, change how eDist is calculated
        if lastReset:
            eDist = round(ePos, 3); eDists.append(eDist)
        else:
            eDist = round((ePoses[-1] - ePoses[-2]), 3); eDists.append(eDist)
        lastReset = reset
        
        moveTime = moveDist/moveRate
        
        eRate = round(eDist/moveTime,4); eRates.append(eRate)
        
        eRateCommand = erateMap(eRate, i)
        newProg.insert(i+1, eRateCommand)
        
        # print(f"Line {i}, Pos {posNum}, move dist {moveDist}, move rate {moveRate}, eDist {eDist}, eRate {eRate}, eRateCommand {eRateCommand}")
        
    
    # Exit loop early cause testing
    if i > iMax: 
        break
    i+= 1
    
    
### Cleanup a few things
# Iterate throuogh new program and delete useless commands
i=0
while i < len(newProg):
    
    if "Extrud" in newProg[i]: 
        newProg.pop(i)
    elif "M_RunC" in newProg[i]:
        newProg.pop(i)
    else:
        i+=1

# Iterate throuogh new program and re-number
i=0
for i, line in enumerate(newProg):
    
    if ":" in line:
        commands = line.split(":")[1]
        
        newLine = f"{i}: {commands}"
        
        newProg[i] = newLine
    
### Combine with rest of code and save to file
modFileName = f"""{fileName.split(".")[0]} _mod.LS"""
modCodeName = f"""{fileName.split(".")[0]} _mod"""
saveName = os.path.join(fileLocal, modFileName)

# Rename program in code
for index, line in enumerate(startBits):
    
    if "/PROG" in line:
        startBits[index] = f"{line.split()[0]} {modCodeName}"
    
    if "FILE_NAME" in line:
        startBits[index] = f"""{line.split(" ")[0]} {modCodeName};"""
             
# Combine program together again
fullProgram = startBits + newProg + positions


# Write to file
with open(saveName, "w") as file:
    for line in fullProgram:
        file.write(f"{line}\n")