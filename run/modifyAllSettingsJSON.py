############################
# 
# Modify IPs ONLY at current settings.json at Documents/AirSim
# The IPs are provided by args
# There is an alternative script that modifies the IPs at all *.json at Documents/AirSim
# 
# author: damigo@inf.uc3m.es
############################

import os
import fnmatch
import sys
from pathlib import Path
import subprocess
from glob import glob

############################
# Function that return the line(s) number that start with the passed line
#  along its number of tabulations and spaces to replicate it at a modified version
############################
def idx_lines_that_start_with(string, fp):
    a = []
    numTabs = []
    numSpaces = []
    for idx, line in enumerate(fp):
        if string in line:
            a.append(idx)
            numTabs.append(line.count('\\t'))
            numSpaces.append(line.count(' '))
    return a, numTabs, numSpaces

############################
# Function that modifies the provided file with a new IP
############################
def modify_file(filePath, LocalHostIp, ControlIp):
    # Get file's content as a list
    my_file = open(filePath)
    string_list = my_file.readlines()
    my_file.close()

    # Find the line to modify and modify it
    display = 'LocalHostIp'
    a, numTabs, numSpaces = idx_lines_that_start_with(display, string_list)
    if len(a) == 0:
        print("ERROR, NOT DETECTED: "+display)
    if len(a) > 0:
        print("DETECTED: "+display)
        for idxArr, idxLine in enumerate(a):
            strTabs   = '\n' * numTabs[idxArr]
            strSpaces = ' '  * numSpaces[idxArr]
            string_list[a[idxArr]] = strTabs+strSpaces+'"'+display+'"'+': '+'"'+LocalHostIp+'"'+','+'\n'

    # Write the modified file
    my_file = open(filePath, "w")
    new_file_contents = "".join(string_list)
    my_file.write(new_file_contents)
    my_file.close()

############################
# Orchestate all procedure
############################
def main():
    print("--------------------------------------------")
    print("START modifyAllSettingsJSON.py")

    # Receives the parameters by Windows
    LocalHostIp, ControlIp = sys.argv[1], sys.argv[2]
    print("Received: "+LocalHostIp+" "+ControlIp)

    # Get all *.json at Documents/AirSim path
    username = os.path.expanduser('~').split("\\")[-1]
    folderPath = "C:/Users/"+username+"/Documents/AirSim/"

    matches = []
    for root, dirnames, filenames in os.walk(folderPath):
        for filename in fnmatch.filter(filenames, '*.json'):
            matches.append(os.path.join(root, filename))
    
    for filePath in matches:
        # Modifies each file
        modify_file(filePath, LocalHostIp, ControlIp)
        print("Modified file "+filePath)
    
    print("END modifyAllSettingsJSON.py")
    print("--------------------------------------------")

############################
# Run main function
############################
main()