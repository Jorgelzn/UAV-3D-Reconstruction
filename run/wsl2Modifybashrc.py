############################
# 
# Modify IPs at .bashrc file in WSL2
# The IPs are provided by args
# 
# author: damigo@inf.uc3m.es
############################

import sys
from pathlib import Path
import subprocess

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
# Function that modifies the bashrc file with a new IP
############################
def modify_file(filePath, LocalHostIp, ControlIp):
    # Get file's content as a list
    my_file = open(filePath)
    string_list = my_file.readlines()
    my_file.close()

    # Find the line to modify and modify it
    display = "export PX4_SIM_HOST_ADDR="
    a, numTabs, numSpaces = idx_lines_that_start_with(display, string_list)
    if len(a) == 0: # Si no esta ninguna vez, lo pone por primera vez
        aux = "\n"+display+LocalHostIp+"\n"
        string_list = string_list + [aux]
        print("FIRST TIME ADDED: "+display)
    if len(a) == 1:
        string_list[a[0]] = display+LocalHostIp+"\n"
        print("MODIFIED: "+display)
    if len(a) > 1:
        print("ERROR. MULTIPLE INSTANCES OF: "+display)

    # Find the line to modify and modify it
    display = "export DISPLAY="
    a = idx_lines_that_start_with(display, string_list)
    if len(a) == 0: # Si no esta ninguna vez, lo pone por primera vez
        aux = "\n"+display+LocalHostIp+":0"+"\n"
        string_list = string_list + [aux]
        print("FIRST TIME ADDED: "+display)
    if len(a) == 1:
        string_list[a[0]] = display+LocalHostIp+":0"+"\n"
        print("MODIFIED: "+display)
    if len(a) > 1:
        print("ERROR. MULTIPLE INSTANCES OF: "+display)

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
    print("START wsl2Modifybashrc.py")
    # Receives the parameters by Windows
    LocalHostIp, ControlIp = sys.argv[1], sys.argv[2]
    print("Received: "+LocalHostIp+" "+ControlIp)

    # Get file path
    filePath = str(Path.home())+"/.bashrc"

    # Modifies the file
    modify_file(filePath, LocalHostIp, ControlIp)
    print("Saved file "+filePath)

    # Send the source bashrc (it seems that is not necessary)
    bashCommand=["bash", "-c", "source "+filePath]
    process = subprocess.Popen(bashCommand,stdin=subprocess.PIPE,stdout=subprocess.PIPE, shell=True)
    output, error = process.communicate()
    print("Sent source "+filePath)

    print("END wsl2Modifybashrc.py")
    print("--------------------------------------------")

############################
# Run main function
############################
main()