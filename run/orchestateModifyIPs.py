############################
# 
# Solve IPs problem to link WSL2 to Windows
# Designed to link AirSim from Windows to PX4 in WSL2
# 
# author: damigo@inf.uc3m.es
############################

import subprocess
import os
from glob import glob

print("************************************************************************")
print("START orchestateModifyIPs.py")

############################
# Get Windows and WSL2 usernames
############################
# Windows user
winUsername = os.path.expanduser('~').split("\\")[-1]
print("Get Windows user: ", winUsername)
# WSL2 Linux user
dir = r"\\wsl$\Ubuntu-20.04\home\\"
a = glob(dir+"*", recursive=False)
wslUsername = a[0].split("\\")[-1]
print("Get WSL2 linux user: ", wslUsername)

############################
# Generate scripts paths
############################
# base user path to locate the scripts
pathHomeWSL2           = "/home/"+wslUsername+"/"                 # Userbase at WSL2
pathHomeWindows        = "C:/Users/"+winUsername+"/"              # Userbase at Windows
# Python paths to run from scripts
pathPythonWSL2         = "python3"                                # Path to call python from Ubuntu in WSL2
pathPythonWindows      = "C:/Users/jorge/AppData/Local/Programs/Python/Python38/python.exe"   # Path to call python from Windows in cmd
# AirSim workspace from Windows and WSL2 (this latter uses a symbolic link to the Windows one)
pathAirSimWindows      = pathHomeWindows+"Desktop/Workspace/AirSim/run/"
pathAirSimWSL2         = pathHomeWSL2+"AirSim/run/"
# Script names
scriptNameBashrcWSL2     = "wsl2Modifybashrc.py"                  # Script name to modify bashrc IP and some others
scriptNameMavlinkBugWSL2 = "wsl2ModifyMavlinkBug.py"              # Script name to modify bashrc IP and some others
scriptNameSettingsJSON   = "modifyAllSettingsJSON.py"             # CHOOSE THIS ONE IF you want to modify IPs at all *.json and Documents/AirSim
#scriptNameSettingsJSON  = "modifySettingsJSON.py"                # CHOOSE THIS ONE IF you want to modify IPs at current settings.json at Documents/AirSim

############################
# Identify IPs to link Windows and WSL2
############################
# Get WSL2 IP. The same one as Windows terminal ipconfig
bashCommand=["bash", "-c", "awk '/nameserver / {print $2; exit}' /etc/resolv.conf 2>/dev/null"]
process = subprocess.Popen(bashCommand,stdin=subprocess.PIPE,stdout=subprocess.PIPE, shell=True)
output, error = process.communicate()
ControlIp = output.decode().replace("nameserver ", "")              # POSIBLE ERROR IPs
print("WSL2 IP (ControlIp): "+ControlIp)                            # POSIBLE ERROR IPs
#LocalHostIp = output.decode().replace("nameserver ", "")           # SOLUCION A POSIBLE ERROR IPs
#print("WSL2 IP (LocalHostIp): "+LocalHostIp)                       # SOLUCION A POSIBLE ERROR IPs
# Get Linux's WSL2 IP
bashCommand=["bash", "-c", "hostname -I"] # bashCommand=["bash", "-c", "ip -4 addr show eth0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}'"]
process = subprocess.Popen(bashCommand,stdin=subprocess.PIPE,stdout=subprocess.PIPE, shell=True)
output, error = process.communicate()
LocalHostIp = output.decode()                                       # POSIBLE ERROR IPs
print("Linux's WSL2 IP (LocalHostIp): "+LocalHostIp)                # POSIBLE ERROR IPs
#ControlIp = output.decode()                                        # SOLUCION A POSIBLE ERROR IPs
#print("Linux's WSL2 IP (ControlIp): "+ControlIp)                   # SOLUCION A POSIBLE ERROR IPs

############################
# Modify all *.json in Documents/AirSim using this IPs
# Run at Windows side
############################
command = pathPythonWindows+" "+pathAirSimWindows+scriptNameSettingsJSON+" "+ControlIp+" "+LocalHostIp
command = command.replace("\n", "")
bashCommand=command
process = subprocess.Popen(bashCommand,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
output, error = process.communicate()       # Execution
lines = output.decode().split("\n")         # Printing log
for line in lines:
    print(line)

############################
# Modifies several WSL2 files using the IPs. Calls another Python script in WSL2 home path
# Run at WSL2 side
############################
command = pathPythonWSL2+" "+pathAirSimWSL2+scriptNameBashrcWSL2+" "+ControlIp+" "+LocalHostIp
command = command.replace("\n", "")
bashCommand=["bash", "-c", command]
process = subprocess.Popen(bashCommand,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
output, error = process.communicate()       # Execution
lines = output.decode().split("\n")         # Printing log
for line in lines:
    print(line)

############################
# Modifies several WSL2 files using the IPs. Calls another Python script in WSL2 home path
# Run at WSL2 side
############################
command = pathPythonWSL2+" "+pathAirSimWSL2+scriptNameMavlinkBugWSL2+" "+ControlIp+" "+LocalHostIp
command = command.replace("\n", "")
bashCommand=["bash", "-c", command]
process = subprocess.Popen(bashCommand,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
output, error = process.communicate()       # Execution
lines = output.decode().split("\n")         # Printing log
for line in lines:
    print(line)

print("END orchestateModifyIPs")
print("************************************************************************")
print()
input("Press anything to close.")