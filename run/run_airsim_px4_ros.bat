ECHO %USERNAME%

REM --------------------------------------------------
REM MAIN
REM --------------------------------------------------
ECHO OFF
SET /p choose="Lanzar Unreal Engine 4? Y o N - "
IF /i "%choose%" == "Y" GOTO UE4
IF /i "%choose%" == "y" GOTO UE4
GOTO first

REM --------------------------------------------------
REM UE4
REM --------------------------------------------------
:UE4
start C:\"Program Files"\"Epic Games"\UE_4.26\Engine\Binaries\Win64\UE4Editor.exe
ECHO --- Abriendo UE4, continua mientras se abre
goto first

REM --------------------------------------------------
REM FIRST
REM --------------------------------------------------
:first
SET /p choose="Actualizar direcciones IP? Y o N - "
IF /i "%choose%" == "Y" GOTO updateIP
IF /i "%choose%" == "y" GOTO updateIP
GOTO second

REM --------------------------------------------------
REM updateIP
REM --------------------------------------------------
:updateIP
start C:\Users\%USERNAME%\anaconda3\python.exe C:\Users\%USERNAME%\Desktop\Workspace\AirSim\run\orchestateModifyIPs.py
ECHO --- Actualizadas direcciones IP
goto second

REM --------------------------------------------------
REM second
REM --------------------------------------------------
:second
SET /p choose="Lanzar VScode en la carpeta de AirSim? Y o N - "
IF /i "%choose%" == "Y" GOTO vscode
IF /i "%choose%" == "y" GOTO vscode
GOTO fourth

REM --------------------------------------------------
REM VScode
REM --------------------------------------------------
:vscode
start code C:\Users\%USERNAME%\Desktop\Workspace\AirSim
REM start C:\Users\%USERNAME%\AppData\Local\Programs\"Microsoft VS Code"\Code.exe C:\Users\%USERNAME%\Desktop\Workspace\AirSim
ECHO Abierto VSCode en C:\Users\%USERNAME%\Desktop\Workspace\AirSim
ECHO --- Recuerda abrir el WSL2 con el boton verde (abajo izquierda)
ECHO --- Despues abre la carpeta de tu usuario de WSL2
goto fourth

REM --------------------------------------------------
REM fourth
REM --------------------------------------------------
:fourth
ECHO Lanzar QGroundControl? Y o N - 
ECHO --- Asegurate que PX4 y AirSim estan vinculados antes de lanzarlo
SET /p choose="Y o N - "
IF /i "%choose%" == "Y" GOTO QGC
IF /i "%choose%" == "y" GOTO QGC
GOTO fifth

REM --------------------------------------------------
REM QGroundControl QGC
REM --------------------------------------------------
:QGC
start C:\"Program Files"\QGroundControl\QGroundControl.exe
ECHO --- Lanzado QGC
ECHO --- ASEGURATE DE HABER LANZADO PX4 CORRECTAMENTE
goto fifth

REM --------------------------------------------------
REM fifth
REM --------------------------------------------------
:fifth
SET /p choose="Lanzar GUI Linux (vcxsrv)? Y o N - "
IF /i "%choose%" == "Y" GOTO vcxsrv
IF /i "%choose%" == "y" GOTO vcxsrv
GOTO sixth

REM --------------------------------------------------
REM vcxsrv
REM --------------------------------------------------
:vcxsrv
start C:\Users\%USERNAME%\Desktop\Workspace\AirSim\run\config.xlaunch
REM con esto no funciona, termina petando - start C:\"Program Files"\VcXsrv\vcxsrv.exe -ac -multiwindow -clipboard
ECHO --- Lanzado vcxsrv
goto sixth

REM --------------------------------------------------
REM sixth
REM --------------------------------------------------
:sixth
SET /p choose="Lanzar WSL para ROS? Y o N - "
IF /i "%choose%" == "Y" GOTO ROS
IF /i "%choose%" == "y" GOTO ROS
GOTO end

REM --------------------------------------------------
REM ROS
REM --------------------------------------------------
:ROS
wt wsl.exe ~ ; split-pane -H wsl.exe ~ 
ECHO --- Lanzado ROS
ECHO --- EN UNA TERMINAL
ECHO --- Ir a la carpeta de AirSim: cd /mnt/c/Users/%USERNAME%/Desktop/Workspace/AirSim/ros/src/airsim_tutorial_pkgs
ECHO --- Copiar los settings json del ROS: cp settings/front_stereo_and_center_mono.json /mnt/c/Users/%USERNAME%/Documents/AirSim/settings.json
ECHO --- source ../../devel/setup.bash
ECHO --- export WSL_HOST_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
ECHO --- roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP
ECHO --- DAR AL PLAY AL AIRSIM CON EL SETTINGS.JSON CORRECTO
ECHO --- EN LA OTRA TERMINAL
ECHO --- Ir a la carpeta de AirSim: cd /mnt/c/Users/%USERNAME%/Desktop/Workspace/AirSim/ros/
ECHO --- source devel/setup.bash
ECHO --- roslaunch airsim_tutorial_pkgs front_stereo_and_center_mono.launch

goto end

REM --------------------------------------------------
:end
PAUSE