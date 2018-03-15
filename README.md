# gsoc-2018-jderobot-application
this repository is for gsoc-2018-jderobot's programming challenges and application

## 1. installation challenge

I am sorry for that I didn't save the jderobot installation shell log. I show the install and example's screenshot.

References: http://jderobot.org/Installation

## 2. C++ challenge

```
cd Labyrinth
mkdir build
cd build
cmake ..
make
./Labyrinth
```

## 3. python challenge

```
cd ConwayGOF
python3 main.py
```

## 4.demp
move the file to catkin workspace
```
catkin make
rosrun ompl_gui ompl_gui
rosrun path_planning_ompl OptimalPlanning.py
rqt
rviz #add path
```