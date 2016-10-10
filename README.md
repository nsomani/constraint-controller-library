# Dependencies

1. Robotics Library:
	This code is heavily based on the Robotics Library (www.roboticslibrary.org)

	* Option 1:
	```
	sudo apt-add-repository ppa:roblib/ppa
	sudo apt-get update
	sudo apt-get install librl-dev
	```

	* Option 2:
	Build from source, following the instructions on
	www.roboticslibrary.org

2. NLOpt: 
	```
	sudo apt-get install libnlopt-dev
	```

2. Json-cpp: 
	```
	sudo apt-get install libjsoncpp-dev
	```

# Compilation

```
mkdir build
cd build
cmake .. 
make -j5
```

# Examples
1. Launch the visualization from RL using one of the .sh files in the ```models``` folder.
2. Start a demo using the constraint description in JSON format in the ```models/json``` folder.

## Grasping a cup while avoiding obstacles (Comau Racer 7)
```
cd models && sh coach-racer7-cup.sh
```
In another terminal:
```
cd build && ./run_demo models/json/cup_grasping_side_collision_constraint.json
```

## Grasping a cup while avoiding obstacles (Kuka LWR)
```
cd models && sh coach-lwr-cup.sh
```
In another terminal:
```
cd build && ./run_demo models/json/cup_grasping_side_collision_constraint_lwr.json
```

## Grasping a cylindrical object at its rim (Comau Racer 7)
```
cd models && sh coach-racer7-cylinder.sh
```
In another terminal:
```
cd build && ./run_demo models/json/cylinder_grasping_constraint.json 
```

# Platforms
The code has been tested to work on Ubuntu 14.04 and 16.04.

# Citation
```
@inproceedings{Somani2016a,
	author = {Nikhil Somani and Markus Rickert and Andre Gaschler and Caixia Cai and Alexander Perzylo and Alois Knoll},
	title = {Task level robot programming using prioritized non-linear inequality constraints},
	booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	address = {Daejeon, Republic of Korea},
	month = {October},
	year = {2016}
}
```
