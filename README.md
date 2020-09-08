# task_planner
Solution to a toy problem where N tasks have to be assigned to M robots.

## Build
```
mkdir -p ~/ws_task_planner/src
cd ws_task_planner/src
git clone https://github.com/Yadunund/task_planner.git
git clone https://github.com/osrf/rmf_core.git -b feature/battery_planning

cd ~/ws_task_planner
colcon build --packages-up-to toy_problem
```

## Run
```
./build/top_problem/toy_problem
```


# Result
```
===================================================
Solution found!
  Final queue size: 6256
  Nodes added to queue: 7756
  Nodes expanded: 1499
Assignments: 
 -- 10597.7: <2: [ 10 3 4 5 1 ], 1: [ 9 6 7 2 8 ] -- >
```
