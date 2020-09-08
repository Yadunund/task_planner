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
```bash
./build/toy_problem/toy_problem vnofisnfovbs src/task_planner/task_config.yaml
```
