# task_planner
Solution to a toy problem where N tasks have to be assigned to M robots.

## Build
```
mkdir -p ~/ws_task_planner/src
cd ws_task_planner/src
git clone https://github.com/Yadunund/task_planner.git
git clone https://github.com/osrf/rmf_core.git -b feature/battery_planning

cd ~/ws_task_planner
colcon build --packages-up-to toy_problem --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
```

## Run
1) Run Toy Problem:
```bash
# ./toy_problem {INPUT_TASKS_YAML} {OUTPUT_ALLOCATION_YAML}
./build/toy_problem/toy_problem src/task_planner/scripts/task_config.yaml \
src/task_planner/scripts/allocation.yaml
```
You will notice `allocation.yaml` is populated with the new task allocation.


2) Visualize Task Allocation
```bash
cd task_planner/scripts
# Make sure "alocation.yaml" and "task_config.yaml" are located in /scripts
python3 task_allocation_viz.py 
```
