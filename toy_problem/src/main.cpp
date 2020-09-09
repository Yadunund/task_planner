#include "TaskAllocation.cpp"
#include "ParseTasks.cpp"

//=========================================================================

int main(int argc, char* argv[])
{ 
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;

  if (argc < 3)
  {
    std::cerr<< " Incorrect number of args, input format: \n"
             << " \t ./toy_problem {INPUT_YAML} {OUTPUT_YAML}" << std::endl;
    return 0;
  }

  // load yaml file
  std::string task_config(argv[1]);
  std::string allocation_config(argv[2]);
  TaskConfig cfg = read_task_config(task_config);
  
  rmf_traffic::agv::Graph graph;
  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };
  const std::string map_name = "test_map";
  for (int i = 0; i < cfg.grid_size; ++i)
  {
    for (int j = 0; j < cfg.grid_size; ++j)
    {
      // const auto random = (double) rand() / RAND_MAX;
      const double random = 1.0;
      graph.add_waypoint(map_name, 
        {j*cfg.edge_length*random, -i*cfg.edge_length*random});
    }
  }

  for (int i = 0; i < cfg.grid_size*cfg.grid_size; ++i)
  {
    if ((i+1) % cfg.grid_size != 0)
      add_bidir_lane(i,i+1);
    if (i + cfg.grid_size < cfg.grid_size*cfg.grid_size)
      add_bidir_lane(i,i+4);
  }

  const auto shape = rmf_traffic::geometry::make_final_convex<
    rmf_traffic::geometry::Circle>(1.0);
  const rmf_traffic::Profile profile{shape, shape};
  const rmf_traffic::agv::VehicleTraits traits(
    {1.0, 0.7}, {0.6, 0.5}, profile);
  rmf_traffic::schedule::Database database;
  const auto default_options = rmf_traffic::agv::Planner::Options{
    nullptr};
    
  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
      rmf_traffic::agv::Planner::Configuration{graph, traits},
      default_options);

  rmf_battery::agv::BatterySystem battery_system{24.0, 40.0, 2.0};
  rmf_battery::agv::MechanicalSystem mechanical_system{70.0, 40.0, 0.22};
  rmf_battery::agv::PowerSystem power_system{"processor", 40.0};

  std::shared_ptr<SimpleMotionPowerSink> motion_sink =
    std::make_shared<SimpleMotionPowerSink>(battery_system, mechanical_system);
  std::shared_ptr<SimpleDevicePowerSink> device_sink =
    std::make_shared<SimpleDevicePowerSink>(battery_system, power_system);
  
  std::vector<RobotState> robot_states;
  std::vector<ConstTaskRequestPtr> tasks;
  
  for (auto agent : cfg.agents)
  {
    robot_states.push_back(
      RobotState::make(agent.id, agent.wp, agent.charging_wp));
  };

  for (auto tk : cfg.deliveries)
  {
    tasks.push_back(
      DeliveryTaskRequest::make(
        tk.id, tk.pickup, tk.dropoff, motion_sink, device_sink, planner));
  };

  auto charge_battery_task = ChargeBatteryTaskRequest::make(
    battery_system, motion_sink, device_sink, planner);

  TaskPlanner task_planner(
    tasks,
    robot_states,
    charge_battery_task,
    true,
    true
  );

  const auto begin_time = std::chrono::steady_clock::now();
  const auto solution = task_planner.solve();
  const auto end_time = std::chrono::steady_clock::now();
  const double time_to_solve = rmf_traffic::time::to_seconds(
    end_time - begin_time);

  std::cout << "Time taken to solve: " << time_to_solve << " s" << std::endl;
  write_allocation_config(allocation_config, solution->assigned_tasks);
  
  if (!solution)
  {
    std::cout << "No solution found!" << std::endl;
    return 0;
  }
}
