#include "TaskAllocation.cpp"

int main()
{
  using SimpleMotionPowerSink = rmf_battery::agv::SimpleMotionPowerSink;
  using SimpleDevicePowerSink = rmf_battery::agv::SimpleDevicePowerSink;
  srand(42);
  // Graph
  // 00-01-02-03
  // |  |  |  |
  // 04-05-06-07
  // |  |  |  |
  // 08-09-10-11
  // |  |  |  | 
  // 12-13-14-15

  rmf_traffic::agv::Graph graph;
  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };
  const double edge_length = 1500; //metres
  const std::string map_name = "test_map";
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      // const auto random = (double) rand() / RAND_MAX;
      const double random = 1.0;
      graph.add_waypoint(map_name, {j*edge_length*random, -i*edge_length*random});
    }
  }

  for (int i = 0; i < 16; ++i)
  {
    if ((i+1) % 4 != 0)
      add_bidir_lane(i,i+1);
    if (i + 4 < 16)
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

  // TODO: parse yaml to obtain list of tasks and robots
  std::vector<RobotState> robot_states =
  {
    RobotState::make(1, 13, 13),
    RobotState::make(2, 2, 2),
    // RobotState::make(3, 5, 5),
    // RobotState::make(4, 8, 8),
    // RobotState::make(5, 10, 10),
  };
  

  rmf_battery::agv::BatterySystem battery_system{24.0, 40.0, 2.0};
  rmf_battery::agv::MechanicalSystem mechanical_system{70.0, 40.0, 0.22};
  rmf_battery::agv::PowerSystem power_system{"processor", 20.0};

  std::shared_ptr<SimpleMotionPowerSink> motion_sink =
    std::make_shared<SimpleMotionPowerSink>(battery_system, mechanical_system);
  std::shared_ptr<SimpleDevicePowerSink> device_sink =
    std::make_shared<SimpleDevicePowerSink>(battery_system, power_system);

  const bool drain_battery = true;
  auto charge_battery_task = ChargeBatteryTaskRequest::make(
    battery_system, motion_sink, device_sink, planner, drain_battery);

  std::vector<ConstTaskRequestPtr> tasks =
  {
    DeliveryTaskRequest::make(1, 0 , 3, motion_sink, device_sink, planner, drain_battery),
    DeliveryTaskRequest::make(2, 15, 2, motion_sink, device_sink, planner, drain_battery),
    DeliveryTaskRequest::make(3, 7, 9, motion_sink, device_sink, planner, drain_battery),
    DeliveryTaskRequest::make(4, 8, 11, motion_sink, device_sink, planner, drain_battery),
    // DeliveryTaskRequest::make(5, 10, 0, motion_sink, device_sink, planner, drain_battery),
    // DeliveryTaskRequest::make(6, 4, 8, motion_sink, device_sink, planner, drain_battery),
    // DeliveryTaskRequest::make(7, 8, 14, motion_sink, device_sink, planner, drain_battery),
    // DeliveryTaskRequest::make(8, 5, 11, motion_sink, device_sink, planner, drain_battery),
    // DeliveryTaskRequest::make(9, 9, 0, motion_sink, device_sink, planner, drain_battery),
    // DeliveryTaskRequest::make(10, 1, 3, motion_sink, device_sink, planner, drain_battery),
  };



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
  
  if (!solution)
  {
    std::cout << "No solution found!" << std::endl;
    return 0;
  }
}
