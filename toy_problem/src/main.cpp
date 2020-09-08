#include "TaskAllocation.cpp"

int main()
{  
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
  const double edge_length = 100; //metres
  const std::string map_name = "test_map";
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      graph.add_waypoint(map_name, {j*edge_length, -i*edge_length});
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
    RobotState::make(1, graph.get_waypoint(13).get_location(), 13),
    RobotState::make(2, graph.get_waypoint(2).get_location(), 2),
  };
  
  std::vector<ConstTaskRequestPtr> tasks =
  {
    DeliveryTaskRequest::make(1, 0 , 3, planner),
    DeliveryTaskRequest::make(2, 15, 2, planner),
    DeliveryTaskRequest::make(3, 7, 9, planner),
    DeliveryTaskRequest::make(4, 8, 11, planner),
    DeliveryTaskRequest::make(5, 10, 0, planner),
    DeliveryTaskRequest::make(6, 4, 8, planner),
    DeliveryTaskRequest::make(7, 8, 14, planner),
    DeliveryTaskRequest::make(8, 5, 11, planner),
    DeliveryTaskRequest::make(9, 9, 0, planner),
    DeliveryTaskRequest::make(10, 1, 3, planner)
  };

  std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system =
    std::make_shared<rmf_battery::agv::BatterySystem>(24.0, 40.0, 2.0);

  auto charge_battery_task = ChargeBatteryTaskRequest::make(
    battery_system, planner);

  TaskPlanner task_planner(
    tasks,
    robot_states,
    charge_battery_task,
    true,
    true
  );

  const auto solution = task_planner.solve();
  
  if (!solution)
  {
    std::cout << "No solution found!" << std::endl;
    return 0;
  }
}
