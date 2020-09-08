#include "TaskAllocation.cpp"
#include "yaml-cpp/yaml.h"

class TaskConfig
{
  public:
    struct Agent
    {
      std::size_t id;
      std::size_t wp;
      std::size_t charging_wp;
    };
    
    struct DeliveryTask
    {
      std::size_t id;
      std::size_t pickup;
      std::size_t dropoff;
    };

    double edge_length;
    int grid_size;
    std::vector<DeliveryTask> deliveries;
    std::vector<Agent> agents;
};

TaskConfig load_task_config(const std::string& yaml_path)
{
  YAML::Node yaml_config = YAML::LoadFile(yaml_path);
  TaskConfig config;
  config.grid_size = yaml_config["grid_size"].as<int>();
  config.edge_length = yaml_config["grid_length"].as<double>();

  for ( auto task : yaml_config["tasks"]["delivery"])
  {
    TaskConfig::DeliveryTask delivery = {
      task["id"].as<std::size_t>(),
      task["pickup"].as<std::size_t>(),
      task["dropoff"].as<std::size_t>()
    };   
    config.deliveries.push_back(delivery);
  }

  for ( auto ag: yaml_config["agents"])
  {
    TaskConfig::Agent agent = {
      ag["id"].as<std::size_t>(),
      ag["wp"].as<std::size_t>(),
      ag["charging_wp"].as<std::size_t>()
    };   
    config.agents.push_back(agent);
  }
  return config;
}

//=========================================================================

int main(int argc, char* argv[])
{ 
  // load yaml file
  std::string task_config(argv[argc-1]);
  TaskConfig cfg = load_task_config(task_config);
  
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
      DeliveryTaskRequest::make(tk.id, tk.pickup, tk.dropoff, planner));
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
