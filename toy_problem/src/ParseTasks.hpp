
#include "TaskAllocation.hpp"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>

namespace {

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

/// Parse the Tasks Description
/// \arg filename
/// \return TaskConfig
TaskConfig read_task_config(const std::string& yaml_path)
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

/// Output Tasks Allocation result
/// \arg filename
/// \return Allocated Tasks
void write_allocation_config(
  const std::string& yaml_path,
  const AssignedTasks& allocations )
{
  // allocations
  

  YAML::Emitter out;
  out << YAML::Value << YAML::BeginSeq;
  for(std::size_t i = 0; i < allocations.size(); ++i)
  {
    out << YAML::BeginMap;
    out << YAML::Key << "agent_id" << YAML::Value << i;
    out << YAML::Key << "tasks";
    out << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for( auto assignment : allocations[i])
      out << assignment.task_id;
    out << YAML::EndSeq << YAML::EndMap;
  }
  out << YAML::EndSeq;

  std::ofstream fout(yaml_path);
  fout << out.c_str();
  fout.close();
}

}  // anonymous namespace
