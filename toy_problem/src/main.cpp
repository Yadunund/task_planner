#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_utils/optional.hpp>

#include <queue>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <unordered_map>
#include <map>
#include <iostream>

namespace {

using AssignedTasks = std::unordered_map<std::size_t, std::queue<std::size_t>>;
using UnassignedTasks =
  std::unordered_map<std::size_t, std::size_t>;

struct DeliveryTask
{
  std::size_t id; // task id
  rmf_traffic::Time request_time;
  std::size_t pickup_waypoint;
  std::size_t dropoff_waypoint;
};

struct Robot
{
  std::size_t id; // participant id
  std::size_t next_start_waypoint;
  // // The durations in seconds from now when the robot is next available.
  // // If the robot already has a queue of tasks, this time should reflect the
  // // duration between now and the time when its last task is completed.
  // double next_available_time;
};

struct Node
{
  AssignedTasks assigned_tasks;
  UnassignedTasks unassigned_tasks;
  double cost_estimate;
};

using NodePtr = std::shared_ptr<Node>;
using ConstNodePtr = std::shared_ptr<const Node>;

struct LowestCostEstimate
{
  bool operator()(const ConstNodePtr& a, const ConstNodePtr& b)
  {
    return b->cost_estimate < a->cost_estimate;
  }
};

class TaskPlanner
{
public:
  using PriorityQueue = std::priority_queue<
      ConstNodePtr,
      std::vector<ConstNodePtr>,
      LowestCostEstimate>;

  using Robots = std::unordered_map<std::size_t, Robot>;

  TaskPlanner(
    std::vector<DeliveryTask> tasks,
    std::vector<Robot> robots,
    rmf_traffic::agv::Planner& planner)
  : _planner(std::move(planner))
  {
    _num_tasks = tasks.size();
    _num_robots = robots.size();

    for (const auto& task : tasks)
      _tasks.insert({task.id, task});
    for (const auto& robot : robots)
      _robots.insert({robot.id, robot});

    _graph = _planner.get_configuration().graph();

    // Initialize the starting node and add it to the priority queue
    initialize_start();

  }

  ConstNodePtr solve()
  {
    while (!_priority_queue.empty())
    {
      auto top = _priority_queue.top();

      // Pop the top of the priority queue
      _priority_queue.pop();

      // Check if unassigned tasks is empty -> solution found
      if (top->unassigned_tasks.empty())
      {
        _goal_node = *top;
        print_node(*top);
        return top;
      }

      // Apply possible actions to expand the node
      const auto new_nodes = expand(top);
      ++_total_queue_expansions;
      _total_queue_entries += new_nodes.size();

      // Add copies and with a newly assigned task to queue
      for (const auto&n : new_nodes)
        _priority_queue.push(n);
      
    }

    return nullptr;
  }

private:
  std::size_t _num_tasks = 0;
  std::size_t _num_robots = 0;
  std::size_t _total_queue_entries = 0;
  std::size_t _total_queue_expansions = 0;
  std::unordered_map<std::size_t, DeliveryTask> _tasks;
  Robots _robots;
  rmf_traffic::agv::Graph _graph;
  rmf_traffic::agv::Planner _planner;

  Node _goal_node;
  PriorityQueue _priority_queue;

  void initialize_start()
  {
    auto starting_node = std::make_shared<Node>();
    for (const auto& robot : _robots)
      starting_node->assigned_tasks[robot.first] = {};

    for (const auto& task : _tasks)
      starting_node->unassigned_tasks[task.first] = get_best_assignment(
          *starting_node, _robots, task.first);

    starting_node->cost_estimate = compute_f(*starting_node);
    _priority_queue.push(starting_node);
    _total_queue_entries++;
  }

  std::size_t get_best_assignment(
    const Node& n, const Robots& robots, const uint64_t u)
  {
    std::unordered_map<std::size_t, double> scores;
    // For each robot agent, get the estimated duration of the task
    for (const auto agent : n.assigned_tasks)
    {
      // Compute the duration of the delivery task given the state of the node
      scores[agent.first] = get_delivery_estimate(
        agent.second, _robots[agent.first], u);
    }

    double best_score = std::numeric_limits<double>::max();
    std::size_t best_agent;
    for (const auto& score : scores)
    {
      if (score.second <= best_score)
      {
        best_score = score.second;
        best_agent = score.first;
      }
    }

    return best_agent;
  }

  double get_delivery_estimate(
    const std::queue<uint64_t> queue, const Robot& robot, const uint64_t u)
  {
    // TODO
    return 0.0;
  }

  double compute_g(const Node& n)
  {
    // TODO
    return 0;
  }

  double compute_h(const Node& n)
  {
    // TODO
    return 0;
  }

  double compute_f(const Node& n)
  {
    return compute_g(n) + compute_h(n);
  }

  std::vector<ConstNodePtr> expand(ConstNodePtr parent)
  {
    std::vector<ConstNodePtr> new_nodes;

    return new_nodes;
  }

  void print_node(const Node& node)
  {
    std::cout << "Cost estimate: " << node.cost_estimate << std::endl;
    for (auto assignment: node.assigned_tasks)
    {
      std::cout << "Robot: " << assignment.first <<std::endl;
      while (!assignment.second.empty())
      {
        std::cout << "  " << assignment.second.front() << std::endl;
        assignment.second.pop();
      }
    }
  }

};

rmf_utils::clone_ptr<rmf_traffic::agv::ScheduleRouteValidator>
make_test_schedule_validator(
  const rmf_traffic::schedule::Viewer& viewer,
  rmf_traffic::Profile profile)
{
  return rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
    viewer,
    std::numeric_limits<rmf_traffic::schedule::ParticipantId>::max(),
    std::move(profile));
}

} // anonymous namespace


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
    make_test_schedule_validator(database, profile)};
  rmf_traffic::agv::Planner planner{
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    default_options    
  };

  // TODO: parse yaml to obtain list of tasks and robots
  std::vector<Robot> robots;
  std::vector<DeliveryTask> tasks;
  Robot robot1{1, 13};
  Robot robot2{2, 2};
  robots.emplace_back(robot1);
  robots.emplace_back(robot2);
  
  const auto start_time = std::chrono::steady_clock::now();
  DeliveryTask task1{1, start_time, 0, 3};
  DeliveryTask task2{2, start_time, 15, 2};
  DeliveryTask task3{3, start_time, 7, 9};
  tasks.emplace_back(task1);
  tasks.emplace_back(task2);
  tasks.emplace_back(task3);

  TaskPlanner task_planner{
    tasks,
    robots,
    planner
  };

  task_planner.solve();
  
  return 1;
}

