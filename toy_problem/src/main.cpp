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
#include <limits>

namespace {
// ============================================================================
struct RobotState
{
  std::size_t id;
  Eigen::Vector2d p;
  double finish_time = 0.0;
  static RobotState make(std::size_t id_, Eigen::Vector2d p_)
  {
    return RobotState{id_, p_};
  }
};

// ============================================================================
class TaskRequest
{
public:

  virtual std::size_t id() const = 0;
  virtual RobotState estimate(const RobotState& initial_state) const = 0;

};

using ConstTaskRequestPtr = std::shared_ptr<TaskRequest>;

// ============================================================================
class DeliveryTaskRequest : public TaskRequest
{
public:
  using Planner = rmf_traffic::agv::Planner;
  DeliveryTaskRequest(
    std::size_t id,
    std::size_t pickup,
    std::size_t dropoff,
    std::shared_ptr<Planner> planner)
  : _id(id),
    _pickup_waypoint(pickup),
    _dropoff_waypoint(dropoff),
    _planner(planner)
  {
    // Do nothing
  }

static ConstTaskRequestPtr make(
  std::size_t id,
  std::size_t pickup,
  std::size_t dropoff,
  std::shared_ptr<Planner> planner)
{
  return std::make_shared<DeliveryTaskRequest>(id, pickup, dropoff, planner);
}

std::size_t id() const final
{
  return _id;
}

RobotState estimate(const RobotState& initial_state) const final
{
  RobotState state;
  state.id =initial_state.id;
  const auto& graph = _planner->get_configuration().graph();
  state.p = graph.get_waypoint(_dropoff_waypoint).get_location();
  const auto start_time = std::chrono::steady_clock::now() +
    rmf_traffic::time::from_seconds(initial_state.finish_time);
  rmf_utils::optional<Eigen::Vector2d> location = initial_state.p;
  Planner::Start start{
    start_time,
    _pickup_waypoint,
    0.0,
    std::move(location)};

  Planner::Goal goal{_dropoff_waypoint};
  
  const auto result = _planner->plan(start, goal);
  if (result.success())
  {
    const auto& itinerary = result->get_itinerary();
    state.finish_time = rmf_traffic::time::to_seconds(
        *itinerary.back().trajectory().finish_time() - start_time);
  }
  else
  {
    // If we are unsuccessful in generating a plan, set the finish time to infinity
    state.finish_time = std::numeric_limits<double>::max();
  }

  return state;
  
}

private:
  std::size_t _id; // task id
  std::size_t _pickup_waypoint;
  std::size_t _dropoff_waypoint;
  std::shared_ptr<Planner> _planner;
};

// ============================================================================
class Candidates
{
public:

  // struct Entry
  // {
  //   std::size_t candidate;
  //   RobotState state;
  // };

  // Map finish time to entry
  using Map = std::multimap<double, RobotState>;

  static Candidates make(
      const std::vector<RobotState>& initial_states,
      const TaskRequest& request);

  Candidates(const Candidates& other)
  {
    _value_map = other._value_map;
    _update_map();
  }

  Candidates& operator=(const Candidates& other)
  {
    _value_map = other._value_map;
    _update_map();
    return *this;
  }

  Candidates(Candidates&&) = default;
  Candidates& operator=(Candidates&&) = default;

  // We have have more than one best candidate so we store their iterators in
  // a Range
  struct Range
  {
    Map::const_iterator begin;
    Map::const_iterator end;
  };

  Range best_candidates() const
  {
    assert(!_value_map.empty());

    Range range;
    range.begin = _value_map.begin();
    auto it = range.begin;
    while (it->first == range.begin->first)
      ++it;

    range.end = it;
    return range;
  }

  double best_finish_time() const
  {
    assert(!_value_map.empty());
    return _value_map.begin()->first;
  }

  void update_candidate(RobotState state)
  {
    Map::iterator erase_it;
    for (auto it = _value_map.begin(); it != _value_map.end(); ++it)
      if (it->second.id == state.id)
        erase_it = it;
    _value_map.erase(erase_it);
    _candidate_map[state.id] = _value_map.insert(
      {state.finish_time, state});
  }


private:
  Map _value_map;
  std::map<std::size_t, Map::iterator> _candidate_map;

  Candidates(Map candidate_values)
    : _value_map(std::move(candidate_values))
  {
    _update_map();
  }

  void _update_map()
  {
    for (auto it = _value_map.begin(); it != _value_map.end(); ++it)
    {
      const auto id = it->second.id;
      _candidate_map[id] = it;
    }
  }
};

Candidates Candidates::make(
    const std::vector<RobotState>& initial_states,
    const TaskRequest& request)
{
  Map initial_map;
  for (const auto& state : initial_states)
  {
    const auto finish = request.estimate(state);
    initial_map.insert({finish.finish_time, finish});
  }

  return Candidates(std::move(initial_map));
}

// ============================================================================
struct PendingTask
{
  PendingTask(
      std::vector<RobotState> initial_states,
      ConstTaskRequestPtr request_)
    : request(std::move(request_)),
      candidates(Candidates::make(initial_states, *request))
  {
    // Do nothing
  }

  ConstTaskRequestPtr request;
  Candidates candidates;
};

// ============================================================================
struct Assignment
{
  std::size_t task_id;
  RobotState state;
};

using AssignedTasks =
  std::unordered_map<std::size_t, std::vector<Assignment>>;
using UnassignedTasks =
  std::unordered_map<std::size_t, PendingTask>;

// ============================================================================
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

// ============================================================================
class Filter
{
public:

  Filter(bool passthrough)
    : _passthrough(passthrough)
  {
    // Do nothing
  }

  bool ignore(const Node& node);

private:

  struct TaskTable;

  struct AgentTable
  {
    std::unordered_map<std::size_t, std::unique_ptr<TaskTable>> agent;
  };

  struct TaskTable
  {
    std::unordered_map<std::size_t, std::unique_ptr<AgentTable>> task;
  };

  bool _passthrough;
  AgentTable _root;
};

bool Filter::ignore(const Node& node)
{
  if (_passthrough)
    return false;

  bool new_node = false;

  // TODO(MXG): Consider replacing this tree structure with a hash set

  AgentTable* agent_table = &_root;
  std::size_t a = 0;
  std::size_t t = 0;
  // while(a < node.assignments.size())
  // {
  //   const auto& current_agent = node.assignments.at(a);

  //   if (t < current_agent.size())
  //   {
  //     const auto& task_id = current_agent[t].task_id;
  //     const auto agent_insertion = agent_table->agent.insert({a, nullptr});
  //     if (agent_insertion.second)
  //       agent_insertion.first->second = std::make_unique<TaskTable>();

  //     auto* task_table = agent_insertion.first->second.get();

  //     const auto task_insertion = task_table->task.insert({task_id, nullptr});
  //     if (task_insertion.second)
  //     {
  //       new_node = true;
  //       task_insertion.first->second = std::make_unique<AgentTable>();
  //     }

  //     agent_table = task_insertion.first->second.get();
  //     ++t;
  //   }
  //   else
  //   {
  //     t = 0;
  //     ++a;
  //   }
  // }

  return !new_node;
}

// ============================================================================
class TaskPlanner
{
public:
  using PriorityQueue = std::priority_queue<
      ConstNodePtr,
      std::vector<ConstNodePtr>,
      LowestCostEstimate>;

  TaskPlanner(
    std::vector<ConstTaskRequestPtr> tasks,
    std::vector<RobotState> initial_states,
    const bool use_filter)
  : _use_filter(use_filter)
  {
    // Initialize the starting node and add it to the priority queue
    auto starting_node = std::make_shared<Node>();
    for (const auto& state : initial_states)
      starting_node->assigned_tasks[state.id] = {};

    for (const auto& task : tasks)
      starting_node->unassigned_tasks.insert(
        {task->id(), PendingTask(initial_states, task)});

    starting_node->cost_estimate = compute_f(*starting_node);
    _priority_queue.push(starting_node);
    _total_queue_entries++;

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
      Filter filter{!_use_filter};
      const auto new_nodes = expand(top, filter);
      ++_total_queue_expansions;
      _total_queue_entries += new_nodes.size();

      // Add copies and with a newly assigned task to queue
      for (const auto&n : new_nodes)
        _priority_queue.push(n);
      
    }

    return nullptr;
  }

private:
  std::size_t _total_queue_entries = 0;
  std::size_t _total_queue_expansions = 0;
  bool _use_filter;
  Node _goal_node;
  PriorityQueue _priority_queue;

  double compute_g(const Node& node)
  {
    double cost = 0.0;
    for (const auto& agent : node.assigned_tasks)
    {
      for (const auto& assignment : agent.second)
      {
        cost += assignment.state.finish_time;
      }
    }
  }

  double compute_h(const Node& node)
  {
    double cost = 0.0;
    for (const auto& u : node.unassigned_tasks)
    {
      cost += u.second.candidates.best_finish_time();
    }
  }

  double compute_f(const Node& n)
  {
    return compute_g(n) + compute_h(n);
  }

  std::vector<ConstNodePtr> expand(ConstNodePtr parent, Filter& filter)
  {
    std::vector<ConstNodePtr> new_nodes;
    
    for (const auto& u : parent->unassigned_tasks)
    {
      const auto& range = u.second.candidates.best_candidates();
    }
    return new_nodes;
  }

  void print_node(const Node& node)
  {
    std::cout << "Cost estimate: " << node.cost_estimate << std::endl;
    for (const auto& agent: node.assigned_tasks)
    {
      std::cout << "Robot: " << agent.first <<std::endl;
      for (const auto& assignment : agent.second)
      {
        std::cout << "-- " << assignment.task_id <<std::endl;
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
    
  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
      rmf_traffic::agv::Planner::Configuration{graph, traits},
      default_options);

  // auto planner = std::make_shared<rmf_traffic::agv::Planner>(
  //     rmf_traffic::agv::Planner::Configuration(
  //       std::move(graph),
  //       std::move(traits)),
  //     rmf_traffic::agv::Planner::Options(nullptr));

  // TODO: parse yaml to obtain list of tasks and robots
  std::vector<RobotState> robot_states =
  {
    // RobotState::make(1, graph.get_waypoint(13).get_location());
    // RobotState::make(2, graph.get_waypoint(2).get_location());
  };
  
  std::vector<ConstTaskRequestPtr> tasks =
  {
    // DeliveryTaskRequest::make(1, 0 , 3, planner);
    // DeliveryTaskRequest::make(2, 15, 2, planner);
    // DeliveryTaskRequest::make(3, 7, 9, planner);
  };

  TaskPlanner task_planner(
    tasks,
    robot_states,
    true
  );

  const auto solution = task_planner.solve();
  
  if (!solution)
  {
    std::cout << "No solution found!" << std::endl;
    return 0;
  }
}

