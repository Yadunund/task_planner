#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/Profile.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <rmf_battery/agv/SimpleDevicePowerSink.hpp>
#include <rmf_battery/agv/SimpleMotionPowerSink.hpp>
#include <rmf_battery/agv/BatterySystem.hpp>

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
  std::size_t charging_waypoint;
  rmf_traffic::Time finish_time = std::chrono::steady_clock::now();
  double cost = 0.0;
  double battery_soc = 1.0;
  static RobotState make(
    std::size_t id_, Eigen::Vector2d p_, std::size_t charging_waypoint_)
  {
    return RobotState{id_, p_, charging_waypoint_};
  }
};

// ============================================================================
std::size_t estimate_waypoint(
  const Eigen::Vector2d location, const rmf_traffic::agv::Graph& graph)
{
  auto nearest_dist = std::numeric_limits<double>::infinity();
  std::size_t nearest_wp = 0;
  for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
  {
    auto wp_location = graph.get_waypoint(i).get_location();
    const double dist = (location - wp_location).norm();
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      nearest_wp = i;
    }
  }

  return nearest_wp;
}

// ============================================================================
class TaskRequest
{
public:

  virtual std::size_t id() const = 0;
  virtual rmf_utils::optional<RobotState> estimate(
    const RobotState& initial_state) const = 0;

};

using ConstTaskRequestPtr = std::shared_ptr<TaskRequest>;

// ============================================================================
class ChargeBatteryTaskRequest : public TaskRequest
{
public:
  ChargeBatteryTaskRequest(
    std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system,
    std::shared_ptr<rmf_traffic::agv::Planner> planner)
  : _battery_system(battery_system),
    _planner(planner)
  {
    // Do nothing
  }

  static ConstTaskRequestPtr make(
  std::shared_ptr<rmf_battery::agv::BatterySystem> battery_system,
  std::shared_ptr<rmf_traffic::agv::Planner> planner)
  {
    return std::make_shared<ChargeBatteryTaskRequest>(battery_system, planner);
  }

  std::size_t id() const final
  {
    return _id;
  }

  rmf_utils::optional<RobotState> estimate(const RobotState& initial_state) const final
  {
    rmf_utils::optional<RobotState> state = initial_state;

    // Compute time taken to reach charging waypoint from current location
    const auto& graph = _planner->get_configuration().graph();
    state->p = graph.get_waypoint(
      initial_state.charging_waypoint).get_location();
    const auto now = std::chrono::steady_clock::now();
    rmf_utils::optional<Eigen::Vector2d> location = initial_state.p;
    auto start_wp = estimate_waypoint(initial_state.p, graph);
    rmf_traffic::agv::Planner::Start start{
      initial_state.finish_time,
      start_wp,
      0.0,
      std::move(location)};

    rmf_traffic::agv::Planner::Goal goal{initial_state.charging_waypoint};

    const auto result = _planner->plan(start, goal);
    if (result.success())
    {
      const auto itinerary = result->get_itinerary();
      const auto finish_time = itinerary.back().trajectory().finish_time();
      assert(finish_time);
      state->finish_time = *finish_time;
    }

    // TODO Check if robot has charge to make it back to its charging dock
    //

    double battery_soc = initial_state.battery_soc;
    double delta_soc = _charge_soc - battery_soc;
    assert(delta_soc >= 0.0);
    double time_to_charge =
      (3600 * delta_soc * _battery_system->nominal_capacity()) / _battery_system->charging_current();

    state->finish_time = rmf_traffic::time::apply_offset(state->finish_time, time_to_charge);
    state->cost = rmf_traffic::time::to_seconds(state->finish_time - now);
    return state;
  }

private:
  // fixed id for now
  std::size_t _id = 42;
  std::shared_ptr<rmf_battery::agv::BatterySystem> _battery_system;
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  // soc to always charge the battery up to
  double _charge_soc = 1.0;
 
};

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

rmf_utils::optional<RobotState> estimate(const RobotState& initial_state) const final
{
  rmf_utils::optional<RobotState> state = initial_state;
  const auto& graph = _planner->get_configuration().graph();
  state->p = graph.get_waypoint(_dropoff_waypoint).get_location();
  const auto now = std::chrono::steady_clock::now();
  auto initial_waypoint = estimate_waypoint(initial_state.p, graph);

  rmf_utils::optional<Eigen::Vector2d> location = initial_state.p;
  Planner::Start start{
    initial_state.finish_time,
    initial_waypoint,
    0.0,
    std::move(location)};

  Planner::Goal goal{_pickup_waypoint};
  
  // First move to waypoint on graph
  const auto result_to_pickup = _planner->plan(start, goal);
  if (result_to_pickup.success())
  {
    const auto& itinerary = result_to_pickup->get_itinerary();
    auto finish_time = itinerary.back().trajectory().finish_time();
    assert(finish_time);

    Planner::Start start_2{
      *finish_time,
      _pickup_waypoint,
      0.0};

    Planner::Goal goal_2{_dropoff_waypoint};

    const auto result_to_dropoff = _planner->plan(start_2, goal_2);
    if (result_to_dropoff.success())
    {
      const auto& itinerary = result_to_dropoff->get_itinerary();
      finish_time = itinerary.back().trajectory().finish_time();
      assert(finish_time);
      state->finish_time = *finish_time;
      state->cost = rmf_traffic::time::to_seconds(*finish_time - now);
    }
    else
    {
      return rmf_utils::nullopt;
    }
    
  }
  else
  {
    // If we are unsuccessful in generating a plan, set the cost to infinity
    // state->cost = std::numeric_limits<double>::max();
    return rmf_utils::nullopt;
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

  // Map finish time to RobotState
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

  double best_cost() const
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
      {state.cost, state});
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
    if (finish.has_value())
    {
      initial_map.insert({finish->cost, finish.value()});
    }
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
  auto a = node.assigned_tasks.begin();
  std::size_t t = 0;
  while(a != node.assigned_tasks.end())
  {
    const auto& current_agent = a->second;

    if (t < current_agent.size())
    {
      const auto& task_id = current_agent[t].task_id;
      const auto agent_insertion = agent_table->agent.insert({a->first, nullptr});
      if (agent_insertion.second)
        agent_insertion.first->second = std::make_unique<TaskTable>();

      auto* task_table = agent_insertion.first->second.get();

      const auto task_insertion = task_table->task.insert({task_id, nullptr});
      if (task_insertion.second)
      {
        new_node = true;
        task_insertion.first->second = std::make_unique<AgentTable>();
      }

      agent_table = task_insertion.first->second.get();
      ++t;
    }
    else
    {
      t = 0;
      ++a;
    }
  }

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
  using BatterySystem = rmf_battery::agv::BatterySystem;

  TaskPlanner(
    std::vector<ConstTaskRequestPtr> tasks,
    std::vector<RobotState> initial_states,
    ConstTaskRequestPtr charge_battery,
    const bool use_filter,
    const bool debug)
  : _charge_battery(charge_battery),
    _use_filter(use_filter),
    _debug(debug)
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
    Filter filter{!_use_filter};

    while (!_priority_queue.empty())
    {
      // Print the nodes if debug is true
      if (_debug)
      {
        auto copy = _priority_queue;
        while (!copy.empty())
        {
          const auto top = copy.top();
          copy.pop();
          print_node(*top);
        }
        std::cout<<"==================================================="<<std::endl;
      }

      auto top = _priority_queue.top();

      // Pop the top of the priority queue
      _priority_queue.pop();

      // Check if unassigned tasks is empty -> solution found
      if (top->unassigned_tasks.empty())
      {
        _goal_node = *top;
        std::cout << "Solution found!" << std::endl;
        std::cout << "  Final queue size: " << _priority_queue.size()
                  << std::endl;
        std::cout << "  Nodes added to queue: " << _total_queue_entries
                  << std::endl;
        std::cout << "  Nodes expanded: " << _total_queue_expansions
                  << std::endl;
        std::cout << "Assignments: " << std::endl;
        print_node(*top);
        return top;
      }

      // Apply possible actions to expand the node
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

  ConstTaskRequestPtr _charge_battery;

  bool _use_filter;
  bool _debug;
  Node _goal_node;
  PriorityQueue _priority_queue;


  double compute_g(const Node& node)
  {
    double cost = 0.0;
    for (const auto& agent : node.assigned_tasks)
    {
      for (const auto& assignment : agent.second)
      {
        cost += assignment.state.cost;
      }
    }

    return cost;
  }

  double compute_h(const Node& node)
  {
    double cost = 0.0;
    for (const auto& u : node.unassigned_tasks)
    {
      cost += u.second.candidates.best_cost();
    }
    
    return cost;
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
      for (auto it = range.begin; it!= range.end; it++)
      {
        auto new_node = std::make_shared<Node>(*parent);
        const auto& state = it->second;
        // Assign the unassigned task
        new_node->assigned_tasks[state.id].push_back(
          Assignment{u.first, state});
        
        // Erase the assigned task from unassigned tasks
        new_node->unassigned_tasks.erase(u.first);

        // Update states of unassigned tasks for the candidate
        bool discard = false;
        for (auto& new_u : new_node->unassigned_tasks)
        {
          const auto finish = new_u.second.request->estimate(state);
          if (finish.has_value())
          {
            new_u.second.candidates.update_candidate(
              finish.value());
          }
          else
            discard = true;
        }

        if (discard)
          continue;

        // Update the cost estimate for new_node
        new_node->cost_estimate = compute_f(*new_node);

        // Apply filter
        if (filter.ignore(*new_node))
        {
          std::cout << "Ignoring node: " << std::endl;
          print_node(*new_node);
          std::cout << "==============================================" << std::endl;
          continue;
        }

        new_nodes.push_back(std::move(new_node));
        
      }
    }

    // TODO Assign charging task to each robot
    for (auto agent : parent->assigned_tasks)
    {
      auto new_node = std::make_shared<Node>(*parent);

    }

    return new_nodes;
  }

  void print_node(const Node& node)
  {
    std::cout << " -- " << node.cost_estimate << ": <";
    bool first = true;
    for (const auto& agent : node.assigned_tasks)
    {
      if (first)
        first = false;
      else
        std::cout << ", ";

      std::cout << agent.first << ": [";
      for (const auto i : agent.second)
        std::cout << " " << i.task_id;
      std::cout << " ]";
    }

    std::cout << " -- ";
    first = true;
    for (const auto& u : node.unassigned_tasks)
    {
      if (first)
        first = false;
      else
        std::cout << ", ";

      std::cout << u.first << ":";
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
        std::cout << " " << it->second.id;
    }

    std::cout << ">" << std::endl;
  }

};

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
    nullptr};
    
  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
      rmf_traffic::agv::Planner::Configuration{graph, traits},
      default_options);

  // TODO: parse yaml to obtain list of tasks and robots
  std::vector<RobotState> robot_states =
  {
    RobotState::make(1, graph.get_waypoint(13).get_location(), 13),
    RobotState::make(2, graph.get_waypoint(2).get_location(), 2)
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

  clock_t tStart = clock();

  const auto solution = task_planner.solve();
  
  if (!solution)
  {
    std::cout << "No solution found!" << std::endl;
    return 0;
  }
  printf("Time taken: %.4fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

