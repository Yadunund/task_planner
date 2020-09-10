#ifndef RMF_TASKS__TASK_ALLOCATION_HPP
#define RMF_TASKS__TASK_ALLOCATION_HPP

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
#include <cstdlib>
#include <cassert>
#include <set>
#include <algorithm>

namespace {
// ============================================================================
struct RobotState
{
  std::size_t waypoint;
  std::size_t charging_waypoint;
  double finish_time = 0.0;
  double battery_soc = 1.0;
  double threshold_soc = 0.25;
  static RobotState make(
    std::size_t wp_, std::size_t c_wp_)
  {
    return RobotState{wp_, c_wp_};
  }
};

// ============================================================================
struct Invariant
{
  std::size_t task_id;
  double invariant_cost;
};

struct InvariantLess
{
  bool operator()(const Invariant& a, const Invariant& b) const
  {
    return a.invariant_cost < b.invariant_cost;
  }
};

// ============================================================================
class TaskRequest
{
public:

  virtual std::size_t id() const = 0;

  virtual rmf_utils::optional<RobotState> estimate(
    const RobotState& initial_state) const = 0;

  virtual double invariant_duration() const = 0;

};

using ConstTaskRequestPtr = std::shared_ptr<TaskRequest>;

// ============================================================================
class ChargeBatteryTaskRequest : public TaskRequest
{
public:
  ChargeBatteryTaskRequest(
    rmf_battery::agv::BatterySystem battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    bool drain_battery)
  : _battery_system(battery_system),
    _motion_sink(motion_sink),
    _device_sink(device_sink),
    _planner(planner),
    _drain_battery(drain_battery)
  {
    _invariant_duration = 0.0;
  }

  static ConstTaskRequestPtr make(
    rmf_battery::agv::BatterySystem battery_system,
    std::shared_ptr<rmf_battery::MotionPowerSink> motion_sink,
    std::shared_ptr<rmf_battery::DevicePowerSink> device_sink,
    std::shared_ptr<rmf_traffic::agv::Planner> planner,
    bool drain_battery = true)
  {
    return std::make_shared<ChargeBatteryTaskRequest>(
      battery_system, motion_sink, device_sink, planner, drain_battery);
  }

  std::size_t id() const final
  {
    return _id;
  }

  rmf_utils::optional<RobotState> estimate(const RobotState& initial_state) const final
  {

    if (abs(initial_state.battery_soc - _charge_soc) < 1e-3)
    {
      std::cout << " -- Charge battery: Battery full" << std::endl;
      return rmf_utils::nullopt;
    }

    rmf_utils::optional<RobotState> state = initial_state;

    // Compute time taken to reach charging waypoint from current location
    state->waypoint = initial_state.charging_waypoint;
    const auto start_time = std::chrono::steady_clock::now() +
      rmf_traffic::time::from_seconds(initial_state.finish_time);

    double battery_soc = initial_state.battery_soc;

    if (initial_state.waypoint != initial_state.charging_waypoint)
    {
      // Compute plan to charging waypoint along with battery drain
      rmf_traffic::agv::Planner::Start start{
        start_time,
        initial_state.waypoint,
        0.0};

      rmf_traffic::agv::Planner::Goal goal{initial_state.charging_waypoint};

      const auto result = _planner->plan(start, goal);
      const auto& trajectory = result->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      const double travel_duration = rmf_traffic::time::to_seconds(
        finish_time - start_time);

      state->finish_time += travel_duration;

      if (_drain_battery)
      {
        const double dSOC_motion = _motion_sink->compute_change_in_charge(
          trajectory);
        const double dSOC_device = _device_sink->compute_change_in_charge(
          travel_duration);
        battery_soc = battery_soc - dSOC_motion - dSOC_device;
      }

      if (battery_soc <= state->threshold_soc)
      {
        // If a robot cannot reach its charging dock given its initial battery soc
        std::cout << " -- Charge battery: Unable to reach charger" << std::endl;
        return rmf_utils::nullopt;
      }
    }

    // Default _charge_soc = 1.0
    double delta_soc = _charge_soc - battery_soc;
    assert(delta_soc >= 0.0);
    double time_to_charge =
      (3600 * delta_soc * _battery_system.nominal_capacity()) /
      _battery_system.charging_current();

    state->finish_time += time_to_charge;
    state->battery_soc = _charge_soc;
    return state;

  }

  double invariant_duration() const final
  {
    return _invariant_duration;
  }

private:
  // fixed id for now
  std::size_t _id = 101;
  rmf_battery::agv::BatterySystem _battery_system;
  std::shared_ptr<rmf_battery::MotionPowerSink> _motion_sink;
  std::shared_ptr<rmf_battery::DevicePowerSink> _device_sink;
  std::shared_ptr<rmf_traffic::agv::Planner> _planner;
  bool _drain_battery;
  // soc to always charge the battery up to
  double _charge_soc = 1.0;
  double _invariant_duration;
 
};

// ============================================================================
class DeliveryTaskRequest : public TaskRequest
{
public:
  using Planner = rmf_traffic::agv::Planner;
  using MotionPowerSink = rmf_battery::MotionPowerSink;
  using DevicePowerSink = rmf_battery::DevicePowerSink;
  DeliveryTaskRequest(
    std::size_t id,
    std::size_t pickup,
    std::size_t dropoff,
    std::shared_ptr<MotionPowerSink> motion_sink,
    std::shared_ptr<DevicePowerSink> device_sink,
    std::shared_ptr<Planner> planner,
    bool drain_battery)
  : _id(id),
    _pickup_waypoint(pickup),
    _dropoff_waypoint(dropoff),
    _motion_sink(motion_sink),
    _device_sink(device_sink),
    _planner(planner),
    _drain_battery(drain_battery)
  {
    const auto start_time = std::chrono::steady_clock::now();
    Planner::Start start{
      start_time,
      _pickup_waypoint,
      0.0};

    Planner::Goal goal{_dropoff_waypoint};
    const auto result_to_dropoff = _planner->plan(start, goal);

    const auto trajectory = result_to_dropoff->get_itinerary().back().trajectory();
    const auto& finish_time = *trajectory.finish_time();
    _invariant_duration = rmf_traffic::time::to_seconds(
      finish_time - start_time);

    _invariant_battery_drain = 0.0;

    if (_drain_battery)
    {
      // Compute battery drain
      const double dSOC_motion = _motion_sink->compute_change_in_charge(trajectory);
      const double dSOC_device = _device_sink->compute_change_in_charge(_invariant_duration);
      _invariant_battery_drain = dSOC_motion + dSOC_device;
    }
    
  }

  static ConstTaskRequestPtr make(
    std::size_t id,
    std::size_t pickup,
    std::size_t dropoff,
    std::shared_ptr<MotionPowerSink> motion_sink,
    std::shared_ptr<DevicePowerSink> device_sink,
    std::shared_ptr<Planner> planner,
    bool drain_battery = true)
  {
    return std::make_shared<DeliveryTaskRequest>(
      id, pickup, dropoff, motion_sink, device_sink, planner, drain_battery);
  }

  std::size_t id() const final
  {
    return _id;
  }

  rmf_utils::optional<RobotState> estimate(const RobotState& initial_state) const final
  {
    rmf_utils::optional<RobotState> state = initial_state;
    state->waypoint = _dropoff_waypoint;

    const auto now = std::chrono::steady_clock::now();
    auto start_time = now +
      rmf_traffic::time::from_seconds(initial_state.finish_time);

    double battery_soc = initial_state.battery_soc;
    double travel_duration = 0;
    double dSOC_motion = 0.0;
    double dSOC_device = 0.0;

    if (initial_state.waypoint != _pickup_waypoint)
    {
      // Compute plan to pickup waypoint along with battery drain
      Planner::Start start{
      start_time,
      initial_state.waypoint,
      0.0};

      Planner::Goal goal{_pickup_waypoint};

      const auto result_to_pickup = _planner->plan(start, goal);
      // We assume we can always compute a plan
      const auto& trajectory = result_to_pickup->get_itinerary().back().trajectory();
      const auto& finish_time = *trajectory.finish_time();
      travel_duration = rmf_traffic::time::to_seconds(
        finish_time - start_time);

      state->finish_time += travel_duration;

      if(_drain_battery)
      {
        // Compute battery drain
        dSOC_motion = _motion_sink->compute_change_in_charge(trajectory);
        dSOC_device = _device_sink->compute_change_in_charge(travel_duration);
        battery_soc = battery_soc - dSOC_motion - dSOC_device;
      }

      if (battery_soc <= state->threshold_soc)
      {
        std::cout << " -- Delivery: Unable to reach pickup" << std::endl;
        return rmf_utils::nullopt;
      }

      start_time = finish_time;
    }

    // Factor in invariants
    state->finish_time += _invariant_duration;
    battery_soc -= _invariant_battery_drain;

    if (battery_soc <= state->threshold_soc)
    {
      std::cout << " -- Delivery: Unable to reach dropoff" << std::endl;
      return rmf_utils::nullopt;
    }
    
    state->battery_soc = battery_soc;

    // TODO: Check if we have enough charge to head back to nearest charger

    return state;
  }

  double invariant_duration() const final
  {
    return _invariant_duration;
  }

private:
  std::size_t _id; // task id
  std::size_t _pickup_waypoint;
  std::size_t _dropoff_waypoint;
  std::shared_ptr<MotionPowerSink> _motion_sink;
  std::shared_ptr<DevicePowerSink> _device_sink;
  std::shared_ptr<Planner> _planner;
  bool _drain_battery;
  double _invariant_duration;
  double _invariant_battery_drain;
};

// ============================================================================
class Candidates
{
public:

  struct Entry
  {
    std::size_t candidate;
    RobotState state;
  };

  // Map finish time to Entry
  using Map = std::multimap<double, Entry>;

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

  void update_candidate(std::size_t candidate, RobotState state)
  {
    const auto it = _candidate_map.at(candidate);
    _value_map.erase(it);
    _candidate_map[candidate] = _value_map.insert(
      {state.finish_time, Entry{candidate, state}});
  }


private:
  Map _value_map;
  std::vector<Map::iterator> _candidate_map;

  Candidates(Map candidate_values)
    : _value_map(std::move(candidate_values))
  {
    _update_map();
  }

  void _update_map()
  {
    for (auto it = _value_map.begin(); it != _value_map.end(); ++it)
    {
      const auto c = it->second.candidate;
      if (_candidate_map.size() <= c)
        _candidate_map.resize(c+1);

      _candidate_map[c] = it;
    }
  }
};

Candidates Candidates::make(
    const std::vector<RobotState>& initial_states,
    const TaskRequest& request)
{
  Map initial_map;
  for (std::size_t i = 0; i < initial_states.size(); ++i)
  {
    const auto& state = initial_states[i];
    const auto finish = request.estimate(state);
    if (finish.has_value())
    {
      initial_map.insert({finish->finish_time, Entry{i, finish.value()}});
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
  std::vector<std::vector<Assignment>>;
using UnassignedTasks =
  std::unordered_map<std::size_t, PendingTask>;
using InvariantSet = std::multiset<Invariant, InvariantLess>;
// ============================================================================
struct Node
{
  AssignedTasks assigned_tasks;
  UnassignedTasks unassigned_tasks;
  double cost_estimate;
  InvariantSet unassigned_invariants;

  void sort_invariants()
  {
    unassigned_invariants.clear();
    for (const auto& u : unassigned_tasks)
    {
      unassigned_invariants.insert(
            Invariant{u.first, u.second.request->invariant_duration()});
    }
  }

  void pop_unassigned(std::size_t task_id)
  {
    unassigned_tasks.erase(task_id);

    bool popped_invariant = false;
    for (auto it = unassigned_invariants.begin();
         it != unassigned_invariants.end(); ++it)
    {
      if (it->task_id == task_id)
      {
        popped_invariant = true;
        unassigned_invariants.erase(it);
      }
    }

    assert(popped_invariant);
  }
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
class InvariantHeuristicQueue
{
public:

  InvariantHeuristicQueue(std::vector<double> initial_values)
  {
    assert(!initial_values.empty());
    std::sort(initial_values.begin(), initial_values.end());

    for (const auto value : initial_values)
      _stacks.push_back({value});
  }

  void add(double new_value)
  {
    // Add the new value to the smallest stack
    const double value = _stacks[0].back() + new_value;
    _stacks[0].push_back(value);

    // Find the largest stack that is still smaller than the current front
    const auto next_it = _stacks.begin() + 1;
    auto end_it = next_it;
    for (; end_it != _stacks.end(); ++end_it)
    {
      if (value <= end_it->back())
        break;
    }

    if (next_it != end_it)
    {
      // Rotate the vector elements to move the front stack to its new place
      // in the order
      std::rotate(_stacks.begin(), next_it, end_it);
    }
  }

  double compute_cost() const
  {
    double total_cost = 0.0;
    for (const auto& stack : _stacks)
    {
      // NOTE: We start iterating from i=1 because i=0 represents a component of
      // the cost that is already accounted for by g(n) and the variant
      // component of h(n)
      for (std::size_t i=1; i < stack.size(); ++i)
        total_cost += stack[i];
    }

    return total_cost;
  }

private:
  std::vector<std::vector<double>> _stacks;
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
  while(a < node.assigned_tasks.size())
  {
    const auto& current_agent = node.assigned_tasks.at(a);

    if (t < current_agent.size())
    {
      const auto& task_id = current_agent[t].task_id;
      const auto agent_insertion = agent_table->agent.insert({a, nullptr});
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
    _debug(debug),
    _initial_states(initial_states)
  {

    // Initialize the starting node and add it to the priority queue
    auto starting_node = std::make_shared<Node>();
    starting_node->assigned_tasks.resize(initial_states.size());

    for (const auto& task : tasks)
      starting_node->unassigned_tasks.insert(
        {task->id(), PendingTask(initial_states, task)});

    starting_node->cost_estimate = compute_f(*starting_node);
    starting_node->sort_invariants();

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
        std::cout << "Battery SOC:" << std::endl;
        for (std::size_t i = 0; i < top->assigned_tasks.size(); ++i)
        {
          std::cout << "  Agent: " << i << std::endl;
          for (const auto& assignment : top->assigned_tasks[i])
          {
            std::cout << "    " << assignment.task_id << " : " 
                      << assignment.state.battery_soc << std::endl;
          }
        }
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
  std::vector<RobotState> _initial_states;

  double compute_g(const Node& node)
  {
    double cost = 0.0;
    for (const auto& agent : node.assigned_tasks)
    {
      for (const auto& assignment : agent)
      {
        cost += assignment.state.finish_time;
      }
    }

    return cost;
  }

  double compute_h(const Node& node)
  {
    std::vector<double> initial_queue_values;
    initial_queue_values.resize(
          node.assigned_tasks.size(), std::numeric_limits<double>::infinity());

    for (const auto& u : node.unassigned_tasks)
    {
      // We subtract the invariant duration here because otherwise its
      // contribution to the cost estimate will be duplicated in the next section,
      // which could result in an overestimate.
      const double variant_value =
          u.second.candidates.best_finish_time()
          - u.second.request->invariant_duration();

      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
      {
        const std::size_t candidate = it->second.candidate;
        if (variant_value < initial_queue_values[candidate])
          initial_queue_values[candidate] = variant_value;
      }
    }

    for (std::size_t i=0; i < initial_queue_values.size(); ++i)
    {
      auto& value = initial_queue_values[i];
      if (std::isinf(value))
      {
        // Clear out any infinity placeholders. Those candidates simply don't have
        // any unassigned tasks that want to use it.
        const auto& assignments = node.assigned_tasks[i];
        if (assignments.empty())
          value = 0.0;
        else
          value = assignments.back().state.finish_time;
      }
    }

    InvariantHeuristicQueue queue(std::move(initial_queue_values));
    // NOTE: It is crucial that we use the ordered set of unassigned_invariants
    // here. The InvariantHeuristicQueue expects the invariant costs to be passed
    // to it in order of smallest to largest. If that assumption is not met, then
    // the final cost that's calculated may be invalid.
    for (const auto& u : node.unassigned_invariants)
      queue.add(u.invariant_cost);

    return queue.compute_cost();
  }

  double compute_f(const Node& n)
  {
    return compute_g(n) + compute_h(n);
  }

  std::vector<ConstNodePtr> expand(ConstNodePtr parent, Filter& filter)
  {
    std::vector<ConstNodePtr> new_nodes;
    new_nodes.reserve(
      parent->unassigned_tasks.size() + parent->assigned_tasks.size());
    for (const auto& u : parent->unassigned_tasks)
    {
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it!= range.end; it++)
      {
        auto new_node = std::make_shared<Node>(*parent);
        const auto& entry = it->second;
        // Assign the unassigned task
        new_node->assigned_tasks[entry.candidate].push_back(
          Assignment{u.first, entry.state});
        
        // Erase the assigned task from unassigned tasks
        new_node->pop_unassigned(u.first);

        // Update states of unassigned tasks for the candidate
        bool discard = false;
        for (auto& new_u : new_node->unassigned_tasks)
        {
          const auto finish = new_u.second.request->estimate(entry.state);
          if (finish.has_value())
          {
            new_u.second.candidates.update_candidate(
              entry.candidate, finish.value());
          }
          else
          {
            discard = true;
            break;
          }
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

    // Assign charging task to each robot
    for (std::size_t i = 0; i < parent->assigned_tasks.size(); ++i)
    {
      auto new_node = std::make_shared<Node>(*parent);
      // Assign charging task to an agent
      auto charging_task = _charge_battery;
      const auto& assignments = new_node->assigned_tasks[i];
      RobotState state;
      if (assignments.size() > 0)
      {
        const auto& last_assignment = assignments.back();
        state = last_assignment.state;
      }
      else
      {
        // We use the initial state of the robot
        state = _initial_states[i];
      }

      bool discard = false;
      auto new_state = charging_task->estimate(state);
      if (new_state.has_value())
      {
        new_node->assigned_tasks[i].push_back(
          Assignment{charging_task->id(), new_state.value()});
      }
      else
      {
        continue;
      }
      
      // Update unassigned tasks
      for (auto& new_u : new_node->unassigned_tasks)
      {
        const auto finish = new_u.second.request->estimate(state);
        if (finish.has_value())
        {
          new_u.second.candidates.update_candidate(
            i, finish.value());
        }
        else
        {
          discard = true;
          break;
        }
        
      }
      if (discard)
        continue;

      // Update the cost estimate for new_node
      new_node->cost_estimate = compute_f(*new_node);

      new_nodes.push_back(new_node);
    }

    return new_nodes;
  }

  void print_node(const Node& node)
  {
    std::cout << " -- " << node.cost_estimate << ": <";
    for (std::size_t a=0; a < node.assigned_tasks.size(); ++a)
    {
      if (a > 0)
        std::cout << ", ";

      std::cout << a << ": [";
      for (const auto i : node.assigned_tasks[a])
        std::cout << " " << i.task_id;
      std::cout << " ]";
    }

    std::cout << " -- ";
    bool first = true;
    for (const auto& u : node.unassigned_tasks)
    {
      if (first)
        first = false;
      else
        std::cout << ", ";

      std::cout << u.first << ":";
      const auto& range = u.second.candidates.best_candidates();
      for (auto it = range.begin; it != range.end; ++it)
        std::cout << " " << it->second.candidate;
    }

    std::cout << ">" << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
  }

};

} // anonymous namespace

#endif // RMF_TASKS__TASK_ALLOCATION_HPP
