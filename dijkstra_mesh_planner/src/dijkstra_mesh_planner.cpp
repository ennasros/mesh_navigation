/*
 *  Copyright 2020, Sebastian Pütz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *
 */

#include <dijkstra_mesh_planner/dijkstra_mesh_planner.h>
#include <lvr2/util/Meap.hpp>
#include <mbf_msgs/GetPathResult.h>
#include <mesh_map/util.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dijkstra_mesh_planner::DijkstraMeshPlanner, mbf_mesh_core::MeshPlanner);

namespace dijkstra_mesh_planner
{
DijkstraMeshPlanner::DijkstraMeshPlanner()
{
}

DijkstraMeshPlanner::~DijkstraMeshPlanner()
{
}

uint32_t DijkstraMeshPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                       double tolerance, std::vector<geometry_msgs::PoseStamped>& plan, double& cost,
                                       std::string& message)
{
  const auto& mesh = mesh_map->mesh();
  std::list<lvr2::VertexHandle> path;
  ROS_INFO("start dijkstra mesh planner.");

  mesh_map::Vector goal_vec = mesh_map::toVector(goal.pose.position);
  mesh_map::Vector start_vec = mesh_map::toVector(start.pose.position);

  // call dijkstra with the goal pose as seed / start vertex
  uint32_t outcome = dijkstra(goal_vec, start_vec, path);

  path.reverse();

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = mesh_map->mapFrame();

  cost = 0;
  if (!path.empty())
  {
    mesh_map::Vector& vec = start_vec;
    const auto& vertex_normals = mesh_map->vertexNormals();
    mesh_map::Normal normal = vertex_normals[path.front()];

    float dir_length;
    geometry_msgs::PoseStamped pose;
    pose.header = header;

    while (!path.empty())
    {
      // get next position
      const lvr2::VertexHandle& vH = path.front();
      mesh_map::Vector next = mesh.getVertexPosition(vH);

      pose.pose = mesh_map::calculatePoseFromPosition(vec, next, normal, dir_length);
      cost += dir_length;
      vec = next;
      normal = vertex_normals[vH];
      plan.push_back(pose);
      path.pop_front();
    }
    pose.pose = mesh_map::calculatePoseFromPosition(vec, goal_vec, normal, dir_length);
    cost += dir_length;
    plan.push_back(pose);
  }

  ROS_INFO_STREAM("Path length: " << cost << "m");
  nav_msgs::Path path_msg;
  path_msg.poses = plan;
  path_msg.header = header;

  path_pub.publish(path_msg);
  mesh_map->publishVertexCosts(potential, "Potential");

  ROS_INFO_STREAM("Path length: " << cost << "m");

  if (publish_vector_field)
  {
    mesh_map->publishVectorField("vector_field", vector_map, publish_face_vectors);
  }

  return outcome;
}

bool DijkstraMeshPlanner::cancel()
{
  cancel_planning = true;
  return true;
}

bool DijkstraMeshPlanner::initialize(const std::string& plugin_name,
                                     const boost::shared_ptr<mesh_map::MeshMap>& mesh_map_ptr)
{
  mesh_map = mesh_map_ptr;
  name = plugin_name;
  map_frame = mesh_map->mapFrame();
  private_nh = ros::NodeHandle("~/" + name);

  private_nh.param("publish_vector_field", publish_vector_field, false);
  private_nh.param("publish_face_vectors", publish_face_vectors, false);
  private_nh.param("goal_dist_offset", goal_dist_offset, 0.3f);

  path_pub = private_nh.advertise<nav_msgs::Path>("path", 1, true);
  const auto& mesh = mesh_map->mesh();

  reconfigure_server_ptr =
      boost::shared_ptr<dynamic_reconfigure::Server<dijkstra_mesh_planner::DijkstraMeshPlannerConfig>>(
          new dynamic_reconfigure::Server<dijkstra_mesh_planner::DijkstraMeshPlannerConfig>(private_nh));

  config_callback = boost::bind(&DijkstraMeshPlanner::reconfigureCallback, this, _1, _2);
  reconfigure_server_ptr->setCallback(config_callback);

  return true;
}

lvr2::DenseVertexMap<mesh_map::Vector> DijkstraMeshPlanner::getVectorMap()
{
  return vector_map;
}

void DijkstraMeshPlanner::reconfigureCallback(dijkstra_mesh_planner::DijkstraMeshPlannerConfig& cfg, uint32_t level)
{
  ROS_INFO_STREAM("New height diff layer config through dynamic reconfigure.");
  if (first_config)
  {
    config = cfg;
    first_config = false;
    return;
  }
  config = cfg;
}

void DijkstraMeshPlanner::computeVectorMap()
{
  const auto& mesh = mesh_map->mesh();

  for (auto v3 : mesh.vertices())
  {
    const lvr2::VertexHandle& v1 = predecessors[v3];
    // if predecessor is pointing to it self, continue with the next vertex.
    if (v1 == v3)
      continue;

    const auto& vec3 = mesh.getVertexPosition(v3);
    const auto& vec1 = mesh.getVertexPosition(v1);

    // compute the direction vector and store it in the direction vertex map
    const auto dirVec = vec1 - vec3;
    // store the normalized rotated vector in the vector map
    vector_map.insert(v3, dirVec.normalized());
  }
  mesh_map->setVectorMap(vector_map);
}

uint32_t DijkstraMeshPlanner::dijkstra(const mesh_map::Vector& start, const mesh_map::Vector& goal,
                                       std::list<lvr2::VertexHandle>& path)
{
  return dijkstra(start, goal, mesh_map->edgeDistances(), mesh_map->vertexCosts(), path, potential, predecessors);
}

uint32_t DijkstraMeshPlanner::dijkstra(const mesh_map::Vector& original_start, const mesh_map::Vector& original_goal,
                                       const lvr2::DenseEdgeMap<float>& edge_weights,
                                       const lvr2::DenseVertexMap<float>& costs, std::list<lvr2::VertexHandle>& path,
                                       lvr2::DenseVertexMap<float>& distances,
                                       lvr2::DenseVertexMap<lvr2::VertexHandle>& predecessors)
{
  ROS_INFO_STREAM("Init wave front propagation.");
  ros::WallTime t_initialization_start = ros::WallTime::now();

  const auto& mesh = mesh_map->mesh();
  const auto& vertex_costs = mesh_map->vertexCosts();

  auto& invalid = mesh_map->invalid;

  mesh_map->publishDebugPoint(original_start, mesh_map::color(0, 1, 0), "start_point");
  mesh_map->publishDebugPoint(original_goal, mesh_map::color(0, 0, 1), "goal_point");
  
  // Find the closest vertex handle of start and goal
  const auto& start_opt = mesh_map->getNearestVertexHandle(original_start);
  const auto& goal_opt = mesh_map->getNearestVertexHandle(original_goal);
  // reset cancel planning
  cancel_planning = false;

  if (!start_opt)
  {
    std::cout << "INVALID_START" << std::endl;
    return mbf_msgs::GetPathResult::INVALID_START;
  }
  if (!goal_opt)
  {
    std::cout << "INVALID_GOAL" << std::endl;
    return mbf_msgs::GetPathResult::INVALID_GOAL;
  }

  const auto& start_vertex = start_opt.unwrap();
  const auto& goal_vertex = goal_opt.unwrap();

  path.clear();
  distances.clear();
  predecessors.clear();

  if (goal_vertex == start_vertex)
  {
    return mbf_msgs::GetPathResult::SUCCESS;
  }

  lvr2::DenseVertexMap<bool> fixed(mesh.nextVertexIndex(), false);
  lvr2::DenseVertexMap<int> que_order(mesh.nextVertexIndex(), -1);

  // clear vector field map
  vector_map.clear();

  ros::WallTime t_start, t_end;
  t_start = ros::WallTime::now();

  // initialize distances with infinity
  // initialize predecessor of each vertex with itself
  for (auto const& vH : mesh.vertices())
  {
    distances.insert(vH, std::numeric_limits<float>::infinity());
    predecessors.insert(vH, vH);
  }

  lvr2::Meap<lvr2::VertexHandle, float> pq;

  // Set start distance to zero
  // add start vertex to priority queue
  distances[start_vertex] = 0;
  pq.insert(start_vertex, 0);


  float goal_dist = std::numeric_limits<float>::infinity();

  ROS_INFO_STREAM("Start Dijkstra");
  ros::WallTime t_propagation_start = ros::WallTime::now();
  double initialization_duration = (t_propagation_start - t_initialization_start).toNSec() * 1e-6;

  size_t fixed_set_cnt = 0;


  size_t que_set_cnt = 0;
  que_order.insert(start_vertex, que_set_cnt);


  while (!pq.isEmpty() && !cancel_planning)
  {
    lvr2::VertexHandle current_vh = pq.popMin().key();
    
    fixed[current_vh] = true;
    fixed_set_cnt++;
    std::cout << "fixed_set_cnt = " << fixed_set_cnt << std::endl;


    int que_id = que_order[current_vh];
    mesh_map->publishDebugPoint(mesh.getVertexPosition(current_vh), mesh_map::color(0, 0, 1), "que_order_"+std::to_string(que_id));
    sleep(1);

    if (current_vh == goal_vertex)
    {
      ROS_INFO_STREAM("The Dijkstra Mesh Planner reached the goal.");
      goal_dist = distances[current_vh] + goal_dist_offset;
    }

    std::cout << "distances[current_vh] = " << distances[current_vh] << ", goal_dist = " << goal_dist << std::endl;
    if (distances[current_vh] > goal_dist)
    {
      mesh_map->publishDebugPoint(mesh.getVertexPosition(current_vh), mesh_map::color(1, 0, 0), "que_order_"+std::to_string(que_id));
      continue;
    }

    std::cout << "vertex_costs[current_vh] = " << vertex_costs[current_vh] << ", config.cost_limit = " << config.cost_limit << std::endl;
    if (vertex_costs[current_vh] > config.cost_limit)
    {
      mesh_map->publishDebugPoint(mesh.getVertexPosition(current_vh), mesh_map::color(1, 0, 0), "que_order_"+std::to_string(que_id));
      continue;
    }
    std::vector<lvr2::EdgeHandle> edges;
    try
    {
      mesh.getEdgesOfVertex(current_vh, edges);
      std::cout << "=================================================" << std::endl;
      std::cout << "found " << edges.size() << " edges for the vertex" << std::endl;
    }
    catch (lvr2::PanicException exception)
    {
      std::cout << "lvr2 panic exception" << std::endl;
      invalid.insert(current_vh, true);
      continue;
    }
    catch (lvr2::VertexLoopException exception)
    {
      std::cout << "lvr2 vertex loop exception" << std::endl;
      invalid.insert(current_vh, true);
      continue;
    }
    size_t edge_cnt = 1;
    for (auto eH : edges)
    {
      std::cout << "reading edge" << edge_cnt << " of " << edges.size() << std::endl;
      try
      {
        std::array<lvr2::VertexHandle, 2> vertices = mesh.getVerticesOfEdge(eH);
        edge_cnt++;
        auto vH = vertices[0] == current_vh ? vertices[1] : vertices[0];

        if (que_order.containsKey(vH))
        {
          que_id = que_order[vH];
        }
        else
        {
          que_set_cnt++;
          que_id = que_set_cnt;
          que_order.insert(vH, que_id);
        }
        

        if (fixed[vH])
        {
          std::cout << "other vertix was already checked" << std::endl;
          // mesh_map->publishDebugPoint(mesh.getVertexPosition(vH), mesh_map::color(0, 0, 1), "que_order_"+std::to_string(que_id));
          continue;
        }
        if (invalid[vH])
        {
          std::cout << "other vertix was determined invalid" << std::endl;
          // mesh_map->publishDebugPoint(mesh.getVertexPosition(vH), mesh_map::color(1, 0, 0), "que_order_"+std::to_string(que_id));
          continue;
        }

        mesh_map->publishDebugPoint(mesh.getVertexPosition(vH), mesh_map::color(0, 1, 0), "que_order_"+std::to_string(que_id));
        sleep(1);

        float tmp_cost = distances[current_vh] + edge_weights[eH];
        pq.insert(vH, tmp_cost);
        std::cout << "other vertix added to pq with tmp_cost = " << tmp_cost << std::endl;
        
        if (tmp_cost < distances[vH])
        {
          std::cout << "tmp_cost = " << tmp_cost << ", distances[vH] = " << distances[vH] << std::endl;
          std::cout << "new low cost to get to this vertix!" << std::endl;
          distances[vH] = tmp_cost;
          predecessors[vH] = current_vh;
        }
      }
      catch (lvr2::PanicException exception)
      {
        std::cout << "lvr2::PanicException" << std::endl;
        continue;
      }
      catch (lvr2::VertexLoopException exception)
      {
        std::cout << "lvr2::VertexLoopException" << std::endl;
        continue;
      }

      std::cout << "pq.numValues() = " << pq.numValues() << std::endl;
    }
  }

  if (cancel_planning)
  {
    ROS_WARN_STREAM("Wave front propagation has been canceled!");
    return mbf_msgs::GetPathResult::CANCELED;
  }

  ROS_INFO_STREAM("The Dijkstra Mesh Planner finished the propagation.");

  if (goal_vertex == predecessors[goal_vertex])
  {
    ROS_WARN("Predecessor of the goal is not set! No path found!");
    return mbf_msgs::GetPathResult::NO_PATH_FOUND;
  }

  ros::WallTime t_propagation_end = ros::WallTime::now();
  double propagation_duration = (t_propagation_end - t_propagation_start).toNSec() * 1e-6;

  auto vH = goal_vertex;

  while (vH != start_vertex && !cancel_planning)
  {
    vH = predecessors[vH];
    path.push_front(vH);
  };

  t_end = ros::WallTime::now();
  double execution_time = (t_end - t_start).toNSec() * 1e-6;
  ROS_INFO_STREAM("Execution time (ms): " << execution_time << " for " << mesh.numVertices()
                                          << " num vertices in the mesh.");

  computeVectorMap();

  if (cancel_planning)
  {
    std::cout << "Dijkstra has been canceled!" << std::endl;
    ROS_WARN_STREAM("Dijkstra has been canceled!");
    return mbf_msgs::GetPathResult::CANCELED;
  }

  ros::WallTime t_path_backtracking = ros::WallTime::now();
  double path_backtracking_duration = (t_path_backtracking - t_propagation_end).toNSec() * 1e-6;

  ROS_INFO_STREAM("Processed " << fixed_set_cnt << " vertices in the fixed set.");
  ROS_INFO_STREAM("Initialization duration (ms): " << initialization_duration);
  ROS_INFO_STREAM("Execution time wavefront propagation (ms): "<< propagation_duration);
  ROS_INFO_STREAM("Path backtracking duration (ms): " << path_backtracking_duration);

  ROS_INFO_STREAM("Successfully finished Dijkstra back tracking!");
  return mbf_msgs::GetPathResult::SUCCESS;
}

} /* namespace dijkstra_mesh_planner */

