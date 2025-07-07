#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/serialization/graph_binary_serialization.h"

#include "rclcpp/rclcpp.hpp"
#include "hydra_msgs/msg/dsg_update.hpp"

#include <queue>
#include <unordered_map>
#include <limits>
#include <random>
#include <algorithm>

using NodeId = spark_dsg::NodeId;

std::vector<NodeId> runAStar(const spark_dsg::SceneGraphLayer& layer, NodeId start, NodeId goal) {
  std::unordered_map<NodeId, double> g_score;
  std::unordered_map<NodeId, NodeId> came_from;
  std::priority_queue<std::pair<double, NodeId>, std::vector<std::pair<double, NodeId>>, std::greater<>> open_set;

  for (const auto& [id, _] : layer.nodes()) {
    g_score[id] = std::numeric_limits<double>::infinity();
  }

  g_score[start] = 0.0;
  open_set.emplace(0.0, start);

  while (!open_set.empty()) {
    NodeId current = open_set.top().second;
    open_set.pop();

    if (current == goal) {
      std::vector<NodeId> path;
      while (came_from.find(current) != came_from.end()) {
        path.push_back(current);
        current = came_from[current];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    const auto& current_node = layer.getNode(current);
    for (const auto& neighbor_id : current_node.siblings()) {
      const auto& neighbor = layer.getNode(neighbor_id);
      auto neighbor_attrs = neighbor.attributes<spark_dsg::SemanticNodeAttributes>();
      if (!neighbor_attrs.is_active) {
        continue;  // Skip inactive nodes
      }

      // Get current node attributes
      auto current_attrs = current_node.attributes<spark_dsg::SemanticNodeAttributes>();

      double tentative_g = g_score[current] + (current_attrs.position - neighbor_attrs.position).norm();

      if (tentative_g < g_score[neighbor_id]) {
        came_from[neighbor_id] = current;
        g_score[neighbor_id] = tentative_g;
        open_set.emplace(tentative_g, neighbor_id);
      }
    }
  }

  return {};  // No path found
}


class DsgPathPlannerNode : public rclcpp::Node {
public:
  DsgPathPlannerNode() : Node("dsg_path_planner_node") {
    subscription_ = this->create_subscription<hydra_msgs::msg::DsgUpdate>(
      "/hydra/backend/dsg", 10,
      std::bind(&DsgPathPlannerNode::dsg_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "DSG Path Planner Node started");
  }

private:
  void dsg_callback(const hydra_msgs::msg::DsgUpdate::SharedPtr msg) {
    auto graph = spark_dsg::io::binary::readGraph(msg->layer_contents.data(),
                                                  msg->layer_contents.size());

    if (!graph) {
      RCLCPP_WARN(this->get_logger(), "Failed to deserialize DSG!");
      return;
    }

    auto& object_layer = graph->getLayer(spark_dsg::DsgLayers::OBJECTS);
    if (object_layer.numNodes() < 2) {
      RCLCPP_WARN(this->get_logger(), "Not enough OBJECT nodes to plan");
      return;
    }

    // Collect OBJECT node ids
    std::vector<NodeId> object_ids;
    for (const auto& [id, _] : object_layer.nodes()) {
      object_ids.push_back(id);
    }

    std::shuffle(object_ids.begin(), object_ids.end(), std::mt19937{std::random_device{}()});
    NodeId obj1 = object_ids[0];
    NodeId obj2 = object_ids[1];

    auto parent1_opt = graph->getNode(obj1).getParent();
    auto parent2_opt = graph->getNode(obj2).getParent();

    if (!parent1_opt || !parent2_opt) {
      RCLCPP_WARN(this->get_logger(), "One of the OBJECTs has no parent PLACE");
      return;
    }

    NodeId place1 = *parent1_opt;
    NodeId place2 = *parent2_opt;

    auto& place_layer = graph->getLayer(spark_dsg::DsgLayers::PLACES);
    if (!place_layer.hasNode(place1) || !place_layer.hasNode(place2)) {
      RCLCPP_WARN(this->get_logger(), "PLACE node not found");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Running A* from PLACE %lu to %lu", place1, place2);
    auto path = runAStar(place_layer, place1, place2);

    if (path.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path found between PLACE nodes");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Found path of %lu nodes:", path.size());
    for (NodeId id : path) {
      RCLCPP_INFO(this->get_logger(), " -> %lu", id);
    }

    // TODO: publish path as PoseStamped[] or tf
  }

  rclcpp::Subscription<hydra_msgs::msg::DsgUpdate>::SharedPtr subscription_;
};


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DsgPathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}