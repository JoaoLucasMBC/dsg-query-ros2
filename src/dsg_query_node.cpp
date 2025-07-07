#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/node_attributes.h"
#include "rclcpp/rclcpp.hpp"
#include "hydra_msgs/msg/dsg_update.hpp"
#include "spark_dsg/serialization/graph_binary_serialization.h"

class DsgQueryNode : public rclcpp::Node {
public:
  DsgQueryNode() : Node("dsg_query_node") {
    subscription_ = this->create_subscription<hydra_msgs::msg::DsgUpdate>(
      "/hydra/backend/dsg", 10,
      std::bind(&DsgQueryNode::dsg_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "DSG Query Node started");
  }

private:
  void dsg_callback(const hydra_msgs::msg::DsgUpdate::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received DSG update, sequence: %ld", msg->sequence_number);
    RCLCPP_INFO(this->get_logger(), "Number of active nodes: %zu", msg->layer_contents.size());
    RCLCPP_INFO(this->get_logger(), "Deleted nodes: %zu", msg->deleted_nodes.size());
    RCLCPP_INFO(this->get_logger(), "Deleted edges: %zu", msg->deleted_edges.size());

    // You can deserialize msg->layer_contents here if needed with Spark DSG
    auto graph = spark_dsg::io::binary::readGraph(msg->layer_contents.data(),
        msg->layer_contents.size());

    if (!graph) {
       RCLCPP_WARN(this->get_logger(), "Failed to deserialize DSG!");
       return;
    }

    auto layer = graph->findLayer("OBJECTS");

    RCLCPP_INFO(this->get_logger(), "%d", layer->numNodes());

    for (const auto& [id, node_ptr] : layer->nodes()) {
      const auto& node = *node_ptr;
      const auto attrs = node.attributes<spark_dsg::SemanticNodeAttributes>();
      RCLCPP_INFO(this->get_logger(), "Node ID: %ld, Name: %d", node.id,
          attrs.semantic_label);
      RCLCPP_INFO(this->get_logger(), "Pos: %f, %f, %f", attrs.position.x(),
          attrs.position.y(), attrs.position.z());

      auto parent = node.getParent();

      if (parent) {
        auto& parent_node = graph->getNode(*parent);
        auto parent_attrs = parent_node.attributes<spark_dsg::SemanticNodeAttributes>();
        RCLCPP_INFO(this->get_logger(), "Parent ID: %ld, Name: %d", parent_node.id,
            parent_attrs.semantic_label);

        auto layer_id = static_cast<spark_dsg::LayerId>(3);
        if (!graph->hasLayer(layer_id)) {
          continue;
        }

        auto& layer = graph->getLayer(layer_id);
        if (layer.hasNode(parent_node.id)) {
          RCLCPP_INFO(this->get_logger(), "Parent Node Layer: %ld",
              layer_id);

          // Check if the GRANDPARENT exists
          auto grandparent = parent_node.getParent();
          if (grandparent) {
            auto& grandparent_node = graph->getNode(*grandparent);
            auto grandparent_attrs = grandparent_node.attributes<spark_dsg::SemanticNodeAttributes>();
            RCLCPP_INFO(this->get_logger(), "Grandparent ID: %ld, Name: %d", grandparent_node.id, grandparent_attrs.semantic_label);

            // Iterate over the layers to see what is the layer of the grandparent node
            for (int i = 1; i <= 5; ++i) {
              auto layer_id = static_cast<spark_dsg::LayerId>(i);
              if (!graph->hasLayer(layer_id)) {
                continue;
              }

              auto& layer = graph->getLayer(layer_id);
              if (layer.hasNode(grandparent_node.id)) {
                RCLCPP_INFO(this->get_logger(), "Grandparent Node Layer: %ld",
                    layer_id);
                break;
              }
            }
          }
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "RAUL #############");
  }

  rclcpp::Subscription<hydra_msgs::msg::DsgUpdate>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DsgQueryNode>());
  rclcpp::shutdown();
  return 0;
}

