#include "rclcpp/rclcpp.hpp"
#include "hydra_msgs/msg/dsg_update.hpp"
#include "spark_dsg/serialization/graph_binary_serialization.h"
#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/node_attributes.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DsgMapServer : public rclcpp::Node {
public:
  DsgMapServer()
  : Node("dsg_map_server") {
    // Initialize the DSG
    graph_ = spark_dsg::DynamicSceneGraph();

    // Create a listener for DSG updates
    dsg_update_subscriber = this->create_subscription<hydra_msgs::msg::DsgUpdate>(
      "/hydra/backend/dsg", 10,
      std::bind(&DsgMapServer::dsgUpdateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DSG Map Server is ready to receive updates.");

    // Create a service for saving the DSG
    save_dsg_service_ = this->create_service<std_srvs::srv::Empty>(
      "/hydra/backend/save_dsg",
      std::bind(&DsgMapServer::saveDsgCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
      
private:
  void dsgUpdateCallback(const hydra_msgs::msg::DsgUpdate::SharedPtr msg) {
    // Deserialize the DSG from the message
    auto graph = spark_dsg::io::binary::readGraph(msg->layer_contents.data(),
                                                  msg->layer_contents.size());
    if (!graph) {
      RCLCPP_WARN(this->get_logger(), "Failed to deserialize DSG!");
      return;
    }

    // Update the internal graph
    graph_ = *graph;

    RCLCPP_INFO(this->get_logger(), "DSG updated with %lu nodes and %lu edges.",
                graph_.numNodes(), graph_.numEdges());
  }

  void saveDsgCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                       std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    
    // Get the mesh data from the graph
    auto mesh = graph_.mesh();

    if (!mesh) {
      RCLCPP_WARN(this->get_logger(), "No mesh data available to save.");
      return;
    }

    // Convert the mesh to a point cloud
    auto cloud = meshToPointCloud(*mesh);

    if (!cloud) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert mesh to point cloud.");
      return;
    }

    // Save the point cloud to a file
    std::string filename = "/home/joaolucasmbc/Desktop/illinois/parasol/hydra_ws/dsg_mesh.pcd";
    if (pcl::io::savePCDFileBinary(filename, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save point cloud to file: %s", filename.c_str());
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "DSG mesh saved to %s", filename.c_str());
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr meshToPointCloud(const spark_dsg::Mesh& mesh) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    for (size_t i = 0; i < mesh.numVertices(); ++i) {
      pcl::PointXYZRGBA pt;
      const auto& pos = mesh.pos(i);
      pt.x = pos.x();
      pt.y = pos.y();
      pt.z = pos.z();

      if (mesh.has_colors) {
        const auto& color = mesh.color(i);
        pt.r = color.r;
        pt.g = color.g;
        pt.b = color.b;
        pt.a = 255;
      } else {
        pt.r = pt.g = pt.b = 255;
        pt.a = 255;
      }

      cloud->points.push_back(pt);
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    return cloud;
  }



  spark_dsg::DynamicSceneGraph graph_;
  rclcpp::Subscription<hydra_msgs::msg::DsgUpdate>::SharedPtr dsg_update_subscriber_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_dsg_service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DsgMapServer>());
  rclcpp::shutdown();
  return 0;
}
