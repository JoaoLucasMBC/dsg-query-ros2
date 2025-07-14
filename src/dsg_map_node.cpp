#include "rclcpp/rclcpp.hpp"
#include "hydra_msgs/msg/dsg_update.hpp"
#include "spark_dsg/serialization/graph_binary_serialization.h"
#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/mesh.h"

#include "std_srvs/srv/empty.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>


class DsgMapServer : public rclcpp::Node {
public:
  DsgMapServer()
  : Node("dsg_map_node") {
    // Initialize the graph
    graph_ = std::make_shared<spark_dsg::DynamicSceneGraph>(true);

    // Create a listener for DSG updates
    dsg_update_subscriber_ = this->create_subscription<hydra_msgs::msg::DsgUpdate>(
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
    graph_ = graph;

    RCLCPP_INFO(this->get_logger(), "DSG updated with %lu nodes and %lu edges.",
                graph_->numNodes(), graph_->numEdges());
  }

  void saveDsgCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>) {
    const auto& mesh = *graph_->mesh();

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    for (size_t i = 0; i < mesh.numVertices(); ++i) {
      pcl::PointXYZRGBA pt;
      const auto& pos = mesh.pos(i);
      pt.x = pos.x();
      pt.y = pos.y();
      pt.z = pos.z();

      if (mesh.has_colors) {
        const auto& c = mesh.color(i);
        pt.r = c.r;
        pt.g = c.g;
        pt.b = c.b;
        pt.a = 255;
      } else {
        pt.r = pt.g = pt.b = 255;
        pt.a = 255;
      }

      cloud.points.push_back(pt);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;

    // Convert cloud to PCLPointCloud2
    pcl::PCLPointCloud2 cloud_blob;
    pcl::toPCLPointCloud2(cloud, cloud_blob);

    // Convert faces
    pcl::PolygonMesh pcl_mesh;
    pcl_mesh.cloud = cloud_blob;
    for (const auto& face : mesh.faces) {
      pcl::Vertices v;
      v.vertices = {static_cast<uint32_t>(face[0]),
                    static_cast<uint32_t>(face[1]),
                    static_cast<uint32_t>(face[2])};
      pcl_mesh.polygons.push_back(v);
    }

    // Save to PLY or OBJ for Gazebo
    std::string filename = "/tmp/dsg_mesh.ply";
    pcl::io::savePLYFileBinary(filename, pcl_mesh);

    RCLCPP_INFO(this->get_logger(), "Saved mesh to %s", filename.c_str());
  }

  std::shared_ptr<spark_dsg::DynamicSceneGraph> graph_;
  std::shared_ptr<pcl::PolygonMesh> mesh_;
  rclcpp::Subscription<hydra_msgs::msg::DsgUpdate>::SharedPtr dsg_update_subscriber_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_dsg_service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DsgMapServer>());
  rclcpp::shutdown();
  return 0;
}
