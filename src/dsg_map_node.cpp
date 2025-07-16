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
#include <pcl/io/ply_io.h>
#include <pcl/features/from_meshes.h>


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
    // Extract the mesh from the graph
    const auto& mesh = *graph_->mesh();
    
    // Create a point cloud from the mesh's vertices and colors
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    for (size_t i = 0; i < mesh.numVertices(); ++i) {
      pcl::PointXYZRGB pt;
      const auto& pos = mesh.pos(i);
      pt.x = pos.x();
      pt.y = pos.y();
      pt.z = pos.z();

      if (mesh.has_colors) {
        const auto& c = mesh.color(i);
        pt.r = c.r;
        pt.g = c.g;
        pt.b = c.b;
      } else {
        pt.r = pt.g = pt.b = 255;
      }

      cloud.points.push_back(pt);
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;

    // Prepare faces (polygons)
    std::vector<pcl::Vertices> pcl_polygons;
    pcl_polygons.reserve(mesh.faces.size()); // Pre-allocate memory
    for (const auto& face : mesh.faces) {
      if (face.size() == 3) { // Ensure it's a triangle
        pcl::Vertices v;
        v.vertices = {static_cast<uint32_t>(face[0]),
                      static_cast<uint32_t>(face[1]),
                      static_cast<uint32_t>(face[2])};
        pcl_polygons.push_back(v);
      } else {
          RCLCPP_WARN(this->get_logger(), "Skipping non-triangular face with %zu vertices.", face.size());
      }
    }

    // Compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::features::computeApproximateNormals(cloud, pcl_polygons, *normals);

    // Combine point cloud with normals
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(cloud, *normals, *cloud_with_normals);

    // Convert cloud to PCLPointCloud2
    pcl::PCLPointCloud2 cloud_blob;
    pcl::toPCLPointCloud2(*cloud_with_normals, cloud_blob);

    // Create a PolygonMesh to hold the cloud and polygons
    pcl::PolygonMesh pcl_mesh;
    pcl_mesh.cloud = cloud_blob;
    pcl_mesh.polygons = pcl_polygons;

    // Check for NaN normals (uncomment if needed)
    // int nan_count = 0;
    // for (size_t i = 0; i < normals->points.size(); ++i) {
    //   const auto& n = normals->points[i];
    //   if (!pcl::isFinite(n)) {
    //     ++nan_count;
    //     // Optionally replace with a default direction
    //     normals->points[i].normal_x = 0.0f;
    //     normals->points[i].normal_y = 0.0f;
    //     normals->points[i].normal_z = 1.0f;
    //   }
    // }
    // RCLCPP_WARN(this->get_logger(), "Found %d NaN normals out of %lu points", nan_count, normals->points.size());

    // Save to PLY
    std::string filename = "/tmp/dsg_mesh_normals.ply";
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
