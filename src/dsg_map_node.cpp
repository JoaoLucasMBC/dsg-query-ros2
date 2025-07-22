#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hydra_msgs/msg/dsg_update.hpp"
#include "spark_dsg/serialization/graph_binary_serialization.h"
#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/mesh.h"

#include "std_srvs/srv/empty.hpp"
#include "dsg_query/srv/list_rooms.hpp"
#include "dsg_query/srv/list_room_objects.hpp"
#include "dsg_query/srv/move_to_object.hpp"
#include "dsg_query/msg/room.hpp"
#include "dsg_query/msg/room_object.hpp"

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/from_meshes.h>
#include <pcl/common/io.h>

#include "dsg_query/pcl_mesh_to_assimp.h"
#include <assimp/scene.h>
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/material.h>

#include <cstdio>
#include <filesystem>
#include <chrono>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

using Room = dsg_query::msg::Room;
using RoomObject = dsg_query::msg::RoomObject;
using ListRooms = dsg_query::srv::ListRooms;
using ListRoomObjects = dsg_query::srv::ListRoomObjects;
using MoveToObject = dsg_query::srv::MoveToObject;

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
    
    // Create a service for listing the rooms
    list_rooms_service_ = this->create_service<ListRooms>(
      "/hydra/backend/rooms",
      std::bind(&DsgMapServer::listRoomsCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create a service to list all objects in a given room
    list_objects_service_ = this->create_service<ListRoomObjects>(
      "/hydra/backend/objects",
      std::bind(&DsgMapServer::listObjectsCallback, this, std::placeholders::_1, std::placeholders::_2));

    nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // Create a service to move to an object, receives a MoveToObject request
    move_to_object_service_ = this->create_service<MoveToObject>(
      "/hydra/backend/moveToObject",
      std::bind(&DsgMapServer::moveToObjectCallback, this, std::placeholders::_1, std::placeholders::_2));
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

    for (auto &n : normals->points) {
      n.normal_x *= -1.0f;
      n.normal_y *= -1.0f;
      n.normal_z *= -1.0f;
    }

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


    // Save to PLY in a temporary file
    // std::string tmpl = "/tmp/dsg_mesh.ply";
    
    // pcl::io::savePLYFileBinary(tmpl, pcl_mesh);
    // RCLCPP_INFO(this->get_logger(), "Saved temp mesh to %s", tmpl.c_str());

    // PclMeshToAssimpConverter::convertPLYToAssimp(tmpl, "/tmp/mesh.dae");

    // Convert PCL mesh to Assimp scene
    aiScene* scene = PclMeshToAssimpConverter::createAssimpSceneFromPolygonMesh(pcl_mesh);
    
    if (!scene) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert PCL mesh to Assimp scene.");
      return;
    }

    // Export the Assimp scene to OBJ format
    std::string output_file = "/tmp/dsg_mesh.obj";
    if (PclMeshToAssimpConverter::exportAssimpScene(scene, "obj", output_file)) {
      RCLCPP_INFO(this->get_logger(), "Successfully exported DSG mesh to %s", output_file.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to export DSG mesh to OBJ format.");
    }

    //std::remove(tmpl.c_str());
  }

  void listRoomsCallback(
      const std::shared_ptr<ListRooms::Request>,
      std::shared_ptr<ListRooms::Response> response) {
    // List all rooms in the DSG
    auto rooms = graph_->findLayer("ROOMS");

    for (const auto& [id, node_ptr] : rooms->nodes()) {
      const auto& node = *node_ptr;
      Room room_msg;
      room_msg.room_id = node.id;
      response->rooms.push_back(room_msg);
    }
  }

  void listObjectsCallback(
      const std::shared_ptr<ListRoomObjects::Request> request,
      std::shared_ptr<ListRoomObjects::Response> response) {
    
    // List all objects in a given room
    auto objects = graph_->findLayer("OBJECTS");
    for (const auto& [id, node_ptr] : objects->nodes()) {
      const auto& node = *node_ptr;

      auto place = node.getParent();
      auto& place_node = graph_->getNode(*place);

      auto room = place_node.getParent();
      auto& room_node = graph_->getNode(*room);

      if (room_node.id == request->room.room_id) {
        const auto attrs = node.attributes<spark_dsg::SemanticNodeAttributes>();

        RoomObject object_msg;
        object_msg.object_id = node.id;
        object_msg.semantic_label = attrs.semantic_label;
        object_msg.room_id = room_node.id;
        object_msg.x = attrs.position.x();
        object_msg.y = attrs.position.y();
        object_msg.z = attrs.position.z();
        response->objects.push_back(object_msg);
      }
    }
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    // Make sure the Nav2 action server is up
    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
      return;
    }

    // Fill in the goal message
    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    // Optional: set up callbacks for feedback/result
    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // send_opts.goal_response_callback =
    //   [](GoalHandle::SharedPtr goal_handle) {
    //     if (!goal_handle) {
    //       RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
    //                   "NavigateToPose goal was rejected by server");
    //     } else {
    //       RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
    //                   "NavigateToPose goal accepted, waiting for result");
    //     }
    //   };
    send_opts.feedback_callback =
      [](GoalHandleNav::SharedPtr,
         const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Distance remaining: %.2f",
                    feedback->distance_remaining);
      };
    send_opts.result_callback =
      [](const GoalHandleNav::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Navigation succeeded!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Navigation was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                        "Navigation was canceled");
            break;
          default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Unknown result code");
            break;
        }
      };

    // Finally send the goal
    nav_to_pose_client_->async_send_goal(goal_msg, send_opts);
  }

  void moveToObjectCallback(
      const std::shared_ptr<MoveToObject::Request> request,
      std::shared_ptr<MoveToObject::Response>) {
  
    if (!graph_->hasNode(request->object.object_id)) {
      RCLCPP_ERROR(this->get_logger(), "Object with ID %lu not found", request->object.object_id);
      return;
    }
      
    // Find the object in the DSG
    auto objects = graph_->findLayer("OBJECTS");
    auto& object_node = objects->getNode(request->object.object_id);

    const auto attrs = object_node.attributes<spark_dsg::SemanticNodeAttributes>();
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = attrs.position.x();
    goal_pose.pose.position.y = attrs.position.y();
    goal_pose.pose.position.z = attrs.position.z();
    goal_pose.pose.orientation.w = 1.0;

    sendGoal(goal_pose);
  }

  std::shared_ptr<spark_dsg::DynamicSceneGraph> graph_;
  std::shared_ptr<pcl::PolygonMesh> mesh_;
  rclcpp::Subscription<hydra_msgs::msg::DsgUpdate>::SharedPtr dsg_update_subscriber_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_dsg_service_;
  rclcpp::Service<ListRooms>::SharedPtr list_rooms_service_;
  rclcpp::Service<ListRoomObjects>::SharedPtr list_objects_service_;
  rclcpp::Service<MoveToObject>::SharedPtr move_to_object_service_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DsgMapServer>());
  rclcpp::shutdown();
  return 0;
}
