#include "dsg_query/pcl_mesh_to_assimp.h" // Corrected include path

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> // For pcl::fromPCLPointCloud2

// Assimp headers
#include <assimp/scene.h> // This is crucial for aiScene, aiMesh, aiNode, aiColor3D, aiColor4D etc.
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/material.h> // For material properties like AI_MATKEY_COLOR_DIFFUSE
// #include <assimp/postprocess.h> // Only if you use post-processing flags explicitly

#include <iostream>
#include <vector>
#include <map>
#include <tuple> // For std::tuple as map key


// --- Private Helper Function Implementation ---
// 'toAssimpColor' is a static member function, so it must be called with PclMeshToAssimpConverter::
aiColor3D PclMeshToAssimpConverter::toAssimpColor(unsigned char r, unsigned char g, unsigned char b) {
    return aiColor3D(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f, static_cast<float>(b) / 255.0f);
}

// --- Public Method Implementations ---

aiScene* PclMeshToAssimpConverter::createAssimpSceneFromPolygonMesh(const pcl::PolygonMesh& pcl_mesh) {
    aiScene* scene = new aiScene();
    scene->mRootNode = new aiNode(); // Root node

    // Extract point cloud from PCLPointCloud2
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_mesh.cloud, *cloud);

    if (cloud->points.empty() || pcl_mesh.polygons.empty()) {
        std::cerr << "Warning: Empty point cloud or polygons in mesh. Returning empty scene." << std::endl;
        // Proper cleanup for a null return
        if (scene->mRootNode) { // Check if mRootNode was actually allocated
            delete scene->mRootNode;
        }
        delete scene;
        return nullptr;
    }

    // Create a single Assimp mesh (assuming one mesh for the whole PolygonMesh)
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1];
    aiMesh* mesh = new aiMesh();
    scene->mMeshes[0] = mesh;
    scene->mRootNode->mMeshes = new unsigned int[1];
    scene->mRootNode->mMeshes[0] = 0; // Link mesh to root node
    scene->mRootNode->mNumMeshes = 1;

    // Allocate memory for vertices
    mesh->mNumVertices = cloud->points.size();
    mesh->mVertices = new aiVector3D[mesh->mNumVertices];
    mesh->mColors[0] = new aiColor4D[mesh->mNumVertices];

    // Copy vertex positions and colors
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& p = cloud->points[i];
        mesh->mVertices[i] = aiVector3D(p.x, p.y, p.z);
        // Note: aiColor4D includes alpha, set to 1.0 (opaque)
        mesh->mColors[0][i] = aiColor4D(static_cast<float>(p.r) / 255.0f,
                                        static_cast<float>(p.g) / 255.0f,
                                        static_cast<float>(p.b) / 255.0f,
                                        1.0f);
    }

    // Allocate memory for faces
    mesh->mPrimitiveTypes = 0; // Will set dynamically based on face size
    mesh->mNumFaces = pcl_mesh.polygons.size();
    mesh->mFaces = new aiFace[mesh->mNumFaces];

    // Map to store unique colors and their material indices
    std::map<std::tuple<unsigned char, unsigned char, unsigned char>, unsigned int> unique_colors_to_material_idx;
    std::vector<aiMaterial*> materials; // Use vector for easy adding

    for (size_t i = 0; i < pcl_mesh.polygons.size(); ++i) {
        const auto& pcl_polygon = pcl_mesh.polygons[i];
        aiFace& face = mesh->mFaces[i];

        face.mNumIndices = pcl_polygon.vertices.size();
        face.mIndices = new unsigned int[face.mNumIndices];

        // Copy face indices
        for (size_t j = 0; j < pcl_polygon.vertices.size(); ++j) {
            face.mIndices[j] = pcl_polygon.vertices[j];
        }

        // Set primitive type based on face size
        if (face.mNumIndices == 3) {
            mesh->mPrimitiveTypes |= aiPrimitiveType_TRIANGLE;
        } else if (face.mNumIndices == 4) {
            // aiPrimitiveType_QUAD might not be available or commonly used in modern Assimp,
            // or the header might be missing. Usually, 4-vertex polys are treated as polygons
            // or triangulated by post-processing.
            // For robustness, consider just aiPrimitiveType_POLYGON, or ensure triangulation.
            // If you know your mesh is only triangles, you can simplify this.
            mesh->mPrimitiveTypes |= aiPrimitiveType_POLYGON; // More general for quads/other polys
        } else if (face.mNumIndices > 4) {
            mesh->mPrimitiveTypes |= aiPrimitiveType_POLYGON;
        } else if (face.mNumIndices == 2) {
            mesh->mPrimitiveTypes |= aiPrimitiveType_LINE;
        } else if (face.mNumIndices == 1) {
            mesh->mPrimitiveTypes |= aiPrimitiveType_POINT;
        }


        // Determine face color for material assignment
        // Using the first vertex's color. Adjust if you need average or another heuristic.
        unsigned char r = cloud->points[pcl_polygon.vertices[0]].r;
        unsigned char g = cloud->points[pcl_polygon.vertices[0]].g;
        unsigned char b = cloud->points[pcl_polygon.vertices[0]].b;
        std::tuple<unsigned char, unsigned char, unsigned char> face_color_tuple = {r, g, b};

        unsigned int material_idx;
        auto it = unique_colors_to_material_idx.find(face_color_tuple);

        if (it == unique_colors_to_material_idx.end()) {
            // New unique color, create a new material
            material_idx = materials.size();
            aiMaterial* new_material = new aiMaterial();
            // Call toAssimpColor as a static member function of PclMeshToAssimpConverter
            aiColor3D diffuse_color = PclMeshToAssimpConverter::toAssimpColor(r, g, b); // <--- FIX HERE
            new_material->AddProperty(&diffuse_color, 1, AI_MATKEY_COLOR_DIFFUSE);
            new_material->AddProperty(&diffuse_color, 1, AI_MATKEY_COLOR_AMBIENT); // Often same as diffuse
            aiColor3D specular_color(0.0f, 0.0f, 0.0f); // Non-shiny
            new_material->AddProperty(&specular_color, 1, AI_MATKEY_COLOR_SPECULAR);
            float shininess = 0.0f;
            new_material->AddProperty(&shininess, 1, AI_MATKEY_SHININESS);
            float opacity = 1.0f;
            new_material->AddProperty(&opacity, 1, AI_MATKEY_OPACITY);


            std::string mat_name = "material_" + std::to_string(material_idx);
            new_material->AddProperty(mat_name.c_str(), static_cast<unsigned int>(mat_name.length()), AI_MATKEY_NAME);

            materials.push_back(new_material);
            unique_colors_to_material_idx[face_color_tuple] = material_idx;
        } else {
            material_idx = it->second; // Use existing material index
        }
        mesh->mMaterialIndex = material_idx; // Assign material to the mesh
    }

    // Add materials to the scene
    scene->mNumMaterials = materials.size();
    scene->mMaterials = new aiMaterial*[scene->mNumMaterials];
    for (size_t i = 0; i < materials.size(); ++i) {
        scene->mMaterials[i] = materials[i];
    }

    return scene;
}

bool PclMeshToAssimpConverter::exportAssimpScene(aiScene* scene, const std::string& format_id, const std::string& filename) {
    if (!scene) {
        std::cerr << "Error: Cannot export null Assimp scene." << std::endl;
        return false;
    }

    Assimp::Exporter exporter;

    if (exporter.Export(scene, format_id, filename) == AI_SUCCESS) {
        std::cout << "Successfully exported mesh to " << filename << std::endl;
        return true;
    } else {
        std::cerr << "Error exporting mesh to " << filename << ": " << exporter.GetErrorString() << std::endl;
        return false;
    }
} 

// void convertPLYToAssimp(const std::string& ply_file, const std::string& out_dae) {
//     Assimp::Importer importer;
//     const aiScene* scene = importer.ReadFile(
//       ply_file,
//       aiProcess_Triangulate | aiProcess_JoinIdenticalVertices
//     );
//     if (!scene)
//       throw std::runtime_error(importer.GetErrorString());

//     Assimp::Exporter exporter;
//     if (exporter.Export(scene, "collada", out_dae) != AI_SUCCESS) {
//       throw std::runtime_error("DAE export failed: " +
//                               std::string(exporter.GetErrorString()));
//     }

//     RCLCPP_INFO(this->get_logger(), "Converted PLY to DAE: %s", out_dae.c_str());
// }