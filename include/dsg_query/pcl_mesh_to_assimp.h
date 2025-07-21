#ifndef PCL_MESH_TO_ASSIMP_H
#define PCL_MESH_TO_ASSIMP_H

#include <pcl/PolygonMesh.h>
#include <string>

#include <assimp/scene.h> // This is crucial for aiScene, aiMesh, aiNode, aiColor3D, aiColor4D etc.
#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/material.h> 

// Forward declarations for Assimp types to avoid full headers in .h
struct aiScene;

/**
 * @brief Utility class to convert PCL PolygonMesh to Assimp's aiScene and export to various formats.
 */
class PclMeshToAssimpConverter {
public:
    /**
     * @brief Converts a pcl::PolygonMesh into an Assimp aiScene.
     * This function maps PCL vertices and faces to Assimp's data structures.
     * It also creates materials based on unique vertex colors, assigning them
     * to the mesh for proper export to formats like OBJ+MTL.
     *
     * @param pcl_mesh The input pcl::PolygonMesh containing point cloud and polygon data.
     * @return A pointer to the newly created aiScene. The caller is responsible for deleting this scene
     * if it's not passed to an Assimp Exporter for ownership transfer.
     * Returns nullptr if the input mesh is empty or invalid.
     */
    static aiScene* createAssimpSceneFromPolygonMesh(const pcl::PolygonMesh& pcl_mesh);

    /**
     * @brief Exports an Assimp aiScene to a specified file format.
     *
     * @param scene The aiScene to export. Must not be nullptr.
     * @param format_id The Assimp format ID string (e.g., "obj", "collada", "stl").
     * @param filename The desired output filename (e.g., "my_model.obj").
     * @return True if export was successful, false otherwise.
     */
    static bool exportAssimpScene(aiScene* scene, const std::string& format_id, const std::string& filename);

    /**
     * @brief Converts a PLY file to an Assimp scene and exports it.
     *
     * @param ply_file The input PLY file path.
     * @param out_dae The output DAE file path.
     */
    static void convertPLYToAssimp(const std::string& ply_file, const std::string& out_dae);

private:
    // Helper to convert PCL color (0-255) to Assimp color (0.0-1.0)
    // Declared static to be part of the class, but can also be a free function if preferred.
    static aiColor3D toAssimpColor(unsigned char r, unsigned char g, unsigned char b);
};

#endif // PCL_MESH_TO_ASSIMP_H