// Basic mesh operations: welding, normals, tangents, cache optimisation, transforms.
#pragma once
#include <vector>
#include "rx/scene.h"

namespace rx {

// Exact duplicate-vertex removal over all attributes.
void weldMesh(Mesh& m);
// Remap of every vertex to the first vertex sharing its position (ids are vertex indices).
std::vector<uint32_t> positionIds(const Mesh& m);
// Smooth normals; faces meeting at more than `angleDeg` keep hard edges (may split vertices).
void computeNormals(Mesh& m, float angleDeg = 180.0f);
// Per vertex tangents (glTF / MikkTSpace-like handedness: bitangent = cross(N, T) * w).
void computeTangents(Mesh& m);
void optimizeMesh(Mesh& m);
void removeDegenerateTriangles(Mesh& m);
void transformMesh(Mesh& m, const Mat4& t);
// Compacts the vertex arrays to the vertices referenced by `indices`.
void compactVertices(Mesh& m);
// Expands the mesh so every triangle corner has its own vertex.
void unweld(Mesh& m);

std::vector<Mat4> worldMatrices(const Scene& s);
// Bakes node transforms into vertex data; result has one identity node per mesh instance.
void flattenScene(Scene& s);
// Wraps the scene in a root that converts axis / units.
void convertAxisAndUnits(Scene& s, UpAxis to, double metersPerUnit);
void dropUnusedResources(Scene& s);

}  // namespace rx
