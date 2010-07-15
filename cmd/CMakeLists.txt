# contrib/mleotta/cmd/mesh/CMakeLists.txt
# Command line programs for mesh computations

INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )

ADD_EXECUTABLE(mesh_subdivide mesh_subdivide.cxx)
TARGET_LINK_LIBRARIES(mesh_subdivide imesh_algo vul)

ADD_EXECUTABLE(mesh_extract_features mesh_extract_features.cxx)
TARGET_LINK_LIBRARIES(mesh_extract_features imesh_algo modrec vul)

ADD_EXECUTABLE(make_vehicle_mesh make_vehicle_mesh.cxx)
TARGET_LINK_LIBRARIES(make_vehicle_mesh modrec imesh_algo vul)

ADD_EXECUTABLE(fit_mesh_body fit_mesh_body.cxx)
TARGET_LINK_LIBRARIES(fit_mesh_body modrec imesh_algo vul)

ADD_EXECUTABLE(fit_parts fit_parts.cxx)
TARGET_LINK_LIBRARIES(fit_parts modrec vul)

ADD_EXECUTABLE(parts2svg parts2svg.cxx)
TARGET_LINK_LIBRARIES(parts2svg modrec vul)

ADD_EXECUTABLE(pca_fit_mesh_body pca_fit_mesh_body.cxx)
TARGET_LINK_LIBRARIES(pca_fit_mesh_body imesh_algo vul)

ADD_EXECUTABLE(mesh_pca mesh_pca.cxx)
TARGET_LINK_LIBRARIES(mesh_pca imesh_algo vul)

ADD_EXECUTABLE(parts_pca parts_pca.cxx)
TARGET_LINK_LIBRARIES(parts_pca modrec vul)

ADD_EXECUTABLE(mesh_exterior mesh_exterior.cxx)
TARGET_LINK_LIBRARIES(mesh_exterior imesh_algo vul)

