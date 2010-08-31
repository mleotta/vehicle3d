
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# cmd/CMakeLists.txt
# Command line programs for mesh computations

INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )

ADD_EXECUTABLE(mesh_subdivide mesh_subdivide.cxx)
TARGET_LINK_LIBRARIES(mesh_subdivide imesh_algo vul)

ADD_EXECUTABLE(mesh_extract_features mesh_extract_features.cxx)
TARGET_LINK_LIBRARIES(mesh_extract_features imesh_algo dml vul)

ADD_EXECUTABLE(make_vehicle_mesh make_vehicle_mesh.cxx)
TARGET_LINK_LIBRARIES(make_vehicle_mesh dml imesh_algo vul)

ADD_EXECUTABLE(fit_mesh_body fit_mesh_body.cxx)
TARGET_LINK_LIBRARIES(fit_mesh_body dml imesh_algo vul)

ADD_EXECUTABLE(fit_parts fit_parts.cxx)
TARGET_LINK_LIBRARIES(fit_parts dml vul)

ADD_EXECUTABLE(parts2svg parts2svg.cxx)
TARGET_LINK_LIBRARIES(parts2svg dml vul)

ADD_EXECUTABLE(pca_fit_mesh_body pca_fit_mesh_body.cxx)
TARGET_LINK_LIBRARIES(pca_fit_mesh_body imesh_algo vul)

ADD_EXECUTABLE(mesh_pca mesh_pca.cxx)
TARGET_LINK_LIBRARIES(mesh_pca imesh_algo vul)

ADD_EXECUTABLE(parts_pca parts_pca.cxx)
TARGET_LINK_LIBRARIES(parts_pca dml vul)

ADD_EXECUTABLE(mesh_exterior mesh_exterior.cxx)
TARGET_LINK_LIBRARIES(mesh_exterior imesh_algo vul)

