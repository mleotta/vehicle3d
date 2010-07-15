# modrec/CMakeLists.txt


include_directories( ${VXL_GEL_INCLUDE_DIR}/mrc )
include_directories( ${VXL_RPL_INCLUDE_DIR} )
include_directories( ${VXL_BRL_INCLUDE_DIR}/bbas )

set(modrec_sources
  modrec_proj_cost_func.cxx         modrec_proj_cost_func.h
  modrec_proj_lsqr_cost_func.cxx    modrec_proj_lsqr_cost_func.h

  modrec_vehicle_mesh.cxx           modrec_vehicle_mesh.h
  modrec_vehicle_parts.cxx          modrec_vehicle_parts.h
  modrec_vehicle_fit.cxx            modrec_vehicle_fit.h
  modrec_vehicle_fit_multiview.cxx  modrec_vehicle_fit_multiview.h
  modrec_vehicle_fit_video.cxx      modrec_vehicle_fit_video.h
  modrec_vehicle_state.cxx          modrec_vehicle_state.h
  modrec_vehicle_tracker.cxx        modrec_vehicle_tracker.h
  modrec_vehicle_track_init.cxx     modrec_vehicle_track_init.h
  modrec_pca_vehicle.cxx            modrec_pca_vehicle.h
  modrec_pca_vehicle_projector.cxx  modrec_pca_vehicle_projector.h

  modrec_edgel.cxx                  modrec_edgel.h
)

add_library(modrec  ${modrec_sources})

target_link_libraries(modrec dbpro dbpro_filters bbgm rrel imesh_algo vpgl vil_algo klt)

if( BUILD_TESTING )
#  add_subdirectory(tests)
endif()

# Make the Python Module for running experiemnts
include( ${VXL_CMAKE_DIR}/NewCMake/FindPython.cmake )
if( PYTHON_FOUND )
  include_directories( ${PYTHON_INCLUDE_PATH} )
  
  set(py_modrec_sources
      py_modrec.cxx
      py_modrec_manager.cxx   py_modrec_manager.h
  )

  add_library(py_modrec MODULE ${py_modrec_sources})

  if(WIN32)
    #library has to have different name depending on debug or release version.
    set_target_properties(py_modrec PROPERTIES OUTPUT_NAME py_modrec DEBUG_POSTFIX _d SUFFIX .pyd)
  endif(WIN32)
  set_target_properties(py_modrec PROPERTIES PREFIX "")

  target_link_libraries(py_modrec modrec dbul)

  if (PYTHON_DEBUG_LIBRARIES)
    target_link_libraries( py_modrec debug ${PYTHON_DEBUG_LIBRARY})
  endif(PYTHON_DEBUG_LIBRARIES )

  if(PYTHON_LIBRARIES)
    target_link_libraries( py_modrec optimized ${PYTHON_LIBRARY})
  endif(PYTHON_LIBRARIES)
endif( PYTHON_FOUND )

