
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# dml/CMakeLists.txt


include_directories( ${VXL_GEL_INCLUDE_DIR}/mrc )
include_directories( ${VXL_RPL_INCLUDE_DIR} )
include_directories( ${VXL_BRL_INCLUDE_DIR}/bbas )

set(dml_sources
  dml_proj_cost_func.cxx         dml_proj_cost_func.h
  dml_proj_lsqr_cost_func.cxx    dml_proj_lsqr_cost_func.h

  dml_vehicle_mesh.cxx           dml_vehicle_mesh.h
  dml_vehicle_parts.cxx          dml_vehicle_parts.h
  dml_vehicle_fit.cxx            dml_vehicle_fit.h
  dml_vehicle_fit_multiview.cxx  dml_vehicle_fit_multiview.h
  dml_vehicle_fit_video.cxx      dml_vehicle_fit_video.h
  dml_vehicle_state.cxx          dml_vehicle_state.h
  dml_vehicle_tracker.cxx        dml_vehicle_tracker.h
  dml_vehicle_track_init.cxx     dml_vehicle_track_init.h
  dml_pca_vehicle.cxx            dml_pca_vehicle.h
  dml_pca_vehicle_projector.cxx  dml_pca_vehicle_projector.h

  dml_solar_position.cxx         dml_solar_position.h

  dml_edgel.cxx                  dml_edgel.h
)

add_library(dml  ${dml_sources})

target_link_libraries(dml vpro vpro_filters bbgm rrel imesh_algo vpgl vil_algo klt)

if( BUILD_TESTING )
  add_subdirectory(tests)
endif()

# Make the Python Module for running experiemnts
include( ${VXL_CMAKE_DIR}/NewCMake/FindPython.cmake )
if( PYTHON_FOUND )
  include_directories( ${PYTHON_INCLUDE_PATH} )
  
  set(py_dml_sources
      py_dml.cxx
      py_dml_manager.cxx   py_dml_manager.h
  )

  add_library(py_dml MODULE ${py_dml_sources})

  if(WIN32)
    #library has to have different name depending on debug or release version.
    set_target_properties(py_dml PROPERTIES OUTPUT_NAME py_dml DEBUG_POSTFIX _d SUFFIX .pyd)
  endif(WIN32)
  set_target_properties(py_dml PROPERTIES PREFIX "")

  target_link_libraries(py_dml dml)

  if (PYTHON_DEBUG_LIBRARIES)
    target_link_libraries( py_dml debug ${PYTHON_DEBUG_LIBRARY})
  endif(PYTHON_DEBUG_LIBRARIES )

  if(PYTHON_LIBRARIES)
    target_link_libraries( py_dml optimized ${PYTHON_LIBRARY})
  endif(PYTHON_LIBRARIES)
endif( PYTHON_FOUND )

