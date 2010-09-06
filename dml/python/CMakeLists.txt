
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# dml/python/CMakeLists.txt


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

