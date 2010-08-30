
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# vpro/CMakeLists.txt

project(vpro)

include_directories( ${VXL_BRL_INCLUDE_DIR} )

find_package( Threads )

configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/vpro_config.h.in
                ${CMAKE_CURRENT_BINARY_DIR}/vpro_config.h )


set(vpro_sources
   vpro_mutex.cxx                  vpro_mutex.h
   vpro_parameters.cxx             vpro_parameters.h             vpro_parameters_sptr.h
   vpro_parameters.txx
   vpro_process.cxx                vpro_process.h                vpro_process_sptr.h
                                   vpro_basic_processes.h
                                   vpro_vsl_processes.h
                                   vpro_ios_processes.h
   vpro_process_factory.cxx        vpro_process_factory.h
                                   vpro_storage.h                vpro_storage_sptr.h
                                   vpro_observer.h               vpro_observer_sptr.h
   vpro_connector.cxx              vpro_connector.h              vpro_connector_sptr.h
   vpro_delay.cxx                  vpro_delay.h
   vpro_try_option.cxx             vpro_try_option.h
   vpro_executive.cxx              vpro_executive.h
)



aux_source_directory(Templates vpro_sources)

add_library(vpro  ${vpro_sources})


target_link_libraries(vpro vul vbl)

if( CMAKE_USE_PTHREADS_INIT )
  set_target_properties(vpro PROPERTIES COMPILE_FLAGS -pthread)
  target_link_libraries(vpro pthread)
endif()


add_subdirectory( vis )

add_subdirectory( filters )


if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
