
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# spl/CMakeLists.txt

project(spl)

include_directories( ${VXL_BRL_INCLUDE_DIR} )

find_package( Threads )

configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/spl_config.h.in
                ${CMAKE_CURRENT_BINARY_DIR}/spl_config.h )


set(spl_sources
   spl_mutex.cxx                  spl_mutex.h
   spl_parameters.cxx             spl_parameters.h             spl_parameters_sptr.h
   spl_parameters.txx
   spl_process.cxx                spl_process.h                spl_process_sptr.h
                                   spl_basic_processes.h
                                   spl_vsl_processes.h
                                   spl_ios_processes.h
   spl_process_factory.cxx        spl_process_factory.h
                                   spl_storage.h                spl_storage_sptr.h
                                   spl_observer.h               spl_observer_sptr.h
   spl_connector.cxx              spl_connector.h              spl_connector_sptr.h
   spl_delay.cxx                  spl_delay.h
   spl_try_option.cxx             spl_try_option.h
   spl_executive.cxx              spl_executive.h
)



aux_source_directory(Templates spl_sources)

add_library(spl  ${spl_sources})


target_link_libraries(spl vul vbl)

if( CMAKE_USE_PTHREADS_INIT )
  set_target_properties(spl PROPERTIES COMPILE_FLAGS -pthread)
  target_link_libraries(spl pthread)
endif()


add_subdirectory( vis )

add_subdirectory( filters )


if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
