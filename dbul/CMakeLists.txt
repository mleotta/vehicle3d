# /basic/dbul/CMakeLists.txt
# Development Brown Utility Library


SET(dbul_sources 
    dbul_solar_position.h   dbul_solar_position.cxx
   )

ADD_LIBRARY(dbul  ${dbul_sources})

if (UNIX)
  target_link_libraries(dbul m)
endif (UNIX)

IF( BUILD_TESTING )
  SUBDIRS(tests)
ENDIF( BUILD_TESTING )
