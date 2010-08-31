
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# modrec/tests/CMakeLists.txt
# Tests for modrec library

ADD_EXECUTABLE( modrec_test_all
  test_driver.cxx
  test_solar_position.cxx
)

TARGET_LINK_LIBRARIES( modrec_test_all modrec testlib  )


ADD_TEST( modrec_test_solar_position      modrec_test_all test_solar_position )

ADD_EXECUTABLE( modrec_test_include test_include.cxx )

TARGET_LINK_LIBRARIES( modrec_test_include modrec )
