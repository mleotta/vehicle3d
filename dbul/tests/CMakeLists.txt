
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# dbul/tests/CMakeLists.txt
# Tests for dbul library

ADD_EXECUTABLE( dbul_test_all
  test_driver.cxx
  test_solar_position.cxx
)

TARGET_LINK_LIBRARIES( dbul_test_all dbul testlib  )


ADD_TEST( dbul_test_solar_position         dbul_test_all test_solar_position )

ADD_EXECUTABLE( dbul_test_include test_include.cxx )

TARGET_LINK_LIBRARIES( dbul_test_include dbul )
