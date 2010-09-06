
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# dml/tests/CMakeLists.txt
# Tests for dml library

ADD_EXECUTABLE( dml_test_all
  test_driver.cxx
  test_solar_position.cxx
)

TARGET_LINK_LIBRARIES( dml_test_all dml testlib  )


ADD_TEST( dml_test_solar_position   ${EXECUTABLE_OUTPUT_PATH}/dml_test_all test_solar_position )

ADD_EXECUTABLE( dml_test_include test_include.cxx )

TARGET_LINK_LIBRARIES( dml_test_include dml )
