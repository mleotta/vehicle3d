
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# spl/tests/CMakeLists.txt


ADD_EXECUTABLE( spl_test_all
  test_driver.cxx
  test_process.cxx
  test_process_factory.cxx
  test_delay_filter.cxx
  test_storage.cxx
  spl_sample_processes.cxx
)

TARGET_LINK_LIBRARIES( spl_test_all spl testlib  )

ADD_TEST( spl_test_process          ${EXECUTABLE_OUTPUT_PATH}/spl_test_all test_process )
ADD_TEST( spl_test_process_factory  ${EXECUTABLE_OUTPUT_PATH}/spl_test_all test_process_factory )
ADD_TEST( spl_test_delay_filter     ${EXECUTABLE_OUTPUT_PATH}/spl_test_all test_delay_filter )
ADD_TEST( spl_test_storage          ${EXECUTABLE_OUTPUT_PATH}/spl_test_all test_storage )


ADD_EXECUTABLE( spl_test_include test_include.cxx )
TARGET_LINK_LIBRARIES( spl_test_include spl)
