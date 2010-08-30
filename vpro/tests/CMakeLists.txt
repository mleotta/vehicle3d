
#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

# vpro/tests/CMakeLists.txt


ADD_EXECUTABLE( vpro_test_all
  test_driver.cxx
  test_process.cxx
  test_process_factory.cxx
  test_delay_filter.cxx
  test_storage.cxx
  vpro_sample_processes.cxx
)

TARGET_LINK_LIBRARIES( vpro_test_all vpro testlib  )

ADD_TEST( vpro_test_process          vpro_test_all test_process )
ADD_TEST( vpro_test_process_factory  vpro_test_all test_process_factory )
ADD_TEST( vpro_test_delay_filter     vpro_test_all test_delay_filter )
ADD_TEST( vpro_test_storage          vpro_test_all test_storage )


ADD_EXECUTABLE( vpro_test_include test_include.cxx )
TARGET_LINK_LIBRARIES( vpro_test_include vpro)
