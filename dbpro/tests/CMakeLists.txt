# basic/dbpro/tests/CMakeLists.txt


ADD_EXECUTABLE( dbpro_test_all
  test_driver.cxx
  test_process.cxx
  test_process_factory.cxx
  test_delay_filter.cxx
  test_storage.cxx
  dbpro_sample_processes.cxx
)

TARGET_LINK_LIBRARIES( dbpro_test_all dbpro testlib  )

ADD_TEST( dbpro_test_process          dbpro_test_all test_process )
ADD_TEST( dbpro_test_process_factory  dbpro_test_all test_process_factory )
ADD_TEST( dbpro_test_delay_filter     dbpro_test_all test_delay_filter )
ADD_TEST( dbpro_test_storage          dbpro_test_all test_storage )


ADD_EXECUTABLE( dbpro_test_include test_include.cxx )
TARGET_LINK_LIBRARIES( dbpro_test_include dbpro)
