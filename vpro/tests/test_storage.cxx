//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <testlib/testlib_test.h>
#include <vpro/vpro_storage.h>
#include <vcl_iostream.h>


MAIN( test_storage )
{
  START ("storage");

  vpro_storage_sptr ds_int = new vpro_storage_type<int>(10);
  int test = ds_int->data<int>();


  TEST("get data (int)", test, 10);

  SUMMARY();
}
