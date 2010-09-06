//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <testlib/testlib_test.h>
#include "spl_sample_processes.h"
#include <vcl_iostream.h>
#include <spl/spl_process_factory.h>




MAIN( test_process_factory )
{
  START ("process_factory");


  //TEST("number of iterations", count, data.size());

  typedef spl_process_factory::Reg_Type Reg_Type;
  const Reg_Type& reg = spl_process_factory::registry();
  vcl_cout << "Registered processes" << vcl_endl;
  for (Reg_Type::const_iterator i = reg.begin(); i != reg.end(); ++i)
    vcl_cout << i->first << vcl_endl;


  SUMMARY();
}
