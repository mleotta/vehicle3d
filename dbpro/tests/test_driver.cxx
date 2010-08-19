//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <testlib/testlib_register.h>


DECLARE( test_process );
DECLARE( test_process_factory );
DECLARE( test_delay_filter );
DECLARE( test_storage );


void
register_tests()
{
  REGISTER( test_process );
  REGISTER( test_process_factory );
  REGISTER( test_delay_filter );
  REGISTER( test_storage );
}

DEFINE_MAIN;


