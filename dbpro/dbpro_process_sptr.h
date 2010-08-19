// This is dbpro/dbpro_process_sptr.h
#ifndef dbpro_process_sptr_h
#define dbpro_process_sptr_h
//--------------------------------------------------------------------------------
//:
// \file
// \brief Defines a smart pointer to a process.
//
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 5/30/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
//
// \verbatim
//  Modifications:
// \endverbatim
//--------------------------------------------------------------------------------

class dbpro_process;

#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<dbpro_process> dbpro_process_sptr;

#endif // dbpro_process_sptr_h
