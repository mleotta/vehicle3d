// This is vpro/vpro_storage_sptr.h
#ifndef vpro_storage_sptr_h
#define vpro_storage_sptr_h
///--------------------------------------------------------------------------------
//:
// \file
// \brief Defines a smart pointer to a storage wrapper.
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

class vpro_storage;

#include <vbl/vbl_smart_ptr.h>

typedef vbl_smart_ptr<vpro_storage> vpro_storage_sptr;


#endif // vpro_storage_sptr_h
