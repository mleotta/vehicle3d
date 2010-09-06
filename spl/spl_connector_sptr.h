// This is spl/spl_connector_sptr.h
#ifndef spl_connector_sptr_h
#define spl_connector_sptr_h
//--------------------------------------------------------------------------------
//:
// \file
// \brief Defines a smart pointer to a connector.
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


class spl_connector;

#include <spl/spl_observer_sptr.h>

//: Smart-pointer to a spl_connector.
typedef spl_observer_sptr_t<spl_connector> spl_connector_sptr;

#endif // spl_connector_sptr_h
