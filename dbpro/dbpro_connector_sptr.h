// This is dbpro/dbpro_connector_sptr.h
#ifndef dbpro_connector_sptr_h
#define dbpro_connector_sptr_h
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


class dbpro_connector;

#include <dbpro/dbpro_observer_sptr.h>

//: Smart-pointer to a dbpro_connector.
typedef dbpro_observer_sptr_t<dbpro_connector> dbpro_connector_sptr;

#endif // dbpro_connector_sptr_h
