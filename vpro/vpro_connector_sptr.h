// This is vpro/vpro_connector_sptr.h
#ifndef vpro_connector_sptr_h
#define vpro_connector_sptr_h
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


class vpro_connector;

#include <vpro/vpro_observer_sptr.h>

//: Smart-pointer to a vpro_connector.
typedef vpro_observer_sptr_t<vpro_connector> vpro_connector_sptr;

#endif // vpro_connector_sptr_h
