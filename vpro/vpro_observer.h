// This is vpro/vpro_observer.h
#ifndef vpro_observer_h_
#define vpro_observer_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief The output observer base class
//
//  Using this observer design pattern, this class implements the observers of
//  output from a process.  The process will notify it's output observers
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 5/30/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications:
// \endverbatim
//--------------------------------------------------------------------------------


#include <vbl/vbl_ref_count.h>

#include <vpro/vpro_process_sptr.h>
#include <vpro/vpro_storage_sptr.h>
#include <vpro/vpro_observer_sptr.h>

//: This abstract class is the base class for observer objects

class vpro_observer : public vbl_ref_count
{
 public:

  //: Called by the process when the data is ready
  virtual bool notify(const vpro_storage_sptr& data, unsigned long timestamp) = 0;

  //: Indicate that the observer will no longer be notified
  virtual bool unlink() { return true; }

 protected:
  //: Disable direct construction and destruction of the base class
  vpro_observer(const vpro_observer& /*other*/) : vbl_ref_count() {}
  vpro_observer(){}
  virtual ~vpro_observer(){}

};




#endif // vpro_observer_h_
