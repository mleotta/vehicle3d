// This is spl/spl_observer.h
#ifndef spl_observer_h_
#define spl_observer_h_
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

#include <spl/spl_process_sptr.h>
#include <spl/spl_storage_sptr.h>
#include <spl/spl_observer_sptr.h>

//: This abstract class is the base class for observer objects

class spl_observer : public vbl_ref_count
{
 public:

  //: Called by the process when the data is ready
  virtual bool notify(const spl_storage_sptr& data, unsigned long timestamp) = 0;

  //: Indicate that the observer will no longer be notified
  virtual bool unlink() { return true; }

 protected:
  //: Disable direct construction and destruction of the base class
  spl_observer(const spl_observer& /*other*/) : vbl_ref_count() {}
  spl_observer(){}
  virtual ~spl_observer(){}

};




#endif // spl_observer_h_
