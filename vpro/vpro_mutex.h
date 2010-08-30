// This is vpro/vpro_mutex.h
#ifndef vpro_mutex_h_
#define vpro_mutex_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief A mutex used if threads are enabled 
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 3/23/09
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


#include <vcl_memory.h>

//: A mutex used if threads are enabled 
// If threads are not enabled, do nothing
class vpro_mutex
{
public:
  //: Default Constructor
  vpro_mutex();
  
  //: Destructor
  ~vpro_mutex();
  
  //: Lock the mutex
  void lock();
  
  //: Unlock the mutex
  void unlock();

private:
  
  //: private implementation struct
  struct pimpl;
  //: private implementation instance
  vcl_auto_ptr<pimpl> data_;

};



#endif // vpro_mutex_h_
