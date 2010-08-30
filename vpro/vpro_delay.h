// This is vpro/vpro_delay.h
#ifndef vpro_delay_h_
#define vpro_delay_h_

//:
// \file
// \brief A filter that intoduces a time delay
// \author Matt Leotta
// \date 6/2/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <vpro/vpro_process.h>
#include <vpro/vpro_connector.h>
#include <vpro/vpro_mutex.h>
#include <vcl_deque.h>


//: A process that intoduces a time delay
class vpro_delay : public vpro_process
{
 public:

  //: Constructor
  vpro_delay(unsigned int size = 1);

  //: Constructor
  template <class T>
  vpro_delay(unsigned int size, const T& data)
  : max_size_(size), recieved_timestamp_(0), recieved_signal_(VPRO_VALID),
    queue_(size,new vpro_storage_type<T>(data)) 
  {
  }

  //: Destructor
  virtual ~vpro_delay();

  //: Runs the process
  vpro_signal run(unsigned long timestamp,
                   vpro_debug_observer* const debug = NULL);

  //: Execute this process
  vpro_signal execute();
  
  //: Request data from dependents and shift the queue 
  void shift_queue(unsigned long timestamp,
                   vpro_debug_observer* const debug = NULL);

 private:
  unsigned max_size_;
  unsigned long recieved_timestamp_;
  vpro_signal recieved_signal_;
  vcl_deque<vpro_storage_sptr> queue_;

  //: mutex for updating the queue
  vpro_mutex queue_mutex_;

};


#endif // vpro_delay_h_
