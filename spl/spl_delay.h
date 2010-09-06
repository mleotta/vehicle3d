// This is spl/spl_delay.h
#ifndef spl_delay_h_
#define spl_delay_h_

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


#include <spl/spl_process.h>
#include <spl/spl_connector.h>
#include <spl/spl_mutex.h>
#include <vcl_deque.h>


//: A process that intoduces a time delay
class spl_delay : public spl_process
{
 public:

  //: Constructor
  spl_delay(unsigned int size = 1);

  //: Constructor
  template <class T>
  spl_delay(unsigned int size, const T& data)
  : max_size_(size), recieved_timestamp_(0), recieved_signal_(SPL_VALID),
    queue_(size,new spl_storage_type<T>(data)) 
  {
  }

  //: Destructor
  virtual ~spl_delay();

  //: Runs the process
  spl_signal run(unsigned long timestamp,
                   spl_debug_observer* const debug = NULL);

  //: Execute this process
  spl_signal execute();
  
  //: Request data from dependents and shift the queue 
  void shift_queue(unsigned long timestamp,
                   spl_debug_observer* const debug = NULL);

 private:
  unsigned max_size_;
  unsigned long recieved_timestamp_;
  spl_signal recieved_signal_;
  vcl_deque<spl_storage_sptr> queue_;

  //: mutex for updating the queue
  spl_mutex queue_mutex_;

};


#endif // spl_delay_h_
