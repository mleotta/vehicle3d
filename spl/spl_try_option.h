// This is spl/spl_try_option.h
#ifndef spl_try_option_h_
#define spl_try_option_h_

//:
// \file
// \brief A filter that transmits the first non-failing input
// \author Matt Leotta
// \date 10/17/06
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
#include <vcl_deque.h>


//: A filter that transmits the first non-failing input
// This filter will request data from input pins sequentially.
// If a pin fails (produces invalid data) then the next pin is tried.
// As soon as a pin does not fail (sends a VALID, WAIT, EOF signal)
// the output from this pin is forwarded to output pin 0.  No further
// input requests are made on the remaining pins.  The output of this
// filter is invalid only if all inputs are invalid.
class spl_try_option : public spl_process
{
 public:

  //: Constructor
  spl_try_option() {}


  //: Destructor
  virtual ~spl_try_option() {}

  //: Runs the process
  spl_signal run(unsigned long timestamp,
                   spl_debug_observer* const debug = NULL);

  //: Execute this process
  spl_signal execute();

};

#endif // spl_try_option_h_
