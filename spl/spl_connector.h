// This is spl/spl_connector.h
#ifndef spl_connector_h_
#define spl_connector_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief An observer that connects a process output to a process input
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
//
// \verbatim
//  Modifications:
// \endverbatim
//--------------------------------------------------------------------------------


#include <spl/spl_observer.h>
#include <spl/spl_process.h>

//: An observer that connects a process output to a process input
class spl_connector : public spl_observer
{
 public:
  //: Constructor
  spl_connector() : data_(NULL), source_(NULL), source_pin_(0), timestamp_(0) {}

  //: Transmit the data to the input of a process
  bool notify(const spl_storage_sptr& data, unsigned long timestamp);

  //: Access the data
  spl_storage_sptr data() const { return data_; }

  //: Access the timestamp
  unsigned long timestamp() const { return timestamp_; }

  //: Disconnect from a process output pin
  bool unlink();

  //: Make a request to the source for data
  spl_signal request_data(unsigned long timestamp,
                           spl_debug_observer* const debug = NULL) const;

  //: Connect this connector to the output of a process
  bool link(const spl_process_sptr& process, unsigned int pin);

 private:
  spl_storage_sptr data_;
  spl_process* source_;
  unsigned int source_pin_;
  unsigned long timestamp_;
};




#endif // spl_connector_h_
