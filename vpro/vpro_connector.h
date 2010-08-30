// This is vpro/vpro_connector.h
#ifndef vpro_connector_h_
#define vpro_connector_h_
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


#include <vpro/vpro_observer.h>
#include <vpro/vpro_process.h>

//: An observer that connects a process output to a process input
class vpro_connector : public vpro_observer
{
 public:
  //: Constructor
  vpro_connector() : data_(NULL), source_(NULL), source_pin_(0), timestamp_(0) {}

  //: Transmit the data to the input of a process
  bool notify(const vpro_storage_sptr& data, unsigned long timestamp);

  //: Access the data
  vpro_storage_sptr data() const { return data_; }

  //: Access the timestamp
  unsigned long timestamp() const { return timestamp_; }

  //: Disconnect from a process output pin
  bool unlink();

  //: Make a request to the source for data
  vpro_signal request_data(unsigned long timestamp,
                           vpro_debug_observer* const debug = NULL) const;

  //: Connect this connector to the output of a process
  bool link(const vpro_process_sptr& process, unsigned int pin);

 private:
  vpro_storage_sptr data_;
  vpro_process* source_;
  unsigned int source_pin_;
  unsigned long timestamp_;
};




#endif // vpro_connector_h_
