// This is spl/spl_connector.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include <spl/spl_connector.h>
#include <spl/spl_process.h>


//: Transmit the data to the input of a process
bool
spl_connector::notify(const spl_storage_sptr& data,
                        unsigned long timestamp)
{
  data_ = data;
  timestamp_ = timestamp;
  return true;
}


//: disconnect from a process output pin
bool
spl_connector::unlink()
{
  if(source_){
    if(!source_->remove_output_observer(source_pin_,this))
      return false;
  }
  source_ = NULL;
  source_pin_ = 0;
  return true;
}

//: Connect this connector to the output of a process
bool
spl_connector::link(const spl_process_sptr& process, unsigned int pin)
{
  if(!unlink() || !process )
    return false;

  process->add_output_observer(pin,this);
  source_ = process.ptr();
  source_pin_ = pin;
  return true;
}


//: Make a request to the source for data
spl_signal
spl_connector::request_data(unsigned long timestamp,
                              spl_debug_observer* const debug) const
{
  if(!source_)
    return SPL_INVALID;
  spl_signal result = source_->run(timestamp,debug);
  if(result == SPL_VALID && !data_)
    return SPL_INVALID;
  return result;
}
