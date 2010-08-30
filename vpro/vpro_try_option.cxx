// This is vpro/vpro_try_option.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vpro_try_option.h"
#include "vpro_connector.h"
#include <vcl_iostream.h>

//: Runs the filter
vpro_signal
vpro_try_option::run(unsigned long timestamp,
                      vpro_debug_observer* const debug)
{
  // notify the debugger if available
  if (debug) debug->notify_enter(this, timestamp);

  update_mutex_.lock();
  
  if(timestamp > this->timestamp_){

    this->timestamp_ = timestamp;

    if(this->input_request_active_){
      vcl_cerr << "Warning detected cycle in data flow" <<vcl_endl;
      return VPRO_INVALID;
    }
    this->input_request_active_ = true;
    vpro_signal retval = VPRO_INVALID;
    typedef vcl_map<unsigned int, vpro_connector_sptr >::iterator Itr;
    for(Itr i = input_connectors_.begin(); i != input_connectors_.end(); ++i)
    {
      vpro_connector_sptr connector = i->second;
      if(!connector->data()){
        retval = connector->request_data(timestamp,debug);
        if(retval == VPRO_INVALID)
          continue;
      }

      vpro_signal m = connector->data()->info();
      if(m == VPRO_INVALID){
        retval = m;
        continue;
      }

      set_output(0,connector->data());
      break;
    }
    this->input_request_active_ = false;

    this->notify_observers(retval);
    this->clear();
    this->last_signal_ = retval;

  }

  update_mutex_.unlock();


  // notify the debugger if available
  if (debug) debug->notify_exit(this, timestamp);

  return last_signal_;
}

//: Run the process on the current frame
vpro_signal 
vpro_try_option::execute()
{
  return VPRO_INVALID;
}




