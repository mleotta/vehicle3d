// This is dbpro/filters/vidl_multi_source.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vidl_multi_source.h"
#include <vidl/vidl_istream.h>


//: Run the process on the current frame
dbpro_signal 
vidl_multi_source::execute()
{
  bool all_passed = true;
  for(unsigned int i=0; i<istreams_.size(); ++i){
    if(istreams_[i])
      all_passed = istreams_[i]->advance() && all_passed;
  }
  if(!all_passed)
    return DBPRO_EOS;

  vidl_frame_sptr frame = NULL;
  for(unsigned int i=0; i<istreams_.size(); ++i){
    if(istreams_[i]){
      frame = istreams_[i]->current_frame();
      all_passed = bool(frame) && all_passed;
      output(i, frame);
    }
  }
  if(!all_passed)
    return DBPRO_INVALID;

  return DBPRO_VALID;
}




