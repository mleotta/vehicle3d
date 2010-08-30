// This is vpro/filters/vidl_source.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vidl_source.h"
#include <vidl/vidl_istream.h>


//: Run the process on the current frame
vpro_signal 
vidl_source::execute()
{
  if(!istream_->advance()){
    return VPRO_EOS;
  }

  vidl_frame_sptr frame = istream_->current_frame();
  if(!frame)
    return VPRO_INVALID;

  output(0,frame);

  return VPRO_VALID;
}




