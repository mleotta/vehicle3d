// This is spl/filters/vidl_sink.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vidl_sink.h"
#include <vidl/vidl_ostream.h>



//: Run the process on the current frame
spl_signal
vidl_sink::execute()
{
  if(!ostream_)
    return SPL_VALID;

  assert(input_type_id(0) == typeid(vidl_frame_sptr));
  vidl_frame_sptr frame = input<vidl_frame_sptr>(0);
  if(!frame)
    return SPL_INVALID;

  if(!ostream_->write_frame(frame))
    return SPL_INVALID;

  return SPL_VALID;
}




