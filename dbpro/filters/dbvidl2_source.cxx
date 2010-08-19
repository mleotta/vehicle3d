// This is dbpro/filters/dbvidl2_source.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "dbvidl2_source.h"
#include <vidl/vidl_istream.h>


//: Run the process on the current frame
dbpro_signal 
dbvidl2_source::execute()
{
  if(!istream_->advance()){
    return DBPRO_EOS;
  }

  vidl_frame_sptr frame = istream_->current_frame();
  if(!frame)
    return DBPRO_INVALID;

  output(0,frame);

  return DBPRO_VALID;
}




