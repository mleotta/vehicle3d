// This is basic/dbvidl2/pro/dbvidl2_source.cxx

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




