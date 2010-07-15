// This is basic/dbvidl2/pro/dbvidl2_sink.cxx

//:
// \file

#include "dbvidl2_sink.h"
#include <vidl/vidl_ostream.h>



//: Run the process on the current frame
dbpro_signal
dbvidl2_sink::execute()
{
  if(!ostream_)
    return DBPRO_VALID;

  assert(input_type_id(0) == typeid(vidl_frame_sptr));
  vidl_frame_sptr frame = input<vidl_frame_sptr>(0);
  if(!frame)
    return DBPRO_INVALID;

  if(!ostream_->write_frame(frame))
    return DBPRO_INVALID;

  return DBPRO_VALID;
}




