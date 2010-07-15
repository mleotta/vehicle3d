// This is basic/dbvidl2/pro/dbvidl2_resource_to_frame.cxx

//:
// \file

#include "dbvidl2_resource_to_frame.h"
#include <vidl/vidl_convert.h>
#include <vil/vil_image_resource.h>


//: Run the process on the current frame
dbpro_signal
dbvidl2_resource_to_frame::execute()
{
  assert(input_type_id(0) == typeid(vil_image_resource_sptr));
  vil_image_resource_sptr image = input<vil_image_resource_sptr>(0);
  if(!image)
    return DBPRO_INVALID;

  vidl_frame_sptr frame = vidl_convert_to_frame(image->get_view());

  if(!frame)
    return DBPRO_INVALID;

  output(0,frame);

  return DBPRO_VALID;
}




