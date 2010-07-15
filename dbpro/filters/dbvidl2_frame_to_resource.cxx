// This is basic/dbvidl2/pro/dbvidl2_frame_to_resource.cxx

//:
// \file

#include "dbvidl2_frame_to_resource.h"
#include <vidl/vidl_convert.h>
#include <vil/vil_new.h>


//: Run the process on the current frame
dbpro_signal
dbvidl2_frame_to_resource::execute()
{
  assert(input_type_id(0) == typeid(vidl_frame_sptr));
  vidl_frame_sptr frame = input<vidl_frame_sptr>(0);
  if(!frame)
    return DBPRO_INVALID;

  // try to wrap the frame in a view
  if( wrap_mode_ == REQUIRE_WRAP ||
      wrap_mode_ == PREFER_WRAP ||
      wrap_mode_ == ALLOW_WRAP )
  {
    vil_image_view_base_sptr view = vidl_convert_wrap_in_view(*frame);
    if(view){
      vil_image_resource_sptr image = vil_new_image_resource_of_view(*view);
      if( wrap_mode_ != ALLOW_WRAP ||
          ( output_format_ == image->pixel_format() &&
            color_ == vidl_pixel_format_color(frame->pixel_format()) ) )
      {
        output(0,image);
        return DBPRO_VALID;
      }
    }
    else if(wrap_mode_ == REQUIRE_WRAP)
      return DBPRO_INVALID;
  }

  // create a new memory image and copy/convert the frame into this resource
  unsigned ni = frame->ni(), nj = frame->nj();
  unsigned np = vidl_pixel_color_num_channels(color_);
  if(np == 0)
    np = vidl_pixel_format_num_channels(frame->pixel_format());

  // Allocate a new image if required by the user
  // or if the old image is not the correct size or type
  if( !image_rsc_ || buffer_mode_ == ALLOCATE_MEMORY ||
      image_rsc_->ni() != ni || image_rsc_->nj() != nj || image_rsc_->nplanes() != np ||
      image_rsc_->pixel_format() != output_format_)
  {
    if(channel_mode_ == INTERLEAVED)
      image_rsc_ = vil_new_image_resource_interleaved(ni,nj,np,output_format_);
    else
      image_rsc_ = vil_new_image_resource(ni,nj,np,output_format_);
  }

  if(!vidl_convert_to_view(*frame,*image_rsc_->get_view(),color_))
    return DBPRO_INVALID;

  output(0,image_rsc_);

  return DBPRO_VALID;
}




