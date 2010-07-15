// This is basic/dbvidl2/pro/dbvidl2_frame_to_resource.h
#ifndef dbvidl2_frame_to_resource_h_
#define dbvidl2_frame_to_resource_h_

//:
// \file
// \brief Convert a vidl_frame into a vil_image_resource
// \author Matt Leotta
// \date 6/1/06
//
// \verbatim
//  Modifications
// \endverbatim


#include <dbpro/dbpro_process.h>
#include <vil/vil_pixel_format.h>
#include <vidl/vidl_pixel_format.h>
#include <vil/vil_image_resource.h>


//: Convert a vidl_frame into a vil_image_resource
class dbvidl2_frame_to_resource : public dbpro_filter
{
 public:

  //: Specifies the different options for when to wrap the frame data in a resource
  // \li \c REQUIRE_WRAP - wrap the frame if possible, if not return INVALID
  // \li \c PREFER_WRAP - wrap the frame if possible, if not convert and copy
  // \li \c ALLOW_WRAP - wrap the frame only if the pixel type matches matches the requested type
  // \li \c NEVER_WRAP - never wrap, always convert and copy the data 
  enum wrap_t { REQUIRE_WRAP, PREFER_WRAP, ALLOW_WRAP, NEVER_WRAP };

  //: Specifies the options for generating multichannel images
  enum channel_t { INTERLEAVED, PLANES };

  //: Specifies whether or not the same image buffer is reused for output.
  //  In general, it is not safe to do this, but allocating new memory
  //  each time can be very slow.
  enum buffer_t { ALLOCATE_MEMORY, REUSE_MEMORY };

  //: Default Constructor
  dbvidl2_frame_to_resource() : image_rsc_(NULL),
                                output_format_(VIL_PIXEL_FORMAT_BYTE),
                                color_(VIDL_PIXEL_COLOR_UNKNOWN),
                                channel_mode_(INTERLEAVED),
                                buffer_mode_(ALLOCATE_MEMORY),
                                wrap_mode_(REQUIRE_WRAP) {}

  //: Constructor
  dbvidl2_frame_to_resource( wrap_t wrap,
                             vil_pixel_format of=VIL_PIXEL_FORMAT_BYTE,
                             vidl_pixel_color color=VIDL_PIXEL_COLOR_UNKNOWN,
                             channel_t channel=PLANES,
                             buffer_t buffer=ALLOCATE_MEMORY)
  : image_rsc_(NULL), output_format_(of), color_(color),
    channel_mode_(channel), buffer_mode_(buffer),
    wrap_mode_(wrap) {}

  //: Destructor
  virtual ~dbvidl2_frame_to_resource(){}

  //: Execute this process
  dbpro_signal execute();

 private:
  vil_image_resource_sptr image_rsc_;
  
  vil_pixel_format   output_format_;
  vidl_pixel_color   color_;
  channel_t          channel_mode_;
  buffer_t           buffer_mode_;
  wrap_t             wrap_mode_;

};

#endif // dbvidl2_frame_to_resource_h_
