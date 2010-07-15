// This is basic/dbvidl2/pro/dbvidl2_resource_to_frame.h
#ifndef dbvidl2_resource_to_frame_h_
#define dbvidl2_resource_to_frame_h_

//:
// \file
// \brief Convert a vil_image_resource into a vidl_frame
// \author Matt Leotta
// \date 6/2/06
//
// \verbatim
//  Modifications
// \endverbatim


#include <dbpro/dbpro_process.h>
#include <vil/vil_pixel_format.h>


//: Convert a vil_image_resource into a vidl_frame
class dbvidl2_resource_to_frame : public dbpro_filter
{
 public:

  //: Constructor
   dbvidl2_resource_to_frame(){}

  //: Destructor
  virtual ~dbvidl2_resource_to_frame(){}

  //: Execute this process
  dbpro_signal execute();

 private:

};

#endif // dbvidl2_resource_to_frame_h_
