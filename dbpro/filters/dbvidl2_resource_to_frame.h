// This is dbpro/filters/dbvidl2_resource_to_frame.h
#ifndef dbvidl2_resource_to_frame_h_
#define dbvidl2_resource_to_frame_h_

//:
// \file
// \brief Convert a vil_image_resource into a vidl_frame
// \author Matt Leotta
// \date 6/2/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
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
