// This is vpro/filters/vil_morphology_filters.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vil_morphology_filters.h"
#include <vil/vil_image_resource.h>
#include <vil/vil_image_view.h>
#include <vil/vil_new.h>
#include <vil/algo/vil_binary_erode.h>
#include <vil/algo/vil_binary_dilate.h>


//: Execute this process
vpro_signal
vil_binary_erode_filter::execute()
{
  assert(input_type_id(0) == typeid(vil_image_resource_sptr));
  vil_image_resource_sptr in_img = input<vil_image_resource_sptr>(0);


  vil_image_view<bool> image_in, image_out;
  image_in = in_img->get_view();

  vil_binary_erode(image_in, image_out, se_);

  // create the output storage class
  vil_image_resource_sptr out_img =  vil_new_image_resource_of_view(image_out);
  output(0,out_img);

  return VPRO_VALID;
}


//: Execute this process
vpro_signal
vil_binary_dilate_filter::execute()
{
  assert(input_type_id(0) == typeid(vil_image_resource_sptr));
  vil_image_resource_sptr in_img = input<vil_image_resource_sptr>(0);


  vil_image_view<bool> image_in, image_out;
  image_in = in_img->get_view();

  vil_binary_dilate(image_in, image_out, se_);

  // create the output storage class
  vil_image_resource_sptr out_img =  vil_new_image_resource_of_view(image_out);
  output(0,out_img);

  return VPRO_VALID;
}




