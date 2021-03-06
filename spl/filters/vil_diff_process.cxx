// This is spl/filters/vil_diff_process.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vil_diff_process.h"
#include <spl/spl_storage.h>
#include <spl/spl_parameters.h>
#include <vil/vil_image_resource.h>
#include <vil/vil_new.h>
#include <vil/vil_image_view.h>
#include <vil/vil_math.h>


//: Return the default set of parameters for the process
spl_parameters_sptr
vil_diff_process::factory::default_params() const
{
  // no parameters to add
  spl_parameters_sptr p = new spl_parameters();
  return p;
}


//: Construct a process from a set of parameters
spl_process_sptr
vil_diff_process::factory::create(const spl_parameters_sptr& params) const
{
  return new vil_diff_process();
}


//: Execute this process
spl_signal
vil_diff_process::execute()
{
  assert(input_type_id(0) == typeid(vil_image_resource_sptr));
  assert(input_type_id(1) == typeid(vil_image_resource_sptr));
  vil_image_resource_sptr in_img1 = input<vil_image_resource_sptr>(0);
  vil_image_resource_sptr in_img2 = input<vil_image_resource_sptr>(1);


  vil_image_view<vxl_byte> image1, image2;
  image1 = in_img1->get_view();
  image2 = in_img2->get_view();

  vil_image_view<vxl_byte> diff_img;
  vil_math_image_abs_difference(image2, image1, diff_img);

  // create the output storage class
  vil_image_resource_sptr out_img =  vil_new_image_resource_of_view(diff_img);
  output(0,out_img);

  return SPL_VALID;
}





