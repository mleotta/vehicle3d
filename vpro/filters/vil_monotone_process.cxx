// This is vpro/filters/vil_monotone_process.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vil_monotone_process.h"
#include <vpro/vpro_parameters.h>
#include <vpro/vpro_storage.h>
#include <vil/vil_image_resource.h>
#include <vil/vil_new.h>
#include <vil/vil_image_view.h>
#include <vil/vil_convert.h>


//: Return the default set of parameters for the process
vpro_parameters_sptr
vil_monotone_process::factory::default_params() const
{
  vpro_parameters_sptr p = new vpro_parameters();
  if(p->add( "Red Weight" ,   "rw",  0.2125f ) &&
     p->add( "Green Weight",  "gw",  0.7154f ) &&
     p->add( "Blue Weight",   "bw",  0.0721f ) )
    return p;

  vcl_cerr << "ERROR: Adding parameters in " __FILE__ << vcl_endl;
  return NULL;
}


//: Construct a process from a set of parameters
vpro_process_sptr
vil_monotone_process::factory::create(const vpro_parameters_sptr& params) const
{
  float rw=0,gw=0,bw=0;
  if( !params->get_value( "rw" , rw ) ||
      !params->get_value( "gw" , gw ) ||
      !params->get_value( "bw" , bw ) ){
    return NULL;
  }

  return new vil_monotone_process(rw,gw,bw);
}



//: Execute this process
vpro_signal
vil_monotone_process::execute()
{
  assert(input_type_id(0) == typeid(vil_image_resource_sptr));
  vil_image_resource_sptr in_img = input<vil_image_resource_sptr>(0);

  vil_image_view_base_sptr grey_view;
  grey_view = vil_convert_to_grey_using_rgb_weighting(rw_,gw_,bw_, in_img->get_view());
  vil_image_resource_sptr out_img = vil_new_image_resource_of_view(*grey_view);

  output(0,out_img);

  return VPRO_VALID;
}

