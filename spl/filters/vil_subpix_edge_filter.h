// This is spl/filters/vil_subpix_edge_filter.h
#ifndef vil_subpix_edge_filter_h_
#define vil_subpix_edge_filter_h_

//:
// \file
// \brief Suppress all non-maximal edges and estimate subpixel locations
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 5/8/09
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim

#include <vil/vil_new.h>
#include <vil/vil_plane.h>
#include <vil/algo/vil_suppress_non_max_edges.h>
#include <spl/spl_process.h>

//: Suppress all non-maximal edges and estimate subpixel locations
template <class srcT, class destT>
class vil_subpix_edge_filter : public spl_filter
{
public:
  //: Constructor
  vil_subpix_edge_filter(double mag_thresh=2.0, bool reuse_output=true) 
  : mag_thresh_(mag_thresh), reuse_output_(reuse_output) {}
  
  //: Execute this process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(vil_image_resource_sptr));
    vil_image_resource_sptr in_img = input<vil_image_resource_sptr>(0);
    vil_image_view<srcT> image = in_img->get_view();
    vil_image_view<srcT> grad_i = vil_plane(image,0);
    vil_image_view<srcT> grad_j = vil_plane(image,1);
    
    if(!reuse_output_)
      grad_moo_ = vil_image_view<destT>();
    vil_suppress_non_max_edges_subpixel(grad_i,grad_j,mag_thresh_,grad_moo_);
    output(0, vil_new_image_resource_of_view(grad_moo_));
    return SPL_VALID;
  }

  double mag_thresh_;
  bool reuse_output_;
  vil_image_view<destT> grad_moo_;

};


#endif // vil_subpix_edge_filter_h_
