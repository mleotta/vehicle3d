// This is spl/filters/vil_math_filters.h
#ifndef vil_math_filters_h_
#define vil_math_filters_h_

//:
// \file
// \brief Filters to apply vil_math operations
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 6/9/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim

#include <vil/vil_math.h>
#include <vil/vil_new.h>
#include <spl/spl_process.h>

//: Filter to cast the image to a new type
// Warning this filter modifies the input data
template <class T>
class vil_math_scale_filter : public spl_filter
{
  public:
    //: Constructor
    vil_math_scale_filter(double scale) : scale_(scale) {}
    //: Execute this process
    spl_signal execute()
    {
      assert(input_type_id(0) == typeid(vil_image_resource_sptr));
      vil_image_resource_sptr in_img = input<vil_image_resource_sptr>(0);
      vil_image_view<T> image = in_img->get_view();
      vil_math_scale_values(image,scale_);
      output(0, in_img);
      return SPL_VALID;
    }
  private:
    double scale_;

};


//: Filter to cast the image to a new type
// Warning this filter modifies the input data
template <class T>
class vil_math_scale_and_offset_filter : public spl_filter
{
  public:
    //: Constructor
    vil_math_scale_and_offset_filter(double scale, double offset)
      : scale_(scale), offset_(offset) {}
    //: Execute this process
    spl_signal execute()
    {
      assert(input_type_id(0) == typeid(vil_image_resource_sptr));
      vil_image_resource_sptr in_img = input<vil_image_resource_sptr>(0);
      vil_image_view<T> image = in_img->get_view();
      vil_math_scale_and_offset_values(image,scale_,offset_);
      output(0, in_img);
      return SPL_VALID;
    }
  private:
    double scale_;
    double offset_;

};

#endif // vil_math_filters_h_
