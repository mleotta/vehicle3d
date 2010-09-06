// This is spl/filters/vil_morphology_filters.h
#ifndef vil_morphology_filters_h_
#define vil_morphology_filters_h_

//:
// \file
// \brief Filters for morphological operations
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 6/12/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <spl/spl_process.h>
#include <vil/algo/vil_structuring_element.h>


//: Filter for binary erosion
class vil_binary_erode_filter : public spl_filter
{
  public:
    //: Constructor
    vil_binary_erode_filter(const vil_structuring_element& s) : se_(s) {}
    //: Destructor
    virtual ~vil_binary_erode_filter() {}

    //: Execute this process
    spl_signal execute();

  private:
    vil_structuring_element se_;

};


//: Filter for binary dilation
class vil_binary_dilate_filter : public spl_filter
{
  public:
    //: Constructor
    vil_binary_dilate_filter(const vil_structuring_element& s) : se_(s) {}
    //: Destructor
    virtual ~vil_binary_dilate_filter() {}

    //: Execute this process
    spl_signal execute();

  private:
    vil_structuring_element se_;

};

#endif
