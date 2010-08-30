// This is vpro/filters/vil_gaussian_blur_process.h
#ifndef vil_gaussian_blur_process_h_
#define vil_gaussian_blur_process_h_

//:
// \file
// \brief A process that smooths an image with a Gaussian filter
// \author Matt Leotta
// \date 12/20/04
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <vcl_vector.h>
#include <vcl_string.h>
#include <vpro/vpro_process.h>
#include <vpro/vpro_process_factory.h>


class vil_gaussian_blur_process : public vpro_filter
{
 public:
  class factory : public vpro_process_factory
  {
   public:
    //: Return the default set of parameters for the process
    virtual vpro_parameters_sptr default_params() const;

    //: Construct a process from a set of parameters
    virtual vpro_process_sptr create(const vpro_parameters_sptr& params) const;

    //: The name of the process
    virtual vcl_string name() const { return "Gaussian Blur"; }

    virtual ~factory() {}
  };

  //: Constructor
  vil_gaussian_blur_process(float sigma) : sigma_(sigma) {}

  //: Destructor
  virtual ~vil_gaussian_blur_process(){}


  //: Execute this process
  vpro_signal execute();

 private:
  float sigma_;
};

#endif // vil_gaussian_blur_process_h_
