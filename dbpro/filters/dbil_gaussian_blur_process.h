// This is dbpro/filters/dbil_gaussian_blur_process.h
#ifndef dbil_gaussian_blur_process_h_
#define dbil_gaussian_blur_process_h_

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
#include <dbpro/dbpro_process.h>
#include <dbpro/dbpro_process_factory.h>


class dbil_gaussian_blur_process : public dbpro_filter
{
 public:
  class factory : public dbpro_process_factory
  {
   public:
    //: Return the default set of parameters for the process
    virtual dbpro_parameters_sptr default_params() const;

    //: Construct a process from a set of parameters
    virtual dbpro_process_sptr create(const dbpro_parameters_sptr& params) const;

    //: The name of the process
    virtual vcl_string name() const { return "Gaussian Blur"; }

    virtual ~factory() {}
  };

  //: Constructor
  dbil_gaussian_blur_process(float sigma) : sigma_(sigma) {}

  //: Destructor
  virtual ~dbil_gaussian_blur_process(){}


  //: Execute this process
  dbpro_signal execute();

 private:
  float sigma_;
};

#endif // dbil_gaussian_blur_process_h_
