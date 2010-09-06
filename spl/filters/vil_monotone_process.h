// This is spl/filters/vil_monotone_process.h
#ifndef vil_monotone_process_h_
#define vil_monotone_process_h_

//:
// \file
// \brief Process that converts an image into monotone (grey-scale)
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 12/21/04
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
#include <spl/spl_process.h>
#include <spl/spl_process_factory.h>

//: Process that converts an image into monotone (grey-scale)
class vil_monotone_process : public spl_filter
{
public:
  //: Constructor
  vil_monotone_process(float rw, float gw, float bw)
   : rw_(rw), gw_(gw), bw_(bw) {}
  //: Destructor
  virtual ~vil_monotone_process() {}

  class factory : public spl_process_factory
  {
   public:
    //: Return the default set of parameters for the process
    virtual spl_parameters_sptr default_params() const;

    //: Construct a process from a set of parameters
    virtual spl_process_sptr create(const spl_parameters_sptr& params) const;

    //: The name of the process
    virtual vcl_string name() const { return "Convert to Monotone"; }

    virtual ~factory() {}
  };

  //: Execute this process
  spl_signal execute();

 private:
  //: The weights for combining color channels
  float rw_, gw_, bw_;
};

#endif // vil_monotone_process_h_
