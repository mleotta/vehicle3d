// This is spl/tests/spl_sample_processes.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "spl_sample_processes.h"
#include <vcl_iostream.h>
#include <spl/spl_parameters.h>
#include <vxl_config.h>



//: Return the default set of parameters for the process
template <class T>
spl_parameters_sptr
spl_sum_factory<T>::default_params() const
{
  // no parameters to add
  spl_parameters_sptr p = new spl_parameters();
  return p;
}


//: Construct a process from a set of parameters
template <class T>
spl_process_sptr
spl_sum_factory<T>::create(const spl_parameters_sptr& params) const
{
  return new spl_sum<T>;
}


spl_process_factory::registrar sample_reglist[] =
{
  new spl_sum_factory<int>,
  new spl_sum_factory<unsigned int>,
  new spl_sum_factory<vxl_byte>,
  new spl_sum_factory<vxl_uint_16>
};
