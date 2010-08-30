// This is vpro/tests/vpro_sample_processes.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vpro_sample_processes.h"
#include <vcl_iostream.h>
#include <vpro/vpro_parameters.h>
#include <vxl_config.h>



//: Return the default set of parameters for the process
template <class T>
vpro_parameters_sptr
vpro_sum_factory<T>::default_params() const
{
  // no parameters to add
  vpro_parameters_sptr p = new vpro_parameters();
  return p;
}


//: Construct a process from a set of parameters
template <class T>
vpro_process_sptr
vpro_sum_factory<T>::create(const vpro_parameters_sptr& params) const
{
  return new vpro_sum<T>;
}


vpro_process_factory::registrar sample_reglist[] =
{
  new vpro_sum_factory<int>,
  new vpro_sum_factory<unsigned int>,
  new vpro_sum_factory<vxl_byte>,
  new vpro_sum_factory<vxl_uint_16>
};
