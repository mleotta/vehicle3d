// This is dbpro/tests/dbpro_sample_processes.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "dbpro_sample_processes.h"
#include <vcl_iostream.h>
#include <dbpro/dbpro_parameters.h>
#include <vxl_config.h>



//: Return the default set of parameters for the process
template <class T>
dbpro_parameters_sptr
dbpro_sum_factory<T>::default_params() const
{
  // no parameters to add
  dbpro_parameters_sptr p = new dbpro_parameters();
  return p;
}


//: Construct a process from a set of parameters
template <class T>
dbpro_process_sptr
dbpro_sum_factory<T>::create(const dbpro_parameters_sptr& params) const
{
  return new dbpro_sum<T>;
}


dbpro_process_factory::registrar sample_reglist[] =
{
  new dbpro_sum_factory<int>,
  new dbpro_sum_factory<unsigned int>,
  new dbpro_sum_factory<vxl_byte>,
  new dbpro_sum_factory<vxl_uint_16>
};
