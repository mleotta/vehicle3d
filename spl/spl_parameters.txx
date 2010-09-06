// This is spl/spl_parameters.txx
#ifndef spl_parameters_txx_
#define spl_parameters_txx_
//:
// \file
// \brief Templated code for spl parameters.
//
// \author Matt Leotta, (mleotta@lems.brown.edu)
// \date 7/2/2004
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//


#include "spl_parameters.h"
#include <vcl_utility.h>
#include <vcl_iostream.h>


//: Set the current value to \p val
template<class T>
bool
spl_param_type<T>::set_value( const T& val )
{
  if( has_bounds_ && (val < min_value_ || max_value_ < val) )
    return false;

  value_ = val;
  return true;
}


//: Create a string representation of the value
template<class T>
vcl_string
spl_param_type<T>::create_string(const T& val) const
{
  vcl_stringstream stm;
  stm << val;
  return stm.str();
}


//: Parse a string representation of the value
template<class T>
T
spl_param_type<T>::parse_string(const vcl_string& input) const
{
  T val;
  vcl_istringstream strm(input);
  strm >> val;
  return val;
}

//===========================================================================================

//: Use this macro to instantiate spl_parameters for each parameter type
#define SPL_PARAMETERS_INSTANTIATE(T) \
template class spl_param_type< T >;

#endif // spl_parameters_txx_
