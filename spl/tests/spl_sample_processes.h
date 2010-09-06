// This is spl/tests/spl_sample_processes.h
#ifndef spl_sample_processes_h_
#define spl_sample_processes_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief Sample processes used in test cases
//
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 5/31/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications:
// \endverbatim
//--------------------------------------------------------------------------------


#include <spl/spl_process.h>
#include <spl/spl_process_factory.h>
#include <vcl_cmath.h>
#include <vcl_vector.h>

//=============================================================================
// Filters
//=============================================================================

template <class T>
class spl_sum : public spl_filter
{
 public:
  //: Execute the process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(T));
    assert(input_type_id(1) == typeid(T));
    T val1 = input<T>(0);
    T val2 = input<T>(1);
    output(0, T(val1+val2));
    return SPL_VALID;
  }
};


template <class T>
class spl_diff : public spl_filter
{
 public:
  //: Execute the process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(T));
    assert(input_type_id(1) == typeid(T));
    T val1 = input<T>(0);
    T val2 = input<T>(1);
    output(0, T(val1-val2));
    return SPL_VALID;
  }
};


template <class inT, class outT>
class spl_static_cast : public spl_filter
{
 public:
  //: Execute the process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(inT));
    output(0, static_cast<outT>(input<inT>(0)));
    return SPL_VALID;
  }
};



class spl_sqrt : public spl_filter
{
  public:
  //: Execute the process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(double));
    output(0, vcl_sqrt(input<double>(0)));
    return SPL_VALID;
  }
};


//=============================================================================
// Sources
//=============================================================================


template <class T>
class spl_input_queue : public spl_source
{
 public:
  spl_input_queue(const vcl_vector<T>& d) : data(d) {}

  //: Execute the process
  spl_signal execute()
  {
    if(data.empty()){
      output(0, SPL_EOS);
      return SPL_EOS;
    }

    output(0, data.back());
    data.pop_back();
    return SPL_VALID;
  }

  vcl_vector<T> data;
};


//=============================================================================
// Sinks
//=============================================================================


template <class T>
class spl_output_queue : public spl_sink
{
 public:
  spl_output_queue() : data() {}

  //: Execute the process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(T));
    data.push_back(input<T>(0));
    return SPL_VALID;
  }

  vcl_vector<T> data;
};


//=============================================================================
// Factories
//=============================================================================

template <class T>
class spl_sum_factory : public spl_process_factory
{
  public:
    //: Return the default set of parameters for the process
    virtual spl_parameters_sptr default_params() const;

    //: Construct a process from a set of parameters
    virtual spl_process_sptr create(const spl_parameters_sptr& params) const;

    //: The name of the process
    virtual vcl_string name() const { return vcl_string("Sum<")+typeid(T).name()+">"; }

    virtual ~spl_sum_factory() {}
};

#endif // spl_sample_processes_h_
