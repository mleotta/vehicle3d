// This is vpro/tests/vpro_sample_processes.h
#ifndef vpro_sample_processes_h_
#define vpro_sample_processes_h_
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


#include <vpro/vpro_process.h>
#include <vpro/vpro_process_factory.h>
#include <vcl_cmath.h>
#include <vcl_vector.h>

//=============================================================================
// Filters
//=============================================================================

template <class T>
class vpro_sum : public vpro_filter
{
 public:
  //: Execute the process
  vpro_signal execute()
  {
    assert(input_type_id(0) == typeid(T));
    assert(input_type_id(1) == typeid(T));
    T val1 = input<T>(0);
    T val2 = input<T>(1);
    output(0, T(val1+val2));
    return VPRO_VALID;
  }
};


template <class T>
class vpro_diff : public vpro_filter
{
 public:
  //: Execute the process
  vpro_signal execute()
  {
    assert(input_type_id(0) == typeid(T));
    assert(input_type_id(1) == typeid(T));
    T val1 = input<T>(0);
    T val2 = input<T>(1);
    output(0, T(val1-val2));
    return VPRO_VALID;
  }
};


template <class inT, class outT>
class vpro_static_cast : public vpro_filter
{
 public:
  //: Execute the process
  vpro_signal execute()
  {
    assert(input_type_id(0) == typeid(inT));
    output(0, static_cast<outT>(input<inT>(0)));
    return VPRO_VALID;
  }
};



class vpro_sqrt : public vpro_filter
{
  public:
  //: Execute the process
  vpro_signal execute()
  {
    assert(input_type_id(0) == typeid(double));
    output(0, vcl_sqrt(input<double>(0)));
    return VPRO_VALID;
  }
};


//=============================================================================
// Sources
//=============================================================================


template <class T>
class vpro_input_queue : public vpro_source
{
 public:
  vpro_input_queue(const vcl_vector<T>& d) : data(d) {}

  //: Execute the process
  vpro_signal execute()
  {
    if(data.empty()){
      output(0, VPRO_EOS);
      return VPRO_EOS;
    }

    output(0, data.back());
    data.pop_back();
    return VPRO_VALID;
  }

  vcl_vector<T> data;
};


//=============================================================================
// Sinks
//=============================================================================


template <class T>
class vpro_output_queue : public vpro_sink
{
 public:
  vpro_output_queue() : data() {}

  //: Execute the process
  vpro_signal execute()
  {
    assert(input_type_id(0) == typeid(T));
    data.push_back(input<T>(0));
    return VPRO_VALID;
  }

  vcl_vector<T> data;
};


//=============================================================================
// Factories
//=============================================================================

template <class T>
class vpro_sum_factory : public vpro_process_factory
{
  public:
    //: Return the default set of parameters for the process
    virtual vpro_parameters_sptr default_params() const;

    //: Construct a process from a set of parameters
    virtual vpro_process_sptr create(const vpro_parameters_sptr& params) const;

    //: The name of the process
    virtual vcl_string name() const { return vcl_string("Sum<")+typeid(T).name()+">"; }

    virtual ~vpro_sum_factory() {}
};

#endif // vpro_sample_processes_h_
