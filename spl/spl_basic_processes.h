// This is spl/spl_basic_processes.h
#ifndef spl_basic_processes_h_
#define spl_basic_processes_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief Basic processes that use standard types or are templated
//
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 6/07/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
//
// \verbatim
//  Modifications:
// \endverbatim
//--------------------------------------------------------------------------------


#include <spl/spl_process.h>
#include <vcl_vector.h>

//=============================================================================
// Filters
//=============================================================================


//: Wrap a function (or functor) with one argument
template <class _functor, class T1 >
class spl_functor1_filter : public spl_filter
{
 public:
  spl_functor1_filter(const _functor& fun) : f(fun) {}
  //: Execute the process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(T1));
    output(0, f(input<T1>(0)));
    return SPL_VALID;
  }
  _functor f;
};


//: Wrap a function (or functor) with two arguments
template <class _functor, class T1, class T2 >
class spl_functor2_filter : public spl_filter
{
 public:
  spl_functor2_filter(const _functor& fun) : f(fun) {}
  //: Execute the process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(T1));
    assert(input_type_id(1) == typeid(T2));
    output(0, f(input<T1>(0),input<T2>(1)));
    return SPL_VALID;
  }
  _functor f;
};


//: Wrap a function (or functor) with three arguments
template <class _functor, class T1, class T2, class T3 >
class spl_functor3_filter : public spl_filter
{
 public:
  spl_functor3_filter(const _functor& fun) : f(fun) {}
  //: Execute the process
  spl_signal execute()
  {
    assert(input_type_id(0) == typeid(T1));
    assert(input_type_id(1) == typeid(T2));
    assert(input_type_id(2) == typeid(T3));
    output(0, f(input<T1>(0),input<T2>(1),input<T3>(2)));
    return SPL_VALID;
  }
  _functor f;
};


//: Static cast from \c inT to \c outT
template <class inT, class outT>
class spl_static_cast_filter : public spl_filter
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



//=============================================================================
// Sources
//=============================================================================


//: Wrap a function (or functor) with no arguments
template <class _functor >
class spl_functor_source : public spl_source
{
 public:
  spl_functor_source(const _functor& fun) : f(fun) {}

  //: Execute the process
  spl_signal execute()
  {
    output(0, f());
    return SPL_VALID;
  }
  _functor f;
};


//: A source that always provides the same data
// (unless modified explictly)
template <class T>
class spl_static_source : public spl_source
{
  public:
    spl_static_source() {}
    spl_static_source(const T& d) : data(d) {}
    
    //: Execute the process
    spl_signal execute()
    {
      output(0, data);
      return SPL_VALID;
    }
    
    T data;
};


template <class T>
class spl_input_queue : public spl_source
{
 public:
  spl_input_queue(const vcl_vector<T>& d) : data(d) {}

  //: Execute the process
  spl_signal execute()
  {
    if(data.empty()){
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


//: A sink that does nothing
// helps pump data to other observers
class spl_null_sink : public spl_sink
{
 public:
  //: Execute the process
  spl_signal execute() { return SPL_VALID; }
};



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



#endif // spl_basic_processes_h_
