// This is vpro/vpro_vsl_processes.h
#ifndef vpro_vsl_processes_h_
#define vpro_vsl_processes_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief Basic sources and sinks using vsl binary IO
//
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 1/22/08
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications:
// \endverbatim
//--------------------------------------------------------------------------------


#include <vpro/vpro_process.h>
#include <vcl_vector.h>
#include <vcl_iostream.h>
#include <vsl/vsl_binary_io.h>
#include <vcl_memory.h>


//=============================================================================
// Sources
//=============================================================================


//: Read data from a (vsl) binary istream (base class)
class vpro_b_istream_source_base : public vpro_source
{
  public:
    vpro_b_istream_source_base(const vcl_string& filename) 
    : is(new vsl_b_ifstream(filename)) {}
    
    void open(const vcl_string& filename) 
    { is = vcl_auto_ptr<vsl_b_istream>(new vsl_b_ifstream(filename)); }
    
    //: Execute the process
    vpro_signal execute()
    {
      return VPRO_INVALID;
    }
    vcl_auto_ptr<vsl_b_istream> is;
};


//: Read data of type T from a (vsl) binary istream
template <class T >
class vpro_b_istream_source : public vpro_b_istream_source_base
{
 public:
  vpro_b_istream_source(const vcl_string& filename) 
  : vpro_b_istream_source_base(filename) {}

  //: Execute the process
  vpro_signal execute()
  {
    if(is->is().eof()){
      return VPRO_EOS;
    }
    T data;
    vsl_b_read(*is.get(),data);
    output(0, data);
    return VPRO_VALID;
  }
  
};


//=============================================================================
// Sinks
//=============================================================================


//: Write data to a (vsl) binary ostream (base class)
class vpro_b_ostream_sink_base : public vpro_sink
  {
  public:
    vpro_b_ostream_sink_base(const vcl_string& filename) 
    : os(new vsl_b_ofstream(filename)) {}
    
    void open(const vcl_string& filename) 
    { os = vcl_auto_ptr<vsl_b_ostream>(new vsl_b_ofstream(filename)); }
    
    bool enabled() const { return (os.get() != NULL && os->os().good()); }
    
    //: Execute the process
    vpro_signal execute()
    {
      return VPRO_INVALID;
    }
    vcl_auto_ptr<vsl_b_ostream> os;
  };


//: Write data of type T to a (vsl) binary ostream
template <class T >
class vpro_b_ostream_sink : public vpro_b_ostream_sink_base
  {
  public:
    vpro_b_ostream_sink(const vcl_string& filename) 
    : vpro_b_ostream_sink_base(filename) {}
    
    //: Execute the process
    vpro_signal execute()
    {
      assert(input_type_id(0) == typeid(T));
      vsl_b_write(*os.get(), input<T>(0));
      return VPRO_VALID;
    }
    
  };




#endif // vpro_vsl_processes_h_
