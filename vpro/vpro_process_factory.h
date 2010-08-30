// This is vpro/vpro_process_factory.h
#ifndef vpro_process_factory_h_
#define vpro_process_factory_h_
//--------------------------------------------------------------------------------
//:
// \file
// \brief A factory for creating processes from names and params
//
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 5/31/06
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

#include <vcl_set.h>
#include <vcl_map.h>
#include <vcl_string.h>
#include <vbl/vbl_ref_count.h>

#include <vpro/vpro_parameters_sptr.h>
#include <vpro/vpro_process_sptr.h>


//: A factory for creating processes from names and params
class vpro_process_factory
{
  public:
  //: Return the default set of parameters for the process
  virtual vpro_parameters_sptr default_params() const = 0;

  //: Construct a process from a set of parameters
  virtual vpro_process_sptr create(const vpro_parameters_sptr& params) const = 0;

  //: The name of the process
  virtual vcl_string name() const = 0;

  typedef vcl_map<vcl_string, const vpro_process_factory*> Reg_Type;

  //: Return a const reference to the global registry of storage classes
  static Reg_Type const & registry() { return mut_registry(); }

  //: Return the default parameters for a process with the given name
  static vpro_parameters_sptr default_params(const vcl_string& name);

  //: Construct a process with the given name and parameters
  static vpro_process_sptr create(const vcl_string& name,
                                   const vpro_parameters_sptr& params);

  //: Create static instances of this struct to register a storage class
  struct registrar{
    registrar(const vpro_process_factory* example);
  };

  friend struct registrar;


 private:
  //: Return a reference to the global registry of storage classes
  static Reg_Type & mut_registry();

 protected:
  //: Copy Constructor
  vpro_process_factory(const vpro_process_factory& /*other*/) {}
  vpro_process_factory() {}
  virtual ~vpro_process_factory() {}
};



#endif // vpro_process_factory_h_
