// This is spl/spl_process_factory.h
#ifndef spl_process_factory_h_
#define spl_process_factory_h_
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

#include <spl/spl_parameters_sptr.h>
#include <spl/spl_process_sptr.h>


//: A factory for creating processes from names and params
class spl_process_factory
{
  public:
  //: Return the default set of parameters for the process
  virtual spl_parameters_sptr default_params() const = 0;

  //: Construct a process from a set of parameters
  virtual spl_process_sptr create(const spl_parameters_sptr& params) const = 0;

  //: The name of the process
  virtual vcl_string name() const = 0;

  typedef vcl_map<vcl_string, const spl_process_factory*> Reg_Type;

  //: Return a const reference to the global registry of storage classes
  static Reg_Type const & registry() { return mut_registry(); }

  //: Return the default parameters for a process with the given name
  static spl_parameters_sptr default_params(const vcl_string& name);

  //: Construct a process with the given name and parameters
  static spl_process_sptr create(const vcl_string& name,
                                   const spl_parameters_sptr& params);

  //: Create static instances of this struct to register a storage class
  struct registrar{
    registrar(const spl_process_factory* example);
  };

  friend struct registrar;


 private:
  //: Return a reference to the global registry of storage classes
  static Reg_Type & mut_registry();

 protected:
  //: Copy Constructor
  spl_process_factory(const spl_process_factory& /*other*/) {}
  spl_process_factory() {}
  virtual ~spl_process_factory() {}
};



#endif // spl_process_factory_h_
