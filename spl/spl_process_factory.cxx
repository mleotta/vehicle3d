// This is spl/spl_process_factory.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include <spl/spl_process_factory.h>
#include <vcl_iostream.h>



//: Return a reference to the global registry of storage classes
spl_process_factory::Reg_Type &
spl_process_factory::mut_registry()
{
  static Reg_Type reg;
  return reg;
}


//: The Constructor for this struct registers an exemplar storage object with the base class
spl_process_factory::registrar::registrar(const spl_process_factory* example)
{
  vcl_string name = example->name();
  // register with the base class
  Reg_Type& reg = spl_process_factory::mut_registry();
  Reg_Type::iterator result = reg.find(name);
  if(result != reg.end()){
    delete result->second;
    vcl_cerr << "Warning multiple processes registered with the name: "<< name <<vcl_endl;
  }

  reg[name] = example;
}


//: Return the default parameters for a process with the given name
spl_parameters_sptr
spl_process_factory::default_params(const vcl_string& name)
{
  const Reg_Type& reg = spl_process_factory::registry();
  Reg_Type::const_iterator result = reg.find(name);
  if(result == reg.end())
    return NULL;

  return result->second->default_params();
}


//: Construct a process with the given name and parameters
spl_process_sptr
spl_process_factory::create(const vcl_string& name,
                              const spl_parameters_sptr& params)
{
  const Reg_Type& reg = spl_process_factory::registry();
  Reg_Type::const_iterator result = reg.find(name);
  if(result == reg.end())
    return NULL;

  return result->second->create(params);
}
