// This is vpro/vpro_parameters.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include <vpro/vpro_parameters.h>

#include <vcl_utility.h>
#include <vcl_iostream.h>
#include <vcl_sstream.h>


//: Output stream operator for vpro_params
vcl_ostream& operator<<(vcl_ostream& os, const vpro_param& p)
{
  os << "parameter{\n  Description: " << p.description();
  if(p.has_bounds())
    os << "\n  Range: " << p.min_str() << " to " << p.max_str();
  os << "\n  Default: " << p.default_str();
  os << "\n  Value: " << p.value_str() << "\n}\n";

  return os;
}

//===========================================================================================

//: Constructor
vpro_parameters::vpro_parameters()
{
}


//: Destructor
vpro_parameters::~vpro_parameters()
{
  for( vcl_vector< vpro_param * >::iterator it = param_list_.begin();
       it != param_list_.end();
       it++ ) {
    delete (*it);
  }
}

//: Deep copy constructor
vpro_parameters::vpro_parameters(const vpro_parameters_sptr& old_params)
{
  for( vcl_vector< vpro_param * >::iterator it = old_params->param_list_.begin();
      it != old_params->param_list_.end();
      it++ ) {

    //deep copy this param
    vpro_param * new_param = (*it)->clone();

    param_list_.push_back( new_param );
    name_param_map_.insert( vcl_pair< vcl_string , vpro_param* >( new_param->name() , new_param ) );
  }
}


//: Returns true if a parameter exists with \p flag
bool
vpro_parameters::valid_parameter( const vcl_string& name ) const
{
  vcl_map< vcl_string , vpro_param * >::const_iterator itr = name_param_map_.find( name );
  return (itr != name_param_map_.end());
}


//: Reset all parameters to their default values
bool
vpro_parameters::reset_all()
{
  for( vcl_vector< vpro_param * >::iterator it = param_list_.begin();
       it != param_list_.end();
       it++ ) {
    (*it)->reset();
  }
  return true;
}


//: Reset the parameter named \p name to its default value
bool
vpro_parameters::reset( const vcl_string& name )
{
  vcl_map< vcl_string , vpro_param * >::iterator it = name_param_map_.find( name );
  if( it == name_param_map_.end() ) {
    return false;
  }

  it->second->reset();

  return true;
}


//: Return a vector of base class pointers to the parameters
vcl_vector< vpro_param* >
vpro_parameters::get_param_list() const
{
  return param_list_;
}


//: Return the description of the parameter named \p name
vcl_string
vpro_parameters::get_desc( const vcl_string& name ) const
{
  vcl_map< vcl_string , vpro_param * >::const_iterator it = name_param_map_.find( name );
  if( it == name_param_map_.end() ) {
    return "";
  }
  return it->second->description();
}


//: Print all parameters to \p os
void
vpro_parameters::print_all(vcl_ostream& os) const
{
  for( vcl_vector< vpro_param * >::const_iterator it = param_list_.begin();
       it != param_list_.end();
       it++ ) {
    os << *it;
  }
}


//: Add parameter helper function
bool 
vpro_parameters::add( vpro_param* param )
{
  if( !param )
    return false;
  vcl_string name = param->name();
  vcl_string desc = param->description();
  if( name_param_map_.find( name ) != name_param_map_.end() ||
      desc == "" || name == "" ) {
    delete param;
    return false;
  }

  param_list_.push_back( param );
  name_param_map_.insert( vcl_pair< vcl_string , vpro_param* >( name , param ) );

  return true;
}

//===========================================================================================

//: Less than operator for vpro_filepath objects
bool operator<( const vpro_filepath& lhs, const vpro_filepath& rhs )
{
  return lhs.path < rhs.path;
}

//: Output stream operator for vpro_filepath objects
vcl_ostream& operator<<( vcl_ostream& strm, const vpro_filepath& fp )
{
  strm << fp.path << '\n' << fp.ext << vcl_ends;
  return strm;
}

//: Input stream operator for vpro_filepath objects
vcl_istream& operator>>( vcl_istream& strm, const vpro_filepath& fp )
{
  strm >> fp.path >> fp.ext;
  return strm;
}

