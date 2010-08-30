// This is vpro/vpro_parameters.h
#ifndef vpro_parameters_h_
#define vpro_parameters_h_
//:
// \file
// \brief classes to represent parameters to vpro processes
// \author Matt Leotta, (mleotta@lems.brown.edu)
// \date 7/1/2004
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
//    Matt Leotta  12/15/04  Migrated from vidpro
// \endverbatim

#include <vcl_string.h>
#include <vcl_sstream.h>
#include <vcl_iostream.h>
#include <vcl_cassert.h>
#include <vcl_vector.h>
#include <vcl_map.h>

#include <vbl/vbl_ref_count.h>

#include <vpro/vpro_parameters_sptr.h>


//: The abstract base class for a parameter
class vpro_param
{
 public:

  //: Destructor
  virtual ~vpro_param() {}

  //: Clone this parameter
  virtual vpro_param * clone() const = 0;

  //: Return the parameter name
  vcl_string name() const { return name_; }
  //: Return the parameter description
  vcl_string description() const { return description_; }
  //: Returns true if the valid range of parameter values is bounded
  bool has_bounds() const { return has_bounds_; }

  //: Reset the value to its default
  virtual void reset() = 0;
  //: Attempt to set the value from the temporary reference
  virtual bool set_from_temp() = 0;

  //: Return a string representation of the current value
  virtual vcl_string value_str() const = 0;
  //: Return a string representation of the default value
  virtual vcl_string default_str() const = 0;
  //: Return a string representation of the minimum value
  virtual vcl_string min_str() const = 0;
  //: Return a string representation of the maximium value
  virtual vcl_string max_str() const = 0;

  //: Set the current value by parsing a string
  virtual bool parse_value_str(const vcl_string& input) = 0;

 protected:
  //: Constructor
  vpro_param(bool has_bounds, const vcl_string& name, const vcl_string& desc)
   : has_bounds_(has_bounds), name_(name), description_(desc) {}


  //: Describes whether or not the parameter has bounds
  const bool has_bounds_;
  //: Name of the parameter
  const vcl_string name_;
  //: Description of the parameter
  const vcl_string description_;
};

//: Output stream operator for vpro_params
vcl_ostream& operator<<(vcl_ostream& os, const vpro_param& p);

//===========================================================================================

//: A Templated parameter class
template< class T >
class vpro_param_type : public vpro_param
{
 public:
  // Constructor - with bounds
  vpro_param_type<T>(const vcl_string& name, const vcl_string& desc, const T& dflt, const T& min, const T& max)
   : vpro_param(true, name, desc), value_(dflt), default_(dflt), temp_value_(dflt),
     min_value_(min), max_value_(max) { assert( value_ < max_value_ &&  min_value_ < value_ ); }

  // Constructor - without bounds
  vpro_param_type<T>(const vcl_string& name, const vcl_string& desc, const T& dflt)
   : vpro_param(false, name, desc), value_(dflt), default_(dflt), temp_value_(dflt),
     min_value_(dflt), max_value_(dflt) {}

  //: Accessor for the default value;
  T default_value() const { return default_; }
  //: Accessor for the default value;
  T min_value() const { return min_value_; }
  //: Accessor for the default value;
  T max_value() const { return max_value_; }
  
  //: Accessor for the current value;
  T value() const { return value_; }
  //: A reference for temporary storage of values
  T& temp_ref() { temp_value_ = value_; return temp_value_; }
  //: Attempt to set the value from the temporary reference
  bool set_from_temp() { return set_value(temp_value_); }
  //: Set the current value to \p val
  bool set_value( const T& val );

  //: Reset the value to its default
  virtual void reset() { value_ = default_; }

  //: Clone the parameter
  virtual vpro_param * clone() const { return new vpro_param_type<T>(*this); }

  //: Return a string representation of the current value
  virtual vcl_string value_str() const { return create_string(value_); }
  //: Return a string representation of the default value
  virtual vcl_string default_str() const { return create_string(default_); }
  //: Return a string representation of the minimum value
  virtual vcl_string min_str() const { return has_bounds_? create_string(min_value_) : ""; }
  //: Return a string representation of the maximium value
  virtual vcl_string max_str() const { return has_bounds_? create_string(max_value_) : ""; }

  //: Set the current value by parsing a string
  virtual bool parse_value_str(const vcl_string& input) { return set_value(parse_string(input)); }

 private:
  //: Create a string representation of the value
  vcl_string create_string(const T& val) const;

  //: Parse a string representation of the value
  T parse_string(const vcl_string& input) const;

  //: The current parameter value
  T value_;
  //: The default parameter value
  const T default_;
  //: A temporary value for assignments by reference
  T temp_value_;
  //: The minimum allowed parameter value
  const T min_value_;
  //: The maximum allowed parameter value
  const T max_value_;
};

//===========================================================================================

//: This class maintains all parameters for a process
class vpro_parameters : public vbl_ref_count
{
 public:

  //: Constructor
  vpro_parameters();
  //: Destructor
  ~vpro_parameters();

  //: Deep psuedo copy constructor
  vpro_parameters( const vpro_parameters_sptr& old_params);

  //: Returns true if a parameter exists with \p name
  bool valid_parameter( const vcl_string& name ) const;

  //: Returns true if a parameter exists with \p name and type \p T
  template<class T>
  bool valid_parameter_type( const vcl_string& name, const T&) const
  {
    vcl_map< vcl_string, vpro_param* >::const_iterator 
      itr = name_param_map_.find( name );
    if( itr == name_param_map_.end() ) {
      return false; // Not Found
    }
    return (dynamic_cast<vpro_param_type<T> *>(itr->second) != NULL);
  }

  //: Add a new parameter with no bounds
  template<class T>
  bool add( const vcl_string& desc, const vcl_string& name, const T& default_val )
  { return add(new vpro_param_type<T>(name, desc, default_val)); }

  //: Add a new parameter with bounds
  template<class T>
  bool add( const vcl_string& desc, const vcl_string& name, const T& default_val,
            const T& min_val, const T& max_val )
  { return add(new vpro_param_type<T>(name, desc, default_val, min_val, max_val)); }

  //: Set the value of the existing parameter named \p name
  template<class T>
  bool set_value( const vcl_string& name , const T& value )
  {
    vpro_param_type<T> * param = NULL;
    if( get_param(name, param) && param ){
      return param->set_value(value);
    }
    return false;
  }

  //: Return the value of the parameter named \p name by reference
  template<class T>
  bool get_value( const vcl_string& name , T& value ) const
  {
    vpro_param_type<T> * param = NULL;
    if( get_param(name, param) && param ){
      value = param->value();
      return true;
    }
    return false;
  }

  //: Return the default value of the parameter named \p name by reference
  template<class T>
  bool get_default( const vcl_string& name , T& deflt ) const
  {
    vpro_param_type<T> * param = NULL;
    if( get_param(name, param) && param ){
      deflt = param->default_value();
      return true;
    }
    return false;
  }

  //: Return the bounds of the parameter named \p name by reference
  template<class T>
  bool get_bounds( const vcl_string& name, T & min, T & max ) const
  {
    vpro_param_type<T> * param = NULL;
    if( get_param(name, param) && param ){
      min = param->min_value();
      max = param->max_value();
      return true;
    }
    return false;
  }

  //: Reset all parameters to their default values
  bool reset_all();
  //: Reset the parameter named \p name to its default value
  bool reset( const vcl_string& name );

  //: Return a vector of base class pointers to the parameters
  vcl_vector< vpro_param* > get_param_list() const;
  //: Return the description of the parameter named \p name
  vcl_string get_desc( const vcl_string& name ) const;
  //: Print all parameters to \p os
  void print_all(vcl_ostream& os) const;
  
 private:
  //: Add parameter helper function
  bool add( vpro_param* param );

  template<class T>
  bool get_param( const vcl_string& name, 
                  vpro_param_type<T> * &param) const
  {
    vcl_map< vcl_string, vpro_param* >::const_iterator 
      itr = name_param_map_.find( name );
    if( itr == name_param_map_.end() ) {
      return false; // Not Found
    }
    param = dynamic_cast<vpro_param_type<T> *>(itr->second);
    if( !param )
      vcl_cerr << "WARNING: parameter \""<< name 
               << "\" was found but has incorrect type" << vcl_endl;
    return true;
  }

  //: The map from names to parameters
  vcl_map< vcl_string , vpro_param* > name_param_map_;
  //: The vector of parameters in order of declaration
  vcl_vector< vpro_param* > param_list_;
};


//===========================================================================================


//: A simple class to represent a file (for use with parameters)
class vpro_filepath
{
 public:
  //: Constructor
  vpro_filepath(const vcl_string& p = "", const vcl_string& e = "*")
   : path(p), ext(e) {}

  vcl_string path;
  vcl_string ext;
};

//: Less than operator for vpro_filepath objects
bool operator<( const vpro_filepath& lhs, const vpro_filepath& rhs );
//: Output stream operator for vpro_filepath objects
vcl_ostream& operator<<( vcl_ostream& strm, const vpro_filepath& fp );
//: Input stream operator for vpro_filepath objects
vcl_istream& operator>>( vcl_istream& strm, const vpro_filepath& fp );


#endif // vpro_parameters_h_
