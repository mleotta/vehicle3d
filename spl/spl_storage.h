// This is spl/spl_storage.h
#ifndef spl_storage_h_
#define spl_storage_h_

//:
// \file
// \brief The storage class
// \author Matt Leotta
// \date 5/26/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim

#include <vcl_typeinfo.h>
#include <vcl_cassert.h>
#include <vcl_iostream.h>
#include <vbl/vbl_ref_count.h>
#include "spl_fwd.h"
#include "spl_storage_sptr.h"

template< class T > class spl_storage_type;

//: This abstract class is the base class for storage objects
class spl_storage : public vbl_ref_count
{
 public:
  //: Constructor
  spl_storage() {}

  //: Destructor
  virtual ~spl_storage() {}

  //: Return a signal (SPL_VALID for valid data)
  virtual spl_signal info() const = 0;

  //: Return the type of the stored data
  virtual const vcl_type_info& type_id() const = 0;

  //: Returns true if a parameter exists with \p name and type \p T
  template<class T>
  const T& data() const
  {
    return static_cast<const spl_storage_type<T>*>(this)->data();
  }

 private:
  spl_storage(const spl_storage& other);

};


//=============================================================================


//: A Templated parameter class
template< class T >
class spl_storage_type : public spl_storage
{
 public:
  //: Default Constructor
  spl_storage_type<T>() {}

  //: Constructor
  spl_storage_type<T>(const T& data)
   : data_(data) {}

  //: Return a message flag
  spl_signal info() const { return SPL_VALID; }

  //: Return a const reference to the data
  const T& data() const { return data_; }

  //: Return the type of the stored data
  const vcl_type_info& type_id() const { return typeid(T); }

 private:
  //: The stored data
  T data_;
};


//=============================================================================

typedef spl_storage_type<spl_signal> spl_storage_signal;

//: A storage class that contains only a message
VCL_DEFINE_SPECIALIZATION
class spl_storage_type<spl_signal> : public spl_storage
{
 public:
  //: Default Constructor
  spl_storage_type<spl_signal>(spl_signal info=SPL_INVALID) : info_(info) {}

  //: Return a message flag
  spl_signal info() const { return info_; }

  //: Return the type of the stored data
  const vcl_type_info& type_id() const { return typeid(spl_signal); }

  private:
  //: The message
  spl_signal info_;
};


#endif // spl_storage_h_
