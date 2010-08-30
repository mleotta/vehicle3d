// This is vpro/vpro_storage.h
#ifndef vpro_storage_h_
#define vpro_storage_h_

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
#include "vpro_fwd.h"
#include "vpro_storage_sptr.h"

template< class T > class vpro_storage_type;

//: This abstract class is the base class for storage objects
class vpro_storage : public vbl_ref_count
{
 public:
  //: Constructor
  vpro_storage() {}

  //: Destructor
  virtual ~vpro_storage() {}

  //: Return a signal (VPRO_VALID for valid data)
  virtual vpro_signal info() const = 0;

  //: Return the type of the stored data
  virtual const vcl_type_info& type_id() const = 0;

  //: Returns true if a parameter exists with \p name and type \p T
  template<class T>
  const T& data() const
  {
    return static_cast<const vpro_storage_type<T>*>(this)->data();
  }

 private:
  vpro_storage(const vpro_storage& other);

};


//=============================================================================


//: A Templated parameter class
template< class T >
class vpro_storage_type : public vpro_storage
{
 public:
  //: Default Constructor
  vpro_storage_type<T>() {}

  //: Constructor
  vpro_storage_type<T>(const T& data)
   : data_(data) {}

  //: Return a message flag
  vpro_signal info() const { return VPRO_VALID; }

  //: Return a const reference to the data
  const T& data() const { return data_; }

  //: Return the type of the stored data
  const vcl_type_info& type_id() const { return typeid(T); }

 private:
  //: The stored data
  T data_;
};


//=============================================================================

typedef vpro_storage_type<vpro_signal> vpro_storage_signal;

//: A storage class that contains only a message
VCL_DEFINE_SPECIALIZATION
class vpro_storage_type<vpro_signal> : public vpro_storage
{
 public:
  //: Default Constructor
  vpro_storage_type<vpro_signal>(vpro_signal info=VPRO_INVALID) : info_(info) {}

  //: Return a message flag
  vpro_signal info() const { return info_; }

  //: Return the type of the stored data
  const vcl_type_info& type_id() const { return typeid(vpro_signal); }

  private:
  //: The message
  vpro_signal info_;
};


#endif // vpro_storage_h_
