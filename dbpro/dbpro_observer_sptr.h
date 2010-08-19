// This is dbpro/dbpro_observer_sptr.h
#ifndef dbpro_observer_sptr_h
#define dbpro_observer_sptr_h
//--------------------------------------------------------------------------------
//:
// \file
// \brief Defines a smart pointer for observers.
//
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 5/30/06
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

class dbpro_observer;
#include <vbl/vbl_smart_ptr.h>

struct dbpro_observer_sptr : public vbl_smart_ptr<dbpro_observer> {
  typedef vbl_smart_ptr<dbpro_observer> base;

  dbpro_observer_sptr() {}
  dbpro_observer_sptr(dbpro_observer* p): base(p) {}
  void vertical_cast(dbpro_observer_sptr const& that) { *this = that; }
  void vertical_cast(dbpro_observer* t) { *this = t; }
};

// Stop doxygen documenting the B class
#ifndef DOXYGEN_SHOULD_SKIP_THIS
template <class T, class B = dbpro_observer_sptr>
struct dbpro_observer_sptr_t : public B {
  dbpro_observer_sptr_t(): B() {}
  dbpro_observer_sptr_t(T* p): B(p) {}
  dbpro_observer_sptr_t(dbpro_observer_sptr_t<T> const& r): B(r) {}
  void operator=(dbpro_observer_sptr_t<T> const& r) { B::operator=(r); }
  T* operator->() const { return (T*)this->as_pointer(); }
  T& operator*() const { return *((T*)this->as_pointer()); }
};
#endif // DOXYGEN_SHOULD_SKIP_THIS


#endif // dbpro_observer_sptr_h
