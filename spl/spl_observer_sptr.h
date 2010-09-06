// This is spl/spl_observer_sptr.h
#ifndef spl_observer_sptr_h
#define spl_observer_sptr_h
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

class spl_observer;
#include <vbl/vbl_smart_ptr.h>

struct spl_observer_sptr : public vbl_smart_ptr<spl_observer> {
  typedef vbl_smart_ptr<spl_observer> base;

  spl_observer_sptr() {}
  spl_observer_sptr(spl_observer* p): base(p) {}
  void vertical_cast(spl_observer_sptr const& that) { *this = that; }
  void vertical_cast(spl_observer* t) { *this = t; }
};

// Stop doxygen documenting the B class
#ifndef DOXYGEN_SHOULD_SKIP_THIS
template <class T, class B = spl_observer_sptr>
struct spl_observer_sptr_t : public B {
  spl_observer_sptr_t(): B() {}
  spl_observer_sptr_t(T* p): B(p) {}
  spl_observer_sptr_t(spl_observer_sptr_t<T> const& r): B(r) {}
  void operator=(spl_observer_sptr_t<T> const& r) { B::operator=(r); }
  T* operator->() const { return (T*)this->as_pointer(); }
  T& operator*() const { return *((T*)this->as_pointer()); }
};
#endif // DOXYGEN_SHOULD_SKIP_THIS


#endif // spl_observer_sptr_h
