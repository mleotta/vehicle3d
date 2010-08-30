// This is vpro/vpro_mutex.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include <vpro/vpro_mutex.h>

#include <vpro/vpro_config.h>
#if VPRO_HAS_PTHREADS
#include <pthread.h>
#endif

//: private implementation struct for thread data
struct vpro_mutex::pimpl
{
public:
  pimpl()
  {
#if VPRO_HAS_PTHREADS
    pthread_mutex_init(&mutex_, NULL);
#endif
  }
  
  ~pimpl()
  {
#if VPRO_HAS_PTHREADS
    pthread_mutex_destroy(&mutex_);
#endif
  }
  

#if VPRO_HAS_PTHREADS
  pthread_mutex_t mutex_;
#endif
};


//: Default Constructor
vpro_mutex::vpro_mutex()
: data_(new pimpl())
{
}


//: Destructor
vpro_mutex::~vpro_mutex()
{
}


//: Lock the mutex
void vpro_mutex::lock()
{
#if VPRO_HAS_PTHREADS
  pthread_mutex_lock(&data_->mutex_);
#endif
}


//: Unlock the mutex
void vpro_mutex::unlock()
{
#if VPRO_HAS_PTHREADS
  pthread_mutex_unlock(&data_->mutex_);
#endif
}
