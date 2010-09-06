// This is spl/spl_mutex.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include <spl/spl_mutex.h>

#include <spl/spl_config.h>
#if SPL_HAS_PTHREADS
#include <pthread.h>
#endif

//: private implementation struct for thread data
struct spl_mutex::pimpl
{
public:
  pimpl()
  {
#if SPL_HAS_PTHREADS
    pthread_mutex_init(&mutex_, NULL);
#endif
  }
  
  ~pimpl()
  {
#if SPL_HAS_PTHREADS
    pthread_mutex_destroy(&mutex_);
#endif
  }
  

#if SPL_HAS_PTHREADS
  pthread_mutex_t mutex_;
#endif
};


//: Default Constructor
spl_mutex::spl_mutex()
: data_(new pimpl())
{
}


//: Destructor
spl_mutex::~spl_mutex()
{
}


//: Lock the mutex
void spl_mutex::lock()
{
#if SPL_HAS_PTHREADS
  pthread_mutex_lock(&data_->mutex_);
#endif
}


//: Unlock the mutex
void spl_mutex::unlock()
{
#if SPL_HAS_PTHREADS
  pthread_mutex_unlock(&data_->mutex_);
#endif
}
