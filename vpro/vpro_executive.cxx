// This is vpro/vpro_executive.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include "vpro_executive.h"
#include <vpro/vpro_config.h>

#include "vpro_delay.h"
#include "vpro_mutex.h"
#include <vcl_iostream.h>
#include <vcl_sstream.h>
#include <vcl_vector.h>
#include <vcl_map.h>
#include <vul/vul_timer.h>

namespace{ // anonymous namespace

vcl_ostream& operator<<(vcl_ostream& os, vpro_signal s){
  switch(s){
    case VPRO_VALID:   os << "VALID"; break;
    case VPRO_INVALID: os << "INVALID"; break;
    case VPRO_EOS:     os << "EOS"; break;
    case VPRO_WAIT:    os << "WAIT"; break;
  }
  return os;
}

class executive_debug: public vpro_debug_observer
{
  public:
    executive_debug(vpro_executive* exec, vcl_ostream& os) : exec_(exec), os_(os) {}
    virtual ~executive_debug(){}
    virtual void notify_enter(const vpro_process* const pro, unsigned int timestamp)
    {
      process_stack_.push_back(pro);
      vcl_string pname = exec_->process_name(pro);
      name_stack_.push_back(pname);
      
      display_mutex_.lock();
#if VPRO_HAS_PTHREADS
      os_ << pthread_self();
#endif
      
      for(unsigned i=0; i<process_stack_.size(); ++i)
        os_ << "  ";
      os_ << "enter: "<< pname << "  timestamp = " << timestamp <<vcl_endl;
      
      display_mutex_.unlock();
    }

    virtual void notify_exit(const vpro_process* const pro, unsigned int /*timestamp*/)
    {
      assert(pro == process_stack_.back());
      vcl_string pname = name_stack_.back();
      display_mutex_.lock();
#if VPRO_HAS_PTHREADS
      os_ << pthread_self();
#endif
      for(unsigned i=0; i<process_stack_.size(); ++i)
        os_ << "  ";
      os_ << "exit: "<< pname << "  return = "<< pro->last_signal() 
          <<" (time: "<< runtimes_[pro] <<")"<< vcl_endl;
      
      display_mutex_.unlock();

      process_stack_.pop_back();
      name_stack_.pop_back();
    }
  
    virtual void notify_pre_exec(const vpro_process* const /*pro*/)
    {
      timer_.mark();
    }
  
    virtual void notify_post_exec(const vpro_process* const pro)
    {
      long int t = timer_.all();
      vcl_map<const vpro_process* const, long int>::iterator f = runtimes_.find(pro);
      if( f == runtimes_.end() )
        runtimes_[pro] = t;
      else
        f->second += t;
    }
  
    //: Copy constructor
    executive_debug(const executive_debug& other)
      : exec_(other.exec_), process_stack_(other.process_stack_),
        name_stack_(other.name_stack_),
        runtimes_(runtimes_), os_(other.os_)
    {      
    }
  
    //: clone the debug observer 
    //  clones are made for each thread during a branch
    virtual vpro_debug_observer* clone() const
    {
      return new executive_debug(*this);
    }
  
    //: merge the data from another into this
    //  clones are merged when threads are joined threads
    virtual void merge(const vpro_debug_observer& /*other*/)
    {
      //const executive_debug& eother = dynamic_cast<const executive_debug&>(other);
      //os_ << eother.os_.str();
    }

  private:
    vpro_executive* exec_;
    vcl_vector<const vpro_process*> process_stack_;
    vcl_vector<vcl_string> name_stack_;
    vcl_map<const vpro_process* const, long int> runtimes_;
    vul_timer timer_;
    vcl_ostream& os_;
    static vpro_mutex display_mutex_;
};
  
vpro_mutex executive_debug::display_mutex_;

}; //end anonymous namespace



//: Find a process by name or return NULL
vpro_process_sptr
vpro_executive::find(const vcl_string& name) const
{
  typedef vcl_map<vcl_string, vpro_process_sptr >::const_iterator Itr;
  Itr result = pmap_.find(name);
  if(result == pmap_.end())
    return NULL;
  return result->second;
}


//: Run the entire graph and iterate until error or end of stream
vpro_signal
vpro_executive::run_all() const
{
  vpro_signal retval = VPRO_VALID;
  while(retval == VPRO_VALID || retval == VPRO_WAIT)
    retval = run_step();
  return retval;
}


//: Run one step of the stream processing
vpro_signal
vpro_executive::run_step() const
{
  ++time_;
  vpro_signal retval = VPRO_VALID;
  typedef vcl_set<vpro_process_sptr >::const_iterator Itr;
  for(Itr i = sinks_.begin(); i != sinks_.end(); ++i){
    vpro_signal s = (*i)->run(time_,debug_);
    if(s != VPRO_VALID)
      retval = s;
  }
  return retval;
}


//: Initialize the process graph
//: Call this after adding processes but before running
void
vpro_executive::init()
{
  typedef vcl_map<vcl_string, vpro_process_sptr >::const_iterator Itr;

  time_ = 0;

  sinks_.clear();
  for(Itr i = pmap_.begin(); i != pmap_.end(); ++i)
    if(dynamic_cast<vpro_sink*>(i->second.ptr()))
      sinks_.insert(i->second);

#ifndef NDEBUG
  if(sinks_.empty()){
    vcl_cerr<< "WARNING: There are no data sinks in this process graph" << vcl_endl;
  }
#endif
}


//: enable debug statements
void
vpro_executive::enable_debug()
{
  if(debug_)
    delete debug_;
  debug_ = new executive_debug(this,vcl_cerr);
}


//: Lookup a process name by process pointer
vcl_string
vpro_executive::process_name(const vpro_process* const pro) const
{
  typedef vcl_map<vcl_string, vpro_process_sptr >::const_iterator Itr;
  for(Itr i = pmap_.begin(); i != pmap_.end(); ++i)
    if(pro == i->second.ptr())
      return i->first;
  return "(unknown process)";
}



