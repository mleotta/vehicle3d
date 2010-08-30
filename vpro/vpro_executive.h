// This is vpro/vpro_executive.h
#ifndef vpro_executive_h_
#define vpro_executive_h_

//:
// \file
// \brief An object to contain and execute a process graph
// \author Matt Leotta
// \date 6/7/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <vcl_map.h>
#include <vcl_string.h>
#include <vpro/vpro_process.h>



//: An object to contain and execute a process graph
class vpro_executive 
{
 public:
   class proxy{
     public:
       friend class vpro_executive;
       //: Copy constructor
       proxy(const proxy& other) : exec(other.exec), name(other.name) {}
       operator vpro_process_sptr() const { return exec.find(name); }
       proxy& operator= (const vpro_process_sptr& p)
       { exec.pmap_[name] = p; return *this; }
       vpro_process* operator->() { return exec.find(name).as_pointer(); }
       vpro_process* ptr() { return exec.find(name).as_pointer(); }
     private:
       proxy(vpro_executive& e, const vcl_string& n)
       : exec(e), name(n) {}
       vpro_executive& exec;
       vcl_string name;
   };

   //: Constructor
   vpro_executive() : time_(0), debug_(NULL) {}

   //: Destructor
   ~vpro_executive() { delete debug_; }

  //: Accessor for assignment
  proxy operator[](const vcl_string& name) { return proxy(*this,name); }

  //: Accessor for const access
  vpro_process_sptr operator[](const vcl_string& name) const  { return find(name); }

  //: Initialize the process graph
  //: Call this after adding processes but before running
  void init();

  //: Run the entire graph and iterate until error or end of stream
  vpro_signal run_all() const;

  //: Run one step of the stream processing
  vpro_signal run_step() const;

  //: enable debug statements
  void enable_debug();
  
  //: Lookup a process name by process pointer
  vcl_string process_name(const vpro_process* const pro) const;

 private:
  //: Find a process by name or return NULL
  vpro_process_sptr find(const vcl_string& name) const;
  //: Map from names to processes
  vcl_map<vcl_string, vpro_process_sptr> pmap_;
  //: The set of sinks
  vcl_set<vpro_process_sptr> sinks_;

  //: The current time
  mutable unsigned long time_;

  vpro_debug_observer* debug_;


};

#endif // vpro_executive_h_
