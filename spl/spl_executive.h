// This is spl/spl_executive.h
#ifndef spl_executive_h_
#define spl_executive_h_

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
#include <spl/spl_process.h>



//: An object to contain and execute a process graph
class spl_executive 
{
 public:
   class proxy{
     public:
       friend class spl_executive;
       //: Copy constructor
       proxy(const proxy& other) : exec(other.exec), name(other.name) {}
       operator spl_process_sptr() const { return exec.find(name); }
       proxy& operator= (const spl_process_sptr& p)
       { exec.pmap_[name] = p; return *this; }
       spl_process* operator->() { return exec.find(name).as_pointer(); }
       spl_process* ptr() { return exec.find(name).as_pointer(); }
     private:
       proxy(spl_executive& e, const vcl_string& n)
       : exec(e), name(n) {}
       spl_executive& exec;
       vcl_string name;
   };

   //: Constructor
   spl_executive() : time_(0), debug_(NULL) {}

   //: Destructor
   ~spl_executive() { delete debug_; }

  //: Accessor for assignment
  proxy operator[](const vcl_string& name) { return proxy(*this,name); }

  //: Accessor for const access
  spl_process_sptr operator[](const vcl_string& name) const  { return find(name); }

  //: Initialize the process graph
  //: Call this after adding processes but before running
  void init();

  //: Run the entire graph and iterate until error or end of stream
  spl_signal run_all() const;

  //: Run one step of the stream processing
  spl_signal run_step() const;

  //: enable debug statements
  void enable_debug();
  
  //: Lookup a process name by process pointer
  vcl_string process_name(const spl_process* const pro) const;

 private:
  //: Find a process by name or return NULL
  spl_process_sptr find(const vcl_string& name) const;
  //: Map from names to processes
  vcl_map<vcl_string, spl_process_sptr> pmap_;
  //: The set of sinks
  vcl_set<spl_process_sptr> sinks_;

  //: The current time
  mutable unsigned long time_;

  spl_debug_observer* debug_;


};

#endif // spl_executive_h_
