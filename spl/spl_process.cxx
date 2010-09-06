// This is spl/spl_process.cxx

//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

//:
// \file

#include <spl/spl_process.h>

#include <vcl_cassert.h>
#include <vcl_cstdlib.h>

#include <spl/spl_parameters.h>
#include <spl/spl_storage.h>
#include <spl/spl_observer.h>
#include <spl/spl_connector.h>

#include <spl/spl_config.h>
#if SPL_HAS_PTHREADS
#include <pthread.h>
#endif



//: Return the last signal
spl_signal spl_process::last_signal() const 
{ 
  return this->last_signal_; 
}


//: Runs the filter
spl_signal
spl_filter::run(unsigned long timestamp,
                  spl_debug_observer* const debug)
{
  // notify the debugger if available
  if (debug) debug->notify_enter(this, timestamp);

  update_mutex_.lock();
  
  if(timestamp > this->timestamp_){
    this->timestamp_ = timestamp;
    this->last_signal_ = this->request_inputs(timestamp,debug);
    if(this->last_signal_ == SPL_VALID){
      if (debug){
        debug->notify_pre_exec(this);
        this->last_signal_ = this->execute();
        debug->notify_post_exec(this);
      }
      else
       this->last_signal_ = this->execute();
    }
    this->notify_observers(this->last_signal_);
    this->clear();
  }
  
  update_mutex_.unlock();
  
  // notify the debugger if available
  if (debug) debug->notify_exit(this, timestamp);

  return this->last_signal_;
}


//: Runs the filter
spl_signal
spl_source::run(unsigned long timestamp,
                  spl_debug_observer* const debug)
{
  // notify the debugger if available
  if (debug) debug->notify_enter(this, timestamp);

  update_mutex_.lock();
  
  if(timestamp > this->timestamp_){
    this->timestamp_ = timestamp;
    this->last_signal_ = SPL_VALID;
    if (debug){
      debug->notify_pre_exec(this);
      this->last_signal_ = this->execute();
      debug->notify_post_exec(this);
    }
    else
      this->last_signal_ = this->execute();
    this->notify_observers(this->last_signal_);
  }
  
  update_mutex_.unlock();

  // notify the debugger if available
  if (debug) debug->notify_exit(this, timestamp);

  return this->last_signal_;
}


//: Runs the filter
spl_signal
spl_sink::run(unsigned long timestamp,
                spl_debug_observer* const debug)
{
  // notify the debugger if available
  if (debug) debug->notify_enter(this, timestamp);

  if(!this->enabled()){
    // notify the debugger if available
    if (debug) debug->notify_exit(this, timestamp);
    return SPL_VALID;
  }
  
  update_mutex_.lock();

  if(timestamp > this->timestamp_){
    this->timestamp_ = timestamp;
    this->last_signal_ = this->request_inputs(timestamp,debug);
    if(this->last_signal_ == SPL_VALID){
      if (debug){
        debug->notify_pre_exec(this);
        this->last_signal_ = this->execute();
        debug->notify_post_exec(this);
      }
      else
        this->last_signal_ = this->execute();
    }
    this->clear();
  }
  
  update_mutex_.unlock();

  // notify the debugger if available
  if (debug) debug->notify_exit(this, timestamp);

  return this->last_signal_;
}


//: Constructor
spl_process::spl_process() 
  : timestamp_(0), input_request_active_(false),
    last_signal_(SPL_INVALID)
{
}


//: Destructor
spl_process::~spl_process()
{
  typedef vcl_map<unsigned int, spl_connector_sptr >::iterator Itr;
  for(Itr i = input_connectors_.begin(); 
      i != input_connectors_.end(); ++i)
  {
    i->second->unlink();
  }

  typedef vcl_map<unsigned int, vcl_set<spl_observer_sptr> >::iterator OItr;
  for(OItr obs = output_observers_.begin(); 
      obs != output_observers_.end(); ++obs)
  {
    for( vcl_set<spl_observer_sptr>::iterator ob = obs->second.begin();
         ob != obs->second.end();  ++ob )
      (*ob)->unlink();
  }
}


//: Clear data sitting on the input connectors
void
spl_process::clear()
{
  typedef vcl_map<unsigned int, spl_connector_sptr >::iterator Itr;
  for(Itr i = input_connectors_.begin(); 
      i != input_connectors_.end(); ++i)
  {
    i->second->notify(NULL,this->timestamp_);
  }
}


#if SPL_HAS_PTHREADS
// helper code to launch and manage pthreads
namespace{
  struct pthread_data
  {
    spl_connector_sptr connector;
    unsigned long timestamp;
    spl_debug_observer* debug;
    pthread_t thread;
  };
  
  void* pthread_launcher(void *arg)
  {
    pthread_data *pd = reinterpret_cast<pthread_data*>(arg);
    spl_signal s = pd->connector->request_data(pd->timestamp,pd->debug);
    return reinterpret_cast<void*>(s);
  }
}
#endif 


//: Request any missing inputs
spl_signal
spl_process::request_inputs(unsigned long timestamp,
                              spl_debug_observer* const debug)
{
  if(this->input_request_active_){
    vcl_cerr << "Warning detected cycle in data flow" <<vcl_endl;
    return SPL_INVALID;
  }
  this->input_request_active_ = true;
  spl_signal retval = SPL_VALID;
  typedef vcl_map<unsigned int, spl_connector_sptr >::iterator Itr;

#if SPL_HAS_PTHREADS
  // the threaded version
  // find the processes that need to be run
  vcl_vector<pthread_data> pd;
  for(Itr i = input_connectors_.begin(); 
      i != input_connectors_.end(); ++i)
  {
    spl_connector_sptr connector = i->second;
    if(!connector->data()){
      pd.push_back(pthread_data());
      pd.back().connector = connector;
      pd.back().timestamp = timestamp;
      pd.back().debug = debug ? debug->clone() : NULL;
    }
    else
    {
      spl_signal m = connector->data()->info();
      if(m != SPL_VALID){
        retval = m;
      }
    }
  }
  if(pd.size() == 1) // serial, no threads needed
  {
    delete pd[0].debug;
    spl_signal s = pd[0].connector->request_data(timestamp,debug);
    if(s != SPL_VALID)
      retval = s;
    spl_signal m = pd[0].connector->data()->info();
    if(m != SPL_VALID){
      retval = m;
    }
  }
  else // use threads
  {
    // start the threads
    for(unsigned int i=0; i<pd.size(); ++i)
    {
      int rc = pthread_create(&pd[i].thread, NULL, pthread_launcher, (void*) &pd[i] );
      if(rc)
      {
        vcl_cerr << "Error creating pthread, return code: "<<rc<<vcl_endl;
        vcl_exit(-1);
      }
    }
    // join the threads
    for(unsigned int i=0; i<pd.size(); ++i)
    {
      void *status;
      int rc = pthread_join(pd[i].thread, &status);
      if(debug && pd[i].debug){
        debug->merge(*(pd[i].debug));
        delete pd[i].debug;
      }
      spl_signal s = reinterpret_cast<spl_signal&> (status);
      if(rc)
      {
        vcl_cerr << "Error joining pthread, return code: "<<rc<<vcl_endl;
        vcl_exit(-1);
      }
      if(s != SPL_VALID)
        retval = s;
      spl_signal m = pd[i].connector->data()->info();
      if(m != SPL_VALID)
        retval = m;
    }
  }
#else
  // the serial (non-threaded) version 
  for(Itr i = input_connectors_.begin(); i != input_connectors_.end(); ++i)
  {
    spl_connector_sptr connector = i->second;
    if(!connector->data()){
      spl_signal s = connector->request_data(timestamp,debug);
      if(s != SPL_VALID)
        retval = s;
      if(s == SPL_INVALID)
        break;
    }
    
    spl_signal m = connector->data()->info();
    if(m != SPL_VALID){
      retval = m;
    }
  }
#endif
  
  this->input_request_active_ = false;
  return retval;
}

//: Request any missing inputs
spl_signal
spl_process::request_inputs_serial(unsigned long timestamp,
                              spl_debug_observer* const debug)
{
  if(this->input_request_active_){
    vcl_cerr << "Warning detected cycle in data flow" <<vcl_endl;
    return SPL_INVALID;
  }
  this->input_request_active_ = true;
  spl_signal retval = SPL_VALID;
  typedef vcl_map<unsigned int, spl_connector_sptr >::iterator Itr;

  // the serial (non-threaded) version 
  for(Itr i = input_connectors_.begin(); i != input_connectors_.end(); ++i)
  {
    spl_connector_sptr connector = i->second;
    if(!connector->data()){
      spl_signal s = connector->request_data(timestamp,debug);
      if(s != SPL_VALID)
        retval = s;
      if(s == SPL_INVALID)
        break;
    }
    
    spl_signal m = connector->data()->info();
    if(m != SPL_VALID){
      retval = m;
    }
  }
  
  this->input_request_active_ = false;
  return retval;
}


//: Notify the output observers
bool
spl_process::notify_observers(spl_signal message)
{
  typedef vcl_map<unsigned int, spl_storage_sptr >::const_iterator DItr;
  typedef vcl_map<unsigned int, vcl_set<spl_observer_sptr> >::iterator OItr;
  // Do not send the message SPL_VALID unless the data is valid
  if(message == SPL_VALID)
    message = SPL_INVALID;
  bool retval = true;
  // for each output pin with observers attached
  for(OItr obs = output_observers_.begin(); 
      obs != output_observers_.end(); ++obs)
  {
    if(obs->second.empty())
      continue;
    // find the output data produced for that pin (if any)
    DItr d = output_data_.find(obs->first);
    spl_storage_sptr data;
    if(d == output_data_.end()){
      // if no data, notify with a signal storage class
      data = new spl_storage_signal(message);
      retval = false;
    }
    else
      data = d->second;

    // notify all of the observers of this pin
    for( vcl_set<spl_observer_sptr>::iterator ob = obs->second.begin();
         ob != obs->second.end();  ++ob )
      (*ob)->notify(data,this->timestamp_);
  }
  output_data_.clear();
  return retval;
}


//: Connect pin \p in_idx to the output pin \p out_idx of the process \p source
void
spl_process::connect_input(unsigned int in_idx,
                             const spl_process_sptr& source, unsigned int out_idx)
{
  typedef vcl_map<unsigned int, spl_connector_sptr >::iterator Itr;
  assert(source);
  spl_connector_sptr connector;
  Itr result = input_connectors_.find(in_idx);
  if(result == input_connectors_.end()){
    connector = new spl_connector();
    input_connectors_[in_idx] = connector;
  }
  else{
    connector = result->second;
  }
  connector->link(source,out_idx);
}


//: Disconnect pin \p in_idx
void
spl_process::disconnect_input(unsigned int in_idx)
{
  typedef vcl_map<unsigned int, spl_connector_sptr >::iterator Itr;
  Itr result = input_connectors_.find(in_idx);
  if(result != input_connectors_.end()){
    result->second->unlink();
    input_connectors_.erase(result);
  }
}


//: Add an output observer on pin \p idx
void
spl_process::add_output_observer(unsigned int idx, const spl_observer_sptr& obs)
{
  output_observers_[idx].insert(obs);
}


//: Remove an output observer on pin \p idx if it exists
bool
spl_process::remove_output_observer(unsigned int idx, const spl_observer_sptr& obs)
{
  typedef vcl_map<unsigned int, vcl_set<spl_observer_sptr> >::iterator Itr;
  Itr result = output_observers_.find(idx);
  if(result == output_observers_.end())
    return false;

  return result->second.erase(obs) > 0;
}


//: Access the storage class on the given input pin
spl_storage_sptr
spl_process::get_input(unsigned int idx) const
{
  typedef vcl_map<unsigned int, spl_connector_sptr>::const_iterator Itr;
  Itr result = input_connectors_.find(idx);
  if(result == input_connectors_.end() || !result->second)
    return NULL;
  return result->second->data();
}


//: Returns the typeinfo struct for the data type stored on the given pin
const vcl_type_info&
spl_process::input_type_id(unsigned int idx) const
{
  spl_storage_sptr ptr = get_input(idx);
  if(!ptr)
    return typeid(void);
  return ptr->type_id();
}


//: Set the output storage class on the given pin
// all observers of this output are notified
bool
spl_process::set_output(unsigned int idx, const spl_storage_sptr& data)
{
  output_data_[idx] = data;
  return true;
}


//: Returns the number of observers of a particular output pin
unsigned int
spl_process::num_observers(unsigned int idx) const
{
  typedef vcl_map<unsigned int, vcl_set<spl_observer_sptr> >::const_iterator Itr;
  Itr result = output_observers_.find(idx);
  if(result == output_observers_.end())
    return 0;

  return result->second.size();
}


//: Returns the set of all pins that have observers
vcl_set<unsigned int>
spl_process::observed_outputs() const
{
  vcl_set<unsigned int> observed;
  typedef vcl_map<unsigned int, vcl_set<spl_observer_sptr> >::const_iterator OItr;
  for(OItr obs = output_observers_.begin(); 
      obs != output_observers_.end(); ++obs)
    observed.insert(obs->first);
  return observed;
}

