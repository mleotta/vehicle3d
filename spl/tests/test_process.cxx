//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <testlib/testlib_test.h>
#include <spl/spl_process.h>
#include "spl_sample_processes.h"
#include <vcl_iostream.h>
#include <spl/spl_observer.h>


//: Print a message when notified
template <class T>
class spl_message : public spl_observer
{
  public:
    //: Transmit the data to the input of a process
    bool notify(const spl_storage_sptr& data, unsigned long timestamp)
    {
      if(data->info() == SPL_VALID)
        vcl_cout << "Received: "<< data->template data<T>() <<" at time "<<timestamp<< vcl_endl;
      return true;
    }
};


MAIN( test_process )
{
  START ("process");

  vcl_vector<int> data;
  data.push_back(1);
  data.push_back(2);
  data.push_back(4);
  data.push_back(6);
  data.push_back(10);

  spl_process_sptr iq = new spl_input_queue<int>(data);

  spl_output_queue<double>* oq1_ptr = new spl_output_queue<double>;
  spl_output_queue<float>* oq2_ptr = new spl_output_queue<float>;
  spl_process_sptr oq1(oq1_ptr);
  spl_process_sptr oq2(oq2_ptr);

  spl_process_sptr conv = new spl_static_cast<int,double>;
  spl_process_sptr root = new spl_sqrt;
  spl_process_sptr sum = new spl_sum<double>;

  spl_process_sptr conv2 = new spl_static_cast<double,float>;

  oq1->connect_input(0,sum,0);
  root->connect_input(0,conv,0);
  sum->connect_input(0,root,0);
  sum->connect_input(1,conv,0);
  conv->connect_input(0,iq,0);

  conv2->connect_input(0,root,0);
  oq2->connect_input(0,conv2,0);

  conv->add_output_observer(0,new spl_message<double>);

  unsigned int count = 0;
  for(; count<100 && oq1->run(count+1) == SPL_VALID
                  && oq2->run(count+1) == SPL_VALID; ++count);

  TEST("number of iterations", count, data.size());

  vcl_cout << "Resulting data output 1\n";
  for(unsigned int i=0; i<oq1_ptr->data.size(); ++i)
    vcl_cout << oq1_ptr->data[i] << " ";
  vcl_cout << vcl_endl;

  vcl_cout << "Resulting data output 2\n";
  for(unsigned int i=0; i<oq2_ptr->data.size(); ++i)
    vcl_cout << oq2_ptr->data[i] << " ";
  vcl_cout << vcl_endl;

  SUMMARY();
}
