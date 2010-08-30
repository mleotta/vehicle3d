//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <testlib/testlib_test.h>
#include <vpro/vpro_process.h>
#include <vpro/vpro_delay.h>
#include "vpro_sample_processes.h"
#include <vcl_iostream.h>



MAIN( test_delay_filter )
{
  START ("delay_filter");


  vpro_process_sptr delay = new vpro_delay(2,1.0);

  vpro_output_queue<double>* oq_ptr = new vpro_output_queue<double>;
  vpro_process_sptr oq(oq_ptr);

  vpro_process_sptr sum = new vpro_sum<double>;


  oq->connect_input(0,delay,1);
  sum->connect_input(0,delay,0);
  sum->connect_input(1,delay,1);
  delay->connect_input(0,sum,0);


  for(unsigned int count = 0; count<10; ++count){
    vpro_signal s1 = oq->run(count);
    vpro_signal s2 = delay->run(count);
    if(s1 == VPRO_INVALID || s2 == VPRO_INVALID){
      vcl_cerr << "Error stream failure" << vcl_endl;
      break;
    }
    if(s1 == VPRO_EOS || s2 == VPRO_EOS){
      break;
    }
  }

  //TEST("number of iterations", count, data.size());

  vcl_cout << "Resulting data output\n";
  for(unsigned int i=0; i<oq_ptr->data.size(); ++i)
    vcl_cout << oq_ptr->data[i] << " ";
  vcl_cout << vcl_endl;


  SUMMARY();
}
