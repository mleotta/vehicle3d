// This is dbpro/filters/vidl_multi_source.h
#ifndef vidl_multi_source_h_
#define vidl_multi_source_h_

//:
// \file
// \brief A synchronized multiple video stream source
// \author Matt Leotta
// \date 6/1/06
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <vcl_string.h>
#include <vcl_vector.h>
#include <dbpro/dbpro_process.h>
#include <vidl/vidl_istream_sptr.h>

//: A source that provides frames from multiple video streams in sync
class vidl_multi_source : public dbpro_source
{
 public:

  //: Constructor
  vidl_multi_source() {}

  //: Constructor
  vidl_multi_source(const vcl_vector<vidl_istream_sptr>& streams)
   : istreams_(streams) {}

  //: Destructor
  virtual ~vidl_multi_source(){}

  //: add an istream
  void add_stream(const vidl_istream_sptr& is) { istreams_.push_back(is); }
  
  //: Set the istream
  void set_stream(unsigned int idx, const vidl_istream_sptr& is) { istreams_[idx] = is; }

  vidl_istream_sptr stream(unsigned int idx) const { return istreams_[idx]; }

  //: Execute this process
  dbpro_signal execute();

 private:
  vcl_vector<vidl_istream_sptr> istreams_;
};

#endif // vidl_multi_source_h_
