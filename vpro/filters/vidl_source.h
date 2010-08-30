// This is vpro/filters/vidl_source.h
#ifndef vidl_source_h_
#define vidl_source_h_

//:
// \file
// \brief A video stream source
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
#include <vpro/vpro_process.h>
#include <vidl/vidl_istream_sptr.h>

//: Convert a vidl_istream into a vpro_source
class vidl_source : public vpro_source
{
 public:

  //: Constructor
  vidl_source(const vidl_istream_sptr& i) : istream_(i) {}

  //: Destructor
  virtual ~vidl_source(){}

  //: Set the istream
  void set_stream(const vidl_istream_sptr& i) { istream_ = i; }
  
  //: Return the stream
  vidl_istream_sptr stream() const { return istream_; }

  //: Execute this process
  vpro_signal execute();

 private:
  vidl_istream_sptr istream_;
};

#endif // vidl_source_h_
