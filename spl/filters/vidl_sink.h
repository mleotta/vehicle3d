// This is spl/filters/vidl_sink.h
#ifndef vidl_sink_h_
#define vidl_sink_h_

//:
// \file
// \brief A video stream sink
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
#include <spl/spl_process.h>
#include <vidl/vidl_ostream_sptr.h>

//: Convert a vidl_ostream into a spl_sink
class vidl_sink : public spl_sink
{
 public:

  //: Constructor
  vidl_sink(const vidl_ostream_sptr& o) : ostream_(o) {}

  //: Destructor
  virtual ~vidl_sink(){}

  //: Set the ostream
  void set_stream(const vidl_ostream_sptr& o) { ostream_ = o; }

  bool enabled() const { return (ostream_ != NULL); }


  //: Execute this process
  spl_signal execute();

 private:
  vidl_ostream_sptr ostream_;
};

#endif // vidl_sink_h_
