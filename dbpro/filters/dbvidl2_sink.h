// This is dbpro/filters/dbvidl2_sink.h
#ifndef dbvidl2_sink_h_
#define dbvidl2_sink_h_

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
#include <dbpro/dbpro_process.h>
#include <vidl/vidl_ostream_sptr.h>

//: Convert a vidl_ostream into a dbpro_sink
class dbvidl2_sink : public dbpro_sink
{
 public:

  //: Constructor
  dbvidl2_sink(const vidl_ostream_sptr& o) : ostream_(o) {}

  //: Destructor
  virtual ~dbvidl2_sink(){}

  //: Set the ostream
  void set_stream(const vidl_ostream_sptr& o) { ostream_ = o; }

  bool enabled() const { return (ostream_ != NULL); }


  //: Execute this process
  dbpro_signal execute();

 private:
  vidl_ostream_sptr ostream_;
};

#endif // dbvidl2_sink_h_
