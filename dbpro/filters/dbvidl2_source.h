// This is basic/dbvidl2/pro/dbvidl2_source.h
#ifndef dbvidl2_source_h_
#define dbvidl2_source_h_

//:
// \file
// \brief A video stream source
// \author Matt Leotta
// \date 6/1/06
//
// \verbatim
//  Modifications
// \endverbatim


#include <vcl_string.h>
#include <dbpro/dbpro_process.h>
#include <vidl/vidl_istream_sptr.h>

//: Convert a vidl_istream into a dbpro_source
class dbvidl2_source : public dbpro_source
{
 public:

  //: Constructor
  dbvidl2_source(const vidl_istream_sptr& i) : istream_(i) {}

  //: Destructor
  virtual ~dbvidl2_source(){}

  //: Set the istream
  void set_stream(const vidl_istream_sptr& i) { istream_ = i; }
  
  //: Return the stream
  vidl_istream_sptr stream() const { return istream_; }

  //: Execute this process
  dbpro_signal execute();

 private:
  vidl_istream_sptr istream_;
};

#endif // dbvidl2_source_h_
