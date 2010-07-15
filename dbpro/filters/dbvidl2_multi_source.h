// This is basic/dbvidl2/pro/dbvidl2_multi_source.h
#ifndef dbvidl2_multi_source_h_
#define dbvidl2_multi_source_h_

//:
// \file
// \brief A synchronized multiple video stream source
// \author Matt Leotta
// \date 6/1/06
//
// \verbatim
//  Modifications
// \endverbatim


#include <vcl_string.h>
#include <vcl_vector.h>
#include <dbpro/dbpro_process.h>
#include <vidl/vidl_istream_sptr.h>

//: A source that provides frames from multiple video streams in sync
class dbvidl2_multi_source : public dbpro_source
{
 public:

  //: Constructor
  dbvidl2_multi_source() {}

  //: Constructor
  dbvidl2_multi_source(const vcl_vector<vidl_istream_sptr>& streams)
   : istreams_(streams) {}

  //: Destructor
  virtual ~dbvidl2_multi_source(){}

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

#endif // dbvidl2_multi_source_h_
