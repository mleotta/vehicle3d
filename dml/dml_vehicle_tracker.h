// This is dml/dml_vehicle_tracker.h
#ifndef dml_vehicle_tracker_h_
#define dml_vehicle_tracker_h_
//=========================================================================
//:
// \file
// \brief manager vehicle tracking processes
// \author Matt Leotta (mleotta)
// \date 04/20/2009
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
//   04/20/2009 - File created. (mleotta)
// \endverbatim
//=========================================================================


#include <vpro/vpro_executive.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vidl/vidl_istream_sptr.h>
#include <dml/dml_pca_vehicle.h>

// forward declarations
class dml_vehicle_fit_video;

//: manager vehicle tracking processes
class dml_vehicle_tracker
{
public:
  dml_vehicle_tracker(dml_vehicle_fit_video* optimizer);
  
  //: Assign a camera
  void set_camera(const vpgl_perspective_camera<double>& camera);
  
  //: Set the sun direction for shadow casting
  void set_sun_direction(const vgl_vector_3d<double>& sun_dir);
  
  //: Assign the PCA vehicle model
  void set_vehicle_model(const dml_pca_vehicle& vehicle);
  
  //: set the video input stream
  void set_istream(const vidl_istream_sptr& istream);
  
  //: add an observer of input video frames 
  void add_video_observer(const vpro_observer_sptr& obs);
  
  //: add an observer of the background detection image
  void add_bg_observer(const vpro_observer_sptr& obs);
  
  //: add an observer of detected vehicle silhouette polygons
  void add_silhouette_observer(const vpro_observer_sptr& obs);
  
  //: add an observer of detected vehicle hypotheses
  void add_hypotheses_observer(const vpro_observer_sptr& obs);
  
  //: add an observer of the vehicle tracks
  void add_track_observer(const vpro_observer_sptr& obs);
  
  //: add an observer of the edge map image
  void add_edgemap_observer(const vpro_observer_sptr& obs);
  
  //: add an observer of the point map image
  void add_pointmap_observer(const vpro_observer_sptr& obs);
  
  //: add an observer of the points
  void add_point_observer(const vpro_observer_sptr& obs);
  
  //: add an observer of the optical flow
  void add_optical_flow_observer(const vpro_observer_sptr& obs);
  
  //: enable tracking
  void enable_tracking(bool enable_track);
  
  //: enable processes for display
  void enable_display(bool enable_display);
  
  //: enable or disable shape estimation
  void set_estimate_shape(bool val);
  
  //: advance the video and process on frame
  vpro_signal process_once() { return graph_.run_step(); }

private:
  //: initialize the intensity background modeling processes
  void init_bgm_pro();
  
  //: initialize the edge background modeling processes
  void init_ebgm_pro();
  
  //: store the process graph
  vpro_executive graph_;
};



#endif // dml_vehicle_tracker_h_
