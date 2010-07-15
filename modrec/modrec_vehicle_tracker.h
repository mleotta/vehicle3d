// This is mleotta/gui/pca_vehicle/modrec_vehicle_tracker.h
#ifndef modrec_vehicle_tracker_h_
#define modrec_vehicle_tracker_h_
//=========================================================================
//:
// \file
// \brief manager vehicle tracking processes
// \author Matt Leotta (mleotta)
// \date 04/20/2009
//
// \verbatim
//  Modifications
//   04/20/2009 - File created. (mleotta)
// \endverbatim
//=========================================================================


#include <dbpro/dbpro_executive.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vidl/vidl_istream_sptr.h>
#include <modrec/modrec_pca_vehicle.h>

// forward declarations
class modrec_vehicle_fit_video;

//: manager vehicle tracking processes
class modrec_vehicle_tracker
{
public:
  modrec_vehicle_tracker(modrec_vehicle_fit_video* optimizer);
  
  //: Assign a camera
  void set_camera(const vpgl_perspective_camera<double>& camera);
  
  //: Set the sun direction for shadow casting
  void set_sun_direction(const vgl_vector_3d<double>& sun_dir);
  
  //: Assign the PCA vehicle model
  void set_vehicle_model(const modrec_pca_vehicle& vehicle);
  
  //: set the video input stream
  void set_istream(const vidl_istream_sptr& istream);
  
  //: add an observer of input video frames 
  void add_video_observer(const dbpro_observer_sptr& obs);
  
  //: add an observer of the background detection image
  void add_bg_observer(const dbpro_observer_sptr& obs);
  
  //: add an observer of detected vehicle silhouette polygons
  void add_silhouette_observer(const dbpro_observer_sptr& obs);
  
  //: add an observer of detected vehicle hypotheses
  void add_hypotheses_observer(const dbpro_observer_sptr& obs);
  
  //: add an observer of the vehicle tracks
  void add_track_observer(const dbpro_observer_sptr& obs);
  
  //: add an observer of the edge map image
  void add_edgemap_observer(const dbpro_observer_sptr& obs);
  
  //: add an observer of the point map image
  void add_pointmap_observer(const dbpro_observer_sptr& obs);
  
  //: add an observer of the points
  void add_point_observer(const dbpro_observer_sptr& obs);
  
  //: add an observer of the optical flow
  void add_optical_flow_observer(const dbpro_observer_sptr& obs);
  
  //: enable tracking
  void enable_tracking(bool enable_track);
  
  //: enable processes for display
  void enable_display(bool enable_display);
  
  //: enable or disable shape estimation
  void set_estimate_shape(bool val);
  
  //: advance the video and process on frame
  dbpro_signal process_once() { return graph_.run_step(); }

private:
  //: initialize the intensity background modeling processes
  void init_bgm_pro();
  
  //: initialize the edge background modeling processes
  void init_ebgm_pro();
  
  //: store the process graph
  dbpro_executive graph_;
};



#endif // modrec_vehicle_tracker_h_
