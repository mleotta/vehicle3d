// This is dml/dml_vehicle_track_init.h
#ifndef dml_vehicle_track_init_h_
#define dml_vehicle_track_init_h_
//=========================================================================
//:
// \file
// \brief Initialize vehicle tracks from foreground outlines and optical flow
// \author Matt Leotta (mleotta)
// \date 05/27/2009
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
//   05/27/2009 - File created. (mleotta)
// \endverbatim
//=========================================================================


#include <vpgl/vpgl_perspective_camera.h>
#include <dml/dml_pca_vehicle.h>
#include <dml/dml_vehicle_fit.h>
#include <dml/dml_vehicle_state.h>

//: Initialize vehicle tracks from foreground outlines and optical flow
class dml_vehicle_track_init
{
public:
  typedef vcl_pair<vgl_point_2d<double>,vgl_vector_2d<double> > pv_pair;
  
  enum vehicle_model { DODECAHEDRAL, FERRYMAN, DETAILED1, DETAILED2, DETAILED3 };
  
  dml_vehicle_track_init(const vpgl_perspective_camera<double>& cam = 
                         vpgl_perspective_camera<double>());
  
  //: Set the camera
  void set_camera(const vpgl_perspective_camera<double>& camera);
  //: Set the sun direction for shadow casting
  void set_sun_direction(const vgl_vector_3d<double>& sun_dir);
  //: Set the vehicle model
  void set_vehicle(const dml_pca_vehicle& vehicle);
  
  //: enable or disable shape estimation
  void set_estimate_shape(bool val) { estimate_shape_ = val; }
  
  //: estimate the Z rotation angle from optical flow vectors
  double estimate_angle(const vcl_vector<pv_pair>& flow);
  
  //: estimate the 3-d translation to align the silhouette centroid to \a c. 
  vgl_vector_3d<double> align_centroids(const vgl_point_2d<double>& c,
                                        const vgl_rotation_3d<double>& R);
  
  //: test the silhouette polygon to see if it is a plausible vehicle
  bool silhouette_plausible(const vgl_polygon<double>& silhouette);
  
  //: compute the median silhouette_error
  double median_error(const vgl_polygon<double>& sil,
                      const vgl_rotation_3d<double>& R,
                      vgl_vector_3d<double>& t);
  
  //: compute the errors in silhouette alignment
  vcl_vector<double> silhouette_error(const vgl_polygon<double>& silhouette,
                                      vcl_vector<vgl_vector_2d<double> >& norms);
  
  //: compute the tranlation and mesh params to align the silhouettes
  double fit_to_silhouette(const vgl_polygon<double>& silhouette,
                           const vgl_rotation_3d<double>& R,
                           vgl_vector_3d<double>& t);
  
  //: match observed silhouettes and flow vectors to existing states
  //  return a vector of unmatched silhouettes
  vcl_vector<vgl_polygon<double> >
  match_silhouettes(const vcl_vector<vgl_polygon<double> >& silhouettes,
                    const vcl_vector<pv_pair>& flow,
                          vcl_vector<dml_vehicle_state>& states);
  
  
  //: Process the silhouettes and flow and estimate initial states.
  //  Also merge in states predicted from the previous frame
  void find_states(const vcl_vector<vgl_polygon<double> >& silhouettes,
                   const vcl_vector<pv_pair>& flow,
                         vcl_vector<dml_vehicle_state>& states);

private:
  //: The active camera
  vpgl_perspective_camera<double> camera_;
  //: the direction of sunlight for shadow casting
  vgl_vector_3d<double> sun_dir_;
  //: The deformable vehicle model
  dml_pca_vehicle vehicle_;
  //: The type of vehicle model
  vehicle_model vehicle_type_;
  //: A class to project the vehicle into images
  dml_pca_vehicle_projector mesh_projector_;
  
  //: parameters used for initialization of shape
  vcl_vector<vcl_vector<vnl_vector<double> > > init_params_;
  
  //: flag to enable shape estimation in addition to pose
  bool estimate_shape_;
  
  
  //: precomputed homographies between image and ground
  vgl_h_matrix_2d<double> H_g2i_, H_i2g_;
  //: precomputed homographies between image and plane Z=1
  vgl_h_matrix_2d<double> H_h2i_, H_i2h_;
};



#endif // dml_vehicle_track_init_h_
