// This is mleotta/gui/pca_vehicle/modrec_vehicle_state.h
#ifndef modrec_vehicle_state_h_
#define modrec_vehicle_state_h_
//=========================================================================
//:
// \file
// \brief  The state of a PCA vehicle for shape estimation while tracking
// \author Matt Leotta (mleotta)
// \date 7/20/2009
//
// \verbatim
//  Modifications
//   7/20/2009 - File created. (mleotta)
// \endverbatim
//=========================================================================


#include <vcl_string.h>
#include <vcl_map.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vgl/vgl_polygon.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/algo/vgl_rotation_3d.h>


//: A structure to store complete vehicle state 
class modrec_vehicle_state
{
public:
  modrec_vehicle_state()
  : t_velocity(0), a_velocity(0), num_frames(0) {}
  
  modrec_vehicle_state(const vnl_vector<double>& p,
                       const vgl_rotation_3d<double>& r,
                       const vgl_vector_3d<double>& t,
                       double vt = 0.0,
                       double va = 0.0,
                       bool estimate_shape = true)
  : params(p), rotation(r), translation(t), t_velocity(vt), a_velocity(va), 
    num_frames(0), mean_params(p.size(),0.0)
  { 
    init_covar(estimate_shape); 
    unique_id = next_id++;
  }
  
  //: stack pose, velocity, and shape into a single state vector
  vnl_vector<double> state_vector() const;
  
  //: extract pose, velocity, and shape from a single state vector
  void set_state(const vnl_vector<double>& s);
  
  //: initialize the covariance matrix 
  void init_covar(bool estimate_shape);
  
  //: shape parameters
  vnl_vector<double> params;
  //: orientation parameters
  vgl_rotation_3d<double> rotation;
  //: position parameters
  vgl_vector_3d<double> translation;
  //: translational velocity
  double t_velocity;
  //: angular velocity
  double a_velocity;
  
  unsigned int unique_id;
  
  //: the number of frames the vehicle has been observed in
  unsigned int num_frames;
  //: last known orientation parameters
  vgl_rotation_3d<double> last_rotation;
  //: last known position parameters
  vgl_vector_3d<double> last_translation;
  //: a running average of shape parameters
  vnl_vector<double> mean_params;
  
  //: state vector covariance
  vnl_matrix<double> covar;
  
  //: optional detected silhouette  
  vgl_polygon<double> silhouette;
  
private:
  
  static unsigned int next_id;
};


//: predict the next state (after time t elapsed) using the circular motion model
modrec_vehicle_state 
modrec_circ_motion_predict(const modrec_vehicle_state& s, double t);

//: predict the position only (after time t elapsed) using the circular motion model
vgl_vector_3d<double> 
modrec_circ_motion_predict_position(const modrec_vehicle_state& s, double t);

//: read a tracking result file
unsigned int 
modrec_read_track_file(const vcl_string& filename, 
                       vcl_string& vid_file, vcl_string& cam_file, vcl_string& model_type,
                       vcl_map<unsigned int,vcl_vector<modrec_vehicle_state> >& state_map);


#endif // modrec_vehicle_state_h_
