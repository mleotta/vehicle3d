// This is mleotta/gui/pca_vehicle/modrec_vehicle_state.cxx
//=========================================================================
//:
// \file
// \brief The state of a PCA vehicle for shape estimation while tracking
//
//=========================================================================

#include "modrec_vehicle_state.h"

#include <vnl/vnl_double_3.h>
#include <vcl_fstream.h>


//: global count of unique tracks
unsigned int modrec_vehicle_state::next_id = 0;


//: stack shape, pose, and velocity into a single state vector
vnl_vector<double> 
modrec_vehicle_state::state_vector() const
{
  vnl_vector<double> s(params.size()+8);
  s[0] = translation.x();
  s[1] = translation.y();
  s[2] = translation.z();
  s.update(rotation.as_rodrigues(),3);
  s[6] = t_velocity;
  s[7] = a_velocity;
  s.update(params,8);
  return s;
}


//: extract pose, velocity, and shape from a single state vector
void 
modrec_vehicle_state::set_state(const vnl_vector<double>& s)
{
  translation.set(s[0],s[1],s[2]);
  vnl_vector_fixed<double,3> r(s.data_block()+3);
  rotation = vgl_rotation_3d<double>(r);
  t_velocity = s[6];
  a_velocity = s[7];
  params.set_size(s.size()-8);
  params.set(s.data_block()+8);
}


//: initialize the covariance matrix 
void modrec_vehicle_state::init_covar(bool estimate_shape)
{
  unsigned int num = 8;
  if(estimate_shape)
    num += params.size();
  covar.set_size(num,num);
  covar.set_identity();
  //covar *= 0.5;
  covar(2,2) = 0.01; //tz
  covar(3,3) = 0.01; //rx
  covar(4,4) = 0.01; //ry
  covar(6,6) = 10.0; // t_velocity
  covar(7,7) = 0.5; // a_velocity
}


//: predict the next state (after time t elapsed) using the circular motion model
modrec_vehicle_state 
modrec_circ_motion_predict(const modrec_vehicle_state& s, double t)
{
  modrec_vehicle_state st;
  st.last_translation = s.translation;
  st.last_rotation = s.rotation;
  st.num_frames = s.num_frames;
  st.unique_id = s.unique_id;
  
  const double& va = s.a_velocity;
  const double& vt = s.t_velocity;
  vnl_double_3 w = s.rotation.as_rodrigues();
  double a_z = w[2]; 
  w[0] = 0.0;
  w[1] = 0.0;
  w[2] += va*t;
  st.rotation = vgl_rotation_3d<double>(w);
  bool no_rotate = vcl_abs(va) < 1e-8;
  double da_x = no_rotate ? t*vcl_cos(a_z) : (vcl_sin(w[2])-vcl_sin(a_z))/va;
  double da_y = no_rotate ? -t*vcl_sin(a_z) : (vcl_cos(w[2])-vcl_cos(a_z))/va;
  st.translation = vgl_vector_3d<double>(s.translation.x()+vt*da_x,
                                         s.translation.y()-vt*da_y, 0.0);
  st.t_velocity = vt;
  st.a_velocity = va;
  double a = 1.0/(s.num_frames);
  st.mean_params = (1-a)*s.mean_params + a*s.params;
  st.params = s.params;
  
  st.covar = s.covar;
  vnl_matrix<double>& C = st.covar;
  
  
  if(C.rows() == C.cols() && (C.rows() == st.params.size()+8 || C.rows() == 8))
  {
    // compute F'*C*F without explicitly computing sparse matrix F
    double dda_x = no_rotate ? -0.5*t*t*vcl_sin(a_z) : (t*vcl_cos(w[2])-da_x)/va;
    double dda_y = no_rotate ? 0.5*t*t*vcl_cos(a_z) : (t*vcl_sin(w[2])+da_y)/va;
    C.set_row(2,0.0);
    C.set_row(3,0.0);
    C.set_row(4,0.0);
    C.set_row(0,C.get_row(0) 
              + (vt*da_y)*C.get_row(5)
              + da_x*C.get_row(6)
              + (vt*dda_x)*C.get_row(7));
    C.set_row(1,C.get_row(1) 
              + (vt*da_x)*C.get_row(5)
              + -da_y*C.get_row(6)
              + (vt*dda_y)*C.get_row(7));
    C.set_row(5,C.get_row(5) 
              + t*C.get_row(7));
    
    C.set_column(2,0.0);
    C.set_column(3,0.0);
    C.set_column(4,0.0);
    C.set_column(0,C.get_column(0) 
                 + (vt*da_y)*C.get_column(5)
                 + da_x*C.get_column(6)
                 + (vt*dda_x)*C.get_column(7));
    C.set_column(1,C.get_column(1) 
                 + (vt*da_x)*C.get_column(5)
                 + -da_y*C.get_column(6)
                 + (vt*dda_y)*C.get_column(7));
    C.set_column(5,C.get_column(5) 
                 + t*C.get_column(7));
    
    // add Q
    C(2,2) = 0.01;
    C(3,3) = 0.01;
    C(4,4) = 0.01;
    C(0,0) += 0.1/(s.num_frames+1) + 0.01;
    C(1,1) += 0.1/(s.num_frames+1) + 0.01;
    C(5,5) += 0.05/(s.num_frames+1) + 0.005;
    C(6,6) += 1.0/(s.num_frames+1) + 0.1;
    C(7,7) += 0.5/(s.num_frames+1) + 0.05;
    for(unsigned int i=8; i<C.rows(); ++i)
      C(i,i) += 0.1/(s.num_frames+1);
  }
  else
    vcl_cout << "covariance not correct size"<<vcl_endl;

  
  return st;
}


//: predict the position only (after time t elapsed) using the circular motion model
vgl_vector_3d<double> 
modrec_circ_motion_predict_position(const modrec_vehicle_state& s, double t)
{
  const double& va = s.a_velocity;
  const double& vt = s.t_velocity;
  double w = s.rotation.as_rodrigues()[2];
  double a_z = w; 
  w += va*t;
  double da_x = (va==0.0) ? t*vcl_cos(a_z) : (vcl_sin(w)-vcl_sin(a_z))/va;
  double da_y = (va==0.0) ? -t*vcl_sin(a_z) : (vcl_cos(w)-vcl_cos(a_z))/va;
  return vgl_vector_3d<double>(s.translation.x()+vt*da_x,
                               s.translation.y()-vt*da_y, 0.0);
}


//: read a tracking result file
unsigned int
modrec_read_track_file(const vcl_string& filename, 
                       vcl_string& vid_file, vcl_string& cam_file, vcl_string& model_type,
                       vcl_map<unsigned int,vcl_vector<modrec_vehicle_state> >& state_map)
{
  vcl_ifstream ifs(filename.c_str());

  // load model type
  ifs >> model_type;
  
  // read the starting frame
  unsigned int start_frame;
  ifs >> start_frame;

  // read video file path
  ifs >> vid_file;
  
  // load camera file path
  ifs >> cam_file;
  
  
  unsigned int frame_number = 0;
  while(ifs >> frame_number)
  {
    modrec_vehicle_state state;
    // read the unique id 
    ifs >> state.unique_id;
    
    char data[4096];
    ifs.getline(data,4096);
    vcl_stringstream sdata(data);
    
    vnl_vector_fixed<double,3> r;
    double tv,av;
    vnl_vector<double> p;
    sdata >> state.translation >> r >> state.t_velocity >> state.a_velocity >> state.params;
    state.rotation = vgl_rotation_3d<double>(r);

    state_map[frame_number].push_back(state);
  }
  return start_frame;
}

