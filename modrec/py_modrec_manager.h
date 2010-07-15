// This is contrib/mleotta/modrec/py_modrec_manager.h
#ifndef py_modrec_manager_h_
#define py_modrec_manager_h_
//----------------------------------------------------------------------------
//:
// \file
// \brief Manager for python interface
// \author
//   Matt Leotta
//

//-----------------------------------------------------------------------------


#include <modrec/modrec_vehicle_fit_multiview.h>
#include <modrec/modrec_vehicle_fit_video.h>
#include <imesh/algo/imesh_kd_tree.h>


//: A manager for the gui
class py_modrec_manager
{
public:
  enum vehicle_model { DODECAHEDRAL, FERRYMAN, DETAILED1, DETAILED2, DETAILED3 };
  
  static py_modrec_manager *instance();
  
  //: load a camera matrix for frame \a i
  bool set_camera(unsigned int i, const vcl_string& filename);
  
  //: compute the sun direction for shadow casting
  void compute_sun_direction();
  
  //: load an image file for frame \a i
  bool set_image(unsigned int i, const vcl_string& filename);
  
  //: load input video file 
  bool set_video(const vcl_string& filename);
  
  //: clear all images and cameras
  void reset_views();
  
  //: load the parts from a file
  bool set_parts(const vcl_string& filename);
  
  //: load the pca parameters from a file
  bool set_pca(const vcl_string& filename);
  
  //: load the mesh and projected into PCA space
  bool set_mesh(const vcl_string& filename, 
                const vcl_string& partsfile = "");
  
  //: Choose the type of vehicle model from the enum
  void set_vehicle_model(vehicle_model vm);
  
  //: load the polyhedral approximation mesh
  bool set_poly_mesh(const vcl_string& filename);
  
  //: set the options for which parameters to optimize
  void set_options(const vcl_vector<bool>& options,
                   unsigned int num_pc);
  
  //: set the scale of the initial uncertainty
  void set_init_uncert(double uncert);
  
  //: enable or disable tracking with silhouette terms
  void set_track_with_silhouette(bool val);
  
  //: set the translation vector
  void set_translation(double tx, double ty, double tz);
  
  //: set the rotation vector
  void set_rotation(double rx, double ry, double rz);
  
  //: set the parameter vector
  void set_params(const vcl_vector<double>& params);
  
  //: set the regularization factor
  void set_lambda(double lambda);
  
  //: set the states produced by the tracker
  void set_tracking_states(const vcl_vector<modrec_vehicle_state>& states);
  
  //: Enable video mode (true), or multiview mode (false)
  void set_fit_mode(bool use_video);
  
  //: Is the fitting mode video (true) or multiview (false)
  bool is_fit_mode_video() const;  
  
  //: fit the vehicle model to the data
  void fit_model(unsigned int max_itr);
  
  //: write an svg file with the last projected curves in view \a i
  bool write_svg_curves(unsigned int i, const vcl_string& filename) const;
  
  //: load a mesh as the ground truth mesh
  bool set_truth_mesh(const vcl_string& filename);
  
  //: compute the RMS error between the PCA mesh and Ground truth
  double compute_error() const;
  
  //: return the number of principal components optimized
  unsigned int num_pc() const;
  
  //: enable or disable tracking
  void enable_tracking(bool enable_track);
  
  //: seek the video to the frame number
  void video_seek(int frame);
  
  //: advance the video to the next frame
  bool advance_video();
  
  //: return the current video frame number
  int current_frame() const;
  
  //: evaluate the fitting residual at the current state and given scale
  double evaluate_residual(double scale, bool compute_visibility);
  
  //: get the translation vector
  void get_translation(double& tx, double& ty, double& tz) const;
  
  //: get the rotation vector
  void get_rotation(double& rx, double& ry, double& rz) const;
  
  //: get the parameter vector
  void get_params(vcl_vector<double>& params) const;
  
  //: get the type of vehicle model used
  void get_vehicle_model(vehicle_model& vm);
  
  //: get the current vehicle tracking states
  const vcl_vector<modrec_vehicle_state>& get_vehicle_states() const;
  
  //: compute the edgel coverage of the contours and part boundaries in the last projection
  void compute_edgel_coverage(double dist_thresh,
                              unsigned int& num_contour_match,
                              unsigned int& num_contour_total,
                              unsigned int& num_part_match,
                              unsigned int& num_part_total) const;
  
  //: compute edge coverage for on-vehicle-edges for mesh and poly_mesh
  void compare_relative_coverage(double dist_thesh,  
                                 unsigned int& num_match,
                                 unsigned int& num_poly_match,
                                 unsigned int& num_total);
  
  
private:
  py_modrec_manager();
  ~py_modrec_manager() {}
  void init_mesh();
  //: Initialize the texture map for detailed meshes
  void init_mesh_tex(modrec_pca_vehicle& mesh);
  
  static py_modrec_manager *instance_;
  
  modrec_vehicle_fit* optimizer_;
  modrec_vehicle_fit_multiview mv_optimizer_;
  modrec_vehicle_fit_video video_optimizer_;
  
  modrec_pca_vehicle *mesh_;
  //: dodecahedral mesh
  modrec_pca_vehicle dodec_mesh_;
  //: Ferryman's mesh
  modrec_pca_vehicle ferryman_mesh_;
  //: detailed mesh with parts at 3 resolutions
  modrec_pca_vehicle detailed1_mesh_,detailed2_mesh_,detailed3_mesh_;
  
  vgl_vector_3d<double> t_;
  vgl_rotation_3d<double> R_;
  
  vcl_vector<modrec_vehicle_state> vehicle_states_;
  
  //: the video stream
  vidl_istream_sptr istream_;
  int frame_number_;
  
  
  bool norm_cam_;
  vpgl_calibration_matrix<double> norm_K_;
  
  //: data for computing the direction of the sun for shadow casting
  vgl_vector_3d<double> sun_dir_;
  double solar_lat_;
  double solar_lon_;
  int solar_day_;
  double solar_utc_;
  double solar_atn_;
  
  imesh_mesh truth_mesh_;
  imesh_mesh poly_mesh_;
  vcl_auto_ptr<imesh_kd_tree_node> truth_kd_tree_;

};

#endif // py_modrec_manager_h_
