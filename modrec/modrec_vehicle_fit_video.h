// This is mleotta/gui/pca_vehicle/modrec_vehicle_fit_video.h
#ifndef modrec_vehicle_fit_video_h_
#define modrec_vehicle_fit_video_h_
//=========================================================================
//:
// \file
// \brief  Fit the PCA vehicle in a video sequence
// \author Matt Leotta (mleotta)
// \date 04/11/2009
//
// \verbatim
//  Modifications
//   04/11/2009 - File created. (mleotta)
// \endverbatim
//=========================================================================


#include <modrec/modrec_vehicle_fit.h>
#include <modrec/modrec_vehicle_tracker.h>
#include <modrec/modrec_vehicle_state.h>
#include <vidl/vidl_istream_sptr.h>

//: Fit a PCA vehicle model with parts to video
class modrec_vehicle_fit_video : public modrec_vehicle_fit
{
public:
  
  //: Constructor
  modrec_vehicle_fit_video();
  
  //: clear all the data and reset to the initial single frame setup
  virtual void reset();
  
  //: Assign an edge map
  void set_edge_map(const vil_image_view<float>& edge_map);
  //: Assign a vehicle silhouette
  void set_silhouette(const vgl_polygon<double>& sil);
  //: Compute the edge map from the image \a img
  void detect_and_set_edges(const vil_image_view<vxl_byte>& img);
  //: Assign a camera
  void set_camera(const vpgl_perspective_camera<double>& camera);
  //: Set the sun direction for shadow casting
  void set_sun_direction(const vgl_vector_3d<double>& sun_dir);
  //: set the video input stream
  void set_istream(const vidl_istream_sptr& istream);
  //: Assign the PCA vehicle model
  void set_vehicle_model(const modrec_pca_vehicle& vehicle);
  
  //: enable or disable tracking using silhouette matching terms
  void set_track_with_silhouette(bool val);
    
  //: Access the edge map
  const vil_image_view<float>& edge_map() const;
  //: Access the silhouette polygon
  const vgl_polygon<double>& silhouette() const;
  //: Access the camera
  const vpgl_perspective_camera<double>& camera() const;
  //: Access the vehicle projector
  const modrec_pca_vehicle_projector& projector() const;
  
  //: Apply fitting to correct the predicted state
  bool correct_state(modrec_vehicle_state& state);
  
  
  //: advance the video and process on frame
  dbpro_signal process_once() { return tracker_.process_once(); }
  
  //: enable tracking (otherwise only BG modeling is run)
  void enable_tracking(bool enable) { tracker_.enable_tracking(enable); }

  //: Provide access to the tracker
  modrec_vehicle_tracker& tracker() { return tracker_; }
  
  //: Compute silhouette errors
  static void 
  compute_silhouette_errors(const vcl_vector<vgl_point_2d<double> >& pts,
                            const vcl_vector<vgl_vector_2d<double> >& norms,
                            const vgl_polygon<double>& silhouette,
                            vcl_vector<double>& errors);
  
  //: Estimate the initial M-estimator scale from parameter standard deviations
  virtual double estimate_initial_scale(const modrec_pca_vehicle& mesh,
                                        const vgl_vector_3d<double>& translation,
                                        const vgl_rotation_3d<double>& rotation,
                                        const vnl_vector<double>& sigma);

private:
  
  //: Compute the terms for optimization from all active views.
  //  Adds to the matrix \a M and vector \a b passed in.
  //  Only recompute curve visiblity when \a compute_visibility is true.
  virtual void 
  compute_all_opt_terms(const modrec_pca_vehicle& mesh,
                        const vgl_vector_3d<double>& translation,
                        const vgl_rotation_3d<double>& rotation,
                        vnl_matrix<double>& M,
                        vnl_vector<double>& b,
                        double& total_weight,
                        double& wgt_residual,
                        bool compute_visiblity);
  
  //: Compute the residuals of optimization from all active views.
  virtual void 
  compute_all_residuals(const modrec_pca_vehicle& mesh,
                        const vgl_vector_3d<double>& translation,
                        const vgl_rotation_3d<double>& rotation,
                        double& total_weight,
                        double& wgt_residual);
  
  //: Compute the prior terms and update M and b
  virtual double 
  compute_prior(const modrec_pca_vehicle& mesh,
                const vgl_vector_3d<double>& translation,
                const vgl_rotation_3d<double>& rotation,
                      vnl_matrix<double>& M, 
                      vnl_vector<double>& b);
  
  //: Compute the optimization terms involving the silhouette and update M and b
  double 
  compute_silhouette_opt_terms(vnl_matrix<double>& M, 
                               vnl_vector<double>& b);
  
  //: Rotate the pose rows and columns of the covariance
  vnl_matrix<double>
  rotate_covariance(const vgl_rotation_3d<double>& rotation,
                    const vnl_matrix<double>& covar) const;
  
  //: Construct the fitting priors from the tracking state
  //: \note The state may have additional parameters to be stripped out,
  //        and parameters may be in a different order.
  void set_priors_from_track(const modrec_vehicle_state& state);
  
  //: Update the tracker covariance with the residual fitting covariance
  //  The residual covariance needs to be rotated back to world orientation
  void update_posterior_covariance(const modrec_pca_vehicle& mesh,
                                   const vgl_vector_3d<double>& t,
                                   const vgl_rotation_3d<double>& R,
                                   vnl_matrix<double>& covar,
                                   unsigned int num_frames);
  
  //: Apply the solution vector to update the shape and pose parameters
  virtual void apply_solution(const vnl_vector<double>& soln,
                              modrec_pca_vehicle& mesh,
                              vgl_vector_3d<double>& translation,
                              vgl_rotation_3d<double>& rotation);
  

  //: the edge map image
  vil_image_view<float> edge_map_;
  //: the silhouette from background modeling
  vgl_polygon<double> silhouette_;
  //: the camera
  vpgl_perspective_camera<double> camera_;
  //: the direction of sunlight for shadow casting
  vgl_vector_3d<double> sun_dir_;
  //: the vehicle projector class
  modrec_pca_vehicle_projector projector_;
  //: The deformable vehicle model
  modrec_pca_vehicle vehicle_;
  //: helper class for tracking
  modrec_vehicle_tracker tracker_;
  
  //: enable silhouette terms in tracking optimization
  bool track_with_silhouette_;
  
  //: prior parameter vector mean
  vnl_vector<double> prior_mean_;
  //: prior parameter vector inverse covariance (in vehicle_fit order)
  vnl_matrix<double> prior_inv_covar_;
  //: prior parameter vector covariance (in vehicle_state order)
  vnl_matrix<double> state_inv_covar_;
  
  //: current estimate of translational velocity
  double t_velocity_;
  //: current estimate of angular velocity
  double a_velocity_;

};




#endif // modrec_vehicle_fit_video_h_
