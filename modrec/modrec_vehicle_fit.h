// This is mleotta/gui/pca_vehicle/modrec_vehicle_fit.h
#ifndef modrec_vehicle_fit_h_
#define modrec_vehicle_fit_h_
//=========================================================================
//:
// \file
// \brief  Fit the PCA vehicle (base class)
// \author Matt Leotta (mleotta)
// \date 11/02/2008
//
// \verbatim
//  Modifications
//   11/02/2008 - File created. (mleotta)
//   04/11/2009 - Split off as a base class for code reuse
// \endverbatim
//=========================================================================


#include <modrec/modrec_pca_vehicle_projector.h>
#include <modrec/modrec_edgel.h>
#include <vcl_vector.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <vpgl/vpgl_perspective_camera.h>



//: A base class for fitting a PCA vehicle model with parts
class modrec_vehicle_fit 
{
public:
  
  //: Constructor
  modrec_vehicle_fit();
  
  //: clear all the data and reset to the initial setup
  virtual void reset() = 0;
  
  //: Compute the edge map for image \a img
  void detect_edges(const vil_image_view<vxl_byte>& img,
                    vil_image_view<float>& edge_map) const;
 
  //: set optimization options
  //  The vector of boolean options specifies which parameters to fit.
  //  - [0] is top \a num_pc PCA params
  //  - [1,2,3] is Tx,Ty,Tz respectively
  //  - [4,5,6] is Rx,Ry,Rz respectively
  void set_options(const vcl_vector<bool>& options, 
                   unsigned int num_pc);
  
  //: Return the number of principal components to optimize
  unsigned int num_pc() const { return num_pc_; }
  //: Return the total number of parameters to optimize
  unsigned int num_params() const { return num_params_; }

  //: Set the value of lambda
  void set_lambda(double lambda) { lambda_ = lambda; }
  //: Set the m-estimator scale
  void set_mest_scale(double scale) { mest_scale_ = scale; }
  //: Set the initial uncertaintly
  void set_init_uncert(const vnl_vector<double>& uncert) { init_uncert_ = uncert; }
  //: Set the initial uncertaintly to a scale times the default uncertainty
  void set_init_uncert(double uncert = 1.0);
  
  //: Return the minimum edge strength
  double min_edge_strength() const { return min_e_strength_; }
  //: Return the maximum edge strength
  double max_edge_strength() const { return max_e_strength_; }
  //: Return the value of labmda
  double lambda() const { return lambda_; }
  //: Return the scale of the m-estimator function
  double mest_scale() const { return mest_scale_; }
  //: Return the last covariance matrix estimate (computed during fit_model)
  vnl_matrix<double> last_covar() const { return last_covar_; }
  //: Return the initial uncertainty
  vnl_vector<double> init_uncert() const { return init_uncert_; }

  //: Fits the model parameters: PCA, translation, rotation to the image.
  void fit_model(unsigned int num_itr, 
                 modrec_pca_vehicle& mesh,
                 vgl_vector_3d<double>& t,
                 vgl_rotation_3d<double>& R);
  
  //: One iteration of fitting PCA, translation, rotation to the image.
  // return true if the iteration successfully reached a lower residual
  bool fit_model_once(modrec_pca_vehicle& mesh,
                        vgl_vector_3d<double>& translation,
                        vgl_rotation_3d<double>& rotation,
                        vnl_vector<double>& soln,
                        double& last_residual,
                        bool compute_visibility);
  
  //: Evaluate the residual at the current state and scale
  // this function is for generating a plot of the error surface
  double evaluate_residual(const modrec_pca_vehicle& mesh,
                           const vgl_vector_3d<double>& t,
                           const vgl_rotation_3d<double>& R,
                           double scale,
                           bool compute_visibility = true);
  
  //: Compute the edgel matches and weights for the last projection of \a projector
  // This is primarily for debugging
  void last_edgel_matches(const modrec_pca_vehicle_projector& projector, 
                          const vil_image_view<float>& edge_map,
                          vcl_vector<vgl_point_2d<double> >& edgel_snaps,
                          vcl_vector<vgl_point_2d<double> >& edgels,
                          vcl_vector<double>& edgel_weights) const;
  
  //: Compute the number of pixel size edge points with an edge neighbor within \a dist_thresh
  // return the total samples and number of matches for both contours and part boundaries
  void last_edgel_coverage(const modrec_pca_vehicle_projector& projector, 
                           const vil_image_view<float>& edge_map,
                           double dist_thresh,
                           unsigned int& num_contour_match,
                           unsigned int& num_contour_total,
                           unsigned int& num_part_match,
                           unsigned int& num_part_total) const;
  
  //: Estimate the initial M-estimator scale from parameter standard deviations
  virtual double estimate_initial_scale(const modrec_pca_vehicle& mesh,
                                        const vgl_vector_3d<double>& translation,
                                        const vgl_rotation_3d<double>& rotation,
                                        const vnl_vector<double>& sigma) = 0; 


protected:
  
  //: Finds the set of all edge matches near the line segment from \a p0 to \a p1
  //  Matches are returned as a 3-d vector where index
  // - 0 is a value in [0.0, 1.0] indicating location of the projection between the endpoints
  // - 1 is the perpendicular distance to the edgel
  // - 2 is the weight for use in optimization
  void compute_line_matches(const vgl_point_2d<double>& p0, 
                            const vgl_point_2d<double>& p1,
                            const vil_image_view<float>& edge_map,
                            vcl_vector<vnl_vector_fixed<double,3> >& matches) const;
  
  //: Compute the terms for optimization from a set of curves and Jacobians
  //  adds to the matrix \a M and vector \a b passed in
  void 
  compute_curve_opt_terms(const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves,
                          const vcl_vector<vcl_vector<vnl_matrix<double> > >& J,
                          const vil_image_view<float>& edge_map,
                          vnl_matrix<double>& M,
                          vnl_vector<double>& b,
                          double& total_weight,
                          double& wgt_residual) const;
  
  //: Compute the residuals of the optimization
  void
  compute_curve_residuals(const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves,
                          const vil_image_view<float>& edge_map,
                          double& total_weight,
                          double& wgt_residual) const;
  
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
                        bool compute_visiblity) = 0;
  
  //: Compute the residuals of optimization from all active views.
  virtual void 
  compute_all_residuals(const modrec_pca_vehicle& mesh,
                        const vgl_vector_3d<double>& translation,
                        const vgl_rotation_3d<double>& rotation,
                        double& total_weight,
                        double& wgt_residual) = 0;
  
  //: Compute the prior terms and update M and b
  // \return the residual of the prior
  virtual double compute_prior(const modrec_pca_vehicle& mesh,
                               const vgl_vector_3d<double>& translation,
                               const vgl_rotation_3d<double>& rotation,
                               vnl_matrix<double>& M, 
                               vnl_vector<double>& b);
  
  //: Compute the edgel matches and weights from a set of curves 
  // This is primarily for debugging
  void compute_edgel_matches(const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves,
                             const vil_image_view<float>& edge_map,
                             vcl_vector<vgl_point_2d<double> >& edgel_snaps,
                             vcl_vector<vgl_point_2d<double> >& edgels,
                             vcl_vector<double>& edgel_weights) const;

  //: Compute the number of pixel size edge points with an edge neighbor within \a dist_thresh
  // return the total samples and number of matches
  void compute_edgel_coverage(const vcl_vector<vcl_vector<vgl_point_2d<double> > >& curves, 
                              const vil_image_view<float>& edge_map,
                              double dist_thresh,
                              unsigned int& num_match,
                              unsigned int& num_total) const;
  
  //: Apply the solution vector to update the shape and pose parameters
  virtual void apply_solution(const vnl_vector<double>& soln,
                              modrec_pca_vehicle& mesh,
                              vgl_vector_3d<double>& translation,
                              vgl_rotation_3d<double>& rotation);
  
  //: estimate the initial scale given the set of Jacobians
  double estimate_initial_scale(const vnl_vector<double>& sigma,
                                const vcl_vector<vcl_vector<vnl_matrix<double> > >& J);
 
  
  //: the minimum edge strength considered for weighting
  double min_e_strength_;
  //: the maximum edge strength considered for weighting
  double max_e_strength_;
  
  //: the amount of regularization
  double lambda_;
  //: the distance to search for matching edgels
  double mest_scale_;
  //: the image border size in which weights are linearly droped
  double border_;
  
  //: total number of parameters to optimize
  unsigned int num_params_;
  //: number of shape principal components to optimize
  unsigned int num_pc_;
  //: the options vector
  vcl_vector<bool> options_;
  //: indicies for extrinsic parameters (-1 for disabled)
  int txi_, tyi_, tzi_, rxi_, ryi_, rzi_;
  
  
  //: initial uncertainty (standard deviation) in each parameter
  vnl_vector<double> init_uncert_;
  //: the final covariance estimate after the last fitting operation
  vnl_matrix<double> last_covar_;

};




#endif // modrec_vehicle_fit_h_
