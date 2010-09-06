// This is dml/dml_vehicle_fit_video.cxx
//=========================================================================
//:
// \file
// \brief Fit the PCA vehicle to a video
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
//=========================================================================

#include "dml_vehicle_fit_video.h"
#include <vcl_limits.h>
#include <vcl_cstdlib.h>
#include <vgl/vgl_lineseg_test.h>
#include <vgl/vgl_intersection.h>
#include <dml/dml_vehicle_tracker.h>
#include <vgl/vgl_area.h>
#include <spl/spl_observer.h>
#include <vil/vil_image_resource.h>
#include <spl/filters/vidl_source.h>
#include <vnl/algo/vnl_cholesky.h>

#include <vnl/algo/vnl_symmetric_eigensystem.h>

//: update the tracker with the edge map
class dml_edgemap_observer: public spl_observer
{
public:
  dml_edgemap_observer(dml_vehicle_fit_video& opt)
  : optimizer(opt) {}
  
  //: Called by the process when the data is ready
  virtual bool notify(const spl_storage_sptr& data, unsigned long timestamp)
  {
    assert(data);
    if(data->info() == SPL_VALID){
      assert(data->type_id() == typeid(vil_image_resource_sptr));
      vil_image_resource_sptr edge_rsc = data->data<vil_image_resource_sptr>();
      if(edge_rsc){
        vil_image_view<float> edge_map = edge_rsc->get_view();
        if(edge_map)
          optimizer.set_edge_map(edge_map);
      }
    }
    return true;
  }
  dml_vehicle_fit_video& optimizer;
};



//=============================================================================


//: Constructor
dml_vehicle_fit_video::dml_vehicle_fit_video()
: sun_dir_(0.0,0.0,0.0), tracker_(this), track_with_silhouette_(false)
{
  vgl_point_3d<double> c(5,5,5);
  vpgl_calibration_matrix<double> K(1200,vgl_point_2d<double>(512,386));
  camera_ = vpgl_perspective_camera<double>(K,c,vgl_rotation_3d<double>());
  camera_.look_at(vgl_homg_point_3d<double>(0,0,0.5));
  tracker_.set_camera(camera_);
  
  tracker_.add_edgemap_observer(new dml_edgemap_observer(*this));
}


//: clear all the data and reset to the initial single frame setup
void dml_vehicle_fit_video::reset()
{
  
}


//: Assign an edge map
void dml_vehicle_fit_video::set_edge_map(const vil_image_view<float>& edge_map)
{
  edge_map_ = edge_map;
  projector_.resize(edge_map.ni(),edge_map.nj());
}


//: Assign a vehicle silhouette
void dml_vehicle_fit_video::set_silhouette(const vgl_polygon<double>& sil)
{
  silhouette_ = sil;
}


//: Assign a camera 
void dml_vehicle_fit_video::set_camera(const vpgl_perspective_camera<double>& camera)
{
  camera_ = camera;
  tracker_.set_camera(camera);
}


//: Set the sun direction for shadow casting
void dml_vehicle_fit_video::set_sun_direction(const vgl_vector_3d<double>& sun_dir)
{
  sun_dir_ = sun_dir;
  tracker_.set_sun_direction(sun_dir);
}


//: set the video input stream
void dml_vehicle_fit_video::set_istream(const vidl_istream_sptr& istream)
{
  tracker_.set_istream(istream);
}


//: enable or disable tracking using silhouette matching terms
void dml_vehicle_fit_video::set_track_with_silhouette(bool val)
{
  track_with_silhouette_ = val;
}


//: Assign the PCA vehicle model
void dml_vehicle_fit_video::
set_vehicle_model(const dml_pca_vehicle& vehicle)
{
  vehicle_ = vehicle;
  tracker_.set_vehicle_model(vehicle);
}


//: Access the edge map 
const vil_image_view<float>& 
dml_vehicle_fit_video::edge_map() const
{ 
  return edge_map_;
}

//: Access the silhouette polygon
const vgl_polygon<double>& 
dml_vehicle_fit_video::silhouette() const
{
  return silhouette_;
}


//: Access the camera
const vpgl_perspective_camera<double>& 
dml_vehicle_fit_video::camera() const
{ 
  return camera_; 
}


//: Access the vehicle projector 
const dml_pca_vehicle_projector& 
dml_vehicle_fit_video::projector() const
{ 
  return projector_; 
}


//: Compute the edge map from the image \a img
void dml_vehicle_fit_video::
detect_and_set_edges(const vil_image_view<vxl_byte>& img)
{  
  // detect edges using the base class function
  detect_edges(img,edge_map_);
  projector_.resize(edge_map_.ni(),edge_map_.nj());
}


//: Compute the terms for optimization from all active views
//  appends terms to the vector references passed in
void dml_vehicle_fit_video::
compute_all_opt_terms(const dml_pca_vehicle& mesh,
                      const vgl_vector_3d<double>& translation,
                      const vgl_rotation_3d<double>& rotation,
                      vnl_matrix<double>& M,
                      vnl_vector<double>& b,
                      double& total_weight,
                      double& wgt_residual,
                      bool compute_visiblity)
{  
  if(compute_visiblity)
    projector_.project(camera_,mesh,rotation,translation,sun_dir_,options_,num_pc_);
  else
    projector_.reproject(camera_,mesh,rotation,translation,sun_dir_,options_,num_pc_);
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& contours = projector_.contours();
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& parts = projector_.parts();
  
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jc = projector_.contours_jacobians();
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jp = projector_.parts_jacobians();
      
  compute_curve_opt_terms(contours, Jc, edge_map_, M, b, total_weight, wgt_residual);
  compute_curve_opt_terms(parts, Jp, edge_map_, M, b, total_weight, wgt_residual);
  
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& silhouette = projector_.silhouette();
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jsil = projector_.silhouette_jacobians();
  const vcl_vector<bool>& is_shadow = projector_.silhouette_shadow();
  
  vcl_vector<vcl_vector<vgl_point_2d<double> > > shadow;
  vcl_vector<vcl_vector<vnl_matrix<double> > > Js;
  for(unsigned int i=0; i<silhouette.size(); ++i){
    if(is_shadow[i]){
      shadow.push_back(silhouette[i]);
      Js.push_back(Jsil[i]);
    }
  }
  if(!shadow.empty())
    compute_curve_opt_terms(shadow, Js, edge_map_, M, b, total_weight, wgt_residual);
  
#if 0
  unsigned int num_contour_match=0;
  unsigned int num_contour_total=0;
  unsigned int num_part_match=0;
  unsigned int num_part_total=0;
  last_edgel_coverage(projector_, edge_map_, 1.0,
                      num_contour_match,
                      num_contour_total,
                      num_part_match,
                      num_part_total);
  vcl_cout << "coverage = " << double(num_contour_match+num_part_match)/
  (num_contour_total+num_part_total) << vcl_endl;
#endif
}


//: Compute the residuals of optimization from all active views.
void dml_vehicle_fit_video::
compute_all_residuals(const dml_pca_vehicle& mesh,
                      const vgl_vector_3d<double>& translation,
                      const vgl_rotation_3d<double>& rotation,
                      double& total_weight,
                      double& wgt_residual)
{
  projector_.reproject(camera_,mesh,rotation,translation,sun_dir_,options_,num_pc_);
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& contours = projector_.contours();
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& parts = projector_.parts();
  
  compute_curve_residuals(contours, edge_map_, total_weight, wgt_residual);
  compute_curve_residuals(parts, edge_map_, total_weight, wgt_residual);
}


//: Compute silhouette errors
void dml_vehicle_fit_video::
compute_silhouette_errors(const vcl_vector<vgl_point_2d<double> >& pts,
                          const vcl_vector<vgl_vector_2d<double> >& norms,
                          const vgl_polygon<double>& silhouette,
                                vcl_vector<double>& errors)
{
  assert(pts.size() == norms.size());
  if(silhouette.num_sheets() == 0)
    return;
  const vcl_vector<vgl_point_2d<double> >& tgt = silhouette[0];
  errors.clear();
  errors.resize(pts.size(),vcl_numeric_limits<double>::infinity());
  
  for(unsigned int i=0; i<pts.size(); ++i){
    const vgl_point_2d<double>& pt = pts[i];
    vgl_vector_2d<double> n = normalized(norms[i]);
    vgl_point_2d<double> ptn = pt+n;
    vgl_line_2d<double> rayline(pt,n);
    for(unsigned int j1=0, j2=tgt.size()-1; j1<tgt.size(); j2 = j1++)
    {
      const vgl_point_2d<double>& t1 = tgt[j1];
      const vgl_point_2d<double>& t2 = tgt[j2];
      
      // ignore edges that don't match orientation
      vgl_vector_2d<double> n2(t1.y()-t2.y(),t2.x()-t1.x());
      normalize(n2);
      if(dot_product(n,n2) < 0)
        continue;
      
      // test for intersection
      if(!vgl_lineseg_test_line(pt.x(),pt.y(),ptn.x(),ptn.y(),
                                t1.x(),t1.y(),t2.x(),t2.y()) )
        continue;
      
      // compute intersection
      vgl_point_2d<double> pi;
      if(!vgl_intersection(rayline, vgl_line_2d<double>(t1,t2), pi))
        continue;
      
      // test for closest point
      double d = dot_product(n,(pi-pt));
      if(vcl_abs(d) > vcl_abs(errors[i]))
        continue;
      
      errors[i] = d;
    }
    // repeat tests on source polygon to rule out self intersection
    for(unsigned int j1=0, j2=pts.size()-1; j1<pts.size(); j2 = j1++)
    {
      if(j1==i || j2 == i)
        continue;
      
      const vgl_point_2d<double>& t1 = pts[j1];
      const vgl_point_2d<double>& t2 = pts[j2];
      
      if(t1 == t2)
        continue;
      
      // test for intersection
      if(!vgl_lineseg_test_line(pt.x(),pt.y(),ptn.x(),ptn.y(),
                                t1.x(),t1.y(),t2.x(),t2.y()) )
        continue;
      
      // compute intersection
      vgl_point_2d<double> pi;
      if(!vgl_intersection(rayline, vgl_line_2d<double>(t1,t2), pi))
        continue;
      
      // test for closest point
      double d = dot_product(n,(pi-pt));
      if(vcl_abs(d) > vcl_abs(errors[i]))
        continue;
      
      // reset this match to infinity to disable it
      errors[i] = vcl_numeric_limits<double>::infinity();
      break;
    }
  }
} 


//: Estimate the initial M-estimator scale from parameter standard deviations
double dml_vehicle_fit_video::
estimate_initial_scale(const dml_pca_vehicle& mesh,
                       const vgl_vector_3d<double>& translation,
                       const vgl_rotation_3d<double>& rotation,
                       const vnl_vector<double>& sigma)
{
  
  projector_.project(camera_,mesh,rotation,translation,vgl_vector_3d<double>(0,0,0),options_,num_pc_);
  
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jc = projector_.contours_jacobians();
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jp = projector_.parts_jacobians();
  
  vcl_vector<vcl_vector<vnl_matrix<double> > > J(Jc);
  J.insert(J.end(), Jp.begin(), Jp.end());
  
  return dml_vehicle_fit::estimate_initial_scale(sigma,J);
}


//: Compute the optimization terms involving the silhouette and update M and b
double dml_vehicle_fit_video::
compute_silhouette_opt_terms(vnl_matrix<double>& M, 
                             vnl_vector<double>& b)
{  
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >& sil = projector_.silhouette();
  
  // abort if predicted or detected silhouettes are not found
  if(silhouette_.num_sheets() != 1 || sil.empty())
    return 0.0;
  
  // compute the silhouette points and normal vectors
  vcl_vector<vgl_point_2d<double> > pts;
  vcl_vector<vgl_vector_2d<double> > norms;
  for(unsigned int i=0; i<sil.size(); ++i){
    const vcl_vector<vgl_point_2d<double> >& si = sil[i];
    if(si.size() < 2)
      continue;
    vgl_vector_2d<double> n1(0,0), n2(0,0);
    pts.push_back(si[0]);
    for(unsigned int j=1; j<si.size(); ++j){
      n2 = normalized(vgl_vector_2d<double>(si[j-1].y()-si[j].y(), 
                                            si[j].x()-si[j-1].x()));
      pts.push_back(si[j]);
      norms.push_back(normalized(n1+n2));
      n1 = n2;
    }  
    norms.push_back(n2);
  }
  
  // compute the error vectors
  vcl_vector<double> errors;
  compute_silhouette_errors(pts,norms,silhouette_,errors);
  
  
  const vcl_vector<vcl_vector<vnl_matrix<double> > >& Js = projector_.silhouette_jacobians();
  const vcl_vector<bool>& is_shadow = projector_.silhouette_shadow();

  double residual = 0.0;
  double total_weight = 0.0;
  vnl_matrix<double> M1(M.rows(),M.cols(),0.0);
  vnl_vector<double> b1(b.size(),0.0);
  unsigned int k=0; // error term index
  for(unsigned int i=0; i<sil.size(); ++i){
    double weight = 1.0;
    if(is_shadow[i]) // give less weight to shadow boundaries
      weight /= 10.0;
    weight = weight*weight;
    const vcl_vector<vgl_point_2d<double> >& si = sil[i];
    if(si.size() < 2)
      continue;
    for(unsigned int j=0; j<si.size(); ++j){
      if(vnl_math_isfinite(errors[k])){
        vnl_vector<double> n(2);
        n[0] = norms[k].x();
        n[1] = norms[k].y();
        vnl_vector<double> J = n*Js[i][j];
        M1 += weight*outer_product(J,J);
        b1 += (weight*errors[k])*J;
        residual += weight*errors[k]*errors[k];
        total_weight += weight;
      }
      ++k;
    }
  }
  
  // give silhouettes half the weight of other edges
  total_weight *=2;
  
  M += M1/total_weight;
  b += b1/total_weight;

  return residual/total_weight;
}


//: Compute the prior terms and update M and b
double dml_vehicle_fit_video::
compute_prior(const dml_pca_vehicle& mesh,
              const vgl_vector_3d<double>& translation,
              const vgl_rotation_3d<double>& rotation,
              vnl_matrix<double>& M, 
              vnl_vector<double>& b)
{
  if(prior_mean_.size() != b.size()+2 ||
     prior_inv_covar_.cols() != M.cols()+2 ||
     prior_inv_covar_.rows() != M.rows()+2 )
  {
    vcl_cerr << "prior data not the correct size, using default prior"<<vcl_endl;
    dml_vehicle_fit::compute_prior(mesh,translation,rotation,M,b);
  }
  
  //: Add in the silhouette optimization terms
  double sil_residual = 0.0;
  if(track_with_silhouette_)
    sil_residual = compute_silhouette_opt_terms(M,b);

  unsigned int tvi = num_params_;
  unsigned int avi = num_params_+1;
  
  
  //: compute the difference to the mean
  vnl_vector<double> dp(num_params_+2,0.0);
  const vnl_vector<double>& params = mesh.params();
  for(unsigned int i=0; i<num_pc_; ++i)
    dp[i] = prior_mean_[i] - params[i];
  
  vgl_rotation_3d<double> rinv = rotation.inverse();
  
  vnl_vector_fixed<double,3> dt(0.0);
  if(txi_ >= 0) dt[0] = prior_mean_[txi_] - translation.x();
  if(tyi_ >= 0) dt[1] = prior_mean_[tyi_] - translation.y();
  if(tzi_ >= 0) dt[2] = prior_mean_[tzi_] - translation.z();
  dt = rinv*dt;
  if(txi_ >= 0) dp[txi_] = dt[0];
  if(tyi_ >= 0) dp[tyi_] = dt[1];
  if(tzi_ >= 0) dp[tzi_] = dt[2];
  
  vnl_vector_fixed<double,3> dr(0.0);
  vnl_vector_fixed<double,3> r = rotation.as_rodrigues();
  if(rxi_ >= 0) dr[0] = prior_mean_[rxi_] - r[0];
  if(ryi_ >= 0) dr[1] = prior_mean_[ryi_] - r[1];
  if(rzi_ >= 0) dr[2] = prior_mean_[rzi_] - r[2];
  dr = rinv*dr;
  if(rxi_ >= 0) dp[rxi_] = dr[0];
  if(ryi_ >= 0) dp[ryi_] = dr[1];
  if(rzi_ >= 0) dp[rzi_] = dr[2];
  
  dp[tvi] = prior_mean_[tvi] - t_velocity_;
  dp[avi] = prior_mean_[avi] - a_velocity_;
  
  
  vnl_matrix<double> P = lambda_*rotate_covariance(rinv, prior_inv_covar_);
  vnl_vector<double> res = P*dp;

  
  // expand by two for extra velocity parameters
  vnl_matrix<double> M2(num_params_+2,num_params_+2,0.0);
  M2.update(M);
  M.swap(M2);
  vnl_vector<double> b2(num_params_+2,0.0);
  b2.update(b);
  b.swap(b2);

  
  b += res;
  M += P;
  
  for(unsigned int i=0; i<num_pc_; ++i)
  {
    M(i,i) += lambda_;
    b[i] += -params[i] *lambda_;
    sil_residual += vnl_math_sqr(-params[i] * lambda_);
  }

  return sil_residual+res.two_norm();
}


//: Apply the solution vector to update the shape and pose parameters
void dml_vehicle_fit_video::
apply_solution(const vnl_vector<double>& soln,
               dml_pca_vehicle& mesh,
               vgl_vector_3d<double>& translation,
               vgl_rotation_3d<double>& rotation)
{
  // apply the base class updates
  dml_vehicle_fit::apply_solution(soln,mesh,translation,rotation);
  
  if(soln.size() == num_params_+2){
    t_velocity_ += soln[num_params_];
    a_velocity_ += soln[num_params_+1];
  }
  else {
    vcl_cerr << "velocities not found in the solution" <<vcl_endl;
    vcl_exit(0);
  }
}


//: Rotate the pose rows and columns of the covariance
vnl_matrix<double> dml_vehicle_fit_video::
rotate_covariance(const vgl_rotation_3d<double>& rotation,
                  const vnl_matrix<double>& covar) const
{
  vnl_matrix<double> P(covar);
  vnl_matrix<double> trows(3,covar.cols(),0.0);
  vnl_matrix<double> rrows(3,covar.cols(),0.0);
  vnl_matrix<double> R = rotation.as_matrix();
  if(txi_ >= 0) trows.set_row(0,P.get_row(txi_));
  if(tyi_ >= 0) trows.set_row(1,P.get_row(tyi_));
  if(tzi_ >= 0) trows.set_row(2,P.get_row(tzi_));
  if(rxi_ >= 0) rrows.set_row(0,P.get_row(rxi_));
  if(ryi_ >= 0) rrows.set_row(1,P.get_row(ryi_));
  if(rzi_ >= 0) rrows.set_row(2,P.get_row(rzi_));
  trows = R*trows;
  rrows = R*rrows;
  if(txi_ >= 0) P.set_row(txi_,trows.get_row(0));
  if(tyi_ >= 0) P.set_row(tyi_,trows.get_row(1));
  if(tzi_ >= 0) P.set_row(tzi_,trows.get_row(2));
  if(rxi_ >= 0) P.set_row(rxi_,rrows.get_row(0));
  if(ryi_ >= 0) P.set_row(ryi_,rrows.get_row(1));
  if(rzi_ >= 0) P.set_row(rzi_,rrows.get_row(2));
  vnl_matrix<double> tcols(covar.rows(),3,0.0);
  vnl_matrix<double> rcols(covar.rows(),3,0.0);
  R = R.transpose();
  if(txi_ >= 0) tcols.set_column(0,P.get_column(txi_));
  if(tyi_ >= 0) tcols.set_column(1,P.get_column(tyi_));
  if(tzi_ >= 0) tcols.set_column(2,P.get_column(tzi_));
  if(rxi_ >= 0) rcols.set_column(0,P.get_column(rxi_));
  if(ryi_ >= 0) rcols.set_column(1,P.get_column(ryi_));
  if(rzi_ >= 0) rcols.set_column(2,P.get_column(rzi_));
  tcols = tcols*R;
  rcols = rcols*R;
  if(txi_ >= 0) P.set_column(txi_,tcols.get_column(0));
  if(tyi_ >= 0) P.set_column(tyi_,tcols.get_column(1));
  if(tzi_ >= 0) P.set_column(tzi_,tcols.get_column(2));
  if(rxi_ >= 0) P.set_column(rxi_,rcols.get_column(0));
  if(ryi_ >= 0) P.set_column(ryi_,rcols.get_column(1));
  if(rzi_ >= 0) P.set_column(rzi_,rcols.get_column(2));
  
  return P;
}


//: Construct the fitting priors from the tracking state
//: \note The state may have additional parameters to be stripped out,
//        and parameters may be in a different order.
void dml_vehicle_fit_video::
set_priors_from_track(const dml_vehicle_state& state)
{
  unsigned int tvi = num_params_;
  unsigned int avi = num_params_+1;
  // compute the prior mean and covariance
  prior_mean_.set_size(num_params_+2);
  prior_mean_.fill(0.0);
  // transfer the shape parameters
  if(state.covar.rows() > 8)
    prior_mean_.update(state.params);
  
  
  // set up the inverse covaraiance
  state_inv_covar_ = vnl_cholesky(state.covar).inverse();
 
#if 0
  // try reseting shape priors
  {
    prior_mean_.fill(0.0);
    vnl_matrix<double> M = state.covar;
    for(unsigned int i=0; i<num_pc_; ++i){
      M.set_row(i+8,0.0);
      M.set_column(i+8,0.0);
      M(i+8,i+8) = 1.0;
    }
    state_inv_covar_ = vnl_cholesky(M).inverse();
  }
#endif
  

  vnl_matrix<double>& C = prior_inv_covar_;
  C.set_size(num_params_+2,num_params_+2);
  C.update(state_inv_covar_.extract(num_pc_,num_pc_,8,8));
  
  // fill in the relevant pose data
  if(options_[1]) // tx
  {
    prior_mean_[txi_] = state.translation.x();
    vnl_vector<double> v(num_params_+2);
    for(unsigned int i=0; i<num_pc_; ++i)
      v[i] = state_inv_covar_(0,8+i);
    v[txi_] = state_inv_covar_(0,0);
    if(options_[2]) v[tyi_] = state_inv_covar_(0,1);
    if(options_[3]) v[tzi_] = state_inv_covar_(0,2);
    if(options_[4]) v[rxi_] = state_inv_covar_(0,3);
    if(options_[5]) v[ryi_] = state_inv_covar_(0,4);
    if(options_[6]) v[rzi_] = state_inv_covar_(0,5);
    v[tvi] = state_inv_covar_(0,6);
    v[avi] = state_inv_covar_(0,7);
    C.set_column(txi_,v);
    C.set_row(txi_,v);
  }
  if(options_[2]) // ty
  {
    prior_mean_[tyi_] = state.translation.y();
    vnl_vector<double> v(num_params_+2);
    for(unsigned int i=0; i<num_pc_; ++i)
      v[i] = state_inv_covar_(1,8+i);
    v[tyi_] = state_inv_covar_(1,1);
    if(options_[1]) v[txi_] = state_inv_covar_(1,0);
    if(options_[3]) v[tzi_] = state_inv_covar_(1,2);
    if(options_[4]) v[rxi_] = state_inv_covar_(1,3);
    if(options_[5]) v[ryi_] = state_inv_covar_(1,4);
    if(options_[6]) v[rzi_] = state_inv_covar_(1,5);
    v[tvi] = state_inv_covar_(1,6);
    v[avi] = state_inv_covar_(1,7);
    C.set_column(tyi_,v);
    C.set_row(tyi_,v);
  }
  if(options_[3]) // tz
  {
    prior_mean_[tzi_] = state.translation.z();
    vnl_vector<double> v(num_params_+2);
    for(unsigned int i=0; i<num_pc_; ++i)
      v[i] = state_inv_covar_(2,8+i);
    v[tzi_] = state_inv_covar_(2,2);
    if(options_[1]) v[txi_] = state_inv_covar_(2,0);
    if(options_[2]) v[tyi_] = state_inv_covar_(2,1);
    if(options_[4]) v[rxi_] = state_inv_covar_(2,3);
    if(options_[5]) v[ryi_] = state_inv_covar_(2,4);
    if(options_[6]) v[rzi_] = state_inv_covar_(2,5);
    v[tvi] = state_inv_covar_(2,6);
    v[avi] = state_inv_covar_(2,7);
    C.set_column(tzi_,v);
    C.set_row(tzi_,v);
  }
  vnl_vector_fixed<double,3> r=state.rotation.as_rodrigues();
  if(options_[4]) // rx
  {
    prior_mean_[rxi_] = r[0];
    vnl_vector<double> v(num_params_+2);
    for(unsigned int i=0; i<num_pc_; ++i)
      v[i] = state_inv_covar_(3,8+i);
    v[rxi_] = state_inv_covar_(3,3);
    if(options_[1]) v[txi_] = state_inv_covar_(3,0);
    if(options_[2]) v[tyi_] = state_inv_covar_(3,1);
    if(options_[3]) v[tzi_] = state_inv_covar_(3,2);
    if(options_[5]) v[ryi_] = state_inv_covar_(3,4);
    if(options_[6]) v[rzi_] = state_inv_covar_(3,5);
    v[tvi] = state_inv_covar_(3,6);
    v[avi] = state_inv_covar_(3,7);
    C.set_column(rxi_,v);
    C.set_row(rxi_,v);
  }
  if(options_[5]) // ry
  {
    prior_mean_[ryi_] = r[1];
    vnl_vector<double> v(num_params_+2);
    for(unsigned int i=0; i<num_pc_; ++i)
      v[i] = state_inv_covar_(4,8+i);
    v[ryi_] = state_inv_covar_(4,4);
    if(options_[1]) v[txi_] = state_inv_covar_(4,0);
    if(options_[2]) v[tyi_] = state_inv_covar_(4,1);
    if(options_[3]) v[tzi_] = state_inv_covar_(4,2);
    if(options_[4]) v[rxi_] = state_inv_covar_(4,3);
    if(options_[6]) v[rzi_] = state_inv_covar_(4,5);
    v[tvi] = state_inv_covar_(4,6);
    v[avi] = state_inv_covar_(4,7);
    C.set_column(ryi_,v);
    C.set_row(ryi_,v);
  }
  if(options_[6]) // rz
  {
    prior_mean_[rzi_] = r[2];
    vnl_vector<double> v(num_params_+2);
    for(unsigned int i=0; i<num_pc_; ++i)
      v[i] = state_inv_covar_(5,8+i);
    v[rzi_] = state_inv_covar_(5,5);
    if(options_[1]) v[txi_] = state_inv_covar_(5,0);
    if(options_[2]) v[tyi_] = state_inv_covar_(5,1);
    if(options_[3]) v[tzi_] = state_inv_covar_(5,2);
    if(options_[4]) v[rxi_] = state_inv_covar_(5,3);
    if(options_[5]) v[ryi_] = state_inv_covar_(5,4);
    v[tvi] = state_inv_covar_(5,6);
    v[avi] = state_inv_covar_(5,7);
    C.set_column(rzi_,v);
    C.set_row(rzi_,v);
  }
  
  prior_mean_[tvi] = state.t_velocity;
  for(unsigned int i=0; i<num_pc_; ++i)
    C(tvi,i) = C(i,tvi) = state_inv_covar_(6,8+i);
  C(tvi,tvi) = state_inv_covar_(6,6);
  C(tvi,avi) = C(avi,tvi) = state_inv_covar_(6,7);
  
  prior_mean_[avi] = state.a_velocity;
  for(unsigned int i=0; i<num_pc_; ++i)
    C(avi,i) = C(i,avi) = state_inv_covar_(7,8+i);
  C(avi,avi) = state_inv_covar_(7,7);
  
}


//: Update the tracker covariance with the residual fitting covariance
//  The residual covariance needs to be rotated back to world orientation
void dml_vehicle_fit_video::
update_posterior_covariance(const dml_pca_vehicle& mesh,
                            const vgl_vector_3d<double>& t,
                            const vgl_rotation_3d<double>& R,
                            vnl_matrix<double>& covar,
                            unsigned int num_frames)
{


  // compute the final covariance 
  double old_mest = mest_scale_;
  mest_scale_ = 1;
  bool compute_visibility = true;
  vnl_vector<double> b(num_params_,0.0);
  vnl_matrix<double> M(num_params_,num_params_,0.0);
  double total_weight = 0.0;
  double wgt_residual = 0.0;
  compute_all_opt_terms(mesh, t, R, M, b, 
                        total_weight, wgt_residual, compute_visibility);
  M /= total_weight;
    
  if(track_with_silhouette_){
    vcl_cout << "adding in silhouette terms"<<vcl_endl;
    compute_silhouette_opt_terms(M,b);
  }
  
  //vcl_cout << "divide by "<<old_mest * old_mest<<vcl_endl;
  M /= old_mest * old_mest * 4;
  
  //vcl_cout <<"M=\n"<<M<<vcl_endl;
  // add in the prior 
  M = rotate_covariance(R.inverse(), M);
  
  for(unsigned int i=0; i<num_pc_; ++i)
  {
    M(i,i) += lambda_;
  }
  
#if 0
  vnl_matrix<double> M(num_params_,num_params_,0.0);
  M.set_identity();
  M *= 0.25;
  for(unsigned int i=0; i<num_pc_; ++i)
    M(i,i) += 1.0/(num_frames+1);
  if(txi_ >= 0) M(txi_,txi_) += 1.0/(num_frames+1);
  if(tyi_ >= 0) M(tyi_,tyi_) += 1.0/(num_frames+1);
  if(rzi_ >= 0) M(rzi_,rzi_) += 1.0/(num_frames+1);
#endif
     
  
  // update the PCA params
  for(unsigned int i=0; i<num_pc_; ++i){
    state_inv_covar_(i+8,i+8) += M(i,i);
    for(unsigned int j=i+1; j<num_pc_; ++j)
      state_inv_covar_(i+8,j+8) = state_inv_covar_(j+8,i+8) += M(i,j);
  }
  if(options_[1]) // tx
  {
    for(unsigned int i=0; i<num_pc_; ++i)
      state_inv_covar_(0,8+i) = state_inv_covar_(8+i,0) += M(txi_,i);
    state_inv_covar_(0,0) += M(txi_,txi_);
    if(options_[2]) state_inv_covar_(0,1) = state_inv_covar_(1,0) += M(txi_,tyi_);
    if(options_[3]) state_inv_covar_(0,2) = state_inv_covar_(2,0) += M(txi_,tzi_);
    if(options_[4]) state_inv_covar_(0,3) = state_inv_covar_(3,0) += M(txi_,rxi_);
    if(options_[5]) state_inv_covar_(0,4) = state_inv_covar_(4,0) += M(txi_,ryi_);
    if(options_[6]) state_inv_covar_(0,5) = state_inv_covar_(5,0) += M(txi_,rzi_);
  }
  if(options_[2]) // ty
  {
    for(unsigned int i=0; i<num_pc_; ++i)
      state_inv_covar_(1,8+i) = state_inv_covar_(8+i,1) += M(tyi_,i);
    state_inv_covar_(1,1) += M(tyi_,tyi_);
    if(options_[3]) state_inv_covar_(1,2) = state_inv_covar_(2,1) += M(tyi_,tzi_);
    if(options_[4]) state_inv_covar_(1,3) = state_inv_covar_(3,1) += M(tyi_,rxi_);
    if(options_[5]) state_inv_covar_(1,4) = state_inv_covar_(4,1) += M(tyi_,ryi_);
    if(options_[6]) state_inv_covar_(1,5) = state_inv_covar_(5,1) += M(tyi_,rzi_);
  }
  if(options_[3]) // tz
  {
    for(unsigned int i=0; i<num_pc_; ++i)
      state_inv_covar_(2,8+i) = state_inv_covar_(8+i,2) += M(tzi_,i);
    state_inv_covar_(2,2) += M(tzi_,tzi_);
    if(options_[4]) state_inv_covar_(2,3) = state_inv_covar_(3,2) += M(tzi_,rxi_);
    if(options_[5]) state_inv_covar_(2,4) = state_inv_covar_(4,2) += M(tzi_,ryi_);
    if(options_[6]) state_inv_covar_(2,5) = state_inv_covar_(5,2) += M(tzi_,rzi_);
  }
  if(options_[4]) // rx
  {
    for(unsigned int i=0; i<num_pc_; ++i)
      state_inv_covar_(3,8+i) = state_inv_covar_(8+i,3) += M(rxi_,i);
    state_inv_covar_(3,3) += M(rxi_,rxi_);
    if(options_[5]) state_inv_covar_(3,4) = state_inv_covar_(4,3) += M(rxi_,ryi_);
    if(options_[6]) state_inv_covar_(3,5) = state_inv_covar_(5,3) += M(rxi_,rzi_);
  }
  if(options_[5]) // ry
  {
    for(unsigned int i=0; i<num_pc_; ++i)
      state_inv_covar_(4,8+i) = state_inv_covar_(8+i,4) += M(ryi_,i);
    state_inv_covar_(4,4) += M(ryi_,ryi_);
    if(options_[6]) state_inv_covar_(4,5) = state_inv_covar_(5,4) += M(ryi_,rzi_);
  }
  if(options_[6]) // rz
  {
    for(unsigned int i=0; i<num_pc_; ++i)
      state_inv_covar_(5,8+i) = state_inv_covar_(8+i,5) += M(rzi_,i);
    state_inv_covar_(5,5) += M(rzi_,rzi_);
  }
  
  covar = vnl_cholesky(state_inv_covar_).inverse();
}


//: Apply fitting to correct the predicted state
bool
dml_vehicle_fit_video::correct_state(dml_vehicle_state& state)
{
  vgl_vector_3d<double> t = state.translation;
  vgl_rotation_3d<double> R = state.rotation;
  // remove params if too large
  if(options_[0] && num_pc_ > 0 && num_pc_ < state.params.size()){
    vnl_vector<double> p = state.params.extract(num_pc_);
    state.params = p;
    vnl_matrix<double> M(num_pc_+8,num_pc_+8);
    state.covar.extract(M);
    state.covar = M;
  }
  // add params if too few
  if(num_pc_ > state.params.size()){
    vnl_vector<double> p(num_pc_,0.0);
    p.update(state.params);
    state.params = p;
    vnl_matrix<double> M = state.covar;
    state.init_covar(true);
    state.covar.update(M);
  }
  
  
  set_priors_from_track(state);
  
  //vnl_vector<double> uncert(num_params_);
  //for(unsigned int i=0; i<num_params_; ++i)
  //  uncert[i] = 1.0/prior_inv_covar_(i,i);
  //set_init_uncert(uncert);
  //set_init_uncert(1.0/(state.num_frames+1)*0.75 + 0.25);
  set_init_uncert(1.0);
  vehicle_.set_params(state.params);
  vehicle_.compute_face_normals();
  
  if(state.num_frames < 2){
    // remove prior on position and orientation while estimating velocity
    set_init_uncert(1.0);
    if(options_[1]){ 
      prior_inv_covar_.set_column(txi_,0.0);
      prior_inv_covar_.set_row(txi_,0.0);
    }
    if(options_[2]){ 
      prior_inv_covar_.set_column(tyi_,0.0);
      prior_inv_covar_.set_row(tyi_,0.0);
    }
    //if(options_[6]){ 
    //  prior_inv_covar_.set_column(rzi_,0.0);
    //  prior_inv_covar_.set_row(rzi_,0.0);
    //}
  }

  
  t_velocity_ = state.t_velocity;
  a_velocity_ = state.a_velocity;
  silhouette_ = state.silhouette;
  //bool tws = track_with_silhouette_;
  //if(state.num_frames < 3)
  //  track_with_silhouette_ = true;
  
  fit_model(50, vehicle_, t, R);
  
  //track_with_silhouette_ = tws;
  
#if 0
  vnl_vector<double> b(num_params_,0.0);
  vnl_matrix<double> M(num_params_,num_params_,0.0);
  compute_prior(vehicle_, t, R, M, b);
  vnl_svd<double> svd_M(M);
  last_covar_ = svd_M.inverse();
#endif
  
  if(state.num_frames > 0 && state.num_frames < 2){
#if 0
    double frac = 0.1;
    if(state.num_frames < 10)
      double frac = 1.0/state.num_frames;
    
    state.t_velocity *= (1 - frac);
    state.a_velocity *= (1 - frac);
    state.t_velocity += frac*30*(R*(t-state.last_translation)).x();
    state.a_velocity += frac*30*(R.as_rodrigues()[2]-state.last_rotation.as_rodrigues()[2]);
#endif
    state.t_velocity = 30*(R.inverse()*(t-state.last_translation)).x();
    state.a_velocity = 30*(R.as_rodrigues()[2]-state.last_rotation.as_rodrigues()[2]);
  }
  else{
    state.t_velocity = t_velocity_;
    state.a_velocity = a_velocity_;
  }
  
  state.translation = t;
  state.rotation = R;
  if(options_[0] && num_pc_ > 0)
    state.params = vehicle_.params().extract(num_pc_);
  
  update_posterior_covariance(vehicle_,t,R,state.covar,state.num_frames);
  ++state.num_frames;

  vcl_cout << "num frames = "<<state.num_frames<<vcl_endl;
  vcl_cout << "velocity = "<<state.t_velocity<<" angular = "<<state.a_velocity<<vcl_endl;
  vcl_cout << "est velocity = "<<30*(R.inverse()*(t-state.last_translation)).x()
           <<" est angular = "<<30*(R.as_rodrigues()[2]-state.last_rotation.as_rodrigues()[2])<<vcl_endl;
  //vcl_cout << "final state covar =\n" << state.covar << vcl_endl;
  
  
  return true;
}

