// This is mleotta/gui/pca_vehicle/modrec_vehicle_fit_multiview.cxx
//=========================================================================
//:
// \file
// \brief Fit the PCA vehicle to one or more images
//
//=========================================================================

#include "modrec_vehicle_fit_multiview.h"




//: Constructor
modrec_vehicle_fit_multiview::modrec_vehicle_fit_multiview()
: views_(1), sun_dir_(0.0,0.0,0.0)
{
}


//: clear all the data and reset to the initial single frame setup
void modrec_vehicle_fit_multiview::reset()
{
  views_.clear();
  views_.resize(1);
}


//: Assign an edge map at index \a idx
void modrec_vehicle_fit_multiview::set_edge_map(unsigned int idx, 
                                      const vil_image_view<float>& edge_map)
{
  if(idx >= views_.size()){
    views_.resize(idx+1);
  }
  views_[idx].edge_map = edge_map;
  views_[idx].projector.resize(edge_map.ni(),edge_map.nj());
}


//: Assign a camera at index \a idx
void modrec_vehicle_fit_multiview::set_camera(unsigned int idx, 
                                    const vpgl_perspective_camera<double>& camera)
{
  if(idx >= views_.size()){
    views_.resize(idx+1);
  }
  views_[idx].camera = camera;
}


//: Set the activity of index \a idx
void modrec_vehicle_fit_multiview::set_active(unsigned int idx, bool active)
{
  if(idx >= views_.size()){
    views_.resize(idx+1);
  }
  views_[idx].active = active;
}


//: Access the edge map at index \a idx
// return the last edge map if the index is invalid
const vil_image_view<float>& 
modrec_vehicle_fit_multiview::edge_map(unsigned int idx) const
{ 
  if(idx >= views_.size())
    return views_.back().edge_map;
  
  return views_[idx].edge_map;
}


//: Access the camera at index \a idx
// return the last camera if the index is invalid
const vpgl_perspective_camera<double>& 
modrec_vehicle_fit_multiview::camera(unsigned int idx) const
{ 
  if(idx >= views_.size())
    return views_.back().camera;
  
  return views_[idx].camera; 
}


//: Access the vehicle projector at index \a idx
// return the last projector if the index is invalid
const modrec_pca_vehicle_projector& 
modrec_vehicle_fit_multiview::projector(unsigned int idx) const
{ 
  if(idx >= views_.size())
    return views_.back().projector;
  
  return views_[idx].projector; 
}


//: Return true if index \a idx is active
bool modrec_vehicle_fit_multiview::is_active(unsigned int idx) const
{
  if(idx >= views_.size())
    return false;
  
  return views_[idx].active; 
}


//: Compute the edge map for index \a idx from the image \a img
void modrec_vehicle_fit_multiview::detect_and_set_edges(unsigned int idx, 
                                      const vil_image_view<vxl_byte>& img)
{
  if(idx >= views_.size()){
    views_.resize(idx+1);
  }
  vil_image_view<float>& edge_map = views_[idx].edge_map;
  
  // detect edges using the base class function
  detect_edges(img,edge_map);
  
  views_[idx].projector.resize(edge_map.ni(),edge_map.nj());
}


//: Compute the terms for optimization from all active views
//  appends terms to the vector references passed in
void modrec_vehicle_fit_multiview::
compute_all_opt_terms(const modrec_pca_vehicle& mesh,
                      const vgl_vector_3d<double>& translation,
                      const vgl_rotation_3d<double>& rotation,
                      vnl_matrix<double>& M,
                      vnl_vector<double>& b,
                      double& total_weight,
                      double& wgt_residual,
                      bool compute_visiblity)
{
  for(unsigned int idx=0; idx<views_.size(); ++idx)
  {
    if(!views_[idx].active)
      continue;
    modrec_pca_vehicle_projector& mesh_projector = views_[idx].projector;
    const vpgl_perspective_camera<double>& camera = views_[idx].camera;
    const vil_image_view<float>& edge_map = views_[idx].edge_map;
    
    // set the relative M-estimator scale
    double global_scale = mest_scale_;
    mest_scale_ *= views_[idx].rel_scale;
    if(mest_scale_ < 1.0)
      mest_scale_ = 1.0;
    
    if(compute_visiblity)
      mesh_projector.project(camera,mesh,rotation,translation,sun_dir_,options_,num_pc_);
    else
      mesh_projector.reproject(camera,mesh,rotation,translation,sun_dir_,options_,num_pc_);
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >& contours = mesh_projector.contours();
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >& parts = mesh_projector.parts();
    
    const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jc = mesh_projector.contours_jacobians();
    const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jp = mesh_projector.parts_jacobians();
        
    compute_curve_opt_terms(contours, Jc, edge_map, M, b, total_weight, wgt_residual);
    compute_curve_opt_terms(parts, Jp, edge_map, M, b, total_weight, wgt_residual);
    
    // revert to the global scale
    mest_scale_ = global_scale;
  }
}

//: Compute the residuals of optimization from all active views.
void modrec_vehicle_fit_multiview::
compute_all_residuals(const modrec_pca_vehicle& mesh,
                      const vgl_vector_3d<double>& translation,
                      const vgl_rotation_3d<double>& rotation,
                      double& total_weight,
                      double& wgt_residual)
{
  for(unsigned int idx=0; idx<views_.size(); ++idx)
  {
    if(!views_[idx].active)
      continue;
    modrec_pca_vehicle_projector& mesh_projector = views_[idx].projector;
    const vpgl_perspective_camera<double>& camera = views_[idx].camera;
    const vil_image_view<float>& edge_map = views_[idx].edge_map;
    
    // set the relative M-estimator scale
    double global_scale = mest_scale_;
    mest_scale_ *= views_[idx].rel_scale;
    if(mest_scale_ < 1.0)
      mest_scale_ = 1.0;
    
    mesh_projector.reproject(camera,mesh,rotation,translation,sun_dir_,options_,num_pc_);
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >& contours = mesh_projector.contours();
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >& parts = mesh_projector.parts();
    
    compute_curve_residuals(contours, edge_map, total_weight, wgt_residual);
    compute_curve_residuals(parts, edge_map, total_weight, wgt_residual);
    
    // revert to the global scale
    mest_scale_ = global_scale;
  }
}


//: Default view constructor
modrec_vehicle_fit_multiview::view::view()
: active(true), rel_scale(1.0)
{
  vgl_point_3d<double> c(5,5,5);
  vpgl_calibration_matrix<double> K(1200,vgl_point_2d<double>(512,386));
  camera = vpgl_perspective_camera<double>(K,c,vgl_rotation_3d<double>());
  camera.look_at(vgl_homg_point_3d<double>(0,0,0.5));
}


//: Estimate the initial M-estimator scale from parameter standard deviations
double modrec_vehicle_fit_multiview::estimate_initial_scale(const modrec_pca_vehicle& mesh,
                                                            const vgl_vector_3d<double>& translation,
                                                            const vgl_rotation_3d<double>& rotation,
                                                            const vnl_vector<double>& sigma)
{
  double max_scale = 0.0;
  for(unsigned int idx=0; idx<views_.size(); ++idx)
  {
    if(!views_[idx].active)
      continue;
    modrec_pca_vehicle_projector& mesh_projector = views_[idx].projector;
    const vpgl_perspective_camera<double>& camera = views_[idx].camera;
    
    mesh_projector.project(camera,mesh,rotation,translation,vgl_vector_3d<double>(0,0,0),options_,num_pc_);
    
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >& contours = mesh_projector.contours();
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >& parts = mesh_projector.parts();
    
    const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jc = mesh_projector.contours_jacobians();
    const vcl_vector<vcl_vector<vnl_matrix<double> > >& Jp = mesh_projector.parts_jacobians();
    vcl_vector<vcl_vector<vnl_matrix<double> > > J(Jc);
    J.insert(J.end(), Jp.begin(), Jp.end());
    
    views_[idx].rel_scale = modrec_vehicle_fit::estimate_initial_scale(sigma,J);
    if(views_[idx].rel_scale > max_scale)
      max_scale = views_[idx].rel_scale;
  }
  
  for(unsigned int idx=0; idx<views_.size(); ++idx)
  {
    if(!views_[idx].active)
      continue;
    views_[idx].rel_scale /= max_scale;
  }
  
  return max_scale;
}
