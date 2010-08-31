// This is dml/dml_vehicle_fit_multiview.h
#ifndef dml_vehicle_fit_multiview_h_
#define dml_vehicle_fit_multiview_h_
//=========================================================================
//:
// \file
// \brief  Fit the PCA vehicle to one or more images
// \author Matt Leotta (mleotta)
// \date 11/02/2008
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
//   11/02/2008 - File created. (mleotta)
//   04/11/2009 - Moved to a child class to allow for base class sharing
// \endverbatim
//=========================================================================


#include <dml/dml_vehicle_fit.h>

//: Fit a PCA vehicle model with parts to multiple calibrated views
class dml_vehicle_fit_multiview : public dml_vehicle_fit
{
public:
  
  //: Constructor
  dml_vehicle_fit_multiview();
  
  //: clear all the data and reset to the initial single frame setup
  virtual void reset();
  
  //: Assign an edge map at index \a idx
  void set_edge_map(unsigned int idx, const vil_image_view<float>& edge_map);
  //: Compute the edge map for index \a idx from the image \a img
  void detect_and_set_edges(unsigned int idx, const vil_image_view<vxl_byte>& img);
  //: Assign a camera at index \a idx
  void set_camera(unsigned int idx, const vpgl_perspective_camera<double>& camera);
  //: Set the activity of index \a idx
  void set_active(unsigned int idx, bool active);
    
  //: Access the edge map at index \a idx
  // return the last edge map if the index is invalid
  const vil_image_view<float>& edge_map(unsigned int idx) const;
  //: Access the camera at index \a idx
  // return the last camera if the index is invalid
  const vpgl_perspective_camera<double>& camera(unsigned int idx) const;
  //: Access the vehicle projector at index \a idx
  // return the last projector if the index is invalid
  const dml_pca_vehicle_projector& projector(unsigned int idx) const;
  //: Return true if index \a idx is active
  bool is_active(unsigned int idx) const;
  
  //: Return the number views
  unsigned int num_views() const { return views_.size(); }
  
  //: Estimate the initial M-estimator scale from parameter standard deviations
  virtual double estimate_initial_scale(const dml_pca_vehicle& mesh,
                                        const vgl_vector_3d<double>& translation,
                                        const vgl_rotation_3d<double>& rotation,
                                        const vnl_vector<double>& sigma);
  

private:
  
  //: Compute the terms for optimization from all active views.
  //  Adds to the matrix \a M and vector \a b passed in.
  //  Only recompute curve visiblity when \a compute_visibility is true.
  virtual void 
  compute_all_opt_terms(const dml_pca_vehicle& mesh,
                        const vgl_vector_3d<double>& translation,
                        const vgl_rotation_3d<double>& rotation,
                        vnl_matrix<double>& M,
                        vnl_vector<double>& b,
                        double& total_weight,
                        double& wgt_residual,
                        bool compute_visiblity);
  
  //: Compute the residuals of optimization from all active views.
  virtual void 
  compute_all_residuals(const dml_pca_vehicle& mesh,
                        const vgl_vector_3d<double>& translation,
                        const vgl_rotation_3d<double>& rotation,
                        double& total_weight,
                        double& wgt_residual);
  
  //: A container for all the data assosiated with each camera view
  struct view
  {
    //: Default Constructor
    view();
    
    //: should this view be used
    bool active;
    //: the edge map image
    vil_image_view<float> edge_map;
    //: the camera
    vpgl_perspective_camera<double> camera;
    //: the vehicle projector class
    dml_pca_vehicle_projector projector;
    
    //: the relative M-estimator scale compared to other views
    double rel_scale;
  };
  
  //: A vector containing all the views
  vcl_vector<view> views_;
  //: the direction of the sun for shadow casting
  vgl_vector_3d<double> sun_dir_;

};




#endif // dml_vehicle_fit_multiview_h_
