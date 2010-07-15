// This is mleotta/gui/pca_vehicle/modrec_pca_vehicle_projector.h
#ifndef modrec_pca_vehicle_projector_h_
#define modrec_pca_vehicle_projector_h_
//=========================================================================
//:
// \file
// \brief Projector of a PCA vehicle model with parts into an image
// \author Matt Leotta (mleotta)
// \data 09/09/2008
//
// \verbatim
//  Modifications
//   09/09/2008 - File created. (mleotta)
//   04/27/2009 - Moved from modrec_pca_vehicle.h into its own file (mleotta)
// \endverbatim
//=========================================================================


#include <modrec/modrec_pca_vehicle.h>

#include <vil/vil_image_view.h>
#include <vpgl/vpgl_perspective_camera.h>



//: Projection of the occluding contours and part edges into an image
class modrec_pca_vehicle_projector
{
public:
  //: Constructor
  modrec_pca_vehicle_projector() {}

  //: Constructor - takes width and height of the image
  modrec_pca_vehicle_projector(unsigned int w, unsigned int h);
  
  //: Compute a camera that incorporates the vehicle rotation and translation
  static vpgl_perspective_camera<double>
  compute_relative_camera(const vpgl_perspective_camera<double>& camera,
                          const vgl_rotation_3d<double>& R,
                          const vgl_vector_3d<double>& t);
  
  //: Compute a camera that images the projection onto the ground plane
  static vpgl_proj_camera<double>
  compute_shadow_camera(const vpgl_perspective_camera<double>& camera,
                        const vgl_rotation_3d<double>& R,
                        const vgl_vector_3d<double>& t,
                        const vgl_vector_3d<double>& sun);

  //: Project the vehicle curves using the camera
  void project(const vpgl_perspective_camera<double>& camera,
               const modrec_pca_vehicle& vehicle, 
               const vgl_rotation_3d<double>& R,
               const vgl_vector_3d<double>& t,
               const vgl_vector_3d<double>& sun = vgl_vector_3d<double>(0,0,0),
               const vcl_vector<bool>& options = vcl_vector<bool>(7,false), 
               unsigned int num_pc = 0);
  
  //: Reproject the last vehicle curves using the camera.
  // Do not recompute the depth map or redetermine part and contour visibility.
  // Requires that \c project() has been called at least once first 
  void reproject(const vpgl_perspective_camera<double>& camera,
                 const modrec_pca_vehicle& vehicle, 
                 const vgl_rotation_3d<double>& R,
                 const vgl_vector_3d<double>& t,
                 const vgl_vector_3d<double>& sun = vgl_vector_3d<double>(0,0,0),
                 const vcl_vector<bool>& options = vcl_vector<bool>(7,false), 
                 unsigned int num_pc = 0);
  
  //: Project all mesh edges and no parts using the camera
  void project_all_edges(const vpgl_perspective_camera<double>& camera,
                         const modrec_pca_vehicle& vehicle, 
                         const vgl_rotation_3d<double>& R,
                         const vgl_vector_3d<double>& t,
                         const vgl_vector_3d<double>& sun = vgl_vector_3d<double>(0,0,0),
                         const vcl_vector<bool>& options = vcl_vector<bool>(7,false), 
                         unsigned int num_pc = 0);
  
  //: Project the shadow of the vehicle onto Z=0 along the sun direction
  void project_shadow(const vpgl_perspective_camera<double>& camera,
                      const imesh_mesh& vehicle, 
                      const vgl_rotation_3d<double>& R,
                      const vgl_vector_3d<double>& t,
                      const vgl_vector_3d<double>& sun);
  
  //: use the last produced depth map to estimate the fast back projection.
  vgl_homg_point_3d<double> 
  back_project_fast(const vpgl_perspective_camera<double>& camera,
                    const vgl_point_2d<double>& pt);
  
  //: Project all mesh edges
  vcl_vector<vcl_vector<vgl_point_2d<double> > >
  project_all_edges(const vpgl_perspective_camera<double>& camera,
                    const imesh_mesh& vehicle,
                    const vgl_rotation_3d<double>& R,
                    const vgl_vector_3d<double>& t);

  //: resize the depth map image and clear it
  void resize(unsigned int w, unsigned int h);
  
  //: The last used set of Jacobian computation options
  vcl_vector<bool> options() { return options_; } 
  //: The last used number of principal components in Jacobian computation
  unsigned int num_pc() { return num_pc_; }

  //: Compute the minimum projected depth
  double min_depth() const;
  //: Compute the maximum projected depth
  double max_depth() const;
  //: Return a const reference to the depth map image
  const vil_image_view<double>& depth_map() const { return depth_img_; }

  //: Return a const reference to the projected contours
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >&
  contours() const { return contours_; }
  
  //: Return a const reference to the projected silhouette
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >&
  silhouette() const { return silhouette_; }  
  
  //: Arrange the projected silhouette as a closed polygon
  vgl_polygon<double> silhouette_polygon() const; 
  
  //: Return a const reference to the PCA Jacobians of silhouette vertices
  const vcl_vector<vcl_vector<vnl_matrix<double> > >&
  silhouette_jacobians() const { return silhouette_jac_; }
  
  //: Return a boolean vector, true for each section of the silhouette that comes from the cast shadow
  const vcl_vector<bool>& 
  silhouette_shadow() const { return silhouette_shadow_; }

  //: Return a const reference to the projected contour vertex indices
  const vcl_vector<vcl_vector<unsigned int> >&
  contours_vert_indices() const { return contours_vidx_; }

  //: Return a const reference to the PCA Jacobians of contour vertices
  const vcl_vector<vcl_vector<vnl_matrix<double> > >&
  contours_jacobians() const { return contours_jac_; }
  
  //: Return a const reference to the projected ground shadow
  const vgl_polygon<double>&
  shadow() const { return shadow_; } 

  //: Return a const reference to the projected part boundaries
  const vcl_vector<vcl_vector<vgl_point_2d<double> > >&
  parts() const { return parts_; }

  //: Return a const reference to the projected part point face indices
  const vcl_vector<vcl_vector<vcl_pair<unsigned int,unsigned int> > >&
  parts_indices() const { return parts_idx_; }

  //: Return a const reference to the PCA Jacobians of parts points
  const vcl_vector<vcl_vector<vnl_matrix<double> > >&
  parts_jacobians() const { return parts_jac_; }
  
  //: Return a const reference to the PCA Jacobians of parts points in texture space
  const vcl_vector<vcl_vector<vnl_matrix<double> > >&
  parts_texture_jacobians() const { return parts_tex_jac_; }
  
  //: Store the starting point and ending point along the visible portion of part
  struct part_bounds
  {
    part_bounds(unsigned int pi, unsigned int si, double st, 
                unsigned int ei, double et)
    : part_idx(pi), start_idx(si), s_t(st), end_idx(ei), e_t(et) {}
    
    unsigned int part_idx;
    unsigned int start_idx;
    double s_t; // interpolation in [0,1] between start_idx and start_idx+1
    unsigned int end_idx;
    double e_t; // interpolation in [0,1] between end_idx-1 and end_idx
  };

private:
  //: build a depth map of the vehicle
  void build_depth_image(const vpgl_perspective_camera<double>& camera,
                         const imesh_mesh& vehicle);

  //: Test the depth of point \a x, \a y at depth \a depth against the depth map
  //  Returns true only if deeper than the nearest 4 pixels
  bool depth_test(double x, double y, double depth) const;

  //: project the visible part boundaries into the image
  void project_parts(const vpgl_perspective_camera<double>& camera,
                     const modrec_pca_vehicle& vehicle);

  //: project the visible occluding contours into the image
  void project_contours(const vpgl_perspective_camera<double>& camera,
                        const modrec_pca_vehicle& vehicle);
  
  //: map the silhouette contours back to vehicle mesh vertices
  void map_silhouette(const modrec_pca_vehicle& vehicle,
                      const vcl_vector<vcl_vector<unsigned int> >& edge_loops,
                      const vgl_polygon<double>& sil,
                      const vcl_vector<vcl_pair<unsigned, unsigned> >& sil_idx,
                      const vcl_vector<vcl_pair<double,double> >& sil_range,
                      unsigned int num_vehicle_loops);
  
  //: trace the outer silhoutte of the occluding contours
  bool trace_silhouette(vgl_polygon<double>& p,
                        vcl_vector<vcl_pair<unsigned, unsigned> >& sil_idx,
                        vcl_vector<vcl_pair<double,double> >& sil_frac);
  
  //: reproject the last visible part boundaries into the image
  void reproject_parts(const vpgl_perspective_camera<double>& camera,
                       const modrec_pca_vehicle& vehicle);
  
  //: reproject the last visible occluding contours into the image
  void reproject_contours(const vpgl_perspective_camera<double>& camera,
                          const modrec_pca_vehicle& vehicle);
  
  //: reproject the last valid silhouette edges into the image
  void reproject_silhouette(const vpgl_perspective_camera<double>& camera,
                            const vpgl_proj_camera<double>& shadow_camera,
                            const modrec_pca_vehicle& vehicle);

  //: compute the PCA image Jacobians for both contours and part boundaries
  void compute_jacobians(const vpgl_perspective_camera<double>& camera,
                         const vpgl_proj_camera<double>& shadow_cam,
                         const modrec_pca_vehicle& vehicle);
  
  //: compute the PCA image Jacobians for contour points given the world-to-image Jacobians
  void compute_contour_pca_jacobians(const modrec_pca_vehicle& vehicle,
                                     vcl_vector<vnl_matrix_fixed<double,2,3> >::iterator J);
  
  //: compute the PCA image Jacobians for part boundaries given the world-to-image Jacobians
  void compute_parts_pca_jacobians(const modrec_pca_vehicle& vehicle,
                                   vcl_vector<vnl_matrix_fixed<double,2,3> >::iterator J);


  //: A depth map image for the vehicle
  vil_image_view<double> depth_img_;
  //: projected mesh vertices
  vcl_vector<vgl_point_2d<double> > verts2d_;
  //: projected mesh vertex depths
  vcl_vector<double> depths_;
  
  //: The set of Jacobian computation options
  vcl_vector<bool> options_; 
  //: The number of principal components to use in Jacobians
  unsigned int num_pc_;

  // ==== The projected data ====
  //: the projected contours (not closed)
  vcl_vector<vcl_vector<vgl_point_2d<double> > > contours_;
  //: the mesh vertex indices of the contour points
  vcl_vector<vcl_vector<unsigned int> > contours_vidx_;
  //: PCA Jacobian for contour points
  vcl_vector<vcl_vector<vnl_matrix<double> > > contours_jac_;
  //: the shadow on the ground plane projected to the image
  vgl_polygon<double> shadow_;
  //: edge loop indices for projected shadow points
  vcl_vector<vcl_vector<unsigned int> > shadow_edge_loops_;
  //: the projected silhouette contours
  vcl_vector<vcl_vector<vgl_point_2d<double> > > silhouette_;
  //: the mesh vertex indices of the silhouette contour points
  vcl_vector<vcl_vector<unsigned int> > silhouette_vidx_;
  //: the normalized range in [0,1] that start/end silhouette edges are trimmed
  vcl_vector<vcl_pair<double,double> > silhouette_range_;
  //: true for each section of the silhouette that comes from the cast shadow
  vcl_vector<bool> silhouette_shadow_;
  //: PCA Jacobian for silhouette points
  vcl_vector<vcl_vector<vnl_matrix<double> > > silhouette_jac_;
  //: the projected part boundaries (not closed)
  vcl_vector<vcl_vector<vgl_point_2d<double> > > parts_;
  //: the index pair of the contour points mapping back to (part,point) on the mesh
  vcl_vector<vcl_vector<vcl_pair<unsigned int,unsigned int> > > parts_idx_;
  //: the boundaries of the visible portions of parts
  vcl_vector<part_bounds> parts_bounds_;
  //: PCA Jacobian for parts points
  vcl_vector<vcl_vector<vnl_matrix<double> > > parts_jac_;
  //: PCA Jacobian for parts points in texture space
  vcl_vector<vcl_vector<vnl_matrix<double> > > parts_tex_jac_;
};


//=========================================================
// External functions


//: Save the projected contours as SVG
bool modrec_write_svg_curves(const vcl_string& filename,
                             const modrec_pca_vehicle_projector& projector);


#endif // modrec_pca_vehicle_projector_h_
