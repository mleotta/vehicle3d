// This is mleotta/gui/pca_vehicle/modrec_pca_vehicle.cxx
//=========================================================================
//:
// \file
// \brief  projector of a PCA vehicle model with parts into an image
//
//=========================================================================

#include "modrec_pca_vehicle_projector.h"

#include <vcl_limits.h>
#include <vcl_map.h>
#include <vil/vil_bilin_interp.h>
#include <vgl/vgl_line_3d_2_points.h>
#include <vgl/vgl_triangle_test.h>
#include <vnl/vnl_double_4.h>
#include <imesh/algo/imesh_project.h>
#include <imesh/imesh_detection.h>
#include <imesh/imesh_operations.h>


//: Constructor - takes width and height of the image
modrec_pca_vehicle_projector::
modrec_pca_vehicle_projector(unsigned int w, unsigned int h)
  : depth_img_(w,h)
{
  depth_img_.fill(vcl_numeric_limits<double>::infinity());
}

//: resize the depth map image and clear it
void modrec_pca_vehicle_projector::
resize(unsigned int w, unsigned int h)
{
  depth_img_.set_size(w,h);
  depth_img_.fill(vcl_numeric_limits<double>::infinity());
}


//: Compute the minimum projected depth
double modrec_pca_vehicle_projector::min_depth() const
{
  double min_depth = vcl_numeric_limits<double>::infinity();
  for(vcl_vector<double>::const_iterator i=depths_.begin();
      i!=depths_.end(); ++i)
  {
    if(*i < min_depth) min_depth = *i;
  }
  return min_depth;
}


//: Compute the maximum projected depth
double modrec_pca_vehicle_projector::max_depth() const
{
  double max_depth = -vcl_numeric_limits<double>::infinity();
  for(vcl_vector<double>::const_iterator i=depths_.begin();
      i!=depths_.end(); ++i)
  {
    if(*i > max_depth) max_depth = *i;
  }
  return max_depth;
}


//: Compute a camera that incorporates the vehicle rotation and translation
vpgl_perspective_camera<double>
modrec_pca_vehicle_projector::
compute_relative_camera(const vpgl_perspective_camera<double>& camera,
                        const vgl_rotation_3d<double>& R,
                        const vgl_vector_3d<double>& t)
{
  vpgl_perspective_camera<double> rcam(camera);
  rcam.set_rotation(rcam.get_rotation()*R);
  vgl_point_3d<double> num_p = R.inverse()*(rcam.get_camera_center()-t);
  rcam.set_camera_center(num_p);
  
  return rcam;
}


//: Compute a camera that images the projection onto the ground plane
vpgl_proj_camera<double>
modrec_pca_vehicle_projector::
compute_shadow_camera(const vpgl_perspective_camera<double>& camera,
                      const vgl_rotation_3d<double>& R,
                      const vgl_vector_3d<double>& t,
                      const vgl_vector_3d<double>& sun)
{
  // P [ 1  0 -sx/sz 0 ] [ R t ]
  //   [ 0  1 -sy/sz 0 ] [ 0 1 ]
  //   [ 0  0   0    0 ]
  //   [ 0  0   0    1 ]
  vnl_double_3x4 P = camera.get_matrix();
  P.set_column(2, P.get_column(0)*(-sun.x()/sun.z()) 
                 +P.get_column(1)*(-sun.y()/sun.z()));
  P.set_column(3, P*vnl_double_4(t.x(),t.y(),t.z(),1));
  P.update(P.extract(3,3)*R.as_matrix());
  return vpgl_proj_camera<double>(P);
}


//: Project the vehicle curves using the camera
void modrec_pca_vehicle_projector::
project(const vpgl_perspective_camera<double>& camera,
        const modrec_pca_vehicle& vehicle, 
        const vgl_rotation_3d<double>& R,
        const vgl_vector_3d<double>& t,
        const vgl_vector_3d<double>& sun,
        const vcl_vector<bool>& options, 
        unsigned int num_pc)
{
  // if there are no parts assume this is the dodecahedral model and we want
  // to project all mesh edges 
  if(vehicle.parts().empty()){
    project_all_edges(camera,vehicle,R,t,sun,options,num_pc);
    return;
  }
  
  
  assert(options.size() == 7);
  if(sun.sqr_length() > 0)
    project_shadow(camera,vehicle,R,t,sun);
  else{
    shadow_edge_loops_.clear();
    shadow_.clear();
  }
    
  // compute the camera relative to the vehicle
  vpgl_perspective_camera<double> rel_cam = compute_relative_camera(camera,R,t);
  build_depth_image(rel_cam,vehicle);
  project_contours(rel_cam,vehicle);
  project_parts(rel_cam,vehicle);
  options_ = options;
  num_pc_ = num_pc;
  if((options_[0] && num_pc_>0) || 
     options_[1] || options_[2] || options_[3] ||
     options_[4] || options_[5] || options_[6] )
  {
    vpgl_proj_camera<double> shadow_cam = compute_shadow_camera(camera,R,t,sun);
    compute_jacobians(rel_cam,shadow_cam,vehicle);
  }
  else
  {
    contours_jac_.clear();
    silhouette_jac_.clear();
    parts_jac_.clear();
  }
}


//: Reproject the last vehicle curves using the camera.
// Do not recompute the depth map or redetermine part and contour visibility.
// Requires that \c project() has been called at least once first 
void modrec_pca_vehicle_projector::
reproject(const vpgl_perspective_camera<double>& camera,
          const modrec_pca_vehicle& vehicle, 
          const vgl_rotation_3d<double>& R,
          const vgl_vector_3d<double>& t,
          const vgl_vector_3d<double>& sun,
          const vcl_vector<bool>& options, 
          unsigned int num_pc)
{
  assert(options.size() == 7);
  vpgl_proj_camera<double> shadow_cam;
  if(sun.sqr_length() > 0){
    shadow_cam = compute_shadow_camera(camera,R,t,sun);
  }
  // compute the camera relative to the vehicle
  vpgl_perspective_camera<double> rel_cam = compute_relative_camera(camera,R,t);
  reproject_contours(rel_cam,vehicle);
  reproject_parts(rel_cam,vehicle);
  reproject_silhouette(rel_cam,shadow_cam,vehicle);
  options_ = options;
  num_pc_ = num_pc;
  if((options_[0] && num_pc_>0) || 
     options_[1] || options_[2] || options_[3] ||
     options_[4] || options_[5] || options_[6] )
    compute_jacobians(rel_cam,shadow_cam,vehicle);
  else
  {
    contours_jac_.clear();
    silhouette_jac_.clear();
    parts_jac_.clear();
  }
}


//: Project all mesh edges and no parts using the camera
void modrec_pca_vehicle_projector::
project_all_edges(const vpgl_perspective_camera<double>& camera,
                  const modrec_pca_vehicle& vehicle,
                  const vgl_rotation_3d<double>& R,
                  const vgl_vector_3d<double>& t,
                  const vgl_vector_3d<double>& sun,
                  const vcl_vector<bool>& options, 
                  unsigned int num_pc)
{
  assert(options.size() == 7);
  if(sun.sqr_length() > 0)
    project_shadow(camera,vehicle,R,t,sun);
  else{
    shadow_edge_loops_.clear();
    shadow_.clear();
  }
  
  parts_.clear();
  parts_idx_.clear();
  parts_bounds_.clear();
  contours_.clear();
  contours_vidx_.clear();
  // compute the camera relative to the vehicle
  vpgl_perspective_camera<double> rel_cam = compute_relative_camera(camera,R,t);
  
  build_depth_image(rel_cam,vehicle);
  
  vcl_vector<vcl_vector<unsigned int> > edge_loops;
  vgl_polygon<double> sil;
  
  assert(vehicle.has_half_edges());
  const imesh_half_edge_set& he = vehicle.half_edges();
  const unsigned int num_edges = vehicle.num_edges();
  for(unsigned int i=0; i<num_edges; ++i)
  {
    unsigned int vi1 = he[2*i].vert_index();
    unsigned int vi2 = he[2*i+1].vert_index();    
    const vgl_point_2d<double>& v1 = verts2d_[vi1];
    const vgl_point_2d<double>& v2 = verts2d_[vi2];
    
    vcl_vector<vgl_point_2d<double> > contour(2);
    contour[0] = v1;
    contour[1] = v2;
    sil.push_back(contour);
    
    edge_loops.push_back(vcl_vector<unsigned int>(2));
    edge_loops.back()[0] = 2*i;
    edge_loops.back()[1] = 2*i+1;

    if(depth_test(v1.x(), v1.y(), depths_[vi1]) &&
       depth_test(v2.x(), v2.y(), depths_[vi2]) &&
       depth_test((v1.x()+v2.x())/2, (v1.y()+v2.y())/2, (depths_[vi1]+depths_[vi2])/2))
    {
      contours_.push_back(contour);
      vcl_vector<unsigned int> contour_vidx(2);
      contour_vidx[0] = vi1;
      contour_vidx[1] = vi2;
      contours_vidx_.push_back(contour_vidx);
    }
  }
  
  // added in shadow curves if available
  unsigned int num_vehicle_loops = edge_loops.size();
  if(!shadow_edge_loops_.empty()){
    for(unsigned int i=0; i<shadow_edge_loops_.size(); ++i){
      edge_loops.push_back(shadow_edge_loops_[i]);
      sil.push_back(shadow_[i]);
    }
  }
  
  // trace the part of the contour that makes up the silhouette
  vcl_vector<vcl_pair<unsigned, unsigned> > sil_idx;
  vcl_vector<vcl_pair<double,double> > sil_range;
  if(trace_silhouette(sil, sil_idx, sil_range)){
    // map the silhouette polygon indices back to mesh vertex indices
    map_silhouette(vehicle, edge_loops, sil, sil_idx, sil_range, num_vehicle_loops);
  }else{
    silhouette_vidx_.clear();
    silhouette_range_.clear();
    silhouette_shadow_.clear();
    silhouette_.clear();
  }
  

  // compute the camera relative to the vehicle
  options_ = options;
  num_pc_ = num_pc;
  if((options_[0] && num_pc_>0) || 
     options_[1] || options_[2] || options_[3] ||
     options_[4] || options_[5] || options_[6] )
  {
    vpgl_proj_camera<double> shadow_cam = compute_shadow_camera(camera,R,t,sun);
    compute_jacobians(rel_cam,shadow_cam,vehicle);
  }
  else
  {
    contours_jac_.clear();
    silhouette_jac_.clear();
    parts_jac_.clear();
  }
}


//: use the last produced depth map to estimate the fast back projection.
vgl_homg_point_3d<double> 
modrec_pca_vehicle_projector::
back_project_fast(const vpgl_perspective_camera<double>& camera,
                  const vgl_point_2d<double>& pt)
{
  vgl_line_3d_2_points<double> ray = camera.backproject(vgl_homg_point_2d<double>(pt));
  vgl_vector_3d<double> dir = ray.direction();
  normalize(dir);
  
  double depth = vcl_numeric_limits<double>::infinity();
  if(depth_img_)
  {
    int ni = depth_img_.ni();
    int nj = depth_img_.nj();
    int i = static_cast<int>(vcl_floor(pt.x()));
    int j = static_cast<int>(vcl_floor(pt.y()));
    if(i>=0 && i+1 < ni && j>=0 && j+1 < nj){
      depth = vil_bilin_interp(depth_img_,pt.x(),pt.y());
      // if infinitelook for a nearby pixel that is finite
      if(!vnl_math_isfinite(depth)){
        if(vnl_math_isfinite(depth_img_(i,j)))
          depth = depth_img_(i,j);
        else if(vnl_math_isfinite(depth_img_(i+1,j)))
          depth = depth_img_(i+1,j);
        else if(vnl_math_isfinite(depth_img_(i,j+1)))
          depth = depth_img_(i,j+1);
        else if(vnl_math_isfinite(depth_img_(i+1,j+1)))
          depth = depth_img_(i+1,j+1);
      }
    }
  }
  if(!vnl_math_isfinite(depth))
    return vgl_homg_point_3d<double>(dir.x(),dir.y(),dir.z(),0.0);
  
  return vgl_homg_point_3d<double>(camera.get_camera_center() + depth*dir);
}


//: Project all mesh edges
vcl_vector<vcl_vector<vgl_point_2d<double> > >
modrec_pca_vehicle_projector::
project_all_edges(const vpgl_perspective_camera<double>& camera,
                  const imesh_mesh& vehicle,
                  const vgl_rotation_3d<double>& R,
                  const vgl_vector_3d<double>& t)
{
  // compute the camera relative to the vehicle
  vpgl_perspective_camera<double> rel_cam = compute_relative_camera(camera,R,t);
  
  if(vehicle.faces().regularity() != 3){
    imesh_mesh tri_mesh = vehicle;
    imesh_triangulate(tri_mesh);
    build_depth_image(rel_cam,tri_mesh);
  }
  else
    build_depth_image(rel_cam,vehicle);

  vcl_vector<vcl_vector<vgl_point_2d<double> > > contours;
  assert(vehicle.has_half_edges());
  const imesh_half_edge_set& he = vehicle.half_edges();
  const unsigned int num_edges = vehicle.num_edges();
  for(unsigned int i=0; i<num_edges; ++i)
  {
    unsigned int vi1 = he[2*i].vert_index();
    unsigned int vi2 = he[2*i+1].vert_index();
    
    const vgl_point_2d<double>& v1 = verts2d_[vi1];
    const vgl_point_2d<double>& v2 = verts2d_[vi2];
    if(depth_test(v1.x(), v1.y(), depths_[vi1]) &&
       depth_test(v2.x(), v2.y(), depths_[vi2]) )
    {
      vcl_vector<vgl_point_2d<double> > contour(2);
      contour[0] = verts2d_[vi1];
      contour[1] = verts2d_[vi2];
      contours.push_back(contour);
    }
  }
  return contours;
}


//: Project the shadow of the vehicle onto Z=0 along the sun direction
void modrec_pca_vehicle_projector::
project_shadow(const vpgl_perspective_camera<double>& camera,
               const imesh_mesh& vehicle, 
               const vgl_rotation_3d<double>& R,
               const vgl_vector_3d<double>& t,
               const vgl_vector_3d<double>& sun)
{
  vgl_vector_3d<double> dir = R.inverse()*sun;
  // detect the contour generator
  shadow_edge_loops_ = imesh_detect_contour_generator(vehicle, dir);
  
  vpgl_proj_camera<double> shadow_cam = compute_shadow_camera(camera,R,t,sun);
  
  const imesh_vertex_array<3>& verts3d = vehicle.vertices<3>();
  
  shadow_.clear();
  for(unsigned int i=0; i<shadow_edge_loops_.size(); ++i)
  {
    const vcl_vector<unsigned int>& loop = shadow_edge_loops_[i];
    // convert edge indices to vertex indices
    const unsigned num_pts = loop.size();
    vcl_vector<vgl_point_3d<double> > pts3d(num_pts);
    for(unsigned int j=0; j<num_pts; ++j)
      pts3d[j] = verts3d[vehicle.half_edges()[loop[j]].vert_index()];
    vcl_vector<vgl_point_2d<double> > pts2d;
    imesh_project_verts(pts3d,shadow_cam,pts2d);
    shadow_.push_back(pts2d);
  }
  
}


//: build the depth map of the vehicle
void modrec_pca_vehicle_projector::
build_depth_image(const vpgl_perspective_camera<double>& camera,
                  const imesh_mesh& vehicle)
{
  // clear the depth map
  depth_img_.fill(vcl_numeric_limits<double>::infinity());

  // project the depth image
  //imesh_project_depth(vehicle,camera,depth_img_);
  imesh_project_verts(vehicle.vertices<3>(), camera, verts2d_, depths_);
  imesh_render_faces_interp(vehicle, verts2d_, depths_, depth_img_);
}


//: Test the depth of point \a x, \a y at depth \a depth against the depth map
//  Returns true only if deeper than the nearest 4 pixels
bool modrec_pca_vehicle_projector::
depth_test(double x, double y, double depth) const
{
  int x1 = static_cast<int>(vcl_floor(x));
  if(x1 < 0) return false;
  int x2 = static_cast<int>(vcl_ceil(x));
  if(x2 >= depth_img_.ni()) return false;
  int y1 = static_cast<int>(vcl_floor(y));
  if(y1 < 0) return false;
  int y2 = static_cast<int>(vcl_ceil(y));
  if(y2 >= depth_img_.nj()) return false;

  if(depth < depth_img_(x1,y1) ||
     depth < depth_img_(x1,y2) ||
     depth < depth_img_(x2,y1) ||
     depth < depth_img_(x2,y2) )
    return true;

  return false;
}


//: project the visible part boundaries into the image
void modrec_pca_vehicle_projector::
project_parts(const vpgl_perspective_camera<double>& camera,
              const modrec_pca_vehicle& vehicle)
{
  parts_.clear();
  parts_idx_.clear();
  const vcl_vector<vcl_vector<vgl_point_3d<double> > >& parts3d = vehicle.parts_3d();
  for(unsigned int i=0; i<parts3d.size(); ++i)
  {
    const vcl_vector<vgl_point_3d<double> >& part3d = parts3d[i];
    vcl_vector<vgl_point_2d<double> > verts2d;
    vcl_vector<double> depths;
    imesh_project_verts(part3d,camera,verts2d,depths);

    vcl_vector<bool> visible(part3d.size(),false);
    for(unsigned int j=0; j<part3d.size(); ++j)
    {
      visible[j] = depth_test(verts2d[j].x(), verts2d[j].y(), depths[j]);
    }

    vcl_vector<vgl_point_2d<double> > part;
    vcl_vector<vcl_pair<unsigned int,unsigned int> > part_idx;
    for(unsigned int j1=visible.size()-1, j2=0; j1<visible.size(); j1 = j2++)
    {
      if(visible[j1]){
        part.push_back(verts2d[j1]);
        part_idx.push_back(vcl_pair<unsigned int,unsigned int>(i,j1));
      }
      else
      {
        if(part.size() > 1){
          parts_.push_back(part);
          parts_idx_.push_back(part_idx);
        }
        part.clear();
        part_idx.clear();
      }
    }
    if(part.size() > 1){
      // close the loop for broken parts
      int k = -1;
      if(!parts_idx_.empty() && parts_idx_.back()[0].first == i){
        for(k=parts_idx_.size()-1; k>=0 && parts_idx_[k][0].first == i &&
            parts_idx_[k].front().second != part_idx.back().second; --k);
        if(k<0 || parts_idx_[k][0].first != i)
          k = -1;
      }
      if(k>=0){
        parts_idx_[k].insert(parts_idx_[k].begin(), 
                             part_idx.begin(), part_idx.end()-1);
        parts_[k].insert(parts_[k].begin(), 
                         part.begin(), part.end()-1);
      }
      else{
        parts_.push_back(part);
        parts_idx_.push_back(part_idx);
      }
    }
  }
  
  parts_bounds_.clear();
  typedef modrec_pca_vehicle::uv_point uv_point;
  const vcl_vector<vcl_vector<uv_point> >& parts_uv = vehicle.parts_bary();
  for(unsigned int i=0; i<parts_idx_.size(); ++i)
  {
    const vcl_vector<vcl_pair<unsigned int,unsigned int> >& part_idx = parts_idx_[i];
    unsigned int pi = part_idx[0].first;
    const vcl_vector<uv_point>& part_uv = parts_uv[pi];
    unsigned int si = part_idx[0].second;
    unsigned int ei = part_idx[part_idx.size()-1].second;
    if(si == ei && part_idx.size() > 1)
      parts_bounds_.push_back(part_bounds(pi,0,0.0,0,1.0));
    else{
      double st = part_uv[si].t;
      unsigned int tmp1,tmp2;
      vehicle.pc_to_part_idx(part_uv[si].end_point1,tmp1,tmp2);
      assert(tmp1 == pi);
      si = tmp2;
      double et = part_uv[ei].t;
      vehicle.pc_to_part_idx(part_uv[ei].end_point2,tmp1,tmp2);
      assert(tmp1 == pi);
      ei = tmp2;
      parts_bounds_.push_back(part_bounds(pi,si,st,ei,et));
    }
  }
}


//: project the visible occluding contours into the image
void modrec_pca_vehicle_projector::
project_contours(const vpgl_perspective_camera<double>& camera,
                 const modrec_pca_vehicle& vehicle)
{
  // detect the contour generator
  vcl_vector<vcl_vector<unsigned int> > edge_loops =
      imesh_detect_contour_generator(vehicle, camera.camera_center());

  contours_.clear();
  contours_vidx_.clear();
  vgl_polygon<double> sil;
  for(unsigned int i=0; i<edge_loops.size(); ++i)
  {
    sil.new_sheet();
    const vcl_vector<unsigned int>& loop = edge_loops[i];
    // convert edge indices to vertex indices
    const unsigned num_pts = loop.size();
    vcl_vector<unsigned int> vidx(num_pts);
    for(unsigned int j=0; j<num_pts; ++j)
      vidx[j] = vehicle.half_edges()[loop[j]].vert_index();

    // find visible vertices
    vcl_vector<bool> visible(num_pts,false);
    for(unsigned int j=0; j<num_pts; ++j)
    {
      unsigned int vi = vidx[j];
      const vgl_point_2d<double>& v = verts2d_[vi];
      sil.push_back(v);
      visible[j] = depth_test(v.x(), v.y(), depths_[vi]);
    }

    vcl_vector<vgl_point_2d<double> > contour;
    vcl_vector<unsigned int> contour_vidx;
    for(unsigned int j1=num_pts-1, j2=0; j1<num_pts; j1 = j2++)
    {
      if(visible[j1]){
        contour.push_back(verts2d_[vidx[j1]]);
        contour_vidx.push_back(vidx[j1]);
      }
      else
      {
        if(contour.size() > 1){
          contours_.push_back(contour);
          contours_vidx_.push_back(contour_vidx);
        }
        contour.clear();
        contour_vidx.clear();
      }
    }
    if(contour.size() > 1){
      contours_.push_back(contour);
      contours_vidx_.push_back(contour_vidx);
    }
  }
  
  // added in shadow curves if available
  unsigned int num_vehicle_loops = edge_loops.size();
  if(!shadow_edge_loops_.empty()){
    for(unsigned int i=0; i<shadow_edge_loops_.size(); ++i){
      edge_loops.push_back(shadow_edge_loops_[i]);
      sil.push_back(shadow_[i]);
    }
  }
  
  // trace the part of the contour that makes up the silhouette
  vcl_vector<vcl_pair<unsigned, unsigned> > sil_idx;
  vcl_vector<vcl_pair<double,double> > sil_range;
  if(trace_silhouette(sil, sil_idx, sil_range)){
    // map the silhouette polygon indices back to mesh vertex indices
    map_silhouette(vehicle, edge_loops, sil, sil_idx, sil_range, num_vehicle_loops);
  }else{
    silhouette_vidx_.clear();
    silhouette_range_.clear();
    silhouette_shadow_.clear();
    silhouette_.clear();
  }
}


//: map the silhouette contours back to vehicle mesh vertices
void modrec_pca_vehicle_projector:: 
map_silhouette(const modrec_pca_vehicle& vehicle,
               const vcl_vector<vcl_vector<unsigned int> >& edge_loops,
               const vgl_polygon<double>& sil,
               const vcl_vector<vcl_pair<unsigned, unsigned> >& sil_idx,
               const vcl_vector<vcl_pair<double,double> >& sil_range,
               unsigned int num_vehicle_loops)
{
  silhouette_vidx_.clear();
  silhouette_range_.clear();
  silhouette_shadow_.clear();
  silhouette_.clear();
  const vcl_vector<vgl_point_2d<double> >& sil_pts = sil[0];
  vcl_vector<unsigned int> vidx;
  vcl_vector<vgl_point_2d<double> > pts;
  unsigned int k=0;
  unsigned last_idx = vehicle.num_verts();
  silhouette_range_.push_back(vcl_pair<double,double>(sil_range[0].first,1));
  silhouette_shadow_.push_back(sil_idx[0].first >= num_vehicle_loops);
  for(unsigned int i=0; i<sil_idx.size(); ++i)
  {
    unsigned int he_idx = edge_loops[sil_idx[i].first][sil_idx[i].second];
    const imesh_half_edge& he = vehicle.half_edges()[he_idx];
    const imesh_half_edge& ohe = vehicle.half_edges()[he.pair_index()];
    unsigned int idx = he.vert_index();
    vidx.push_back(idx);
    assert(k < sil_pts.size());
    pts.push_back(sil_pts[k++]);
    
    last_idx = ohe.vert_index();
    if(sil_range[i].second < 1){
      vidx.push_back(last_idx);
      assert(k < sil_pts.size());
      pts.push_back(sil_pts[k]);
      silhouette_vidx_.push_back(vidx);
      silhouette_.push_back(pts);
      vidx.clear();
      pts.clear();
      silhouette_range_.back().second = sil_range[i].second;
      if(i+1<sil_idx.size()){
        silhouette_range_.push_back(vcl_pair<double,double>(sil_range[i+1].first,1));
        silhouette_shadow_.push_back(sil_idx[i+1].first >= num_vehicle_loops);
      }
      last_idx = vehicle.num_verts();
    }
  }
  if(!vidx.empty()){
    // connect the end back to the start
    if(silhouette_vidx_.empty() && last_idx == vidx[0] ){ // a single contour comprises the silhouette
      vidx.push_back(last_idx);
      pts.push_back(sil_pts[0]);
      silhouette_vidx_.push_back(vidx);
      silhouette_.push_back(pts);
    }
    else if(!silhouette_vidx_.empty() && last_idx == silhouette_vidx_[0][0]){
      silhouette_vidx_.front().insert(silhouette_vidx_.front().begin(),
                                      vidx.begin(),vidx.end());
      silhouette_.front().insert(silhouette_.front().begin(),
                                 pts.begin(),pts.end());
      silhouette_range_.front().first = silhouette_range_.back().first;
      silhouette_range_.pop_back();
      silhouette_shadow_.pop_back();
    }
    else{
      vidx.push_back(last_idx);
      //pts.push_back(sil_pts[k]);
      silhouette_vidx_.push_back(vidx);
      //assert(k < sil_pts.size());
      silhouette_.push_back(pts);
    }
  }
}


//: trace the outer silhoutte of the occluding contours
// this function is incredibly complex and ugly to handle the many degenerate case
// there must be a way to make this more elegant
bool modrec_pca_vehicle_projector::
trace_silhouette(vgl_polygon<double>& p,
                 vcl_vector<vcl_pair<unsigned, unsigned> >& sil_idx,
                 vcl_vector<vcl_pair<double,double> >& sil_frac)
{
  typedef vcl_pair<unsigned,unsigned> upair;
  typedef vcl_pair<double,double> dpair;
  const double ftol = vcl_sqrt(vgl_tolerance<double>::position);
  vcl_vector<upair> e1, e2;
  vcl_vector<vgl_point_2d<double> > ip;
  vgl_selfintersections(p,e1,e2,ip);

  
  // create fast lookup of intersections
  vcl_multimap<upair, unsigned> isect_map;
  typedef vcl_multimap<upair, unsigned>::iterator map_itr;
  for(unsigned int i=0; i<e1.size(); ++i){
    isect_map.insert(vcl_pair<upair,unsigned>(e1[i],i));
    isect_map.insert(vcl_pair<upair,unsigned>(e2[i],i));
  }
  
  // the minimum x value should be on the silhouette
  // if multiple edges exit this point, pick the one with
  // the maximum angle from the x-axis
  upair min_pt;
  double max_angle = 0;
  unsigned max_num_pts = ip.size() + 1;
  double min_val = vcl_numeric_limits<double>::infinity();
  for(unsigned s=0; s<p.num_sheets(); ++s){
    for(unsigned i=0; i<p[s].size(); ++i){
      if(p[s][i].x() < min_val){
        unsigned n = (i+1)%p[s].size();
        vgl_vector_2d<double> edge = p[s][n]-p[s][i];
        double angle = vcl_atan2(edge.y(),edge.x());
        if(vnl_math_isfinite(min_val) &&
           (p[min_pt.first][min_pt.second]-p[s][i]).sqr_length() < ftol &&
           angle < max_angle){
          continue;
        }
        min_val = p[s][i].x() + ftol;
        min_pt = upair(s,i);
        max_angle = angle;
      }
      ++max_num_pts;
    }
  }
  
  int dir = 1;
  vcl_vector<vgl_point_2d<double> > sil;
  upair curr = min_pt;
  double curr_frac = ftol;
  vgl_point_2d<double> curr_pt = p[curr.first][curr.second];
  unsigned count;
  for(count=0; count<max_num_pts; ++count){
    sil.push_back(curr_pt);
    sil_idx.push_back(curr);
    
    upair next = curr;
    next.second += p[next.first].size()+dir;
    next.second %= p[next.first].size();
    // find the collection of intersections
    vcl_pair<map_itr, map_itr> range;
    if(dir < 0) // edges are indexed by the previous vertex
      range = isect_map.equal_range(next);
    else
      range = isect_map.equal_range(curr);
    if(range.first == range.second){ // no intersection
      sil_frac.push_back(dpair(0.0,1.0));
      curr = next;
      curr_frac = 0.0;
      curr_pt = p[curr.first][curr.second];
    }
    else{
      double min_frac = 2.0;
      map_itr min_itr = isect_map.end();
      vgl_vector_2d<double> edge1 = p[next.first][next.second]
                                   -p[curr.first][curr.second];
      vgl_vector_2d<double> edge = edge1 / edge1.sqr_length();
      double etol = edge.length()*ftol;
      bool parallel_edge = false;
      // find the first intersection along the edge
      for(map_itr itr=range.first; itr!=range.second; ++itr){
        double frac = dot_product(edge,ip[itr->second] - p[curr.first][curr.second]);
        
        // check for parallel intersections
        unsigned k = itr->second;
        upair other = e1[k];
        if((dir<0 && next == other) || (dir>0 && curr == other))
          other = e2[k];
        upair o_next = other; 
        o_next.second += 1;
        o_next.second %= p[o_next.first].size();
        vgl_vector_2d<double> edge2 = p[o_next.first][o_next.second]
                                     -p[other.first][other.second];
        if(parallel(edge1,edge2,ftol)){
          // find start and end fractions
          double frac1 = dot_product(edge,p[other.first][other.second] 
                                        - p[curr.first][curr.second]);
          double frac2 = dot_product(edge,p[o_next.first][o_next.second] 
                                        - p[curr.first][curr.second]);

          // make sure the other edge goes in the same direction
          if(frac1 > frac2){
            vcl_swap(frac1,frac2);
          }
          if(frac2-etol > 1.0 && frac1+etol <= curr_frac){
            // another parallel edge is longer, so switch to that one
            parallel_edge = true;
            min_frac = 0.5; // don't trigger start or end intersections
            min_itr = itr;
            break;
          }
          continue; 
        }
        
        if(frac < min_frac && frac > curr_frac){
          min_frac = frac;
          min_itr = itr;
        }
      }
      if(min_itr == isect_map.end()){ // intersection are behind the current point
        sil_frac.push_back(dpair(curr_frac,1.0));
        curr = next;
        curr_frac = 0.0;
        curr_pt = p[curr.first][curr.second];
      }
      else{
        bool isect_at_start = vcl_abs(min_frac) < etol;
        bool isect_at_end = vcl_abs(1.0-min_frac) < etol;
        if(isect_at_end)
          sil_frac.push_back(dpair(curr_frac,1.0));
        // special case of intersection at start vertex
        // reconfigure to intersection at end vertex
        if(isect_at_start){
          isect_at_end = true;
          next = curr;
          sil.pop_back();
          sil_idx.pop_back();
          assert(!sil.empty());
          edge = p[next.first][next.second]-sil.back();
          curr = sil_idx.back();
        }
        // special case of intersection at end vertex
        if(isect_at_end){
          double base_angle = -vcl_atan2(-edge.y(),-edge.x());
          // find all possible output edges (might be more than two)
          upair n_next = next;
          n_next.second += p[n_next.first].size()+dir;
          n_next.second %= p[n_next.first].size();
          vgl_vector_2d<double> edge2 = p[n_next.first][n_next.second]
                                       -p[next.first][next.second];
          double min_angle = -vcl_atan2(edge2.y(),edge2.x())-base_angle;
          if(min_angle < ftol) min_angle += 2*vnl_math::pi;
          upair best_pair = next;
          int best_dir = dir;
          for(map_itr itr=range.first; itr!=range.second; ++itr){
            //double frac = dot_product(edge,ip[itr->second] - p[curr.first][curr.second]);
            //if(vcl_abs(1.0-min_frac) < ftol){
            unsigned k = itr->second;
            if((ip[k]-p[next.first][next.second]).sqr_length() 
               < ftol){
              upair other = e1[k];
              if((dir<0 && next == other) || (dir>0 && curr == other))
                other = e2[k];
              upair o_next = other; 
              o_next.second += 1;
              o_next.second %= p[o_next.first].size();
              vgl_vector_2d<double> edge2 = p[o_next.first][o_next.second]
                                           -p[other.first][other.second];
              // try one direction if it extends beyond the point
              double angle = -vcl_atan2(edge2.y(),edge2.x())-base_angle;
              if(angle < 0) angle += 2*vnl_math::pi;
              if(angle < min_angle && angle > ftol &&
                 (p[o_next.first][o_next.second]-p[next.first][next.second]).sqr_length() 
                  > vgl_tolerance<double>::position){
                min_angle = angle;
                best_pair = other;
                best_dir = 1;
              }
              // try the other direction if it extends beyond the point
              angle = -vcl_atan2(-edge2.y(),-edge2.x())-base_angle;
              if(angle < 0) angle += 2*vnl_math::pi;
              if(angle < min_angle && angle > ftol &&
                 (p[other.first][other.second]-p[next.first][next.second]).sqr_length() 
                  > vgl_tolerance<double>::position){
                min_angle = angle;
                best_pair = o_next;
                best_dir = -1;
              }
            }
          }
          if(best_pair == next){ // continue along the same curve
            curr = next;
            curr_frac = 0.0;
            if(isect_at_start)
              curr_frac = etol;
            curr_pt = p[curr.first][curr.second];
          }
          else{
            curr_pt = p[next.first][next.second];
            curr = best_pair;
            dir = best_dir;
            next = curr;
            next.second += p[next.first].size()+dir;
            next.second %= p[next.first].size();
            edge = p[next.first][next.second]-p[curr.first][curr.second];
            edge /= edge.sqr_length();
            curr_frac = dot_product(edge,curr_pt - p[curr.first][curr.second]);
          }
        }
        else{ // a normal edge-edge intersection
          
          const vgl_point_2d<double>& v1 = p[curr.first][curr.second];
          const vgl_point_2d<double>& v2 = p[next.first][next.second];
        
          unsigned k = min_itr->second;
          upair other = e1[k];
          upair last = curr;
          int last_dir = dir;
          if((dir<0 && next == other) || (dir>0 && curr == other))
            other = e2[k];
          
          curr = other;
          next = other;
          dir = 1;
          next.second += p[next.first].size()+dir;
          next.second %= p[next.first].size();
          vgl_point_2d<double> v3 = p[curr.first][curr.second];
          vgl_point_2d<double> v4 = p[next.first][next.second];
          
          // handle the parallel edge case
          if(parallel_edge){
            edge = v2-v1;
            edge /= edge.sqr_length();
            vgl_vector_2d<double> edge2 = v4-v3;
            edge2 /= edge2.sqr_length();
            double frac1 = dot_product(edge,v3-v1);
            double frac2 = dot_product(edge,v4-v1);
            if(frac1>frac2){
              vcl_swap(frac1,frac2);
              dir = -1;
              vcl_swap(curr,next);
              vcl_swap(v3,v4);
              edge2 *= -1;
            }
            double old_frac = dot_product(edge,curr_pt-v1);
            if(frac1<=old_frac){
              sil.pop_back();
              sil_idx.pop_back();
              curr_frac = dot_product(edge2,curr_pt-v3)+ftol;
            }
            else{
              sil_frac.push_back(dpair(old_frac,frac1));
              curr_frac = dot_product(edge2,curr_pt-v3)+ftol;
            }
          }
          else{
            double disc1 = vgl_triangle_test_discriminant(v1.x(),v1.y(), 
                                                          v2.x(),v2.y(), 
                                                          v3.x(),v3.y());
            double disc2 = vgl_triangle_test_discriminant(v1.x(),v1.y(), 
                                                          v2.x(),v2.y(), 
                                                          v4.x(),v4.y());
            if(ftol<disc1){
              dir = -1;
              vcl_swap(curr,next);
              vcl_swap(disc1,disc2);
              vcl_swap(v3,v4);
            }
            
            edge = p[next.first][next.second]-p[curr.first][curr.second];
            edge /= edge.sqr_length();
            
            // test for intersection at endpoint
            bool valid_isect = true;
            if(vcl_abs(disc2) < ftol) // intersects at the end
              valid_isect = false;
            else if(vcl_abs(disc1) < ftol) // intersects at the start
            {
              // it might not be safe to take this path if it intersects at an 
              // endpoint, check the angles to be sure 
              vgl_vector_2d<double> edge1 = v2-v1;
              normalize(edge1);
              // rotation relative to edge1
              double a = edge1.x()*edge.x() + edge1.y()*edge.y();
              double b = -edge1.y()*edge.x() + edge1.x()*edge.y();
              double angle = vcl_atan2(b,a);
              if(angle <= 0){
                valid_isect = false;
              }
            }
            if(valid_isect){
              // use this intersecting edge
              sil_frac.push_back(dpair(curr_frac,min_frac));
              curr_frac = dot_product(edge,ip[k] - p[curr.first][curr.second])+ftol;
              curr_pt = ip[k];
            }
            else{
              // try again looking after min_frac
              sil.pop_back();
              sil_idx.pop_back();
              curr = last;
              dir = last_dir;
              curr_frac = min_frac;
              curr_pt = v1;
            }
          }
        }
      }
      
    }
    if(curr == min_pt && curr_frac <= ftol)
      break;
  }
  
  if(count==max_num_pts){
    vcl_cout << "infinite loop in silhouette trace"<<vcl_endl;
    return false;
  }
  p.clear();
  p.push_back(sil);
  return true;
}


//: reproject the last visible part boundaries into the image
void modrec_pca_vehicle_projector::
reproject_parts(const vpgl_perspective_camera<double>& camera,
                const modrec_pca_vehicle& vehicle)
{
  parts_.clear();
  parts_idx_.clear();
  vcl_vector<vgl_point_2d<double> > verts2d;
  vcl_vector<double> depths;
  typedef modrec_pca_vehicle::uv_point uv_point;
  const vcl_vector<vcl_vector<vgl_point_3d<double> > >& parts3d = vehicle.parts_3d();
  const vcl_vector<vcl_vector<uv_point> >& parts_uv = vehicle.parts_bary();
  for(unsigned int b=0; b<parts_bounds_.size(); ++b)
  {
    const part_bounds& pb = parts_bounds_[b];
    unsigned int i = pb.part_idx;
    const vcl_vector<vgl_point_3d<double> >& part3d = parts3d[i];
    const vcl_vector<uv_point>& part_uv = parts_uv[i];
    if(b==0 || i != parts_bounds_[b-1].part_idx)
      imesh_project_verts(part3d,camera,verts2d,depths);
    
    vcl_vector<vgl_point_2d<double> > part;
    vcl_vector<vcl_pair<unsigned int,unsigned int> > part_idx;
    
    // add the whole part
    if(pb.start_idx == pb.end_idx && pb.s_t == 0.0 && pb.e_t == 1.0)
    {
      for(unsigned int j1=verts2d.size()-1, j2=0; j1<verts2d.size(); j1 = j2++)
      {
        part.push_back(verts2d[j1]);
        part_idx.push_back(vcl_pair<unsigned int,unsigned int>(i,j1));
      }
      parts_.push_back(part);
      parts_idx_.push_back(part_idx);
      continue;
    }
    
    unsigned int pc_base;
    vehicle.part_to_pc_idx(i,0,pc_base);
    const unsigned int psize = vehicle.part_size(i);
    const unsigned int p_uv_size = part_uv.size();
    unsigned int start_j = p_uv_size, end_j = p_uv_size;
    double min_diff_s = psize, min_diff_e = psize;
    for(unsigned int j=0; j<p_uv_size; ++j)
    {
      double diff = pb.start_idx + pb.s_t 
                  - (part_uv[j].end_point1-pc_base + part_uv[j].t);
      if(diff < 0.0)
        diff += psize;
      if(diff >= 0.0 && diff < min_diff_s)
      {
        min_diff_s = diff;
        start_j = j;
      }
      diff = (part_uv[j].end_point2-pc_base - part_uv[j].t)
           - (pb.end_idx - pb.e_t);
      if(diff < 0.0)
        diff += psize;
      if(diff >= 0.0 && diff < min_diff_e)
      {
        min_diff_e = diff;
        end_j = j;
      }
    }
    
    //assert(start_j < p_uv_size);
    //assert(end_j < p_uv_size);
      
    if(start_j < p_uv_size && end_j < p_uv_size){
      for(unsigned int j=start_j; j!=end_j; j = (j+1)%p_uv_size)
      {
        part.push_back(verts2d[j]);
        part_idx.push_back(vcl_pair<unsigned int,unsigned int>(i,j));
      }
    }
    else
      vcl_cerr << "bad part detected"<<vcl_endl;
    parts_.push_back(part);
    parts_idx_.push_back(part_idx);
  }
  
}


//: reproject the last visible occluding contours into the image
void modrec_pca_vehicle_projector::
reproject_contours(const vpgl_perspective_camera<double>& camera,
                   const modrec_pca_vehicle& vehicle)
{
  contours_.clear();
  contours_.resize(contours_vidx_.size());
  const imesh_vertex_array< 3 >& verts3d = vehicle.vertices<3>();
  
  for(unsigned int i=0; i<contours_vidx_.size(); ++i)
  {
    const unsigned int num_pts = contours_vidx_[i].size();
    vcl_vector<vgl_point_3d<double> > points3d_vis(num_pts);
    for(unsigned int j=0; j<num_pts; ++j)
    {
      points3d_vis[j] = verts3d[contours_vidx_[i][j]];
    }
    imesh_project_verts(points3d_vis,camera,contours_[i]);
  }
}


//: reproject the last valid silhouette edges into the image
void modrec_pca_vehicle_projector::
reproject_silhouette(const vpgl_perspective_camera<double>& camera,
                     const vpgl_proj_camera<double>& shadow_camera,
                     const modrec_pca_vehicle& vehicle)
{
  const imesh_vertex_array< 3 >& verts3d = vehicle.vertices<3>();
  
  silhouette_.clear();
  for(unsigned int i=0; i<silhouette_vidx_.size(); ++i)
  {
    const unsigned int num_pts = silhouette_vidx_[i].size();
    vcl_vector<vgl_point_3d<double> > points3d_sil(num_pts);
    vcl_vector<vgl_point_2d<double> > points2d_sil(num_pts);
    for(unsigned int j=0; j<num_pts; ++j)
    {
      points3d_sil[j] = verts3d[silhouette_vidx_[i][j]];
    }
    if(silhouette_shadow_[i])
      imesh_project_verts(points3d_sil,shadow_camera,points2d_sil);
    else
      imesh_project_verts(points3d_sil,camera,points2d_sil);
    
    // the first point may be trimmed
    vgl_vector_2d<double> offset(points2d_sil[1]-points2d_sil[0]);
    offset *= silhouette_range_[i].first;
    points2d_sil[0] += offset;
    
    // the last point may be trimmed
    offset = points2d_sil[num_pts-1]-points2d_sil[num_pts-2];
    offset *= silhouette_range_[i].second;
    points2d_sil[num_pts-1] = points2d_sil[num_pts-2] + offset;
    
    silhouette_.push_back(points2d_sil);
  }
}


//: Arrange the projected silhouette as a closed polygon
vgl_polygon<double> modrec_pca_vehicle_projector::silhouette_polygon() const
{
  vgl_polygon<double> sil(1);
  for(unsigned int i=0; i<silhouette_.size(); ++i){
    for(unsigned int j=0; j<silhouette_[i].size(); ++j){
      sil.push_back(silhouette_[i][j]);
    }
  } 
  return sil;
}

//: compute the PCA image Jacobians for both contours and part boundaries
void modrec_pca_vehicle_projector::
compute_jacobians(const vpgl_perspective_camera<double>& camera,
                  const vpgl_proj_camera<double>& shadow_cam,
                  const modrec_pca_vehicle& vehicle)
{
  contours_jac_.clear();
  silhouette_jac_.clear();
  parts_jac_.clear();
  parts_tex_jac_.clear();
  
  // compute indices for each enabled parameter
  unsigned int c=0;
  int txi = options_[1]?c++:-1;
  int tyi = options_[2]?c++:-1;
  int tzi = options_[3]?c++:-1;
  int rxi = options_[4]?c++:-1;
  int ryi = options_[5]?c++:-1;
  int rzi = options_[6]?c++:-1;
  
  if(!options_[0])
    num_pc_ = 0;
  
  // translation is the same for all points, so set this
  vnl_matrix<double> dir_3d(3,c,0.0);
  if(txi>=0) dir_3d(0,txi) = 1.0;
  if(tyi>=0) dir_3d(1,tyi) = 1.0;
  if(tzi>=0) dir_3d(2,tzi) = 1.0;

  const imesh_vertex_array<3>& verts = vehicle.vertices<3>();
  
  const unsigned int num_contours = contours_vidx_.size();
  const unsigned int num_sil_sections = silhouette_vidx_.size();
  // silhouette and contours share many common points,
  // so share the computation of the Jacobians at those points
  // these vectors mirror the *_vidx_ arrays, 
  // but map to a common pool of Jacobians
  vcl_vector<vcl_vector<unsigned int> > contours_jidx(num_contours);
  vcl_vector<vcl_vector<unsigned int> > silhouette_jidx(num_sil_sections);
  // map mesh vertex indices to Jacobian pool indices
  vcl_map<unsigned,unsigned> vidx_to_jidx;
  // the common vertex pool
  vcl_vector<vgl_point_3d<double> > pts;
  vcl_vector<unsigned> pts_vidx;

  // Add all the visible 3D contour points to a vector
  for(unsigned int i=0; i<num_contours; ++i)
  {
    const vcl_vector<unsigned int>& vidx = contours_vidx_[i];
    vcl_vector<unsigned int>& jidx = contours_jidx[i];
    jidx.resize(vidx.size());
    for(unsigned int j=0; j<vidx.size(); ++j)
    {
      const unsigned& vi = vidx[j];
      vcl_map<unsigned,unsigned>::iterator f = vidx_to_jidx.find(vi);
      if(f==vidx_to_jidx.end())
      {
        vidx_to_jidx[vi] = pts.size();
        jidx[j] = pts.size();
        pts.push_back(verts[vi]);
        pts_vidx.push_back(vi);
      }
      else // already added
      {
        jidx[j] = f->second;
      }
    }
  }
  
  
  // points coming from shadow must be computed separately in their own pool
  vcl_vector<vgl_point_3d<double> > shadow_pts;
  vcl_vector<unsigned> shadow_pts_vidx;
  
  // Add all the silhouette points not previously added
  for(unsigned int i=0; i<num_sil_sections; ++i)
  {
    const vcl_vector<unsigned int>& vidx = silhouette_vidx_[i];
    vcl_vector<unsigned int>& jidx = silhouette_jidx[i];
    jidx.resize(vidx.size());
    for(unsigned int j=0; j<vidx.size(); ++j)
    {
      const unsigned& vi = vidx[j];
      if(!silhouette_shadow_[i]){ // not a shadow point
        vcl_map<unsigned,unsigned>::iterator f = vidx_to_jidx.find(vi);
        if(f==vidx_to_jidx.end())
        {
          vidx_to_jidx[vi] = pts.size();
          jidx[j] = pts.size();
          pts.push_back(verts[vi]);
          pts_vidx.push_back(vi);
        }
        else // already added
        {
          jidx[j] = f->second;
        }
      }
      else{ // this is a shadow point
        jidx[j] = shadow_pts.size();
        shadow_pts.push_back(verts[vi]);
        shadow_pts_vidx.push_back(vi);
      }
    }
  }
  const unsigned int num_contour_pts = pts.size();

  const vcl_vector<vcl_vector<vgl_point_3d<double> > >& pt3d = vehicle.parts_3d();

  // Add all the visible 3D part boundary points
  const unsigned int num_parts = parts_idx_.size();
  for(unsigned int i=0; i<num_parts; ++i)
  {
    const vcl_vector<vcl_pair<unsigned int,unsigned int> >& idx = parts_idx_[i];
    for(unsigned int j=0; j<idx.size(); ++j)
      pts.push_back(pt3d[idx[j].first][idx[j].second]);
  }
  const unsigned int num_vehicle_pts = pts.size();

  // compute the image Jacobians at each point
  vcl_vector<vnl_matrix_fixed<double,2,3> > J = image_jacobians(camera,pts);
  // compute the shadow image Jacobians at each shadow point
  vcl_vector<vnl_matrix_fixed<double,2,3> > Js = image_jacobians(shadow_cam,shadow_pts);
  // append Js to J and shadow_pts to pts
  J.insert(J.end(),Js.begin(),Js.end());
  pts.insert(pts.end(),shadow_pts.begin(),shadow_pts.end());

  // compute the Jacobians for extrinincs at each point
  vcl_vector<vnl_matrix<double> > extrinsics(pts.size());
  for(unsigned int i=0; i<pts.size(); ++i)
  {
    const vgl_point_3d<double>& p = pts[i];
    if(rxi>=0){ dir_3d(1,rxi) =-p.z();  dir_3d(2,rxi) = p.y(); }
    if(ryi>=0){ dir_3d(0,ryi) = p.z();  dir_3d(2,ryi) =-p.x();}
    if(rzi>=0){ dir_3d(0,rzi) =-p.y();  dir_3d(1,rzi) = p.x(); }
    extrinsics[i] = J[i] * dir_3d;
  }
  
  // Allocate the Jacobian matrices and fill in the extrinsic Jacobians
  for(unsigned int i=0; i<num_contours; ++i)
  {
    const unsigned int num = contours_vidx_[i].size();
    contours_jac_.push_back(vcl_vector<vnl_matrix<double> >(num,
                                                            vnl_matrix<double>(2,c+num_pc_)));
    const vcl_vector<unsigned>& jidx = contours_jidx[i];
    for(unsigned int j=0; j<num; ++j)
      contours_jac_.back()[j].update(extrinsics[jidx[j]],0,num_pc_);
  }
  for(unsigned int i=0; i<num_sil_sections; ++i)
  {
    const unsigned int num = silhouette_vidx_[i].size();
    silhouette_jac_.push_back(vcl_vector<vnl_matrix<double> >(num,
                                                            vnl_matrix<double>(2,c+num_pc_)));
    vcl_vector<unsigned>& jidx = silhouette_jidx[i];
    int offset=0;
    if(silhouette_shadow_[i])
      offset = num_vehicle_pts;
    for(unsigned int j=0; j<num; ++j)
      silhouette_jac_.back()[j].update(extrinsics[jidx[j]+offset],0,num_pc_);
    
  }
  unsigned int k=num_contour_pts;
  for(unsigned int i=0; i<num_parts; ++i)
  {
    const unsigned int num = parts_idx_[i].size();
    parts_jac_.push_back(vcl_vector<vnl_matrix<double> >(num,
                                                         vnl_matrix<double>(2,c+num_pc_)));
    for(unsigned int j=0; j<num; ++j)
      parts_jac_.back()[j].update(extrinsics[k++],0,num_pc_);
  }
  
  

  // Compute PCA Jacobians
  if(options_[0] && num_pc_ > 0){
    
    // precompute PCA Jacobians at each common point
    vcl_vector<vnl_matrix<double> > contour_sil_J(num_contour_pts);
    const vnl_matrix<double>& pc = vehicle.principal_comps();
    const vnl_vector<double>& std = vehicle.std_devs();
    for(unsigned int i=0; i<num_contour_pts; ++i)
    {
      vnl_matrix<double> dir_3d(num_pc_,3);
      pc.extract(dir_3d,0,3*pts_vidx[i]);
      for(unsigned int j=0; j<num_pc_; ++j){
        dir_3d(j,0) *= std[j];
        dir_3d(j,1) *= std[j];
        dir_3d(j,2) *= std[j];
      }
      contour_sil_J[i] = J[i]*dir_3d.transpose();
    }
    // update contour Jacobians with PCA submatrix
    for(unsigned int i=0; i<num_contours; ++i)
    {
      const unsigned int num = contours_vidx_[i].size();
      const vcl_vector<unsigned>& jidx = contours_jidx[i];
      for(unsigned int j=0; j<num; ++j)
        contours_jac_[i][j].update(contour_sil_J[jidx[j]]);
    }
    // update silhouette Jacobians with PCA submatrix
    for(unsigned int i=0; i<num_sil_sections; ++i)
    {
      const unsigned int num = silhouette_vidx_[i].size();
      const vcl_vector<unsigned>& jidx = silhouette_jidx[i];
      if(silhouette_shadow_[i]){
        for(unsigned int j=0; j<num; ++j){
          vnl_matrix<double> dir_3d(num_pc_,3);
          pc.extract(dir_3d,0,3*shadow_pts_vidx[jidx[j]]);
          for(unsigned int k=0; k<num_pc_; ++k){
            dir_3d(k,0) *= std[k];
            dir_3d(k,1) *= std[k];
            dir_3d(k,2) *= std[k];
          }
          silhouette_jac_[i][j].update(Js[jidx[j]]*dir_3d.transpose());
        }
      }
      else{
        for(unsigned int j=0; j<num; ++j)
          silhouette_jac_[i][j].update(contour_sil_J[jidx[j]]);
      }
    }
    
    if(num_parts > 0)
      compute_parts_pca_jacobians(vehicle, J.begin()+num_contour_pts);
  }


}


//: compute the PCA image Jacobians for contour points given the world-to-image Jacobians
void modrec_pca_vehicle_projector::
compute_contour_pca_jacobians(const modrec_pca_vehicle& vehicle,
                              vcl_vector<vnl_matrix_fixed<double,2,3> >::iterator J)
{
  const vnl_matrix<double>& pc = vehicle.principal_comps();
  const vnl_vector<double>& std = vehicle.std_devs();
  const unsigned int num_contours = contours_vidx_.size();
  
  // start with the contour points
  for(unsigned int i=0; i<num_contours; ++i)
  {
    const vcl_vector<unsigned int>& vidx = contours_vidx_[i];
    for(unsigned int j=0; j<vidx.size(); ++j)
    {
      vnl_matrix<double> dir_3d(num_pc_,3);
      pc.extract(dir_3d,0,3*vidx[j]);
      for(unsigned int k=0; k<num_pc_; ++k){
        dir_3d(k,0) *= std[k];
        dir_3d(k,1) *= std[k];
        dir_3d(k,2) *= std[k];
      }
      
      contours_jac_[i][j].update((*(J++))*dir_3d.transpose());
    }
  }
}


//: compute the PCA image Jacobians for part boundaries given the world-to-image Jacobians
void modrec_pca_vehicle_projector::
compute_parts_pca_jacobians(const modrec_pca_vehicle& vehicle,
                            vcl_vector<vnl_matrix_fixed<double,2,3> >::iterator J)
{
  // now do the parts points
  assert(vehicle.faces().regularity() == 3);
  const imesh_regular_face_array<3>& triangles =
  static_cast<const imesh_regular_face_array<3>&>(vehicle.faces());
  const imesh_half_edge_set& he = vehicle.half_edges();
  const unsigned int num_parts = parts_idx_.size();
  const vnl_matrix<double>& pc = vehicle.principal_comps();
  const vnl_matrix<double>& ppc = vehicle.parts_principal_comps();
  const vnl_vector<double>& std = vehicle.std_devs();
  
  // pre allocate matrices
  vnl_matrix<double> dir_3d(num_pc_,3),
                     dir_3d_0(num_pc_,3),
                     dir_3d_1(num_pc_,3),
                     dir_3d_2(num_pc_,3);
  
  vnl_matrix<double> dir_uv(num_pc_,2),
                     dir_uv_0(num_pc_,2),
                     dir_uv_1(num_pc_,2);
  
  vnl_matrix<double> M(3,2);
  
  typedef modrec_pca_vehicle::uv_point uv_point;
  const vcl_vector<vcl_vector<uv_point> >& parts_uv = vehicle.parts_bary();
  for(unsigned int i=0; i<num_parts; ++i)
  {
    parts_tex_jac_.push_back(vcl_vector<vnl_matrix<double> >());
    const vcl_vector<vcl_pair<unsigned int,unsigned int> >& idx = parts_idx_[i];
    for(unsigned int j=0; j<idx.size(); ++j)
    {
      unsigned int i1 = idx[j].first, i2 = idx[j].second;
      unsigned int fidx = he[parts_uv[i1][i2].mesh_index>>2].face_index();
      const vgl_point_2d<double>& uv = parts_uv[i1][i2].uv;
      const imesh_regular_face<3>& tri = triangles[fidx];
      
      // The 3d deriviates at triangle vertices 
      pc.extract(dir_3d_0,0,3*tri[0]);
      pc.extract(dir_3d_1,0,3*tri[1]);
      pc.extract(dir_3d_2,0,3*tri[2]);
      
      // The 2d texture space derivative at end points
      ppc.extract(dir_uv_0,0,2*parts_uv[i1][i2].end_point1);
      ppc.extract(dir_uv_1,0,2*parts_uv[i1][i2].end_point2);
      double t = parts_uv[i1][i2].t, tm = 1-t;
      // compute the point offset by one from the current point
      dir_uv = dir_uv_0*tm + dir_uv_1*t;
      parts_tex_jac_.back().push_back(dir_uv);
      
      imesh_project_texture_to_3d_map(vehicle, fidx).extract(M);      
      
      dir_3d = (1-uv.x()-uv.y())*dir_3d_0;
      dir_3d += uv.x()*dir_3d_1;
      dir_3d += uv.y()*dir_3d_2;
      
      for(unsigned int k=0; k<num_pc_; ++k){
        dir_3d(k,0) *= std[k];
        dir_3d(k,1) *= std[k];
        dir_3d(k,2) *= std[k];
        dir_uv(k,0) *= std[k];
        dir_uv(k,1) *= std[k];
      }
      
      dir_3d.inplace_transpose();
      
      dir_3d += M*dir_uv.transpose();
      
      parts_jac_[i][j].update((*(J++))*dir_3d);
    }
  }
}


//=========================================================
// External functions


//: Save the projected contours as SVG
bool modrec_write_svg_curves(const vcl_string& filename,
                             const modrec_pca_vehicle_projector& projector)
{
  vcl_ofstream ofs(filename.c_str());
  if(!ofs.is_open())
    return false;
  
  unsigned int ni = projector.depth_map().ni();
  unsigned int nj = projector.depth_map().nj();
  
  ofs << "<?xml version=\"1.0\" standalone=\"no\"?>\n"
  << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n" 
  << "  \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n"
  << "<svg width=\""<< ni <<"px\" height=\""<< nj <<"px\" version=\"1.1\"\n"
  << "     viewBox=\"0 0 "<< ni <<" "<< nj <<"\" xmlns=\"http://www.w3.org/2000/svg\" preserveAspectRatio=\"none\">\n";
  
  // Draw a box around the image
  ofs << "  <rect x=\"0\" y=\"0\" width=\""<< ni <<"\" height=\""<< nj <<"\" "
  << "fill=\"white\" opacity=\"0.5\" stroke=\"none\" stroke-width=\"1px\" />\n";
  
  ofs << "  <g id=\"parts\">\n";
  // Draw the parts in red
  for(unsigned int i=0; i<projector.parts().size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& curve = projector.parts()[i];
    ofs << "  <polyline fill=\"none\" stroke=\"red\" stroke-width=\"2\"\n"
    << "           points=\"";
    for(unsigned int j=0; j<curve.size(); ++j)
      ofs << curve[j].x()<<','<<curve[j].y()<<' ';
    ofs << "\" />\n";
  }
  
  ofs << "  </g>\n  <g id=\"contours\">\n";
  
  // Draw the occluding contours in green
  for(unsigned int i=0; i<projector.contours().size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& curve = projector.contours()[i];
    if(curve.empty())
      continue;
    ofs << "  <polyline fill=\"none\" stroke=\"green\" stroke-width=\"2\"\n"
    << "           points=\"";
    for(unsigned int j=0; j<curve.size(); ++j)
      ofs << curve[j].x()<<','<<curve[j].y()<<' ';
    ofs << "\" />\n";
  }
  
  ofs << "  </g>\n  <g id=\"silhouette\">\n";
  
  // Draw the silhouette contours in blue
  for(unsigned int i=0; i<projector.silhouette().size(); ++i)
  {
    const vcl_vector<vgl_point_2d<double> >& curve = projector.silhouette()[i];
    if(curve.empty())
      continue;
    ofs << "  <polyline fill=\"none\" stroke=\"blue\" stroke-width=\"2\"\n"
    << "           points=\"";
    for(unsigned int j=0; j<curve.size(); ++j)
      ofs << curve[j].x()<<','<<curve[j].y()<<' ';
    ofs << "\" />\n";
  }
  ofs << "  </g>\n";
  ofs << "</svg>\n";
  ofs.close();
  return true;
}
