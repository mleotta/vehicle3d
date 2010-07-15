// This is mleotta/gui/pca_vehicle/modrec_pca_vehicle.cxx
//=========================================================================
//:
// \file
// \brief  PCA vehicle model with parts
//
//=========================================================================

#include "modrec_pca_vehicle.h"

#include <imesh/algo/imesh_project.h>
#include <imesh/imesh_fileio.h>
#include <modrec/modrec_vehicle_parts.h>

#include <vnl/vnl_matlab_filewrite.h>

//: Constructor
modrec_pca_vehicle::modrec_pca_vehicle(const imesh_pca_mesh& pca_mesh,
                                       const part_map& parts)
  : imesh_pca_mesh(pca_mesh),
    parts_(parts), 
    mean_parts_(parts)
{
  init_part_idx_offsets();
}


//: Constructor from a mesh, parts, mean, standard deviations, and PC matrix
modrec_pca_vehicle::modrec_pca_vehicle(const imesh_mesh& mesh,
                                       const part_map& parts,
                                       const vnl_vector<double>& mean,
                                       const vnl_vector<double>& std_devs,
                                       const vnl_matrix<double>& pc)
: imesh_pca_mesh(mesh),
  parts_(parts), 
  mean_parts_(parts)
{
  init_part_idx_offsets();
  init(mean,std_devs,pc);
}


//: Constructor - compute the pca from a joint collection of meshes and parts
modrec_pca_vehicle::modrec_pca_vehicle(const vcl_vector<imesh_mesh>& meshes,
                                       const vcl_vector<vcl_map<vcl_string,
                                               vgl_polygon<double> > >& part_groups)
  : imesh_pca_mesh(meshes[0])
{
  assert(meshes.size() == part_groups.size());
  
  // Create a sum of all the part points
  vcl_map<vcl_string, unsigned int > part_count;
  unsigned int num_pts = 0;
  const unsigned num_training = meshes.size();

  for(unsigned int i=0; i<part_groups.size(); ++i)
  {
    unsigned int pidx = 0;
    for(part_map::const_iterator itr=part_groups[i].begin();
        itr!=part_groups[i].end(); ++itr)
    {
      vgl_polygon<double>& mean = mean_parts_[itr->first];
      const vgl_polygon<double>& part = itr->second;
      if(mean.num_sheets() == 0){
        mean = part;
        part_count[itr->first] = 1;
        num_pts += part.num_vertices();
        continue;
      }
      assert(mean[0].size() == part[0].size());

      for(unsigned int j=0; j<mean[0].size(); ++j)
      {
        vgl_point_2d<double>& m = mean[0][j];
        const vgl_point_2d<double>& p = part[0][j];
        m = vgl_point_2d<double>(m.x()+p.x(),m.y()+p.y());
      }
      ++part_count[itr->first];
    }
  }

  vnl_matrix<double> M(num_pts*2,num_training);
  vnl_vector<double> mean(num_pts*2,0.0);
  unsigned int pidx = 0;
  // divide by the number of occurances of each part to get the means
  for(part_map::iterator itr=mean_parts_.begin(); itr!=mean_parts_.end(); ++itr)
  {
    vgl_polygon<double>& mean_poly = itr->second;
    unsigned int c = part_count[itr->first];
    unsigned int pidx2 = pidx;
    for(unsigned int j=0; j<mean_poly[0].size(); ++j)
    {
      vgl_point_2d<double>& m = mean_poly[0][j];
      m = vgl_point_2d<double>(m.x()/c, m.y()/c);
      mean[pidx++] = m.x();
      mean[pidx++] = m.y();
    }
    // compute the deviation from the mean for each part
    for(unsigned int i=0; i<part_groups.size(); ++i)
    {
      part_map::const_iterator pitr = part_groups[i].find(itr->first);
      if(pitr == part_groups[i].end()) // missing part, assume mean (zero deviation)
      {
        for(unsigned int j=0; j<mean_poly[0].size(); ++j)
        {
          M(pidx2+2*j,i) = 0;
          M(pidx2+2*j+1,i) = 0;
        }
      }
      else // part is found so use it
      {
        const vgl_polygon<double>& poly = pitr->second;
        for(unsigned int j=0; j<mean_poly[0].size(); ++j)
        {
          M(pidx2+2*j,i) = poly[0][j].x() - mean[pidx2+2*j];
          M(pidx2+2*j+1,i) = poly[0][j].y() - mean[pidx2+2*j+1];
        }
      }
    }
  }
  // initialize parts with the mean parts
  parts_ = mean_parts_;
  
  // mesh shape variations
  vnl_matrix<double> V = compute_mean(meshes);
  
  // concatenate shape and parts matrices
  vnl_matrix<double> Vc(V.rows()+M.rows(),V.columns());
  Vc.update(V);
  Vc.update(M,V.rows());
  
  vnl_svd<double> A(Vc,-1e-8);
  
  std_devs_ = vnl_vector<double>(A.W().diagonal().data_block(), A.rank());
  vnl_matrix<double> PC(A.rank(),Vc.rows());
  A.U().transpose().extract(PC);
  vcl_cout << "pc matrix is "<<PC.rows()<<" x "<<PC.columns()<<vcl_endl;
  
  pc_ = PC.extract(A.rank(),V.rows());
  parts_pc_ = PC.extract(A.rank(),M.rows(),0,V.rows());
  
#if 1
  vnl_matlab_filewrite mfw("/tmp/vehicle_data.mat");
  mfw.write(Vc,"Vc");
  mfw.write(PC,"PC");
  mfw.write(std_devs_,"s");
#endif
  
  params_.set_size(std_devs_.size());
  params_.fill(0.0);
  init_part_idx_offsets();
}


//: Destructor
modrec_pca_vehicle::~modrec_pca_vehicle()
{
}


//: Initialize the PCA data (assuming mesh and parts are already set)
//  use this to add PCA data after mesh and parts are loaded from files
void modrec_pca_vehicle::init(const vnl_vector<double>& mean,
                              const vnl_vector<double>& std_devs,
                              const vnl_matrix<double>& pc)
{
  assert(mean.size() == pc.columns());
  assert(std_devs.size() == pc.rows());
  
  unsigned int num_mesh_pts = this->num_verts()*3;
  if(num_mesh_pts == mean.size() && num_mesh_pts == pc.columns()){
    // only information on mesh variation is provided
    imesh_pca_mesh::init(mean,std_devs,pc);
    return;
  }
  
  // extract the mesh data
  assert(num_mesh_pts < mean.size());
  vnl_vector<double> mesh_mean(mean.data_block(), num_mesh_pts);
  vnl_matrix<double> mesh_pc = pc.extract(std_devs.size(),num_mesh_pts);
  imesh_pca_mesh::init(mesh_mean,std_devs,mesh_pc);
  
  // extract the parts data
  unsigned int pidx = num_mesh_pts;
  // set the mean parts with the mean data
  for(part_map::iterator itr=mean_parts_.begin(); itr!=mean_parts_.end(); ++itr)
  {
    vcl_vector<vgl_point_2d<double> >& pts = itr->second[0];
    for(unsigned int j=0; j<pts.size(); ++j)
    {
      assert(pidx+1 < mean.size());
      pts[j].x() = mean[pidx++];
      pts[j].y() = mean[pidx++];
    }
  }
  parts_pc_ = pc.extract(std_devs.size(),pidx-num_mesh_pts,0,num_mesh_pts);
  //set_params(params_);
}


//: Project mesh vertices and parts into the joint PCA parameter space
vnl_vector<double> 
modrec_pca_vehicle::project(const imesh_vertex_array_base& verts, 
                            const part_map& parts) const
{

  vnl_vector<double> vals(parts_pc_.columns());
  unsigned int pidx = 0;
  // compute the part deviation vector
  for(part_map::const_iterator itr=mean_parts_.begin(); itr!=mean_parts_.end(); ++itr)
  {
    const vgl_polygon<double>& mean_poly = itr->second;
    // find the matching part
    part_map::const_iterator pitr = parts.find(itr->first);
    if(pitr == parts.end()) // missing part, assume mean (zero deviation)
    {
      for(unsigned int j=0; j<mean_poly[0].size(); ++j)
      {
        vals[pidx+2*j] = 0;
        vals[pidx+2*j+1] = 0;
      }
    }
    else // part is found so use it
    {
      const vgl_polygon<double>& poly = pitr->second;
      for(unsigned int j=0; j<mean_poly[0].size(); ++j)
      {
        vals[pidx+2*j] = poly[0][j].x() - mean_poly[0][j].x();
        vals[pidx+2*j+1] = poly[0][j].y() - mean_poly[0][j].y();
      }
    }
    pidx += 2*mean_poly[0].size();
  }
  
  vnl_vector<double> p = parts_pc_*vals;
  for(unsigned int i=0; i<p.size(); ++i)
    p[i] /= std_devs_[i];
  
  return imesh_pca_mesh::project(verts) + p;
}


//: Set the parts
void modrec_pca_vehicle::set_parts(const part_map& parts)
{
  parts_ = parts;
  if(mean_parts_.empty())
    mean_parts_ = parts;
  init_part_idx_offsets();

  // regenerate 3d parts
  project_tex_to_bary();
  project_bary_to_3d();
}


//: Set the pca parameters
void modrec_pca_vehicle::set_params(const vnl_vector<double>& p)
{
  // set shape params in the base class
  imesh_pca_mesh::set_params(p);
  
  if(parts_.empty())
    return;
  
  vnl_vector<double> sp(p.size(),0.0);
  for (unsigned int i=0; i<p.size(); ++i)
    sp[i] = std_devs_[i]*p[i];
  
  unsigned int pidx = 0;
  part_map::const_iterator mitr=mean_parts_.begin();
  for(part_map::iterator itr=parts_.begin(); itr!=parts_.end(); ++itr, ++mitr){
    vcl_vector<vgl_point_2d<double> >& part = itr->second[0];
    const vcl_vector<vgl_point_2d<double> >& mpart = mitr->second[0];
    for(unsigned int i=0; i<part.size(); ++i, pidx+=2){
      part[i] = mpart[i]; // copy from the mean
      for(unsigned j=0; j<sp.size(); ++j){
        part[i].x() += parts_pc_(j, pidx)*sp[j];
        part[i].y() += parts_pc_(j, pidx+1)*sp[j];
      }
    }
  }

  // regenerate 3d parts
  project_tex_to_bary();
  project_bary_to_3d();
}

//: Set an individual pca parameter
// This is done by incremental difference, errors may accumulate
// over many calls.
void modrec_pca_vehicle::set_param(unsigned int idx, double param)
{
  double diff = std_devs_[idx]*(param - params_[idx]);
  
  // set shape params in the base class
  imesh_pca_mesh::set_param(idx,param);
  
  if(parts_.empty())
    return;
    
  unsigned int pidx = 0;
  for(part_map::iterator itr=parts_.begin(); itr!=parts_.end(); ++itr){
    vcl_vector<vgl_point_2d<double> >& part = itr->second[0];
    for(unsigned int i=0; i<part.size(); ++i){
      part[i].x() += parts_pc_(idx, pidx++)*diff;
      part[i].y() += parts_pc_(idx, pidx++)*diff;
    }
  }

  // regenerate 3d parts
  project_tex_to_bary();
  project_bary_to_3d();
}


//: Look up part number \a p and index \a i from the PC index \a pc
void modrec_pca_vehicle::pc_to_part_idx(unsigned int pc, 
                                        unsigned int& p, unsigned int& i) const
{
  unsigned int j=1;
  for(; j<part_idx_offsets_.size(); ++j)
  {
    if(part_idx_offsets_[j] > pc){
      p = j-1;
      i = pc - part_idx_offsets_[p];
      return;
    }
  }
  p = j-1;
  i = pc - part_idx_offsets_[p];
}

//: Look up part PC index \a pc from part number \a p and index \a i  
void modrec_pca_vehicle::part_to_pc_idx(unsigned int p, unsigned int i, 
                                        unsigned int& pc) const
{
  pc = part_idx_offsets_[p]+i;
}


//: Return the number of vertices in part \a p
unsigned int modrec_pca_vehicle::part_size(unsigned int p) const
{
  if(p+1<part_idx_offsets_.size())
    return part_idx_offsets_[p+1]-part_idx_offsets_[p];
  
  return 0;
}


//: Project texture coordinates into barycentric coordinates
void modrec_pca_vehicle::project_tex_to_bary()
{
  assert(this->has_tex_coords());
  //imesh_mesh mesh = imesh_submesh_from_faces(*this,this->faces().group_face_set("body"));
  //imesh_triangulate(mesh);
  //mesh.build_edge_graph();

  unsigned int base_idx = 0;
  parts_uv_.clear();
  for(part_map::const_iterator itr=parts_.begin(); itr!=parts_.end(); ++itr)
  {
    const vcl_vector<vgl_point_2d<double> >& poly = itr->second[0];
    vcl_vector<vgl_point_2d<double> > pts_uv;
    vcl_vector<unsigned long> idx;
    vcl_vector<int> map_back;
    imesh_project_texture_to_barycentric(*this,poly,pts_uv,idx,map_back);
    
    const unsigned int num_s = map_back.size(); 

    parts_uv_.push_back(vcl_vector<uv_point>());
    int first_pt = pts_uv.size();
    int first_idx = -1;
    for(unsigned int i=0; first_pt>0 && i<map_back.size(); ++i)
      if(map_back[i] < first_pt) {
        first_pt = map_back[i];
        first_idx = i;
      }
    if(first_idx != 0)
      first_idx = (first_idx+num_s-1)%num_s;
    int next_idx = (first_idx+1)%num_s;
    vgl_vector_2d<double> v = poly[next_idx] - poly[first_idx];
    double mag = v.sqr_length();
    for(unsigned int i=0; i<pts_uv.size(); ++i)
    {
      if(map_back[next_idx] == i){
        first_idx = next_idx;
        next_idx = (next_idx+1)%num_s;
        v = poly[next_idx] - poly[first_idx];
        mag = v.sqr_length();
        parts_uv_.back().push_back(uv_point(pts_uv[i],idx[i],
                                            base_idx+first_idx,
                                            base_idx+next_idx,0.0));
      }
      else{
        unsigned int fidx = this->half_edges()[idx[i]>>2].face_index();
        vgl_point_2d<double> pt = 
            imesh_project_barycentric_to_texture(*this,pts_uv[i],fidx);
        double t = dot_product(pt - poly[first_idx],v)/mag;
        parts_uv_.back().push_back(uv_point(pts_uv[i],idx[i],
                                            base_idx+first_idx,
                                            base_idx+next_idx,t));
      }
    }
    base_idx += poly.size();
  }
}


//: Project barycentric coordinates into 3D coordinates
void modrec_pca_vehicle::project_bary_to_3d()
{
  assert(this->has_tex_coords());

  parts_3d_.clear();
  for(unsigned int i=0; i<parts_uv_.size(); ++i)
  {
    const vcl_vector<uv_point>& uv_points = parts_uv_[i];
    parts_3d_.push_back( vcl_vector<vgl_point_3d<double> >() );
    for(unsigned int j=0; j<uv_points.size(); ++j)
    {
      const uv_point& pt = uv_points[j];
      unsigned int fidx = this->half_edges()[pt.mesh_index>>2].face_index();
      vgl_point_3d<double> p3 = imesh_project_barycentric_to_mesh(*this,pt.uv,fidx);
      parts_3d_.back().push_back(p3);
    }
  }
}


//: Compute the part index offsets for the part_map
void modrec_pca_vehicle::init_part_idx_offsets()
{
  part_idx_offsets_.resize(parts_.size()+1);
  part_idx_offsets_[0] = 0;
  unsigned int i=1;
  for(part_map::const_iterator itr=mean_parts_.begin(); itr!=mean_parts_.end(); ++itr, ++i)
  {
    part_idx_offsets_[i] = part_idx_offsets_[i-1] + itr->second[0].size();
  }
}



//=========================================================
// External functions


//: Write the mean mesh, mean parts, and PCA file
void modrec_write_pca_vehicle(const vcl_string& mesh_file,
                              const vcl_string& parts_file,
                              const vcl_string& pca_file,
                              const modrec_pca_vehicle& pmesh)
{
  const vnl_vector<double>& std_dev = pmesh.std_devs();
  const vnl_matrix<double>& mesh_pc = pmesh.principal_comps();
  const vnl_matrix<double>& parts_pc = pmesh.parts_principal_comps();
  const imesh_vertex_array<3>& mverts = pmesh.mean_vertices<3>();
  const modrec_pca_vehicle::part_map& mparts = pmesh.mean_parts();
  
  const unsigned int num_mesh_pts = mesh_pc.columns();
  const unsigned int num_parts_pts = parts_pc.columns();
  vnl_vector<double> mean(num_mesh_pts+num_parts_pts);
  
  for(unsigned int i=0; i<num_mesh_pts; ++i)
    mean[i] = mverts[i/3][i%3];

  unsigned int pidx=num_mesh_pts;
  for(modrec_pca_vehicle::part_map::const_iterator pitr=mparts.begin(); 
      pitr!=mparts.end(); ++pitr){
    const vcl_vector<vgl_point_2d<double> >& pts = pitr->second[0];
    for(unsigned int i=0; i<pts.size(); ++i){
      mean[pidx++] = pts[i].x();
      mean[pidx++] = pts[i].y();
    }
  }

  vnl_matrix<double> pc(mesh_pc.rows(),num_mesh_pts+num_parts_pts);
  pc.update(mesh_pc);
  pc.update(parts_pc,0,num_mesh_pts);
  
  vcl_auto_ptr<imesh_vertex_array_base> verts(mverts.clone());
  vcl_auto_ptr<imesh_face_array_base> faces(pmesh.faces().clone());
  imesh_mesh mean_mesh(verts,faces);
  
  imesh_write_pca(pca_file,mean,std_dev,pc);
  imesh_write_obj(mesh_file,mean_mesh);
  modrec_write_vehicle_parts(parts_file,mparts);
}


