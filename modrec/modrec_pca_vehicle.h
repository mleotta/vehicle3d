// This is mleotta/gui/pca_vehicle/modrec_pca_vehicle.h
#ifndef modrec_pca_vehicle_h_
#define modrec_pca_vehicle_h_
//=========================================================================
//:
// \file
// \brief  PCA vehicle model with parts
// \author Matt Leotta (mleotta)
// \data 09/09/2008
//
// \verbatim
//  Modifications
//   09/09/2008 - File created. (mleotta)
// \endverbatim
//=========================================================================


#include <imesh/algo/imesh_pca.h>
#include <vgl/vgl_polygon.h>


//: PCA vehicle model with parts
class modrec_pca_vehicle : public imesh_pca_mesh
{
public:
  typedef vcl_map<vcl_string, vgl_polygon<double> > part_map;

  //: Constructor
  modrec_pca_vehicle() {}

  //: Constructor - compute the pca from a joint collection of meshes and parts
  modrec_pca_vehicle(const vcl_vector<imesh_mesh>& meshes,
                     const vcl_vector<part_map>& part_groups);

  //: Constructor 
  modrec_pca_vehicle(const imesh_pca_mesh& pca_mesh,
                     const part_map& parts);
  
  //: Constructor from a mesh, parts, mean, standard deviations, and PC matrix
  modrec_pca_vehicle(const imesh_mesh& mesh,
                     const part_map& parts,
                     const vnl_vector<double>& mean,
                     const vnl_vector<double>& std_devs,
                     const vnl_matrix<double>& pc);

  //: Destructor
  ~modrec_pca_vehicle();
  
  //: A structure containing information about each barycentric point
  struct uv_point
  {
    //: Constructor
    uv_point(const vgl_point_2d<double>& p, unsigned long mi, 
             unsigned int e1, unsigned int e2, double tp) 
    : uv(p), mesh_index(mi), end_point1(e1), end_point2(e2), t(tp) {}

    //: The uv coordinates within a triangle
    vgl_point_2d<double> uv;
    
    //: Access the coded half edge indices of the part boundary points.
    //  Divide by 4 to get the half edge index adjacent to the barycentric face
    //  the last two bits indicate whether a point is on the face(0), edge(1), or vertex(2)
    unsigned long mesh_index;
    
    //: The index of the first line end point
    unsigned int end_point1;
    //: The index of the last line end point
    unsigned int end_point2;
    
    //: The interpolation [0,1) between the endpoints in texture space
    double t;
  };
  
  //: Initialize the PCA data (assuming mesh and parts are already set)
  //  use this to add PCA data after mesh and parts are loaded from files
  void init(const vnl_vector<double>& mean,
            const vnl_vector<double>& std_devs,
            const vnl_matrix<double>& pc);

  //: Set the pca parameters
  void set_params(const vnl_vector<double>& p);

  //: Set an individual pca parameter
  // This is done by incremental difference, errors may accumulate
  // over many calls.
  void set_param(unsigned int i, double param);

  //: Set the parts
  void set_parts(const part_map& parts);
  
  //: Project mesh vertices and parts into the joint PCA parameter space
  vnl_vector<double> project(const imesh_vertex_array_base& verts, 
                             const part_map& parts) const;

  //: Access the parts map
  const part_map& parts() const { return parts_; }
  
  //: Access the mean parts map
  const part_map& mean_parts() const { return mean_parts_; }
  
  //: Access the principal components of the parts
  const vnl_matrix<double>& parts_principal_comps() const {return parts_pc_; }

  //: Access the 3D surface projection of the parts
  const vcl_vector<vcl_vector<vgl_point_3d<double> > >&
  parts_3d() const { return parts_3d_; }

  //: Access the structures containing barycentric coordinates
  const vcl_vector<vcl_vector<uv_point> >&
  parts_bary() const { return parts_uv_; }

  //: Look up part number \a p and index \a i from the PC index \a pc
  void pc_to_part_idx(unsigned int pc, unsigned int& p, unsigned int& i) const;
  
  //: Look up part PC index \a pc from part number \a p and index \a i  
  void part_to_pc_idx(unsigned int p, unsigned int i, unsigned int& pc) const;

  //: Return the number of vertices in part \a p
  unsigned int part_size(unsigned int p) const;

private:
  //: Project texture coordinates into barycentric coordinates
  void project_tex_to_bary();
  //: Project barycentric coordinates into 3D coordinates
  void project_bary_to_3d();
  
  //: Compute the part index offsets for the part_map
  void init_part_idx_offsets();

  // Member variables -------------------------------------

  part_map parts_;

  part_map mean_parts_;
  
  //: used to covert PC index to (part,point) indices
  vcl_vector<unsigned int> part_idx_offsets_;
  
  //: parts principal components
  vnl_matrix<double> parts_pc_;
  
  //: UV (barycentric) coordinates of all points 
  vcl_vector<vcl_vector<uv_point> > parts_uv_;
  
  //: the UV points mapped into 3-d
  vcl_vector<vcl_vector<vgl_point_3d<double> > > parts_3d_;
};



//=========================================================
// External functions


//: Write the mean mesh, mean parts, and PCA file
void modrec_write_pca_vehicle(const vcl_string& mesh_file,
                              const vcl_string& parts_file,
                              const vcl_string& pca_file,
                              const modrec_pca_vehicle& pmesh);


#endif // modrec_pca_vehicle_h_
