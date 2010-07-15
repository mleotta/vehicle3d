// This is mleotta/cmd/mesh/pca_fit_mesh_body.cxx


#include <vcl_iostream.h>
#include <vcl_limits.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <vul/vul_sprintf.h>
#include <imesh/imesh_mesh.h>
#include <imesh/imesh_fileio.h>
#include <imesh/imesh_operations.h>
#include <imesh/algo/imesh_pca.h>
#include <imesh/algo/imesh_detect.h>
#include <imesh/algo/imesh_transform.h>
#include <imesh/algo/imesh_intersect.h>
#include <imesh/algo/imesh_imls_surface.h>
#include <imesh/algo/imesh_operations.h>


#include <vcl_algorithm.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vnl/vnl_matrix.h>

#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_plane_3d.h>
#include <vnl/algo/vnl_svd.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>


class mesh_to_surface_lsq_func : public vnl_least_squares_function
{
  public:
    mesh_to_surface_lsq_func(const imesh_imls_surface& surf,
                             const imesh_pca_mesh& pca_mesh,
                             const vcl_set<unsigned int>& vert_set,
                                   unsigned int num_comps = 10)
    : vnl_least_squares_function(num_comps,vert_set.size()),
      surf_(surf),
      pca_mesh_(pca_mesh),
      vert_set_(vert_set)
      {}

    void f (vnl_vector< double > const &x, vnl_vector< double > &fx)
    {
      pca_mesh_.set_params(x);
      const unsigned int num_verts = vert_set_.size();
      const imesh_vertex_array<3>& verts = pca_mesh_.vertices<3>();
      vcl_set<unsigned int>::const_iterator vi = vert_set_.begin();
      for(unsigned int i=0; vi != vert_set_.end(); ++vi,++i)
      {
        vgl_vector_3d<double> dp;
        fx[i] = surf_.deriv(verts[*vi],dp);
        fx[i] /= dp.length();
      }
    }

    void gradf(vnl_vector< double > const &x, vnl_matrix< double > &jacobian)
    {
      pca_mesh_.set_params(x);
      const unsigned int num_verts = vert_set_.size();
      const imesh_vertex_array<3>& verts = pca_mesh_.vertices<3>();
      vcl_set<unsigned int>::const_iterator vi = vert_set_.begin();
      for(unsigned int i=0; vi != vert_set_.end(); ++vi,++i)
      {
        vgl_vector_3d<double> dp;
        double val = surf_.deriv(verts[*vi],dp);
        double len2 = dp.sqr_length();
        double len = vcl_sqrt(len2);
        dp *= (len/len2 - val/(len*len2));
        for(unsigned int j=0; j<jacobian.columns(); ++j)
        {
          const double* pc = pca_mesh_.principal_comps()[j]+3*i;
          jacobian(i,j) = pc[0]*dp.x() + pc[1]*dp.y() + pc[2]*dp.z();
        }
      }
    }

  private:
    imesh_imls_surface surf_;
    imesh_pca_mesh pca_mesh_;
    vcl_set<unsigned int> vert_set_;
};

const double pi = 3.14159265358979;



void snap_to_surface_assign_normals(const imesh_imls_surface& f, imesh_mesh& mesh)
{
  for(unsigned int i=0; i<mesh.num_verts(); ++i){
    vgl_point_3d<double> p(mesh.vertices<3>()[i]);
    vgl_point_3d<double> last_p(p);
    if(snap_to_surface(f,p) && (last_p-p).length() < 0.5){
      vgl_vector_3d<double> dp;
      f.deriv(p,dp);
      mesh.vertices<3>()[i] = p;
      mesh.vertices().normal(i) = normalized(dp);
    }
  }
}




void snap_to_surface(const imesh_imls_surface& f, imesh_mesh& mesh,
                     const vcl_set<unsigned int>& verts)
{
  for(vcl_set<unsigned int>::const_iterator vi = verts.begin();
      vi != verts.end(); ++vi){
    vgl_point_3d<double> p(mesh.vertices<3>()[*vi]);
    vgl_point_3d<double> last_p(p);
    vgl_vector_3d<double> dp;
    double val = f.deriv(p,dp);
    const vgl_vector_3d<double>& n = mesh.vertices().normal(*vi);
    double dir = dot_product(dp,n);
    unsigned int i=0;
    for(; i<2 && dir < 0.0; ++i){
      vcl_cout << "backward point " << dir << vcl_endl;
      p += -dir * n;
      val = f.deriv(p,dp);
      dir = dot_product(dp,n);
      if(dir >= 0.0)
        vcl_cout << "-----------------"<<vcl_endl;
    }
    if(i == 2)
      continue;
    double shift = (last_p-p).length();//((last_p-p)-dot_product((last_p-p),n)*n).length();
    if(snap_to_surface(f,p) && shift < 0.2)
      mesh.vertices<3>()[*vi] = p;
  }
}

void snap_to_surface(const imesh_imls_surface& f, imesh_mesh& mesh)
{
  for(unsigned int i=0; i<mesh.num_verts(); ++i){
    vgl_point_3d<double> p(mesh.vertices<3>()[i]);
    snap_to_surface(f,p);
    mesh.vertices<3>()[i] = p;
  }
}


void snap_to_surface_lm(const imesh_imls_surface& f,
                              imesh_pca_mesh& pca_mesh,
                        const vcl_set<unsigned int>& vert_set)
{
  //if(!mesh.has_half_edges())
  //  mesh.build_edge_graph();

  mesh_to_surface_lsq_func lsq_func(f,pca_mesh,vert_set,10);
  vnl_levenberg_marquardt lm(lsq_func);

  vnl_vector<double> x = pca_mesh.params().extract(10);

  lm.set_trace(true);
  lm.minimize_without_gradient(x);
  lm.diagnose_outcome(vcl_cout);

  pca_mesh.set_params(x);
}



void get_verts(const imesh_mesh& mesh, const vcl_set<unsigned int>& faces,
               vcl_set<unsigned int>& in_verts, vcl_set<unsigned int>& bound_verts)
{
  in_verts.clear();
  bound_verts.clear();
  assert(mesh.has_half_edges());
  const imesh_half_edge_set& he = mesh.half_edges();
  typedef imesh_half_edge_set::v_const_iterator vert_itr;
  for(unsigned int v=0; v<mesh.num_verts(); ++v){
    vert_itr vi = he.vertex_begin(v);
    vert_itr end = vi;
    bool inside = false;
    bool outside = false;
    do{
      if(vi->is_boundary() || faces.find(vi->face_index()) == faces.end())
        outside = true;
      else
        inside = true;
      if(inside && outside)
        break;
      ++vi;
    }while(vi != end);
    if(inside){
      if(outside)
        bound_verts.insert(v);
      else
        in_verts.insert(v);
    }
  }
}


void partial_snap(imesh_mesh& model, const imesh_mesh& target,
                  double amount, unsigned int vert)
{
  imesh_vertex<3>& v = model.vertices<3>()[vert];
  vgl_point_3d<double> pt(v);
  vgl_vector_3d<double> dir = -model.vertices().normal(vert);
  normalize(dir);
  pt -= 1.0*dir;
  double dist;
  int ind = imesh_intersect_min_dist(pt,dir,target,dist);
  if(ind >= 0){
    pt += (1.0+(dist-1.0)*amount)*dir;
  }
  else{
    vcl_cout << vert<<" not intersected"<<vcl_endl;
    pt += 1.0*dir;
  }
  model.vertices<3>()[vert] = pt;
  
}


void partial_snap(imesh_mesh& model, const imesh_mesh& target, double amount)
{
  for(unsigned int i=0; i<model.num_verts(); ++i)
  {
    partial_snap(model, target, amount, i);
  }
}


void partial_snap(imesh_mesh& model, const imesh_mesh& target,
                  double amount, const vcl_set<unsigned int>& verts)
{
  for(vcl_set<unsigned int>::const_iterator vi = verts.begin();
      vi != verts.end(); ++vi){
    partial_snap(model, target, amount, *vi);
  }
}


void closest_point(imesh_mesh& model, const imesh_mesh& target,
                   unsigned int vert)
{
  imesh_vertex<3>& v = model.vertices<3>()[vert];
  vgl_point_3d<double> pt(v);
  //vgl_vector_3d<double> dir = -model.vertices().normal(vert);
  //normalize(dir);
  //pt -= 0.1*dir;
  vgl_point_3d<double> cp;
  int ind = imesh_closest_point(pt,target,cp);
  if(ind >= 0){
    pt = cp;
  }
  else{
    vcl_cout << vert<<" no closest point found"<<vcl_endl;
    //pt += 0.1*dir;
  }
  model.vertices<3>()[vert] = pt;
  
}

void closest_point(imesh_mesh& model, const imesh_mesh& target,
                   const vcl_set<unsigned int>& verts)
{
  for(vcl_set<unsigned int>::const_iterator vi = verts.begin();
      vi != verts.end(); ++vi){
    closest_point(model, target, *vi);
  }
}




// The Main Function
int main(int argc, char** argv)
{
  vul_arg<vcl_string>  a_in_file("-i", "input mesh file", "");
  vul_arg<vcl_string>  a_pca_file("-p", "input pca file", "");
  vul_arg<vcl_string>  a_body_file("-b", "input mesh body file", "");
  vul_arg<vcl_string>  a_out_file("-o", "output mesh file", "");
  vul_arg<int>         a_num_subdiv("-subdiv", "number of subdivisions", 0);
  vul_arg_parse(argc, argv);

  if(!a_out_file.set()){
    vcl_cerr << "output file required" << vcl_endl;
    return -1;
  }



  imesh_pca_mesh pca_mesh = imesh_read_pca(a_in_file(),a_pca_file());
  pca_mesh.build_edge_graph();

  vnl_vector<double> params = pca_mesh.params();

  vcl_cout << "params: "<< params << vcl_endl;

  pca_mesh.set_params(pca_mesh.params().extract(5));
  
  imesh_write_obj(a_out_file(), pca_mesh);
  return 0;

  vcl_set<unsigned int> body_faces = pca_mesh.faces().group_face_set("body");

  vcl_set<unsigned int> uc_faces = pca_mesh.faces().group_face_set("undercarriage");
  body_faces.insert(uc_faces.begin(), uc_faces.end());

  vcl_set<unsigned int> body_verts_contained, body_verts_boundary;
  get_verts(pca_mesh, body_faces, body_verts_contained, body_verts_boundary);


  imesh_mesh body_mesh;
  imesh_read(a_body_file(),body_mesh);
  imesh_triangulate(body_mesh);
  body_mesh.compute_face_normals();

  vcl_set<unsigned int> ext_faces;
  vcl_set<unsigned int> ext_backfaces;
  vcl_set<unsigned int> ext_bifaces;
  imesh_detect_exterior_faces(body_mesh, ext_faces,
                              ext_backfaces, ext_bifaces);
  vcl_cout << "exterior:\n  front: "<<ext_faces.size()
           <<"\n  back: "<<ext_backfaces.size()
           <<"\n  both: "<<ext_bifaces.size()<<vcl_endl;

  imesh_flip_faces(body_mesh, ext_backfaces);
  ext_faces.insert(ext_backfaces.begin(),ext_backfaces.end());
  ext_faces.insert(ext_bifaces.begin(),ext_bifaces.end());

  // extract only the exterior faces
  imesh_mesh ext_mesh = imesh_submesh_from_faces(body_mesh,ext_faces);
  // find new indices of bifacing faces
  unsigned int new_fi = 0;
  vcl_set<unsigned int> new_bifaces;
  for(vcl_set<unsigned int>::const_iterator fi=ext_faces.begin();
      fi!=ext_faces.end(); ++fi, ++new_fi)
  {
    if(ext_bifaces.find(*fi) != ext_bifaces.end())
      new_bifaces.insert(new_fi);
  }

  double epsilon = 0.1;
  imesh_imls_surface body_imp(ext_mesh,epsilon,0.01,true);

  snap_to_surface_lm(body_imp,pca_mesh,body_verts_contained);

  vcl_cout << "params: "<< pca_mesh.params() << vcl_endl;


  //snap_to_surface(body_imp, pca_mesh, body_verts_contained);

  vcl_string fname = vul_sprintf(a_out_file().c_str(),1);
  imesh_write_obj(fname, pca_mesh);

  return 0;

  for(unsigned int i=0; i<a_num_subdiv(); ++i){
    epsilon *= 0.5;
    //body_imp.set_epsilon(epsilon);
    body_faces = pca_mesh.faces().group_face_set("body");
    imesh_quad_subdivide(pca_mesh,body_faces);
    pca_mesh.compute_vertex_normals_from_faces();

    body_faces = pca_mesh.faces().group_face_set("body");
    uc_faces = pca_mesh.faces().group_face_set("undercarriage");
    body_faces.insert(uc_faces.begin(), uc_faces.end());
    get_verts(pca_mesh, body_faces, body_verts_contained, body_verts_boundary);

    snap_to_surface(body_imp, pca_mesh, body_verts_contained);

    vcl_string fname = vul_sprintf(a_out_file().c_str(),i+2);
    imesh_write_obj(fname, pca_mesh);
  }



#if 0
  body_mesh.set_faces(vcl_auto_ptr<imesh_face_array_base>(imesh_triangulate(body_mesh.faces())));
  body_mesh.compute_face_normals(false);

  partial_snap(pca_mesh, body_mesh, 1.0, body_verts_contained);
  //closest_point(pca_mesh, body_mesh, body_verts_contained);
  closest_point(pca_mesh, body_mesh, body_verts_boundary);


  for(unsigned int i=0; i<a_num_subdiv(); ++i){
    imesh_quad_subdivide(pca_mesh,pca_mesh.faces().group_face_set("body"));
    pca_mesh.compute_face_normals();
    pca_mesh.compute_vertex_normals_from_faces();

    body_faces = pca_mesh.faces().group_face_set("body");
    vcl_set<unsigned int> new_body_verts_contained, new_body_verts_boundary;
    get_verts(pca_mesh, body_faces, new_body_verts_contained, new_body_verts_boundary);
    //set_difference()

    partial_snap(pca_mesh, body_mesh, 1.0, new_body_verts_contained);
    closest_point(pca_mesh, body_mesh, new_body_verts_boundary);
  }

#endif
  //imesh_write_obj(a_out_file(), model_mesh_sub);

  return 0;

}
