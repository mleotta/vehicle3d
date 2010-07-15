// This is mleotta/cmd/mesh/fit_mesh_body.cxx


#include <vcl_iostream.h>
#include <vcl_limits.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <vul/vul_sprintf.h>
#include <imesh/imesh_mesh.h>
#include <imesh/imesh_fileio.h>
#include <imesh/imesh_operations.h>
#include <imesh/algo/imesh_detect.h>
#include <imesh/algo/imesh_transform.h>
#include <imesh/algo/imesh_intersect.h>
#include <imesh/algo/imesh_imls_surface.h>
#include <imesh/algo/imesh_operations.h>
#include <modrec/modrec_vehicle_mesh.h>


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
                             const imesh_half_edge_set& half_edges,
                             const vcl_vector<vgl_point_3d<double> >& orig_verts,
                             unsigned int num_edge_samples = 3,
                             double orig_weight = 1.0)
    : vnl_least_squares_function(3*half_edges.num_verts(),
                                 2*half_edges.num_verts()
                                    + half_edges.size()/2*num_edge_samples),
      surf_(surf),
      half_edges_(half_edges),
      orig_verts_(orig_verts),
      num_edge_samples_(num_edge_samples),
      orig_weight_(orig_weight)
      {}

    void f (vnl_vector< double > const &x, vnl_vector< double > &fx)
    {
      const unsigned int num_verts = x.size()/3;
      for(unsigned int i=0; i<num_verts; ++i){
        vgl_point_3d<double> p(x[3*i],x[3*i+1],x[3*i+2]);
        fx[2*i] = surf_(p);
        fx[2*i+1] = orig_weight_*(p-orig_verts_[i]).sqr_length();
      }
      const unsigned int num_edges = half_edges_.size()/2;
      for(unsigned int i=0; i<num_edges; ++i){
        unsigned int i1 = half_edges_[2*i].vert_index();
        unsigned int i2 = half_edges_[2*i+1].vert_index();
        const vnl_vector_ref<double>
            p1(3, const_cast<double*>(x.data_block())+3*i1),
            p2(3, const_cast<double*>(x.data_block())+3*i2);
        for(unsigned int j=0; j<num_edge_samples_; ++j){
          double u = double(j+1)/(num_edge_samples_+1);
          vnl_double_3 p = u*p1 + (1.0-u)*p2;
          fx[2*num_verts+num_edge_samples_*i+j] =
              surf_(vgl_point_3d<double>(p[0],p[1],p[2]));
        }
      }
    }

    void gradf(vnl_vector< double > const &x, vnl_matrix< double > &jacobian)
    {
      jacobian.fill(0.0);
      vgl_vector_3d<double> dp;
      const unsigned int num_verts = x.size()/3;
      for(unsigned int i=0; i<num_verts; ++i){
        vgl_point_3d<double> p(x[3*i],x[3*i+1],x[3*i+2]);
        surf_.deriv(p,dp);
        double *data = jacobian[2*i]+3*i;
        data[0] = dp.x();
        data[1] = dp.y();
        data[2] = dp.z();
        data = jacobian[2*i+1]+3*i;
        data[0] = orig_weight_*2*(p.x() - orig_verts_[i].x());
        data[1] = orig_weight_*2*(p.y() - orig_verts_[i].y());
        data[2] = orig_weight_*2*(p.z() - orig_verts_[i].z());
      }
      
      const unsigned int num_edges = half_edges_.size()/2;
      for(unsigned int i=0; i<num_edges; ++i){
        unsigned int i1 = half_edges_[2*i].vert_index();
        unsigned int i2 = half_edges_[2*i+1].vert_index();
        const vnl_vector_ref<double>
            p1(3, const_cast<double*>(x.data_block())+3*i1),
            p2(3, const_cast<double*>(x.data_block())+3*i2);
        for(unsigned int j=0; j<num_edge_samples_; ++j){
          double *data = jacobian[2*num_verts + num_edge_samples_*i +j];
          double *d1 = data+3*i1;
          double *d2 = data+3*i2;
          double u = double(j+1)/(num_edge_samples_+1);
          double v = 1.0 - u;
          vnl_double_3 p = u*p1 + v*p2;
          surf_.deriv(vgl_point_3d<double>(p[0],p[1],p[2]),dp);
          d1[0] = u*dp.x();   d2[0] = v*dp.x();
          d1[1] = u*dp.y();   d2[1] = v*dp.y();
          d1[2] = u*dp.z();   d2[2] = v*dp.z();
        }
      }
    }

  private:
    imesh_half_edge_set half_edges_;
    vcl_vector<vgl_point_3d<double> > orig_verts_;
    imesh_imls_surface surf_;
    unsigned int num_edge_samples_;
    double orig_weight_;
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


void snap_faces_to_surface(const imesh_imls_surface& f, imesh_mesh& mesh,
                           const vcl_set<unsigned int>& faces)
{
  if(!mesh.faces().has_normals())
    mesh.compute_face_normals();
  typedef vcl_map<unsigned int, vgl_plane_3d<double> > plane_map;
  plane_map planes;
  for(vcl_set<unsigned int>::const_iterator fi = faces.begin();
      fi != faces.end(); ++fi){
    const unsigned int num_verts = mesh.faces().num_verts(*fi);
    vcl_vector<vgl_point_3d<double> > pts;
    for(unsigned int j=0; j<num_verts; ++j)
    {
      unsigned int v = mesh.faces()(*fi,j);
      pts.push_back(mesh.vertices<3>()[v]);
    }
    vgl_point_3d<double> c = centre(pts);
    vgl_point_3d<double> last_c(c);

    if(snap_to_surface(f,c) && (last_c-c).length() < 0.1)
    {
      vgl_vector_3d<double> dc;
      f.deriv(c,dc);
      normalize(dc);
      vcl_cout << "dot_prod norm = "<<dot_product(dc,mesh.faces().normal(*fi))<<vcl_endl;
      if(dot_product(dc,mesh.faces().normal(*fi)) > 0.98)
        planes[*fi] = vgl_plane_3d<double>(dc,c);
      else
        planes[*fi] = vgl_plane_3d<double>(mesh.faces().normal(*fi),last_c);
    }else
      planes[*fi] = vgl_plane_3d<double>(mesh.faces().normal(*fi),last_c);
  }

  if(!mesh.has_half_edges())
    mesh.build_edge_graph();
  const imesh_half_edge_set& half_edges = mesh.half_edges();
  const unsigned int num_verts = mesh.num_verts();
  for(unsigned int i=0; i<num_verts; ++i)
  {
    vcl_vector<vgl_plane_3d<double> > local_planes;
    const imesh_vertex<3>& p0 = mesh.vertices<3>()[i];
    vgl_vector_3d<double> v0(p0[0],p0[1],p0[2]);
    typedef imesh_half_edge_set::v_const_iterator vitr;
    vitr end = half_edges.vertex_begin(i), v = end;
    do{
      ++v;
      plane_map::const_iterator pi = planes.find(v->face_index());
      if(pi != planes.end())
      {
        vgl_plane_3d<double> pl = pi->second;
        double d = dot_product(v0,pl.normal());
        pl.set(pl.a(),pl.b(),pl.c(),pl.d()+d);
        local_planes.push_back(pl);
      }
    }while(v!=end);

    if(local_planes.empty())
      continue;

    vnl_matrix<double> A(local_planes.size(), 3);
    vnl_vector<double> b(local_planes.size());
    for(unsigned int j=0; j<local_planes.size(); ++j)
    {
      A(j,0) = local_planes[j].nx();
      A(j,1) = local_planes[j].ny();
      A(j,2) = local_planes[j].nz();
      b(j)   = -local_planes[j].d();
    }

    vnl_svd<double> svd(A);
    svd.zero_out_relative(1e-1);
    vgl_point_3d<double> p1(svd.solve(b).begin());
    p1 += v0;
    mesh.vertices<3>()[i] = imesh_vertex<3>(p1);
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
    for(; i<10 && dir < 0.0; ++i){
      vcl_cout << "backward point " << dir << vcl_endl;
      p += -dir * n;
      val = f.deriv(p,dp);
      dir = dot_product(dp,n);
      if(dir >= 0.0)
        vcl_cout << "-----------------"<<vcl_endl;
    }
    if(i == 2)
      continue;
    //double shift = ((last_p-p)-dot_product((last_p-p),n)*n).length();
    if(snap_to_surface(f,p) && (last_p-p).length() < 0.5)
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


void snap_to_surface_lm(const imesh_imls_surface& f, imesh_mesh& mesh)
{
  if(!mesh.has_half_edges())
    mesh.build_edge_graph();
  vcl_vector<vgl_point_3d<double> > orig_verts;
  for(unsigned int i=0; i<mesh.num_verts(); ++i){
    orig_verts.push_back(mesh.vertices<3>()[i]);
  }
  mesh_to_surface_lsq_func lsq_func(f,mesh.half_edges(),orig_verts,3,4.0);
  vnl_levenberg_marquardt lm(lsq_func);

  vnl_vector<double> x(3*mesh.num_verts());
  for(unsigned int i=0; i<mesh.num_verts(); ++i){
    x[3*i] = mesh.vertices<3>()[i][0];
    x[3*i+1] = mesh.vertices<3>()[i][1];
    x[3*i+2] = mesh.vertices<3>()[i][2];
  }

  lm.set_trace(true);
  lm.minimize(x);
  lm.diagnose_outcome(vcl_cout);

  for(unsigned int i=0; i<mesh.num_verts(); ++i){
    mesh.vertices<3>()[i][0] = x[3*i];
    mesh.vertices<3>()[i][1] = x[3*i+1];
    mesh.vertices<3>()[i][2] = x[3*i+2];
  }
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
  vul_arg<vcl_string>  a_body_file("-b", "input mesh body file", "");
  vul_arg<vcl_string>  a_out_file("-o", "output mesh file", "");
  vul_arg<int>         a_num_subdiv("-subdiv", "number of subdivisions", 0);
  vul_arg_parse(argc, argv);

  if(!a_out_file.set()){
    vcl_cerr << "output file required" << vcl_endl;
    return -1;
  }


#if 0
  vcl_map<vcl_string,double> params;
  modrec_read_vehicle_params(a_in_file(),params);

  double r1 = params["wheel_rad"];
  double r2 = r1+params["tire_thick"];
  double ww = params["wheel_width"];
  double so = params["susp_offset"];

  vcl_auto_ptr<imesh_vertex_array_base> verts(modrec_generate_vehicle_body_verts(params));
  vcl_auto_ptr<imesh_face_array_base> faces(modrec_generate_vehicle_body_faces());

  imesh_mesh model_mesh(verts,faces);
  imesh_transform_inplace(model_mesh,vgl_vector_3d<double>(0,0,r2+so));
#endif

#if 1
  imesh_mesh model_mesh;
  imesh_read_obj(a_in_file(),model_mesh);

  model_mesh.compute_vertex_normals_from_faces();

  vcl_set<unsigned int> body_faces = model_mesh.faces().group_face_set("body");

  vcl_set<unsigned int> uc_faces = model_mesh.faces().group_face_set("undercarriage");
  body_faces.insert(uc_faces.begin(), uc_faces.end());

  vcl_set<unsigned int> body_verts_contained, body_verts_boundary;
  get_verts(model_mesh, body_faces, body_verts_contained, body_verts_boundary);
#endif

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
  imesh_imls_surface body_imp(ext_mesh,epsilon,0.01,false);

#if 0

  vgl_box_3d<double> bbox = body_imp.bounding_box();
  bbox.scale_about_centroid(1.5);
  double msize = vcl_max(bbox.width(),bbox.depth());
  unsigned res = 400;
  unsigned ni = res*bbox.width()/msize;
  unsigned nj = res*bbox.depth()/msize;

  double scale = msize/res;
  double ishift = bbox.min_x();
  double jshift = bbox.min_z();


  vnl_matrix<double> M(nj,ni);
  for(int i=0; i<ni; ++i){
    vcl_cout << "row "<< i<<vcl_endl;
    for(int j=0; j<nj; ++j){
      M(j,i) = body_imp(scale*i + ishift, 0.0, scale*j+jshift);
    }
  }
  vnl_matlab_filewrite mfw("slice.mat");
  mfw.write(M,"M");

  return 0;
#endif
  //imesh_mesh model_mesh_sub(imesh_submesh_from_faces(model_mesh, body_faces));
  //snap_to_surface(body_imp, model_mesh_sub);
  //imesh_quad_subdivide(model_mesh_sub);
  //snap_to_surface_lm(body_imp, model_mesh_sub);
  //vcl_string fname = vul_sprintf(a_out_file().c_str(),1);
  //imesh_write_obj(fname, model_mesh_sub);
  //return 0;

  //snap_to_surface(body_imp, model_mesh, body_verts_contained);
  //snap_faces_to_surface(body_imp,model_mesh,body_faces);
  //imesh_write_obj("test_dual.obj", model_mesh);

  //return 0;


  snap_to_surface(body_imp, model_mesh, body_verts_contained);

  vcl_string fname = vul_sprintf(a_out_file().c_str(),1);
  imesh_write_obj(fname, model_mesh);

  for(unsigned int i=0; i<a_num_subdiv(); ++i){
    epsilon *= 0.5;
    //body_imp.set_epsilon(epsilon);
    body_faces = model_mesh.faces().group_face_set("body");
    imesh_quad_subdivide(model_mesh,body_faces);
    model_mesh.compute_vertex_normals_from_faces();

    body_faces = model_mesh.faces().group_face_set("body");
    uc_faces = model_mesh.faces().group_face_set("undercarriage");
    body_faces.insert(uc_faces.begin(), uc_faces.end());
    get_verts(model_mesh, body_faces, body_verts_contained, body_verts_boundary);

    snap_to_surface(body_imp, model_mesh, body_verts_contained);

    vcl_string fname = vul_sprintf(a_out_file().c_str(),i+2);
    imesh_write_obj(fname, model_mesh);
  }



#if 0
  body_mesh.set_faces(vcl_auto_ptr<imesh_face_array_base>(imesh_triangulate(body_mesh.faces())));
  body_mesh.compute_face_normals(false);

  partial_snap(model_mesh, body_mesh, 1.0, body_verts_contained);
  //closest_point(model_mesh, body_mesh, body_verts_contained);
  closest_point(model_mesh, body_mesh, body_verts_boundary);


  for(unsigned int i=0; i<a_num_subdiv(); ++i){
    imesh_quad_subdivide(model_mesh,model_mesh.faces().group_face_set("body"));
    model_mesh.compute_face_normals();
    model_mesh.compute_vertex_normals_from_faces();

    body_faces = model_mesh.faces().group_face_set("body");
    vcl_set<unsigned int> new_body_verts_contained, new_body_verts_boundary;
    get_verts(model_mesh, body_faces, new_body_verts_contained, new_body_verts_boundary);
    //set_difference()

    partial_snap(model_mesh, body_mesh, 1.0, new_body_verts_contained);
    closest_point(model_mesh, body_mesh, new_body_verts_boundary);
  }

#endif
  //imesh_write_obj(a_out_file(), model_mesh_sub);

  return 0;

}
