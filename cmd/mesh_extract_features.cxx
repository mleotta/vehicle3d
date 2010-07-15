// This is mleotta/cmd/mesh/mesh_extract_features.cxx


#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vcl_map.h>
#include <vcl_cmath.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <vgl/vgl_polygon.h>
#include <vgl/vgl_distance.h>
#include <vgl/vgl_clip.h>
#include <vgl/vgl_area.h>
#include <imesh/imesh_mesh.h>
#include <imesh/imesh_fileio.h>
#include <imesh/imesh_operations.h>
#include <imesh/imesh_detection.h>
#include <imesh/algo/imesh_intersect.h>
#include <modrec/modrec_vehicle_parts.h>



void smooth_vert_normals(imesh_mesh& mesh, double sigma)
{
  const imesh_half_edge_set& half_edges = mesh.half_edges();
  imesh_vertex_array<3>& verts = mesh.vertices<3>();
  vcl_vector<vgl_vector_3d<double> > normals(verts.normals());
  for(unsigned int i=0; i<half_edges.size(); ++i){
    const imesh_half_edge& he = half_edges[i];
    unsigned int vi = he.vert_index();
    unsigned int nvi = half_edges[he.next_index()].vert_index();
    double wgt = vgl_distance(vgl_point_3d<double>(verts[vi]),
                              vgl_point_3d<double>(verts[nvi]));
    wgt = vcl_exp(-0.5*wgt*wgt/sigma);
    normals[vi] += wgt*verts.normals()[nvi];
  }

  for(unsigned int i=0; i<normals.size(); ++i){
    normalize(normals[i]);
  }
  verts.set_normals(normals);
}



vgl_polygon<double> project_part(const vcl_string& name,
                                 const imesh_mesh& body,
                                 const imesh_mesh& surface)
{
  vcl_set<unsigned int> selection = body.faces().group_face_set(name);
  vcl_cout << name<<"\nnum faces: "<< selection.size() << vcl_endl;
  if(selection.empty())
    return vgl_polygon<double>();
  imesh_mesh part = imesh_submesh_from_faces(body,selection);
  part.build_edge_graph();
  part.compute_vertex_normals_from_faces();

  for(unsigned int i=0; i<100; ++i)
    smooth_vert_normals(part,0.01);

  const imesh_half_edge_set& half_edges = part.half_edges();
  vcl_vector<vcl_vector<unsigned int> > loops = imesh_detect_boundary_loops(half_edges);

  if(loops.empty())
  {
    return vgl_polygon<double>();
  }

  vcl_cout << "num loops: "<<loops.size() << vcl_endl;
  const imesh_vertex_array<3>& verts = part.vertices<3>();

  vgl_polygon<double> object;
  for(unsigned int i=0; i<loops.size(); ++i){
    const vcl_vector<unsigned int>& loop = loops[i];
    vcl_vector<vgl_point_2d<double> > tex_path;
    for(unsigned int j=0; j<loop.size(); ++j){
      unsigned int vi = half_edges[loop[j]].vert_index();
      vgl_point_3d<double> pt(verts[vi]);
      vgl_vector_3d<double> dir(-verts.normal(vi));
      normalize(dir);
      double dist,u,v;
      int ind = imesh_intersect_min_dist(pt-1.0*dir,dir,surface,dist,&u,&v);
      double cdist,cu,cv;
      vgl_point_3d<double> cp;
      int cind = imesh_closest_point(pt,surface,cp,&cu,&cv);
      cdist = (cp - pt).length();
      if(ind >=0){
        vgl_vector_3d<double> sn = normalized(surface.faces().normal(ind));
        if(dot_product(sn,dir) > -0.866 || dist > 2*cdist){
          vcl_cout << "poor normal alignment" << vcl_endl;
          ind = -1;
        }
      }
      if(ind < 0){
        ind = cind;
        u = cu;
        v = cv;
        //vgl_point_3d<double> cp;
        //ind = imesh_closest_point(pt,surface,cp,&u,&v);
      }
      if(ind >=0){
        vgl_point_2d<double> pt(surface.texture_map(ind,u,v));
        tex_path.push_back(pt);
      }
      else
        vcl_cerr << "unable to project point" << vcl_endl;
    }
    vgl_polygon<double> part(tex_path);
    if(vgl_area_signed(part) <= 0.0)
      continue;
    object = vgl_clip(object, part, vgl_clip_type_union);
  }
  return object;
}


// The Main Function
int main(int argc, char** argv)
{
  vul_arg<vcl_string>  a_in_file("-i", "input mesh file", "");
  vul_arg<vcl_string>  a_body_file("-b", "input mesh body file", "");
  vul_arg<vcl_string>  a_out_file("-o", "output parts file", "");
  vul_arg<vcl_string>  a_svg_file("-s", "output svg file", "");
  vul_arg<vcl_string>  a_part_file("-p", "part names file", "");
  vul_arg<vcl_string>  a_group("-g", "group name", "");
  vul_arg_parse(argc, argv);

  if(!a_in_file.set()){
    vcl_cerr << "input file required" << vcl_endl;
    return -1;
  }
  if(!a_out_file.set()){
    vcl_cerr << "output file required" << vcl_endl;
    return -1;
  }

  imesh_mesh mesh;
  if(!imesh_read(a_in_file(),mesh))
    return -1;

  imesh_mesh body_mesh;
  if(!imesh_read(a_body_file(),body_mesh))
    return -1;

  vcl_vector<vcl_string> part_names;
  vcl_ifstream ifs(a_part_file().c_str());
  vcl_string name;
  while(ifs >> name) part_names.push_back(name);
  ifs.close();

  vcl_set<unsigned int> selection = mesh.faces().group_face_set(a_group());
  imesh_mesh submesh = imesh_submesh_from_faces(mesh,selection);

  imesh_quad_subdivide(submesh);
  imesh_quad_subdivide(submesh);
  vcl_auto_ptr<imesh_face_array_base> tris(imesh_triangulate(submesh.faces()));
  submesh.set_faces(tris);
  submesh.compute_face_normals(false);


  vcl_map<vcl_string,vgl_polygon<double> > parts;


  for(unsigned int i=0; i<part_names.size(); ++i)
  {
    vgl_polygon<double> poly = project_part(part_names[i],body_mesh,submesh);
    if(poly.num_sheets() > 0)
      parts[part_names[i]] = poly;
  }

  if(a_svg_file.set())
    modrec_write_svg(a_svg_file(), parts);
  modrec_write_vehicle_parts(a_out_file(), parts);

  return 0;
}
