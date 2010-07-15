// This is mleotta/cmd/mesh/mesh_pca.cxx


#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_math.h>
#include <imesh/imesh_mesh.h>
#include <imesh/imesh_fileio.h>
#include <modrec/modrec_pca_vehicle.h>
#include <modrec/modrec_vehicle_parts.h>



// The Main Function
int main(int argc, char** argv)
{
  vul_arg<vcl_string>  a_in_parts_file("-ip", "input parts file list", "");
  vul_arg<vcl_string>  a_in_mesh_file("-im", "input mesh file list", "");
  vul_arg<vcl_string>  a_mean_parts_file("-op", "mean parts file", "");
  vul_arg<vcl_string>  a_mean_mesh_file("-om", "mean mesh file", "");
  vul_arg<vcl_string>  a_pca_file("-p", "pca file", "");
  vul_arg<vcl_string>  a_proj_file("-j", "projection file", "");
  vul_arg_parse(argc, argv);

  if(!a_in_parts_file.set()){
    vcl_cerr << "input parts file required" << vcl_endl;
    return -1;
  }
  if(!a_in_mesh_file.set()){
    vcl_cerr << "input mesh file required" << vcl_endl;
    return -1;
  }
  if(!a_pca_file.set()){
    vcl_cerr << "output pca file required" << vcl_endl;
    return -1;
  }

  typedef vcl_map<vcl_string, vgl_polygon<double> > pmap;
  vcl_vector<pmap> part_groups;
  
  // make hubcaps which are common to all vehicles
  vgl_polygon<double> h1(1),h2(1),h3(1),h4(1);
  for(unsigned i=0; i<16; ++i){
    double s = vcl_sin(vnl_math::pi*i/8.0)/16;
    double c = vcl_cos(vnl_math::pi*i/8.0)/16;
    h1.push_back(.875-s,.875+c);
    h2.push_back(.875+s,.125-c);
    h3.push_back(.125-s,.875+c);
    h4.push_back(.125+s,.125-c);
  }

  {
    vcl_ifstream ifs(a_in_parts_file().c_str());
    vcl_string fname;
    while(ifs >> fname){
      vcl_cout << "loading: " << fname << vcl_endl;
      pmap parts = modrec_read_vehicle_parts(fname);
      parts["hubcap1"] = h1;
      parts["hubcap2"] = h2;
      parts["hubcap3"] = h3;
      parts["hubcap4"] = h4;
      part_groups.push_back(parts);
    }
    ifs.close();
  }

  vcl_vector<imesh_mesh> meshes;
  {
    vcl_ifstream ifs(a_in_mesh_file().c_str());
    vcl_string fname;
    while(ifs >> fname){
      vcl_cout << "loading: " << fname << vcl_endl;
      imesh_mesh model_mesh;
      imesh_read_obj(fname,model_mesh);
      meshes.push_back(model_mesh);
    }
    ifs.close();
  }

  modrec_pca_vehicle pca_vehicle(meshes,part_groups);
  
  modrec_write_pca_vehicle(a_mean_mesh_file(),
                           a_mean_parts_file(),
                           a_pca_file(),
                           pca_vehicle);
  
  if(a_proj_file.set()){
    vcl_ofstream ofs(a_proj_file().c_str());
    for(unsigned int i=0; i<meshes.size(); ++i){
      vnl_vector<double> v = pca_vehicle.project(meshes[i].vertices(),part_groups[i]);
      ofs << v<<vcl_endl;
    }
    ofs.close();
  }

  return 0;
}
