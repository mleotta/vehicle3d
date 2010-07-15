// This is mleotta/cmd/mesh/mesh_exterior.cxx


#include <vcl_iostream.h>
#include <vcl_fstream.h>
#include <vul/vul_arg.h>
#include <imesh/imesh_mesh.h>
#include <imesh/imesh_operations.h>
#include <imesh/imesh_fileio.h>
#include <imesh/algo/imesh_detect.h>



// The Main Function
int main(int argc, char** argv)
{
  vul_arg<vcl_string>  a_in_file("-i", "input mesh_file", "");
  vul_arg<vcl_string>  a_out_file("-o", "output mesh file", "");
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
  imesh_read_obj(a_in_file(),mesh);


  vcl_set<unsigned int> ext_frontfaces;
  vcl_set<unsigned int> ext_backfaces;
  vcl_set<unsigned int> ext_bifaces;
  imesh_detect_exterior_faces(mesh,ext_frontfaces, ext_backfaces, ext_bifaces);
  vcl_cout << "exterior:\n  front: "<<ext_frontfaces.size()
           <<"\n  back: "<<ext_backfaces.size()
           <<"\n  both: "<<ext_bifaces.size()<<vcl_endl;

  imesh_flip_faces(mesh, ext_backfaces);
  ext_frontfaces.insert(ext_backfaces.begin(),ext_backfaces.end());

  ext_frontfaces.insert(ext_bifaces.begin(),ext_bifaces.end());



  imesh_mesh ext_mesh = imesh_submesh_from_faces(mesh,ext_frontfaces);

  imesh_write_obj(a_out_file(), ext_mesh);




  return 0;
}
