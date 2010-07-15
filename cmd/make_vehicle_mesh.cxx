// This is mleotta/cmd/mesh/mesh_make_vehicle.cxx


#include <vcl_iostream.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <imesh/imesh_mesh.h>
#include <imesh/imesh_fileio.h>
#include <modrec/modrec_vehicle_mesh.h>




const double pi = 3.14159265358979;

// The Main Function
int main(int argc, char** argv)
{
  vul_arg<vcl_string>  a_in_file("-i", "input params file", "");
  vul_arg<vcl_string>  a_out_file("-o", "output mesh file", "");
  vul_arg<bool>  a_dodec("-d", "use dodecahedral model", false);
  vul_arg<bool>  a_ferryman("-f", "use ferryman model", false);
  vul_arg_parse(argc, argv);

  if(!a_out_file.set()){
    vcl_cerr << "output file required" << vcl_endl;
    return -1;
  }

  vcl_map<vcl_string,double> params;
  modrec_read_vehicle_params(a_in_file(),params);

  imesh_mesh mesh;
  if(a_dodec())
    modrec_generate_dodec_vehicle(params, mesh);
  else if(a_ferryman())
    modrec_generate_ferryman_vehicle(params, mesh);
  else
    modrec_generate_vehicle(params, mesh);

  imesh_write_obj(a_out_file(), mesh);

  return 0;
}
