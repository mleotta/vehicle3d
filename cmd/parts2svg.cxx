// This is mleotta/cmd/mesh/parts2svg.cxx


#include <vcl_iostream.h>
#include <vul/vul_arg.h>
#include <modrec/modrec_vehicle_parts.h>




// The Main Function
int main(int argc, char** argv)
{
  vul_arg<vcl_string>  a_in_file("-i", "input parts file", "");
  vul_arg<vcl_string>  a_out_file("-o", "output parts file", "");
  vul_arg_parse(argc, argv);

  if(!a_out_file.set()){
    vcl_cerr << "output file required" << vcl_endl;
    return -1;
  }

  typedef vcl_map<vcl_string, vgl_polygon<double> > pmap;

  pmap parts = modrec_read_vehicle_parts(a_in_file());

  modrec_write_svg(a_out_file(),parts);

  return 0;

}
