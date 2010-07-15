// This is contrib/mleotta/modrec/modrec_vehicle_parts.cxx

//:
// \file


#include "modrec_vehicle_parts.h"
#include <vcl_fstream.h>
#include <vcl_sstream.h>


//: read the vehicle parts from a file
vcl_map<vcl_string, vgl_polygon<double> >
modrec_read_vehicle_parts(const vcl_string filename)
{
  vcl_map<vcl_string, vgl_polygon<double> > parts;
  vcl_ifstream fh(filename.c_str());

  char c = fh.peek();
  vgl_polygon<double> poly;
  vcl_string name = "";
  while(fh.good())
  {
    vcl_string line;
    vcl_getline(fh,line);
    vcl_stringstream ss(line);
    if((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'))
    {
      if(name != "")
        parts[name] = poly;
      poly.clear();
      ss >> name;
    }
    else
    {
      poly.new_sheet();
      double x,y;
      while(ss>>x)
      {
        if(ss.peek() == ',')
          ss.ignore();
        ss >> y;
        poly.push_back(vgl_point_2d<double>(x,y));
      }
    }
    c = fh.peek();
  }
  if(name != "")
    parts[name] = poly;

  fh.close();
  return parts;
}


//: write the vehicle parts to a file
void
modrec_write_vehicle_parts(const vcl_string filename,
                           const vcl_map<vcl_string, vgl_polygon<double> >& parts)
{
  typedef vcl_map<vcl_string, vgl_polygon<double> > pmap;
  vcl_ofstream fh(filename.c_str());
  for(pmap::const_iterator itr = parts.begin(); itr != parts.end(); ++itr)
  {
    fh << itr->first << '\n';
    for(unsigned int i=0; i<itr->second.num_sheets(); ++i)
    {
      const vcl_vector<vgl_point_2d<double> >& pts = itr->second[i];
      for(unsigned int j=0; j<pts.size(); ++j)
      {
        fh << pts[j].x()<<','<<pts[j].y()<<' ';
      }
      fh <<'\n';
    }
  }
  fh.close();
}


//: write the vehicle parts as an SVG file
void modrec_write_svg(const vcl_string& file,
                      const vcl_map<vcl_string,vgl_polygon<double> >& paths)
{
  vcl_ofstream fh(file.c_str());
  fh << "<?xml version=\"1.0\" standalone=\"no\"?>\n"
     << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n" 
     << "  \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n"
     << "<svg width=\"1000px\" height=\"1000px\" version=\"1.1\"\n"
     << "     viewBox=\"0 0 1000 1000\" xmlns=\"http://www.w3.org/2000/svg\" preserveAspectRatio=\"none\">\n";

  fh << "  <rect x=\"0\" y=\"0\" width=\"1000\" height=\"1000\" "
     << "fill=\"none\" stroke=\"black\" stroke-width=\"1px\" />\n";

  typedef vcl_map<vcl_string,vgl_polygon<double> > path_map;
  for(path_map::const_iterator itr = paths.begin(); itr != paths.end(); ++itr)
  {
    const vcl_string& name = itr->first;
    fh << "  <desc>"<<name<<"</desc>\n";
    const vgl_polygon<double>& poly = itr->second;
    for(unsigned int j=0; j<poly.num_sheets(); ++j){
      fh << "  <polygon fill=\"blue\" stroke=\"black\" stroke-width=\"1px\"\n"
         << "           points=\"";
      for(unsigned int i=0; i<poly[j].size(); ++i)
        fh << 1000*poly[j][i].x()<<','<<1000*(1-poly[j][i].y())<<' ';
      fh << "\" />\n";
    }
  }

  fh << "</svg>" << vcl_endl;
  fh.close();
}



