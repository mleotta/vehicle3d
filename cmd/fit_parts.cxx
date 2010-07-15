// This is mleotta/cmd/mesh/fit_parts.cxx


#include <vcl_iostream.h>
#include <vcl_limits.h>
#include <vul/vul_arg.h>
#include <modrec/modrec_vehicle_parts.h>
#include <vgl/vgl_closest_point.h>
#include <vgl/vgl_area.h>
#include <vgl/vgl_vector_2d.h>
#include <vgl/vgl_line_2d.h>
#include <vgl/vgl_lineseg_test.h>
#include <vgl/vgl_intersection.h>
#include <vgl/algo/vgl_convex_hull_2d.h>


vgl_line_2d<double> snap_to_edge(const vgl_polygon<double>& target,
                                 const vgl_point_2d<double>& p1,
                                 const vgl_point_2d<double>& p2)
{
  vgl_point_2d<double> c = centre(p1, p2);
  vgl_vector_2d<double> n(p1.y()-p2.y(),p2.x()-p1.x());
  normalize(n);
  vgl_point_2d<double> np(c+n);
  vgl_line_2d<double> rayline(c,np);

  double dist = vcl_numeric_limits<double>::infinity();
  vgl_line_2d<double> cline;
  for(unsigned int s=0; s < target.num_sheets(); ++s){ 
    for(unsigned int i=0, j=target[s].size()-1; i<target[s].size(); j = i++)
    {
      const vgl_point_2d<double>& t1 = target[s][j];
      const vgl_point_2d<double>& t2 = target[s][i];
      if(!vgl_lineseg_test_line(c.x(),c.y(),np.x(),np.y(),
                               t1.x(),t1.y(),t2.x(),t2.y()) )
        continue;
      vgl_point_2d<double> pi;
      if(!vgl_intersection(rayline, vgl_line_2d<double>(t1,t2), pi))
        continue;

      double len = (pi-c).length();
      if(len >= dist)
        continue;

      vgl_vector_2d<double> n2(t1.y()-t2.y(),t2.x()-t1.x());
      normalize(n2);
      if(dot_product(n,n2) < 0)
        continue;

      dist = len;
      cline = vgl_line_2d<double>(t1,t2);
    }
  }
  return cline;
}


void project_edges(const vgl_polygon<double>& target,
                   vcl_vector<vgl_point_2d<double> >& points)
{
  vcl_vector<vgl_line_2d<double> > lines(points.size());
  for(unsigned int i=0, j=points.size()-1; i<points.size(); j = i++)
  {
    lines[j] = snap_to_edge(target, points[j], points[i]);
    vgl_vector_2d<double> n(points[j].y()-points[i].y(),
                            points[i].x()-points[j].x());
    normalize(n);
    if(dot_product(n,lines[j].normal()) < 0.7){
      vcl_cout << "use existing line"<<vcl_endl;
      lines[j] = vgl_line_2d<double>(points[j], points[i]);
    }
  }
  for(unsigned int i=0, j=lines.size()-1; i<lines.size(); j = i++)
  {
    vgl_point_2d<double> pi;
    if(vgl_intersection(lines[j],lines[i],pi))
      points[i] = pi;
  }
}

void subdivide(vcl_vector<vgl_point_2d<double> >& points)
{
  for(unsigned int i=0, j=points.size()-1; i<points.size(); j = i++)
  {
    points.insert(points.begin()+i,centre(points[j],points[i]));
    ++i;
  }
}


// The Main Function
int main(int argc, char** argv)
{
  vul_arg<vcl_string>  a_in_file("-i", "input parts file", "");
  vul_arg<vcl_string>  a_model_file("-m", "model parts file", "");
  vul_arg<vcl_string>  a_out_file("-o", "output parts file", "");
  //vul_arg<int>         a_num_subdiv("-subdiv", "number of subdivisions", 0);
  vul_arg_parse(argc, argv);

  if(!a_out_file.set()){
    vcl_cerr << "output file required" << vcl_endl;
    return -1;
  }

  typedef vcl_map<vcl_string, vgl_polygon<double> > pmap;

  pmap iparts = modrec_read_vehicle_parts(a_in_file());
  pmap mparts = modrec_read_vehicle_parts(a_model_file());

  pmap parts;

  //modrec_write_svg(a_out_file(),parts);
  //return 0;

  for(pmap::const_iterator itr=mparts.begin(); itr!=mparts.end(); ++itr)
  {
    const vgl_polygon<double>& mpart = itr->second;
    const vgl_polygon<double>& ipart = iparts[itr->first];
    if(ipart.num_sheets() == 0)
      continue;

    assert(ipart.num_sheets() == 1);
    if(ipart[0].empty())
      continue;

    vgl_polygon<double>& part = parts[itr->first] = ipart;

    vcl_vector<vgl_point_2d<double> > pts;
    for(unsigned int s=0; s<mpart.num_sheets(); ++s)
      pts.insert(pts.end(), mpart[s].begin(), mpart[s].end());

    vgl_polygon<double> convex_mpart = vgl_convex_hull_2d<double>(pts).hull();
    //part = convex_mpart;
    //continue;


    vcl_cout << "fitting part: "<< itr->first << vcl_endl;

    if(vgl_area_signed(part)<=0.0)
      vcl_cerr << "part_reversed ---------------------"<<vcl_endl;

    vgl_point_2d<double> mc = vgl_centroid(convex_mpart);
    vgl_point_2d<double> c = vgl_centroid(part);
    // compute the shift vector to align centroids
    vgl_vector_2d<double> s(mc-c);


    // recenter and scale
    for(unsigned int i=0; i<part[0].size(); ++i){
      part[0][i] += s;
      part[0][i] = mc+(1.5*(part[0][i]-mc));
    }


    for(unsigned int i=0; i<part[0].size(); ++i)
    {
      part[0][i] = vgl_closest_point(part[0][i],convex_mpart);
    }

    project_edges(convex_mpart,part[0]);

    for(unsigned int i=0; i<part[0].size(); ++i)
    {
      part[0][i] = vgl_closest_point(part[0][i],mpart);
    }

    if(vgl_area_signed(part)<=0.0)
      vcl_cerr << "collapsed part! ---------------------"<<vcl_endl;

    if(itr->first == "front_plate" || itr->first == "rear_plate")
      continue;

    subdivide(part[0]);
    project_edges(convex_mpart,part[0]);

    for(unsigned int i=0; i<part[0].size(); ++i)
    {
      part[0][i] = vgl_closest_point(part[0][i],mpart);
    }
    
    if(vgl_area_signed(part)<=0.0)
      vcl_cerr << "collapsed part! ---------------------"<<vcl_endl;

  }

  modrec_write_vehicle_parts(a_out_file(),parts);

  return 0;

}
