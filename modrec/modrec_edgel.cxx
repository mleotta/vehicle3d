// This is contrib/mleotta/modrec/modrec_edgel.cxx
//:
// \file

#include "modrec_edgel.h"

#include <vnl/vnl_math.h>

#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_triangle_scan_iterator.h>


namespace{

//: measure difference between two angles modulo pi/2 radians
inline double angle_diff(double a1, double a2)
{
  double d = vnl_math_abs(a1-a2);
  if(d > vnl_math::pi)
    d = 2*vnl_math::pi - d;
  if(d > vnl_math::pi_over_2)
    d = vnl_math::pi - d;
  return d;
}

inline void scan_edgels(const vil_image_view<float>& edge_map,
                        double angle, double ang_t,
                        vgl_triangle_scan_iterator<double>& tsi,
                        vcl_vector<modrec_edgel>& edgels)
{
  int ni = edge_map.ni();
  int nj = edge_map.nj();
  for (tsi.reset(); tsi.next(); ) {
    int y = tsi.scany();
    if(y < 0) continue;
    if(y >= nj) break;
    for (int x = tsi.startx(); x <= tsi.endx(); ++x)
    {
      if(x < 0) continue;
      if(x >= ni) break;
      if(edge_map(x,y,0) > 0.0f &&
         angle_diff(angle,edge_map(x,y,1)) < ang_t)
      {
        double theta = edge_map(x,y,1);
        double offset = edge_map(x,y,2);
        double gx = vcl_cos(theta);
        double gy = vcl_sin(theta);
        edgels.push_back(modrec_edgel(x+gx*offset, y+gy*offset,
                                      theta, edge_map(x,y,0)));
      }
    }
  }
}

}


//: Return a vector of all edgels in the edge map
vcl_vector<modrec_edgel>
modrec_find_all_edgels(const vil_image_view<float>& edge_map)
{
  vcl_vector<modrec_edgel> edgels;
  
  const unsigned ni = edge_map.ni();
  const unsigned nj = edge_map.nj();
  for(unsigned int j=0; j<nj; ++j){
    for(unsigned int i=0; i<ni; ++i){
      if(edge_map(i,j,0) > 0.0){
        double theta = edge_map(i,j,1);
        double offset = edge_map(i,j,2);
        double gx = vcl_cos(theta);
        double gy = vcl_sin(theta);
        edgels.push_back(modrec_edgel(i+gx*offset, j+gy*offset,
                                      theta, edge_map(i,j,0)));
      }
    }
  }
  return edgels;
}


//: Search the edge map in the neighborhood of a line segment
//  return a vector of edgels found within \a ndist pixel in the normal direction,
//  with a \a edist pixel padding added in the line direction.
//  Edgel orientation must form an angle with the line less than \a ang_t (modulo pi radians)
vcl_vector<modrec_edgel>
modrec_find_edgel_neighbors(const vil_image_view<float>& edge_map,
                            const vgl_line_segment_2d<double>& line,
                            int ndist, int edist, double ang_t)
{
  vcl_vector<modrec_edgel> edgels;

  vgl_vector_2d<double> v = line.direction();
  double angle = atan2(-v.x(),v.y());
  vgl_vector_2d<double> n(-ndist*v.y(),ndist*v.x());
  v *= edist;
  vgl_point_2d<double> p1 = line.point1() - v + n;
  vgl_point_2d<double> p2 = line.point2() + v + n;
  vgl_point_2d<double> p3 = line.point1() - v - n;
  vgl_point_2d<double> p4 = line.point2() + v - n;

  vgl_triangle_scan_iterator<double> tsi;
  tsi.a.x = p1.x();  tsi.a.y = p1.y();
  tsi.b.x = p2.x();  tsi.b.y = p2.y();
  tsi.c.x = p3.x();  tsi.c.y = p3.y();
  scan_edgels(edge_map, angle, ang_t, tsi, edgels);

  tsi.a.x = p3.x();  tsi.a.y = p3.y();
  tsi.b.x = p2.x();  tsi.b.y = p2.y();
  tsi.c.x = p4.x();  tsi.c.y = p4.y();
  scan_edgels(edge_map, angle, ang_t, tsi, edgels);

  return edgels;
}
