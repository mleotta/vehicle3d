// This is contrib/mleotta/modrec/modrec_vehicle_mesh.cxx

//:
// \file


#include "modrec_vehicle_mesh.h"
#include <vcl_cmath.h>
#include <vcl_fstream.h>
#include <imesh/algo/imesh_transform.h>

namespace{

const double pi = 3.141592653589793238;

double get_param(const vcl_string& name, const vcl_map<vcl_string,double>& params)
{
  vcl_map<vcl_string,double>::const_iterator f = params.find(name);
  if(f == params.end())
    return 0.0;

  return f->second;
}

};


//: Read the vehicle parameters from a file
void modrec_read_vehicle_params(const vcl_string filename,
                                vcl_map<vcl_string,double>& params)
{
  vcl_ifstream fh(filename.c_str());

  vcl_string name;
  double val;
  while(fh >> name >> val){
    params[name] = val;
  }

  fh.close();
}


//: Return the mean vehicle params
vcl_map<vcl_string,double> modrec_read_vehicle_params()
{
  vcl_map<vcl_string,double> p;
  p["base_offset"] = 0.105309;
  p["body_width"] = 1.75217;
  p["door_offset"] = -0.265849;
  p["head_b_hgt"] = 0.215424;
  p["head_b_len"] = 0.11819;
  p["head_hgt"] = 0.480279;
  p["head_len"] = 0.800067;
  p["head_width"] = 1.45385;
  p["hood_hgt"] = 0.679043;
  p["hood_width"] = 1.5716;
  p["rwin_depth"] = 0.471869;
  p["rwin_offset"] = 0.467051;
  p["susp_offset"] = 0.0527417;
  p["tail_b_hgt"] = 0.273948;
  p["tail_b_len"] = 0.107457;
  p["tail_hgt"] = 0.679069;
  p["tail_len"] = 0.939277;
  p["tail_width"] = 1.58981;
  p["tire_thick"] = 0.126787;
  p["top_hgt"] = 0.0417266;
  p["top_width"] = 1.19898;
  p["trunk_hgt"] = 0.74307;
  p["trunk_width"] = 1.53785;
  p["wheel_base"] = 2.8066;
  p["wheel_rad"] = 0.215374;
  p["wheel_wgap"] = 0.061132;
  p["wheel_width"] = 0.227521;
  p["win_depth"] = 0.775742;
  p["win_hgt"] = 1.15237;
  p["win_offset"] = 0.285003;

  return p;
}


//: Generate the complete vehicle mesh
void modrec_generate_vehicle(const vcl_map<vcl_string,double>& params,
                             imesh_mesh& mesh)
{
  double r1 = get_param("wheel_rad",params);
  double r2 = r1+get_param("tire_thick",params);
  double ww = get_param("wheel_width",params);

  vcl_auto_ptr<imesh_vertex_array_base> verts(modrec_generate_vehicle_wheel_verts(r1,r2,ww));
  vcl_auto_ptr<imesh_face_array_base> faces(modrec_generate_vehicle_wheel_faces());
  vcl_vector<vgl_point_2d<double> > texture(modrec_generate_vehicle_wheel_tex());

  imesh_mesh wheel_mesh_1(verts,faces);
  imesh_mesh wheel_mesh_2(wheel_mesh_1);
  imesh_mesh wheel_mesh_3(wheel_mesh_1);
  imesh_mesh wheel_mesh_4(wheel_mesh_1);

  wheel_mesh_1.set_tex_coords(modrec_generate_vehicle_wheel_tex(1));
  wheel_mesh_2.set_tex_coords(modrec_generate_vehicle_wheel_tex(2));
  wheel_mesh_3.set_tex_coords(modrec_generate_vehicle_wheel_tex(3));
  wheel_mesh_4.set_tex_coords(modrec_generate_vehicle_wheel_tex(4));

  double wb = get_param("wheel_base",params);
  double bw = get_param("body_width",params);
  double so = get_param("susp_offset",params);

  imesh_transform_inplace(wheel_mesh_1, vgl_rotation_3d<double>(-pi/2,pi/2,0),
                                        vgl_vector_3d<double>(wb/2,bw/2,r2));
  imesh_transform_inplace(wheel_mesh_2, vgl_rotation_3d<double>(pi/2,pi/2,0),
                                        vgl_vector_3d<double>(wb/2,-bw/2,r2));
  imesh_transform_inplace(wheel_mesh_3, vgl_rotation_3d<double>(-pi/2,pi/2,0),
                                        vgl_vector_3d<double>(-wb/2,bw/2,r2));
  imesh_transform_inplace(wheel_mesh_4, vgl_rotation_3d<double>(pi/2,pi/2,0),
                                        vgl_vector_3d<double>(-wb/2,-bw/2,r2));


  verts = (modrec_generate_vehicle_body_verts(params));
  faces = (modrec_generate_vehicle_body_faces());

  mesh.set_vertices(verts);
  mesh.set_faces(faces);
  imesh_transform_inplace(mesh,vgl_vector_3d<double>(0,0,r2+so));

  mesh.set_tex_coords(modrec_generate_vehicle_body_tex());

  mesh.merge(wheel_mesh_1);
  mesh.merge(wheel_mesh_2);
  mesh.merge(wheel_mesh_3);
  mesh.merge(wheel_mesh_4);
}


//: Generate the mesh faces for the vehicle body
vcl_auto_ptr<imesh_face_array_base> modrec_generate_vehicle_body_faces()
{
  vcl_auto_ptr<imesh_face_array> faces(new imesh_face_array);

  // top surface
  for(unsigned int i=0; i<11; ++i)
    faces->push_back(imesh_quad(i, 15+i, 16+i, 1+i));

  // side surfaces
  faces->push_back(imesh_tri(15,46,45));
  faces->push_back(imesh_quad(16,15,45,44));
  faces->push_back(imesh_quad(17,16,44,43));
  faces->push_back(imesh_quad(18,17,43,42));
  faces->push_back(imesh_quad(23,22,31,30));
  faces->push_back(imesh_quad(24,23,30,29));
  faces->push_back(imesh_quad(25,24,29,28));
  faces->push_back(imesh_quad(26,25,28,27));
  faces->push_back(imesh_quad(37,20,19,18));
  faces->push_back(imesh_quad(37,22,21,20));
  faces->push_back(imesh_quad(37,18,42,70));
  faces->push_back(imesh_quad(37,70,31,22));
  faces->push_back(imesh_quad(32,31,70,69));
  faces->push_back(imesh_quad(33,32,69,68));
  faces->push_back(imesh_quad(34,33,68,67));
  faces->push_back(imesh_quad(42,41,69,70));
  faces->push_back(imesh_quad(41,40,68,69));
  faces->push_back(imesh_quad(40,39,67,68));
  faces->push_back(imesh_quad(35,34,67,36));
  faces->push_back(imesh_quad(39,38,36,67));

  // mirror faces in Y
  unsigned int num_faces = faces->size();
  for(unsigned int i=0; i<num_faces; ++i){
    vcl_vector<unsigned int> new_face;
    for(unsigned int j=faces->num_verts(i); j>0; --j){
      unsigned int v = (*faces)(i,j-1);
      if(v > 14) v += 56;
      new_face.push_back(v);
    }
    faces->push_back(new_face);
  }

  faces->make_group("body");

  unsigned int start_faces = faces->size();

  // bottom surface
  faces->push_back(imesh_quad(26,27,47,11));
  faces->push_back(imesh_quad(47,56,12,11));
  faces->push_back(imesh_quad(56,55,13,12));
  faces->push_back(imesh_quad(55,35,36,13));
  faces->push_back(imesh_quad(36,38,57,13));
  faces->push_back(imesh_quad(57,66,14,13));
  faces->push_back(imesh_quad(66,65,0,14));
  faces->push_back(imesh_quad(65,46,15,0));

  // wheel well surface
  for(unsigned int i=0; i<8; ++i){
    faces->push_back(imesh_quad(47+i,27+i,28+i,48+i));
    faces->push_back(imesh_tri(56,47+i,48+i));
    faces->push_back(imesh_quad(57+i,38+i,39+i,58+i));
    faces->push_back(imesh_tri(66,57+i,58+i));
  }

  // mirror faces in Y
  num_faces = faces->size();
  for(unsigned int i=start_faces; i<num_faces; ++i){
    vcl_vector<unsigned int> new_face;
    for(unsigned int j=faces->num_verts(i); j>0; --j){
      unsigned int v = (*faces)(i,j-1);
      if(v > 14) v += 56;
      new_face.push_back(v);
    }
    faces->push_back(new_face);
  }

  faces->make_group("undercarriage");

  return vcl_auto_ptr<imesh_face_array_base>(faces);
}


//: Generate the mesh vertices for the vehicle body
vcl_auto_ptr<imesh_vertex_array<3> >
modrec_generate_vehicle_body_verts(const vcl_map<vcl_string,double>& params)
{
  vcl_auto_ptr<imesh_vertex_array<3> > verts_ptr(new imesh_vertex_array<3>(127));
  imesh_vertex_array<3>& verts = *verts_ptr;


  // reset all vertices to the origin
  for(unsigned int i=0; i<127; ++i)
    verts[i] = imesh_vertex<3>(0,0,0);

  // set X direction positions
  {
    unsigned int ind[] = {0,1,2,3,4, 14,15,16,17,18,19,
                          38,39,40,41,42,43,44,45,46,
                          57,58,59,60,61,62,63,64,65,66};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = get_param("wheel_base",params)/2;
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] = p;
  }
  {
    unsigned int ind[] = {6,7,8,9,10,11,12,
                          21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,
                          47,48,49,50,51,52,53,54,55,56};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = -get_param("wheel_base",params)/2;
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] = p;
  }
  {
    unsigned int ind[] = {0,1,2, 15,16,17};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = get_param("head_len",params);
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] += p;
  }
  {
    unsigned int ind[] = {8,9,10,11, 23,24,25,26};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = -get_param("tail_len",params);
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] += p;
  }
  {
    double p = get_param("head_b_len",params);
    verts[1][0] += p;
    verts[16][0] += p;
  }
  {
    double p = -get_param("tail_b_len",params);
    verts[10][0] += p;
    verts[11][0] += p;
    verts[25][0] += p;
    verts[26][0] += p;
  }
  {
    double p = -get_param("win_offset",params);
    verts[3][0] += p;
    verts[4][0] += p;
    verts[18][0] += p;
    verts[19][0] += p;
  }
  {
    double p = -get_param("win_depth",params);
    verts[4][0] += p;
    verts[19][0] += p;
  }
  {
    double p = -get_param("rwin_offset",params);
    verts[6][0] += p;
    verts[7][0] += p;
    verts[21][0] += p;
    verts[22][0] += p;
  }
  {
    double p = get_param("rwin_depth",params);
    verts[6][0] += p;
    verts[21][0] += p;
  }
  {
    unsigned int ind[] = {5,20,36,37,67,68,69,70};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = get_param("door_offset",params);
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] += p;
  }


  // adjust the wheel well verts
  {
    double wgr = get_param("wheel_rad",params)
               + get_param("tire_thick",params)
               + get_param("wheel_wgap",params);
    unsigned int ind[] = {27,38,47,57};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    for(unsigned int i=0; i<num; ++i){
      verts[ind[i]][0] -= wgr;
      verts[ind[i]+8][0] += wgr;
    }
    for(unsigned int i=0; i<7; ++i){
      double c = vcl_cos((2.0*pi*i)/12);
      double s = vcl_sin((2.0*pi*i)/12);
      for(unsigned int j=0; j<num; ++j){
        verts[ind[j]+i+1][0] -= wgr*c;
        verts[ind[j]+i+1][2] = wgr*s;
      }
    }
  }


  // set Y direction positions
  {
    double p = get_param("body_width",params)/2;
    for( unsigned int i=15; i<47; ++i)
      verts[i][1] = p;
    verts[67][1] = p;
    verts[68][1] = p;
    verts[69][1] = p;
    verts[70][1] = p;
  }
  {
    double p = get_param("top_width",params)/2;
    verts[19][1] = p;
    verts[20][1] = p;
    verts[21][1] = p;
  }
  {
    double p = get_param("body_width",params)/2
             - get_param("wheel_width",params)
             - get_param("wheel_wgap",params);
    for(unsigned int i=47; i<67; ++i)
      verts[i][1] = p;
  }
  {
    double p = get_param("head_width",params)/2;
    verts[15][1] = p;
    verts[16][1] = p;
    verts[17][1] = p;
  }
  {
    double p = get_param("tail_width",params)/2;
    verts[23][1] = p;
    verts[24][1] = p;
    verts[25][1] = p;
    verts[26][1] = p;
  }
  verts[22][1] = get_param("trunk_width",params)/2;
  verts[18][1] = get_param("hood_width",params)/2;


  // set Z direction positions
  {
    double p = get_param("win_hgt",params)
             + get_param("top_hgt",params);
    verts[5][2] = p;
    verts[20][2] = p;
  }
  {
    double p = get_param("win_hgt",params);
    verts[4][2] = p;
    verts[6][2] = p;
    verts[19][2] = p;
    verts[21][2] = p;
  }
  {
    double p = get_param("trunk_hgt",params);
    verts[7][2] = p;
    verts[22][2] = p;
  }
  {
    double p = get_param("hood_hgt",params);
    verts[3][2] = p;
    verts[18][2] = p;
  }
  {
    double p = get_param("head_hgt",params);
    verts[2][2] = p;
    verts[17][2] = p;
  }
  {
    double p = get_param("tail_hgt",params);
    verts[8][2] = p;
    verts[23][2] = p;
  }
  {
    double p = get_param("head_b_hgt",params);
    verts[1][2] = p;
    verts[16][2] = p;
  }
  {
    double p = get_param("tail_b_hgt",params);
    verts[9][2] = p;
    verts[10][2] = p;
    verts[24][2] = p;
    verts[25][2] = p;
  }
  {
    unsigned int ind[] = {27,35,36,38,46,47,55,13,57,65};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = -get_param("base_offset",params);
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][2] = p;
  }


  // dependent vertices
  double d1 = verts[37][0] - verts[22][0];
  double d2 = verts[18][0] - verts[22][0];
  verts[37][1] = d1/d2*verts[18][1] + (d2-d1)/d2*verts[22][1];
  verts[37][2] = d1/d2*verts[18][2] + (d2-d1)/d2*verts[22][2];
  verts[68][2] = (verts[33][2] + verts[40][2])/2;
  verts[69][2] = (verts[32][2] + verts[41][2])/2;
  verts[70][2] = (verts[31][2] + verts[42][2])/2;


  // mirror vertices in Y
  for(unsigned int i=71; i<127; ++i){
    verts[i][0] = verts[i-56][0];
    verts[i][1] = -verts[i-56][1];
    verts[i][2] = verts[i-56][2];
  }

  return verts_ptr;
}


//: Generate the texture coords for the vehicle body
vcl_vector<vgl_point_2d<double> >
modrec_generate_vehicle_body_tex()
{
  vcl_vector<vgl_point_2d<double> > tex(127,vgl_point_2d<double>(0.5,0.5));

  // set X direction positions
  {
    unsigned int ind[] = {5,20,36,37,67,68,69,70};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    for( unsigned int i=0; i<num; ++i)
      tex[ind[i]].x() = 0.5;
  }
  {
    unsigned int ind[] = {7,22,31,32,33,34,35};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    for( unsigned int i=0; i<num; ++i)
      tex[ind[i]].x() = 0.25;
  }
  {
    unsigned int ind[] = {3,18,38,39,40,41,42};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    for( unsigned int i=0; i<num; ++i)
      tex[ind[i]].x() = 0.75;
  }

  tex[0].x() = tex[15].x() = tex[46].x() = 1.0;
  tex[45].x() = 0.96875;
  tex[1].x() = tex[16].x() = 0.9375;
  tex[44].x() = 0.90625;
  tex[2].x() = tex[17].x() = tex[43].x() = 0.875;
  tex[4].x() = tex[19].x() = 0.625;
  tex[6].x() = tex[21].x() = 0.375;
  tex[8].x() = tex[23].x() = tex[30].x() = 0.125;
  tex[9].x() = tex[24].x() = tex[29].x() = 0.0625;
  tex[10].x() = tex[25].x() = tex[28].x() = 0.03125;
  tex[11].x() = tex[26].x() = tex[27].x() = 0.0;

  // set Y direction positions
  for(unsigned int i=0; i<12; ++i)
    tex[i].y() = 0.5;
  {
    unsigned int ind[] = {15,16,17,18,22,23,24,25,26,37};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    for( unsigned int i=0; i<num; ++i)
      tex[ind[i]].y() = 0.625;
  }
  {
    unsigned int ind[] = {27,28,29,30,31,42,43,44,45,46,70};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    for( unsigned int i=0; i<num; ++i)
      tex[ind[i]].y() = 0.75;
  }
  tex[32].y() = tex[69].y() = tex[41].y() = 0.78125;
  tex[33].y() = tex[68].y() = tex[40].y() = 0.875;
  tex[34].y() = tex[67].y() = tex[39].y() = 0.9375;
  tex[35].y() = tex[36].y() = tex[38].y() = 1.0;

  tex[19].y() = tex[20].y() = tex[21].y() = 0.5625;

  // mirror vertices in Y
  for(unsigned int i=71; i<127; ++i){
    tex[i].x() = tex[i-56].x();
    tex[i].y() = 1.0-tex[i-56].y();
  }
  
  return tex; 
}


//: Generate the mesh faces for the vehicle wheel (with \param rs radial samples)
vcl_auto_ptr<imesh_face_array_base> modrec_generate_vehicle_wheel_faces(unsigned int rs)
{
  vcl_auto_ptr<imesh_face_array> faces(new imesh_face_array);

  for(unsigned int i=0; i<rs; ++i){
    unsigned int j = (i+1)%rs;
    faces->push_back(imesh_tri(0,i+2,j+2));
  }
  faces->make_group("hubcap");

  for(unsigned int i=0; i<rs; ++i){
    unsigned int j = (i+1)%rs;
    faces->push_back(imesh_quad(i+2, i+2+rs, j+2+rs, j+2));
    faces->push_back(imesh_quad(i+2+rs, i+2+2*rs, j+2+2*rs, j+2+rs));
    faces->push_back(imesh_quad(i+2+2*rs, i+2+3*rs, j+2+3*rs, j+2+2*rs));
    faces->push_back(imesh_quad(i+2+3*rs, i+2+4*rs, j+2+4*rs, j+2+3*rs));
  }
  faces->make_group("tire");

  for(unsigned int i=0; i<rs; ++i){
    unsigned int j = (i+1)%rs;
    faces->push_back(imesh_tri(1,j+2+4*rs,i+2+4*rs));
  }
  faces->make_group("wheel_back");

  return vcl_auto_ptr<imesh_face_array_base>(faces);
}


//: Generate the mesh vertices for the vehicle wheel
//  \params r1 is the wheel radius
//  \params r2 is the tire outer radius
//  \params r3 is the tire width
vcl_auto_ptr<imesh_vertex_array<3> >
modrec_generate_vehicle_wheel_verts(double r1, double r2, double w, unsigned int rs)
{
  vcl_auto_ptr<imesh_vertex_array<3> > verts_ptr(new imesh_vertex_array<3>(2+rs*5));

  imesh_vertex_array<3>& verts = *verts_ptr;

  verts[0] = imesh_vertex<3>(0, 0, -w*.1);
  verts[1] = imesh_vertex<3>(0, 0, -w*.9);
  for(unsigned int i=0; i<rs; ++i){
    double a = (2.0*pi*i)/rs;
    double c = vcl_cos(a);
    double s = vcl_sin(a);
    verts[i+2] = imesh_vertex<3>(r1*c, r1*s, 0);
    verts[i+2+rs] = imesh_vertex<3>(r2*c, r2*s, 0);
    verts[i+2+2*rs] = imesh_vertex<3>(r2*c, r2*s, -w/2);
    verts[i+2+3*rs] = imesh_vertex<3>(r2*c, r2*s, -w);
    verts[i+2+4*rs] = imesh_vertex<3>(r1*c, r1*s, -w);
  }

  return verts_ptr;
}


//: Generate the texture coords for the vehicle wheel
// if index == 0, center the wheel at the origin
// else position the wheel in each of the four corners of the unit square
vcl_vector<vgl_point_2d<double> >
modrec_generate_vehicle_wheel_tex(unsigned int index, unsigned int rs)
{
  vcl_vector<vgl_point_2d<double> > tex(2+rs*5);

  double r = 1.0/8.0;

  vgl_vector_2d<double> offset(0,0);
  switch(index)
  {
    case 1: offset.set(1.0-r, 1.0-r); break;
    case 2: offset.set(1.0-r, r); break;
    case 3: offset.set(r, 1.0-r); break;
    case 4: offset.set(r, r); break;
  }

  double r1 = r/2;
  double r2 = 3*r/4;
  double r3 = r;
  int sign = 1 - 2*(index%2);

  tex[0] = tex[1] = vgl_point_2d<double>(0,0)+offset;
  for(unsigned int i=0; i<rs; ++i){
    double a = (2.0*pi*i)/rs;
    double c = sign*-vcl_cos(a);
    double s = sign*vcl_sin(a);
    tex[i+2] = tex[i+2+4*rs] = vgl_point_2d<double>(r1*s, r1*c)+offset;
    tex[i+2+rs] = tex[i+2+3*rs] = vgl_point_2d<double>(r2*s, r2*c)+offset;
    tex[i+2+2*rs] = vgl_point_2d<double>(r3*s, r3*c)+offset;
  }

  return tex;
}

// =============================================================================
// Dodecahedral mesh

//: Generate the dodecahedral vehicle mesh
void modrec_generate_dodec_vehicle(const vcl_map<vcl_string,double>& params,
                                   imesh_mesh& mesh)
{
  double r1 = get_param("wheel_rad",params);
  double r2 = r1+get_param("tire_thick",params);
  double so = get_param("susp_offset",params);
  
  vcl_auto_ptr<imesh_vertex_array_base> verts(modrec_generate_dodec_vehicle_verts(params));
  vcl_auto_ptr<imesh_face_array_base> faces(modrec_generate_dodec_vehicle_faces());
  
  mesh.set_vertices(verts);
  mesh.set_faces(faces);
  imesh_transform_inplace(mesh,vgl_vector_3d<double>(0,0,r2+so));
  
  //mesh.set_tex_coords(modrec_generate_dodec_vehicle_tex());
}


//: Generate the mesh vertices for the dodecahedral body
vcl_auto_ptr<imesh_vertex_array<3> >
modrec_generate_dodec_vehicle_verts(const vcl_map<vcl_string,double>& params)
{
  vcl_auto_ptr<imesh_vertex_array<3> > verts_ptr(new imesh_vertex_array<3>(16));
  imesh_vertex_array<3>& verts = *verts_ptr;
  
  
  // reset all vertices to the origin
  for(unsigned int i=0; i<16; ++i)
    verts[i] = imesh_vertex<3>(0,0,0);
    
    // set X direction positions
  {
    unsigned int ind[] = {0,1,2,3, 8,9,10,11};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = get_param("wheel_base",params)/2;
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] = p;
  }
  {
    unsigned int ind[] = {4,5,6,7, 12,13,14,15};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = -get_param("wheel_base",params)/2;
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] = p;
  }
  {
    unsigned int ind[] = {0,1, 8,9};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = get_param("head_len",params);
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] += p;
  }
  {
    unsigned int ind[] = {6,7, 14,15};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = -get_param("tail_len",params);
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][0] += p;
  }

  {
    double p = -get_param("win_offset",params);
    verts[2][0] += p;
    verts[3][0] += p;
    verts[10][0] += p;
    verts[11][0] += p;
  }
  {
    double p = -get_param("win_depth",params);
    verts[3][0] += p;
    verts[11][0] += p;
  }
  {
    double p = -get_param("rwin_offset",params);
    verts[4][0] += p;
    verts[5][0] += p;
    verts[12][0] += p;
    verts[13][0] += p;
  }
  {
    double p = get_param("rwin_depth",params);
    verts[4][0] += p;
    verts[12][0] += p;
  }

  // set Y direction positions
  {
    double p = get_param("body_width",params)/2;
    verts[2][1] = -p;
    verts[5][1] = -p;
    verts[10][1] = p;
    verts[13][1] = p;
    
    verts[0][1] = -p;
    verts[8][1] = p; 
    verts[7][1] = -p;
    verts[15][1] = p;
    verts[1][1] = -p;
    verts[9][1] = p;
    verts[6][1] = -p;
    verts[14][1] = p;
  }
  {
    double p = get_param("top_width",params)/2;
    verts[3][1] = -p;
    verts[4][1] = -p;
    verts[11][1] = p;
    verts[12][1] = p;
  }
#if 0
  {
    double p = get_param("head_width",params)/2;
    verts[0][1] = -p;
    verts[8][1] = p;
  }
  {
    double p = get_param("tail_width",params)/2;
    verts[7][1] = -p;
    verts[15][1] = p;
  }
  {
    double p = get_param("hood_width",params)/2;
    verts[1][1] = -p;
    verts[9][1] = p;
  }
  {
    double p = get_param("trunk_width",params)/2;
    verts[6][1] = -p;
    verts[14][1] = p;
  }
#endif
  
  
  // set Z direction positions
  {
    double p = get_param("win_hgt",params);
    verts[3][2] = p;
    verts[4][2] = p;
    verts[11][2] = p;
    verts[12][2] = p;
  }
  {
    double p = get_param("trunk_hgt",params);
    verts[5][2] = p;
    verts[13][2] = p;
  }
  {
    double p = get_param("hood_hgt",params);
    verts[2][2] = p;
    verts[10][2] = p;
  }
  {
    double p = get_param("head_hgt",params);
    verts[1][2] = p;
    verts[9][2] = p;
  }
  {
    double p = get_param("tail_hgt",params);
    verts[6][2] = p;
    verts[14][2] = p;
  }
  {
    unsigned int ind[] = {0,7, 8,15};
    unsigned int num = sizeof(ind)/sizeof(unsigned int);
    double p = -get_param("base_offset",params);
    for( unsigned int i=0; i<num; ++i)
      verts[ind[i]][2] = p;
  }

  
  return verts_ptr;
}


//: Generate the mesh faces for the dodecahedral body
vcl_auto_ptr<imesh_face_array_base>
modrec_generate_dodec_vehicle_faces()
{
  vcl_auto_ptr<imesh_face_array> faces(new imesh_face_array);
  
  // top surface
  for(unsigned int i=0; i<7; ++i)
    faces->push_back(imesh_quad(i, 8+i, 9+i, 1+i));
    
  // side surfaces
  faces->push_back(imesh_quad(2,3,4,5));
  unsigned int side1[] = {0,1,2,5,6,7};
  faces->push_back(vcl_vector<unsigned int>(side1,side1+6));
  //faces->push_back(imesh_tri(0,1,2));
  //faces->push_back(imesh_tri(5,6,7));
  //faces->push_back(imesh_quad(0,2,5,7));
  
  faces->push_back(imesh_quad(13,12,11,10));
  unsigned int side2[] = {15,14,13,10,9,8};
  faces->push_back(vcl_vector<unsigned int>(side2,side2+6));
  //faces->push_back(imesh_tri(10,9,8));
  //faces->push_back(imesh_tri(15,14,13));
  //faces->push_back(imesh_quad(15,13,10,8));
  
  faces->make_group("body");
  
  unsigned int start_faces = faces->size();
  
  // bottom surface
  faces->push_back(imesh_quad(0,7,15,8));

  faces->make_group("undercarriage");
  
  return vcl_auto_ptr<imesh_face_array_base>(faces);
}


// =============================================================================
// Ferryman mesh
// J. M. Ferryman, A. D. Worrall, G. D. Sullivan, and K. D. Baker
// "A Generic Deformable Model for Vehicle Recognition", BMVC 1995

//: Generate the ferryman vehicle mesh
void modrec_generate_ferryman_vehicle(const vcl_map<vcl_string,double>& params,
                                      imesh_mesh& mesh)
{
  double r1 = get_param("wheel_rad",params);
  double r2 = r1+get_param("tire_thick",params);
  double so = get_param("susp_offset",params);
  
  vcl_auto_ptr<imesh_vertex_array_base> verts(modrec_generate_ferryman_vehicle_verts(params));
  vcl_auto_ptr<imesh_face_array_base> faces(modrec_generate_ferryman_vehicle_faces());
  
  mesh.set_vertices(verts);
  mesh.set_faces(faces);
  imesh_transform_inplace(mesh,vgl_vector_3d<double>(0,0,r2+so));
}


//: Generate the mesh vertices for the Ferryman body
vcl_auto_ptr<imesh_vertex_array<3> >
modrec_generate_ferryman_vehicle_verts(const vcl_map<vcl_string,double>& params)
{
  // the Dodecahedral model has a subset of the vertices in the Ferryman model
  vcl_auto_ptr<imesh_vertex_array<3> > verts_ptr(modrec_generate_dodec_vehicle_verts(params));
  imesh_vertex_array<3>& verts = *verts_ptr;
  
  unsigned int start_idx = verts.size();
  
  double x = get_param("wheel_base",params)/2;
  double y = get_param("body_width",params)/2;
  double z = -get_param("base_offset",params);
  
  // add the wheel vertices
  for(unsigned int i=0; i<6; ++i)
    verts.push_back(imesh_vertex<3>(-x,-y,z));
  for(unsigned int i=0; i<6; ++i)
    verts.push_back(imesh_vertex<3>(x,-y,z));
  for(unsigned int i=0; i<6; ++i)
    verts.push_back(imesh_vertex<3>(-x,y,z));
  for(unsigned int i=0; i<6; ++i)
    verts.push_back(imesh_vertex<3>(x,y,z));
  
  // compute the wheel well radius
  double r = get_param("wheel_rad",params)
            +get_param("tire_thick",params)
            +get_param("wheel_wgap",params);
  
  // shift the vertices around a semicircle
  for(unsigned int i=0; i<6; ++i){
    double dx = -r*vcl_cos(i*vnl_math::pi/5.0);
    double dy = r*vcl_sin(i*vnl_math::pi/5.0);
    verts[start_idx+i][0] += dx;
    verts[start_idx+i][2] += dy;
    verts[start_idx+6+i][0] += dx;
    verts[start_idx+6+i][2] += dy;
    verts[start_idx+12+i][0] += dx;
    verts[start_idx+12+i][2] += dy;
    verts[start_idx+18+i][0] += dx;
    verts[start_idx+18+i][2] += dy;
  }

  return verts_ptr;
}



//: Generate the mesh faces for the Ferryman body
vcl_auto_ptr<imesh_face_array_base>
modrec_generate_ferryman_vehicle_faces()
{
  vcl_auto_ptr<imesh_face_array> faces(new imesh_face_array);
  
  // top surface
  for(unsigned int i=0; i<7; ++i)
    faces->push_back(imesh_quad(i, 8+i, 9+i, 1+i));
    
  // side surfaces
  faces->push_back(imesh_quad(2,3,4,5));
  unsigned int side1[] = {0,1,2,5,6,7};
  vcl_vector<unsigned int> side1v(side1,side1+6);
  for(unsigned int i=16; i<28; ++i)
    side1v.push_back(i);
  faces->push_back(side1v);

  
  faces->push_back(imesh_quad(13,12,11,10));
  unsigned int side2[] = {15,14,13,10,9,8};
  vcl_vector<unsigned int> side2v(side2,side2+6);
  for(unsigned int i=39; i>=28; --i)
    side2v.push_back(i);
  faces->push_back(side2v);

  
  faces->make_group("body");
  
  unsigned int start_faces = faces->size();
  
  //Not watertight, the bottom surface is missing 
  //faces->push_back(imesh_quad(0,7,15,8));
  //faces->make_group("undercarriage");
  
  return vcl_auto_ptr<imesh_face_array_base>(faces);
}
