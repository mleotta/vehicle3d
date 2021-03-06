// This is dml/dml_vehicle_mesh.h
#ifndef dml_vehicle_mesh_h_
#define dml_vehicle_mesh_h_

//:
// \file
// \brief Functions for generating and manipulated the vehicle mesh
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 5/8/08
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim

#include <imesh/imesh_mesh.h>
#include <vcl_map.h>
#include <vcl_string.h>


//: Read the vehicle parameters from a file
void dml_read_vehicle_params(const vcl_string filename,
                             vcl_map<vcl_string,double>& params);

//: Return the mean vehicle params
vcl_map<vcl_string,double> dml_read_vehicle_params();


//: Generate the complete vehicle mesh
void dml_generate_vehicle(const vcl_map<vcl_string,double>& params,
                          imesh_mesh& mesh);


//: Generate the mesh faces for the vehicle body
vcl_auto_ptr<imesh_face_array_base>
dml_generate_vehicle_body_faces();


//: Generate the mesh vertices for the vehicle body
vcl_auto_ptr<imesh_vertex_array<3> >
dml_generate_vehicle_body_verts(const vcl_map<vcl_string,double>& params);


//: Generate the texture coords for the vehicle body
vcl_vector<vgl_point_2d<double> >
dml_generate_vehicle_body_tex();


//: Generate the mesh faces for the vehicle wheel (with \param rs radial samples)
vcl_auto_ptr<imesh_face_array_base>
dml_generate_vehicle_wheel_faces(unsigned int rs = 16);


//: Generate the mesh vertices for the vehicle wheel
//  \params r1 is the wheel radius
//  \params r2 is the tire outer radius
//  \params r3 is the tire width
vcl_auto_ptr<imesh_vertex_array<3> >
dml_generate_vehicle_wheel_verts(double r1, double r2, double w, unsigned int rs = 16);


//: Generate the texture coords for the vehicle wheel
// if index == 0, center the wheel at the origin
// else position the wheel in each of the four corners of the unit square
vcl_vector<vgl_point_2d<double> >
dml_generate_vehicle_wheel_tex(unsigned int index = 0, unsigned int rs = 16);

// =============================================================================
// Dodecahedral mesh

//: Generate the dodecahedral vehicle mesh
void dml_generate_dodec_vehicle(const vcl_map<vcl_string,double>& params,
                                imesh_mesh& mesh);

//: Generate the mesh vertices for the dodecahedral body
vcl_auto_ptr<imesh_vertex_array<3> >
dml_generate_dodec_vehicle_verts(const vcl_map<vcl_string,double>& params);

//: Generate the mesh faces for the dodecahedral body
vcl_auto_ptr<imesh_face_array_base>
dml_generate_dodec_vehicle_faces();

// =============================================================================
// Ferryman mesh
// J. M. Ferryman, A. D. Worrall, G. D. Sullivan, and K. D. Baker
// "A Generic Deformable Model for Vehicle Recognition", BMVC 1995

//: Generate the ferryman vehicle mesh
void dml_generate_ferryman_vehicle(const vcl_map<vcl_string,double>& params,
                                   imesh_mesh& mesh);

//: Generate the mesh vertices for the Ferryman body
vcl_auto_ptr<imesh_vertex_array<3> >
dml_generate_ferryman_vehicle_verts(const vcl_map<vcl_string,double>& params);

//: Generate the mesh faces for the Ferryman body
vcl_auto_ptr<imesh_face_array_base>
dml_generate_ferryman_vehicle_faces();


#endif // dml_vehicle_mesh_h_
