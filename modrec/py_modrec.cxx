// This is contrib/mleotta/modrec/py_modrec.cxx
//:
// \file
// \brief The python interface for modrec
// \author Matt Leotta (mleotta@lems.brown.edu)
// \date 11/12/08

#include "Python.h"
#include <vcl_iostream.h>
#include "py_modrec_manager.h"


static PyObject *
set_camera(PyObject *self, PyObject *args)
{
  int frame;
  const char* filename;
  if (!PyArg_ParseTuple(args, "is:set_camera", &frame, &filename))
    return NULL;
  
  bool ret = py_modrec_manager::instance()->set_camera(frame,filename);
  
  return Py_BuildValue("b", ret);
}

static PyObject *
set_image(PyObject *self, PyObject *args)
{
  int frame;
  const char* filename;
  if (!PyArg_ParseTuple(args, "is:set_image", &frame, &filename))
    return NULL;
  
  bool ret = py_modrec_manager::instance()->set_image(frame,filename);
  
  return Py_BuildValue("b", ret);
}


static PyObject *
set_video(PyObject *self, PyObject *args)
{
  const char* filename;
  if (!PyArg_ParseTuple(args, "s:set_video", &filename))
    return NULL;
  
  bool ret = py_modrec_manager::instance()->set_video(filename);
  
  return Py_BuildValue("b", ret);
}


static PyObject *
reset_views(PyObject *self, PyObject *args)
{
  py_modrec_manager::instance()->reset_views();
  return Py_BuildValue("b", true);
}


static PyObject *
set_parts(PyObject *self, PyObject *args)
{
  const char* filename;
  if (!PyArg_ParseTuple(args, "s:set_parts", &filename))
    return NULL;
  
  bool ret = py_modrec_manager::instance()->set_parts(filename);
  
  return Py_BuildValue("b", ret);
}

static PyObject *
set_pca(PyObject *self, PyObject *args)
{
  const char* filename;
  if (!PyArg_ParseTuple(args, "s:set_pca", &filename))
    return NULL;
  
  bool ret = py_modrec_manager::instance()->set_pca(filename);
  
  return Py_BuildValue("b", ret);
}

static PyObject *
set_mesh(PyObject *self, PyObject *args)
{
  const char* filename;
  const char* partsname = "";
  if (!PyArg_ParseTuple(args, "s|s:set_mesh", &filename, &partsname))
    return NULL;
  
  bool ret = false;
  ret = py_modrec_manager::instance()->set_mesh(filename,partsname);
  
  return Py_BuildValue("b", ret);
}

static PyObject *
set_vehicle_model(PyObject *self, PyObject *args)
{
  const char* type = "Detailed3";
  if (!PyArg_ParseTuple(args, "s:set_vehicle_model", &type))
    return NULL;
  
  if(vcl_string(type) == "Detailed1")
    py_modrec_manager::instance()->set_vehicle_model(py_modrec_manager::DETAILED1);
  else if(vcl_string(type) == "Detailed2")
    py_modrec_manager::instance()->set_vehicle_model(py_modrec_manager::DETAILED2);
  else if(vcl_string(type) == "Detailed3")
    py_modrec_manager::instance()->set_vehicle_model(py_modrec_manager::DETAILED3);
  else if(vcl_string(type) == "Dodecahedral")
    py_modrec_manager::instance()->set_vehicle_model(py_modrec_manager::DODECAHEDRAL);
  else if(vcl_string(type) == "Ferryman")
    py_modrec_manager::instance()->set_vehicle_model(py_modrec_manager::FERRYMAN);
  else
    return Py_BuildValue("b", false);
  
  return Py_BuildValue("b", true);
}


static PyObject *
set_fit_mode(PyObject *self, PyObject *args)
{
  const char* mode = "multiview";
  if (!PyArg_ParseTuple(args, "s:set_fit_mode_video", &mode))
    return NULL;
  
  if(vcl_string(mode) == "video")
    py_modrec_manager::instance()->set_fit_mode(true);
  else if(vcl_string(mode) == "multiview")
    py_modrec_manager::instance()->set_fit_mode(false);
  else
    return Py_BuildValue("b", false);
  
  return Py_BuildValue("b", true);
}


static PyObject *
set_options(PyObject *self, PyObject *args)
{
  PyObject *list;
  int len;
  if (!PyArg_ParseTuple(args, "O!:set_options", &PyList_Type, &list))
    return NULL;
  
  len = PyList_Size(list);
  if (len < 0) {
    return NULL;
  }
  
  vcl_vector<bool> options(7,false);
  unsigned int num_pc = 0;
  for(int i=0; i<len; ++i){
    PyObject * el;
    if (!(el = PyList_GetItem(list, i))){
      return NULL;
    }
    if (PyString_Check(el)){
      vcl_string p(PyString_AsString(el));
      if(p == "tx")       options[1] = true;
      else if(p == "ty")  options[2] = true;
      else if(p == "tz")  options[3] = true;
      else if(p == "rx")  options[4] = true;
      else if(p == "ry")  options[5] = true;
      else if(p == "rz")  options[6] = true;
      else return NULL;
    }
    else if (PyInt_Check(el)){
      long int num = PyInt_AsLong(el);
      if(num > 0){
        options[0] = true;
        num_pc = num;
      }
    }
    else
      return NULL;
  }
  
  py_modrec_manager::instance()->set_options(options,num_pc);
  
  return Py_BuildValue("b", true);
}

static PyObject *
set_init_uncert(PyObject *self, PyObject *args)
{  
  double uncert;
  if (!PyArg_ParseTuple(args, "d:set_init_uncert", &uncert))
    return NULL;
  py_modrec_manager::instance()->set_init_uncert(uncert);
  
  return Py_BuildValue("b", true);
}

static PyObject *
set_translation(PyObject *self, PyObject *args)
{
  double tx, ty, tz;
  if (!PyArg_ParseTuple(args, "(ddd):set_translation", &tx, &ty, &tz))
    return NULL;
  
  py_modrec_manager::instance()->set_translation(tx, ty, tz);
  
  return Py_BuildValue("b", true);
}

static PyObject *
set_rotation(PyObject *self, PyObject *args)
{
  double rx, ry, rz;
  if (!PyArg_ParseTuple(args, "(ddd):set_rotation", &rx, &ry, &rz))
    return NULL;
  
  py_modrec_manager::instance()->set_rotation(rx, ry, rz);
  
  return Py_BuildValue("b", true);
}

static PyObject *
set_params(PyObject *self, PyObject *args)
{
  PyObject *list;
  int len;
  if (!PyArg_ParseTuple(args, "O!:set_options", &PyList_Type, &list))
    return NULL;
  
  len = PyList_Size(list);
  if (len < 0) {
    return NULL;
  }
  vcl_vector<double> params(len);
  for(int i=0; i<len; ++i){
    PyObject * el;
    if (!(el = PyList_GetItem(list, i))){
      return NULL;
    }
    if (PyFloat_Check(el)){
      params[i] = PyFloat_AsDouble(el);
    }
    else if (PyInt_Check(el)){
      params[i] = (double) PyInt_AsLong(el);
    } else
      return NULL;
  }
    
  py_modrec_manager::instance()->set_params(params);
  
  return Py_BuildValue("b", true);
}

static PyObject *
set_lambda(PyObject *self, PyObject *args)
{  
  double lambda;
  if (!PyArg_ParseTuple(args, "d:set_lambda", &lambda))
    return NULL;
  py_modrec_manager::instance()->set_lambda(lambda);
  
  return Py_BuildValue("b", true);
}


static PyObject *
set_truth_mesh(PyObject *self, PyObject *args)
{
  const char* filename;
  const char* type = "";
  if (!PyArg_ParseTuple(args, "s:set_truth_mesh", &filename, &type))
    return NULL;
  
  bool ret = py_modrec_manager::instance()->set_truth_mesh(filename);
  
  return Py_BuildValue("b", ret);
}


static PyObject *
evaluate_residual(PyObject *self, PyObject *args)
{  
  double scale = 1.0;
  bool compute_vis = true;
  if (!PyArg_ParseTuple(args, "d|b:evaluate_residual", &scale, &compute_vis))
    return NULL;
  
  double error = py_modrec_manager::instance()->evaluate_residual(scale,compute_vis);
  
  return Py_BuildValue("d", error);
}


static PyObject *
compute_error(PyObject *self, PyObject *args)
{  
  double error = py_modrec_manager::instance()->compute_error();
  
  return Py_BuildValue("d", error);
}

static PyObject *
fit_model(PyObject *self, PyObject *args)
{  
  int max_itr = 20;
  if (!PyArg_ParseTuple(args, "|i:fit_model", &max_itr))
    return NULL;
  py_modrec_manager::instance()->fit_model(max_itr);
  
  return Py_BuildValue("b", true);
}

static PyObject *
write_svg_curves(PyObject *self, PyObject *args)
{
  int frame;
  const char* filename;
  if (!PyArg_ParseTuple(args, "is:write_svg_curves", &frame, &filename))
    return NULL;
  
  bool ret = py_modrec_manager::instance()->write_svg_curves(frame,filename);
  
  return Py_BuildValue("b", ret);
}

static PyObject *
enable_tracking(PyObject *self, PyObject *args)
{  
  bool enabled = 0;
  if (!PyArg_ParseTuple(args, "b:enable_tracking", &enabled))
    return NULL;
  py_modrec_manager::instance()->enable_tracking(enabled);
  
  return Py_BuildValue("b", true);
}

static PyObject *
set_track_with_silhouette(PyObject *self, PyObject *args)
{  
  bool val = false;
  if (!PyArg_ParseTuple(args, "b:set_track_with_silhouette", &val))
    return NULL;
  py_modrec_manager::instance()->set_track_with_silhouette(val);
  
  return Py_BuildValue("b", true);
}

static PyObject *
video_seek(PyObject *self, PyObject *args)
{  
  int frame = 0;
  if (!PyArg_ParseTuple(args, "|i:video_seek", &frame))
    return NULL;
  py_modrec_manager::instance()->video_seek(frame);
  
  return Py_BuildValue("b", true);
}

static PyObject *
advance_video(PyObject *self, PyObject *args)
{  
  bool ret = py_modrec_manager::instance()->advance_video();
  return Py_BuildValue("b", ret);
}

static PyObject *
get_current_frame(PyObject *self, PyObject *args)
{  
  int ret = py_modrec_manager::instance()->current_frame();
  return Py_BuildValue("i", ret);
}

static PyObject *
get_translation(PyObject *self, PyObject *args)
{
  double tx,ty,tz;
  py_modrec_manager::instance()->get_translation(tx,ty,tz);
  return Py_BuildValue("(ddd)", tx,ty,tz);
}

static PyObject *
get_rotation(PyObject *self, PyObject *args)
{
  double rx,ry,rz;
  py_modrec_manager::instance()->get_rotation(rx,ry,rz);
  return Py_BuildValue("(ddd)", rx,ry,rz);
}

static PyObject *
get_params(PyObject *self, PyObject *args)
{
  const char * opt = NULL;
  if (!PyArg_ParseTuple(args, "|s:get_params", &opt))
    return NULL;
  
  vcl_vector<double> params;
  py_modrec_manager::instance()->get_params(params);
  unsigned num_params = py_modrec_manager::instance()->num_pc();
  if(opt && vcl_string(opt) == "all")
    num_params = params.size();
  
  PyObject* list = PyList_New(num_params);
  for (int i = 0; i < num_params; i++)
    PyList_SetItem(list, i, Py_BuildValue("d", params[i]));
  
  return list;
}

static PyObject *
get_vehicle_model(PyObject *self, PyObject *args)
{
  vcl_string model;
  py_modrec_manager::vehicle_model vm;
  py_modrec_manager::instance()->get_vehicle_model(vm);
  switch(vm){
    case py_modrec_manager::DODECAHEDRAL:
      model = "Dodecahedral";
      break;
    case py_modrec_manager::FERRYMAN:
      model = "Ferryman";
      break;
    case py_modrec_manager::DETAILED1:
      model = "Detailed1";
      break;
    case py_modrec_manager::DETAILED2:
      model = "Detailed2";
      break;
    case py_modrec_manager::DETAILED3:
      model = "Detailed3";
      break;
  }
  return Py_BuildValue("s", model.c_str());
}

static PyObject *
get_vehicle_states(PyObject *self, PyObject *args)
{
  const vcl_vector<modrec_vehicle_state>& states =
      py_modrec_manager::instance()->get_vehicle_states();
  
  PyObject* list = PyList_New(states.size());
  for(unsigned int i=0; i<states.size(); ++i)
  {
    const modrec_vehicle_state& s = states[i];
    PyObject* dict = PyDict_New();
    PyDict_SetItemString(dict,"translation",Py_BuildValue("(ddd)",s.translation.x(), 
                                                                  s.translation.y(),
                                                                  s.translation.z()));
    vnl_vector_fixed<double,3> r = s.rotation.as_rodrigues();
    PyDict_SetItemString(dict,"rotation",Py_BuildValue("(ddd)",r[0],r[1],r[2]));
    PyObject* plist = PyList_New(s.params.size());
    for(unsigned int j=0; j<s.params.size(); ++j)
      PyList_SetItem(plist,j,Py_BuildValue("d",s.params[j]));
    PyDict_SetItemString(dict,"params",plist);
    PyDict_SetItemString(dict,"ang_vel",Py_BuildValue("d",s.a_velocity));
    PyDict_SetItemString(dict,"trans_vel",Py_BuildValue("d",s.t_velocity));
    PyDict_SetItemString(dict,"id",Py_BuildValue("i",s.unique_id));
    PyList_SetItem(list,i,dict);
  }

  return list;
}

static PyObject *
compute_edgel_coverage(PyObject *self, PyObject *args)
{
  double dist_thresh = 1.0;
  if (!PyArg_ParseTuple(args, "d:compute_edgel_coverage", &dist_thresh))
    return NULL;
  
  unsigned int ncm,nct,npm,npt;
  py_modrec_manager::instance()->compute_edgel_coverage(dist_thresh,ncm,nct,npm,npt);

  return Py_BuildValue("(iiii)", ncm,nct,npm,npt);
}

static PyObject *
relative_coverage(PyObject *self, PyObject *args)
{
  double dist_thresh = 1.0;
  if (!PyArg_ParseTuple(args, "d:relative_coverage", &dist_thresh))
    return NULL;
  
  unsigned int nm,np,nt;
  py_modrec_manager::instance()->compare_relative_coverage(dist_thresh,nm,np,nt);
  
  return Py_BuildValue("(iii)", nm,np,nt);
}


//: Define all the python functions
static PyMethodDef py_modrec_methods[] = {
  {"set_camera", set_camera, METH_VARARGS,
   "set_camera(i,path) loads a camera file to frame i"},
  {"set_image", set_image, METH_VARARGS,
   "set_image(i,path) loads an image file to frame i"},
  {"set_video", set_video, METH_VARARGS,
   "set_video(path) loads a video file"},
  {"reset_views", reset_views, METH_VARARGS,
   "reset_views() clears all images and cameras"},
  {"set_parts", set_parts, METH_VARARGS,
   "set_parts(path) load the vehicle parts file"},
  {"set_pca", set_pca, METH_VARARGS,
   "set_pca(path) load the vehicle pca file"},
  {"set_mesh", set_mesh, METH_VARARGS,
   "set_mesh(path) load a mesh from a file"},
  {"set_vehicle_model", set_vehicle_model, METH_VARARGS,
   "set_vehicle_model(type_string) set the type of vehicle model"},
  {"set_options", set_options, METH_VARARGS,
   "set_options([]) indicate which parameters to optimize"},
  {"set_init_uncert", set_init_uncert, METH_VARARGS,
   "set_init_uncert(d) set the initial uncertainty in the params"},
  {"set_translation", set_translation, METH_VARARGS,
   "set_translation(t) sets the translation vector"},
  {"set_rotation", set_rotation, METH_VARARGS,
   "set_rotation(r) sets the rotation Rodrigues vector"},
  {"set_params", set_params, METH_VARARGS,
   "set_params([]) sets the list of top PCA parameters"},
  {"set_lambda", set_lambda, METH_VARARGS,
   "set_lambda(d) set the regularization factor"},
  {"set_fit_mode", set_fit_mode, METH_VARARGS,
   "set_fit_mode(s) set the fitting mode to video or multiview"},
  {"fit_model", fit_model, METH_VARARGS,
   "fit_model() fits the vehicle model to the data"},
  {"write_svg_curves", write_svg_curves, METH_VARARGS,
   "write_svg_curves(path,i) write the projection of curves as SVG"},
  {"enable_tracking", enable_tracking, METH_VARARGS,
   "enable_tracking(b) enable tracking"},
  {"set_track_with_silhouette", set_track_with_silhouette, METH_VARARGS,
   "set_track_with_silhouette(b) use the silhouette in tracking"},
  {"video_seek", video_seek, METH_VARARGS,
   "video_seek(i) seek to a frame of video"},
  {"advance_video", advance_video, METH_VARARGS,
   "advance_video() run one frame of tracking"},
  {"get_current_frame", get_current_frame, METH_VARARGS,
   "get_current_frame() returns the current frame number"},
  {"set_truth_mesh", set_truth_mesh, METH_VARARGS,
   "set_truth_mesh() loads a mesh as ground truth"},
  {"compute_error", compute_error, METH_VARARGS,
   "compute_error() compute the RMS error to ground truth"},
  {"evaluate_residual", evaluate_residual, METH_VARARGS,
   "evaluate_residual(d,b) evaluate the residual at the current state"},
  {"get_translation", get_translation, METH_VARARGS,
   "get_translation() returns the translation vector as a tuple"},
  {"get_rotation", get_rotation, METH_VARARGS,
   "get_rotation() returns the rotation Rodrigues vector as a tuple"},
  {"get_params", get_params, METH_VARARGS,
   "get_params() returns a list of PCA parameters"},
  {"get_vehicle_model", get_vehicle_model, METH_VARARGS,
   "get_vehicle_model() get the type of vehicle model"},
  {"get_vehicle_states", get_vehicle_states, METH_VARARGS,
   "get_vehicle_states() get the vehicle tracking states"},
  {"compute_edgel_coverage", compute_edgel_coverage, METH_VARARGS,
   "compute_edgel_coverage(d) computes number of curve point near edgels"},
  {"relative_coverage", relative_coverage, METH_VARARGS,
   "relative_coverage(d) compare poly and mesh edge coverage"}
};


PyMODINIT_FUNC
initpy_modrec(void)
{
  Py_InitModule("py_modrec", py_modrec_methods);
}
