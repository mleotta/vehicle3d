#!/usr/bin/python
##########################################################################
# This is mleotta/gui/pca_vehicle/modrec_vehicle_fit.py
##########################################################################

import os.path
import getopt, sys
from math import *


img_path = "/Users/mleotta/research/projects/carshape/real_images/have_model/"
pca_path = "/projects/lems/src/contrib/mleotta/gui/pca_vehicle/"
geom_path = "/Users/mleotta/research/projects/carshape/blender/vehicles/training/"
results_path = "/Users/mleotta/research/projects/carshape/experiments/"
video_path = "/Users/mleotta/data/video/"
rand_pose_path = "/Users/mleotta/research/projects/carshape/run_experiments/rand_pose.txt"
rand_gnd_pose_path = "/Users/mleotta/research/projects/carshape/run_experiments/rand_gnd_pose.txt"


# dictionary of vehicle ground truth models
truth_model = {"Dodge_Stratus"    : "sedan4/Dodge_Stratus_99_E3D_ver1",
               "VW_Beetle_yellow" : "sedan2/VW_Beetle_1998_E3D_ver1",
               "VW_Beetle_black"  : "sedan2/VW_Beetle_1998_E3D_ver1",
               "Dodge_Caravan"    : "/minivan/Dodge_Caravan_96_E3D_ver1",
               #"Volvo_V40"        : "wagon/Volvo_V40_w_Interior_2000_E3D_ver1",
               "Volvo_V70XC_blue" : "wagon/Volvo_V70XC_w_Undercarriage_01_E3D_ver1",
               #"Volvo_V70XC_gray" : "wagon/Volvo_V70XC_w_Undercarriage_01_E3D_ver1",
               "Toyota_4Runner"   : "suv/Toyota_4Runner_1999_E3D_ver1",
               "Chevy_S10_Blazer" : "suv/Chevy_S10_Blazer_1998_ED3_ver1",
               "Toyota_Tundra"    : "pickup/Toyota_Tundra_w_Subframe_Pickup_2000_E3D_ver1",
               "Nissan_Maxima"    : "sedan4/Nissan_Maxima_GLE_99_E3D_ver1",
               # --- below here there are no corresponding image sets ---
               "PT_Cruiser"       : "sedan4/Chrysler_PT_Cruiser_2001_E3D_ver1",
               "VW_Beetle"        : "sedan2/VW_Beetle_1998_E3D_ver1",
               "MarkII"           : "sedan4/Toyota_Mark_II_w_Interior_97_E3D_ver1",
               "Trailblazer"      : "suv/Chevy_Trailblazer_2001_E3D_ver1",
               "Grand_Cherokee"   : "suv/Jeep_Grand_Cherokee_94_E3D_ver1",
               "Cavalier98"       : "sedan2/Chevy_Cavalier_98_2DR_E3D_ver1",
               "Catera"           : "sedan4/Cadillac_Catera_99_E3D_ver1",
               "Grand_Caravan"    : "minivan/Dodge_Grand_Caravan_ES_2001_E3D_ver1"}

# dictionary of vehicle image paths
v_img_path = {"Dodge_Stratus"    : "sedan4/Dodge_Stratus_99/",
              "VW_Beetle_yellow" : "sedan2/VW_Beetle_1998/yellow/",
              "VW_Beetle_black"  : "sedan2/VW_Beetle_1998/black/",
              "Dodge_Caravan"    : "minivan/Dodge_Caravan_96/",
              #"Volvo_V40"        : "wagon/Volvo_V40_2000/",
              "Volvo_V70XC_blue" : "wagon/Volvo_V70XC_01/blue/",
              #"Volvo_V70XC_gray" : "wagon/Volvo_V70XC_01/gray/",
              "Toyota_4Runner"   : "suv/Toyota_4Runner_1999/",
              "Chevy_S10_Blazer" : "suv/Chevy_S10_Blazer_1998/",
              "Toyota_Tundra"    : "pickup/Toyota_Tundra_2000/",
              "Nissan_Maxima"    : "sedan4/Nissan_Maxima_GLE_99/gray/"}
              
# dictionary of model type postfixes
truth_postfix = {"Dodecahedral"  : "_dodec.obj",
                 "Ferryman"      : "_ferryman.obj",
                 "Detailed1"     : "_m1.obj",
                 "Detailed2"     : "_m2.obj",
                 "Detailed3"     : "_m3.obj"}
                 
# dictionary of videos
video_files = {"westin" : "westin/7_6_06/raw/people_full.avi",
               "brook"  : "intersections/2_26_09/brook_george.avi",
               "waterman": "intersections/7_10_09/waterman_brown.avi",
               "waterman2": "intersections/7_10_09/waterman_brown.avi"}
          
# dictionary of videos camera
video_cameras = {"westin" : "westin/7_6_06/raw/people_full.cam",
                 "brook"  : "intersections/2_26_09/brook_george.cam",
                 "waterman": "intersections/7_10_09/waterman_brown_s.cam",
                 "waterman2": "intersections/7_10_09/waterman_brown_s_old.cam"}
                 
# vector of image scale paths
img_scale = ["", "halfsize/", "quartersize/", "eighthsize/", "sixteenthsize/", "32ndsize/"]


# load the ground truth model
def load_truth(vehicle):
  print "--loading ground truth mesh--"
  py_modrec.set_truth_mesh(geom_path+truth_model[vehicle]+"_m3.obj") 

# load the image and camera data from files
def load_data(vehicle, s, views):
  print "--loading images and cameras--"
  curr_img_path = img_path+v_img_path[vehicle]
  opened = []
  py_modrec.reset_views()
  for i in views:
    img_file = curr_img_path+img_scale[s]+("%05d.jpg" % (i+1))
    cam_file = curr_img_path+("%05d.cam" % (i+1))
    if os.path.isfile(img_file) and os.path.isfile(cam_file):
      print ' ', i, img_file
      py_modrec.set_image(i,img_file)
      py_modrec.set_camera(i,cam_file)
      opened.append(i);
    else:
      break
  return opened
  
# load video data
def load_video_data(video):
  print "--loading video and camera--"
  py_modrec.set_fit_mode("video")
  py_modrec.reset_views()
  vid_file = video_path+video_files[video]
  cam_file = video_path+video_cameras[video]
  if os.path.isfile(vid_file) and os.path.isfile(cam_file):
    print ' ', vid_file
    py_modrec.set_video(vid_file)
    py_modrec.set_camera(0,cam_file)
  else:
    print "video or camera file not found"
    print vid_file
    print cam_file

  
# load the random pose initialization
def load_rand_pose(pose_path):
  file = open(pose_path,'r')
  poses = [];
  for line in file:
    words = line.split()
    nums = []
    for word in words:
      nums.append(float(word))
    if len(nums) == 3:
      nums = nums[0:2]+[0.0]*3+[nums[2]]
    poses.append(nums)
  return poses


# fit with this initial pose
def fit(R,t,pc):
  py_modrec.set_translation(t)
  py_modrec.set_rotation(R)
  py_modrec.set_params(pc) 
  e0 = py_modrec.compute_error()
  py_modrec.set_lambda(1.0)
  py_modrec.fit_model(50)
  e1 = py_modrec.compute_error()
  t = py_modrec.get_translation()
  R = py_modrec.get_rotation()
  pc = py_modrec.get_params()
  return ((e0,e1), R, t, pc)
  
  
# write a line of results to the file
def write_results(f, E, R, t, R2, t2, pc):
  f.write(repr(E[0])+' '+repr(E[1])+' ')
  f.write(repr(R[0])+' '+repr(R[1])+' '+repr(R[2])+' ')
  f.write(repr(t[0])+' '+repr(t[1])+' '+repr(t[2])+' ')
  f.write(repr(R2[0])+' '+repr(R2[1])+' '+repr(R2[2])+' ')
  f.write(repr(t2[0])+' '+repr(t2[1])+' '+repr(t2[2])+' ')
  for p in pc:
    f.write(repr(p)+' ')
  f.write('\n')


# fit the model with many sample points of initialization
def fit_in_region(vehicle,scale,options,views):
  py_modrec.set_options(options)
  
  load_truth(vehicle)
  load_data(vehicle, scale,views)
  model = py_modrec.get_vehicle_model()
  rfile = results_path + 'region_' + vehicle + '_'+model+'_s'+str(scale)+'_pc5.txt'
  print "Writting output to:", rfile
  f=open(rfile,'w')
  for i in range(-20,21):
    t = (i/10.0,0,0)
    for j in range(-60,61,3):
      R = (0,0,j*pi/180.0)
      print "fit starting at", i, j
      (E, R2, t2, pc) = fit(R,t,[])
      write_results(f, E, R, t, R2, t2, pc)

  f.close()
  
def find_t_range(ti,thresh):
  R = (0,0,0)
  t = (0,0,0)
  for i in range(1,41):
    t = t[:ti]+(i/20.0,)+t[ti+1:]
    (E, R2, t2, pc) = fit(R,t,[])
    if E[1] > thresh:
      break
  pmax = (i-1)/20.0 
  for i in range(1,41):
    t = t[:ti]+(-i/20.0,)+t[ti+1:]
    (E, R2, t2, pc) = fit(R,t,[])
    if E[1] > thresh:
      break 
  pmin = (-i+1)/20.0
  return pmax-pmin
  
def find_R_range(Ri,thresh):
  R = (0,0,0)
  t = (0,0,0)
  for i in range(1,90):
    R = R[:Ri]+(i*pi/180,)+R[Ri+1:]
    (E, R2, t2, pc) = fit(R,t,[])
    if E[1] > thresh:
      break
  pmax = i-1; 
  for i in range(1,90):
    R = R[:Ri]+(-i*pi/180,)+R[Ri+1:]
    (E, R2, t2, pc) = fit(R,t,[])
    if E[1] > thresh:
      break 
  pmin = -i+1; 
  return pmax-pmin
    
# compute the data for the CVPR paper table
def find_convergence(vehicle,scale,options,views):
  py_modrec.set_options(options)
  load_truth(vehicle)
  opened = load_data(vehicle, scale, views)
  pc = py_modrec.get_params()
  
  R = (0,0,0)
  t = (0,0,0)
  (E, R2, t2, pc) = fit(R,t,[])
    
  #EC = py_modrec.compute_edgel_coverage(1.0)
  #cov_pct = float(EC[0]+EC[2])/(EC[1]+EC[3])*100.0;
  #print "coverage percent is", cov_pct
  #return 
  
  rfile = results_path + vehicle + '_s'+str(scale)+'_v'+str(len(opened))+'_pc'+str(len(pc))
  if len(options) == 4:
    rfile += '_gnd'
  print "Writing output to:", rfile+'_cvg.txt'
  f=open(rfile+'_cvg.txt','w')
  f.write("Vehicle: "+vehicle+"\n")
  f.write("Options: "+str(options)+"\n")
  f.write("Scale: "+str(scale)+"\n")
  f.write("Views: "+str(opened)+"\n\n")
  
  f.write("Error: "+str(E)+"\n")
  f.write("R: "+str(R2)+"\n")
  f.write("t: "+str(t2)+"\n")
  f.write("PC: "+str(pc)+"\n")
  #f.write("edgel coverage: "+str(EC)+" => "+str(cov_pct)+"\n")
  
  t0_range = find_t_range(0, 0.1)
  t1_range = find_t_range(1, 0.1)
  t2_range = find_t_range(2, 0.1)
  f.write("t range: "+str(t0_range)+" ")
  f.write(str(t1_range)+" ")
  f.write(str(t2_range)+"\n")
  
  R0_range = find_R_range(0, 0.1)
  R1_range = find_R_range(1, 0.1)
  R2_range = find_R_range(2, 0.1)
  f.write("R range: "+str(R0_range)+" ")
  f.write(str(R1_range)+" ")
  f.write(str(R2_range)+"\n")
  
  f.close()
  
  
# compute the fit with several predetermined random initial poses
def fit_random_pose(vehicle,scale,options,views):
  py_modrec.set_options(options)
  load_truth(vehicle)
  opened = load_data(vehicle, scale, views)
  pc = py_modrec.get_params()
  
  model_type = py_modrec.get_vehicle_model()
  partsfile = ""
  if model_type[:-1] == "Detailed":
    partsfile = geom_path+truth_model[vehicle]+'.parts'
  print 'parts file = ', partsfile
  py_modrec.set_mesh(geom_path+truth_model[vehicle]+truth_postfix[model_type], partsfile)
  pc = py_modrec.get_params("all")
  num_pc = options[-1]
  print "pc = ", pc[:num_pc] 
  
  pose_path = rand_pose_path
  if len(options) == 4:
    pose_path = rand_gnd_pose_path
  poses = load_rand_pose(pose_path)
  
  rfile = results_path + 'rand_' + vehicle + '_' +model_type + '_s'+str(scale)+'_v'+str(len(opened))+'_'+str(views[0])+'_pc'+str(num_pc)
  if len(options) == 4:
    rfile += '_gnd'
  print "Writing output to:", rfile+'.txt'
  f=open(rfile+'.txt','w')
  
  R = (0,0,0)
  t = (0,0,0)
  py_modrec.set_init_uncert(0.25)
  (E, R2, t2, pc) = fit(R,t,pc[:num_pc])
  write_results(f, E, R, t, R2, t2, pc)
  
  py_modrec.set_init_uncert(1.0)
  for pose in poses:
    t = pose[0:3]
    R = pose[3:6]
    (E, R2, t2, pc) = fit(R,t,[])
    write_results(f, E, R, t, R2, t2, pc)
  
  f.close()
  
  
# compute the data for the CVPR paper table
def make_svg(vehicle,scale,options,views):
  py_modrec.set_options(options)
  opened = load_data(vehicle, scale, views)
  pc = py_modrec.get_params()
  
  R = (0,0,0)
  t = (0,0,0)
  py_modrec.set_translation(t)
  py_modrec.set_rotation(R)
  py_modrec.set_params([]) 
  py_modrec.set_lambda(1.0)
  py_modrec.fit_model(50)
      
  rfile = results_path + vehicle + '_s'+str(scale)+'_v'+str(len(opened))+'_pc'+str(len(pc))
  if len(options) == 4:
    rfile += '_gnd'  
  py_modrec.write_svg_curves(views[0],rfile+'_'+str(opened[0]+1)+'.svg')
  
  
# compute error for each number of params 0-15
def find_best_num_params(vehicle,scale):
  load_truth(vehicle)
  opened = load_data(vehicle, scale, range(10))
  model_type = py_modrec.get_vehicle_model()
  py_modrec.set_mesh(geom_path+truth_model[vehicle]+truth_postfix[model_type])
  pc = py_modrec.get_params("all")
  options = ['tx','ty','tz','rx','ry','rz']# ['tx','ty','rz']
  
  py_modrec.set_params([])
  E = []
  for i in range(min(len(pc),16)):
    py_modrec.set_options(options+[i])
    py_modrec.set_translation((0,0,0))
    py_modrec.set_rotation((0,0,0))
    py_modrec.set_params([])#pc[:i]) 

    py_modrec.set_lambda(1.0)
    py_modrec.fit_model(50)
    E.append(py_modrec.compute_error())
  return E

  
# compute error for each number of params 1-20
def find_best_num_params_all(scale,model_type):
  E = []
  for v in sorted(truth_model.keys()):
    print "running on ", v
    E.append(find_best_num_params(v,scale))
    
  rfile = results_path + 'num_pca' + '_s'+str(scale)+'_'+model_type+'.txt'
  print "Writting output to:", rfile
  f=open(rfile,'w')
  
  names = sorted(truth_model.keys());
  for i in range(len(E)):
    f.write(names[i]+" ")
    for j in E[i]:
      f.write(str(j)+" ")
    f.write("\n");
    
    
# running tracking in video
def track_vehicles(vehicle,video,sf,nf,options,model_type,track_with_silhouette):
  load_video_data(video)
  py_modrec.set_options(options)
  py_modrec.set_track_with_silhouette(track_with_silhouette)
  print "setting track with silhouette ",track_with_silhouette
  
  num_pc = options[-1]
  
  if num_pc == 0:
    model_type = py_modrec.get_vehicle_model()
    partsfile = ""
    if model_type[:-1] == "Detailed":
      partsfile = geom_path+truth_model[vehicle]+'.parts'
    print 'parts file = ', partsfile
    py_modrec.set_mesh(geom_path+truth_model[vehicle]+truth_postfix[model_type], partsfile)
    pc = py_modrec.get_params("all")
  
  gnd = ''
  if len(options)==4:
    gnd = '_gnd'
  tws = ''
  if track_with_silhouette:
    tws = '_tws'
  rfile = results_path + 'track_'+model_type+'_'+video+'_sf'+str(sf)+'_nf'+str(nf)+'_pc'+str(num_pc)+gnd+tws+'.txt'
  f=open(rfile,'w')
  f.write(model_type+'\n')
  f.write(str(sf)+'\n')
  f.write(video_path+video_files[video]+'\n')
  f.write(video_path+video_cameras[video]+'\n')
  
  # prime the BG model with 100 frames
  num_bg_prime = 100
  py_modrec.enable_tracking(False)
  if num_bg_prime > sf:
    num_bg_prime = sf
  py_modrec.video_seek(sf-num_bg_prime)
  for i in range(num_bg_prime):
    print "Current Frame: ",py_modrec.get_current_frame()
    py_modrec.advance_video()
  py_modrec.enable_tracking(True)
  
  for i in range(nf):
    py_modrec.advance_video()
    frame = py_modrec.get_current_frame()
    print "Current Frame: ",py_modrec.get_current_frame()
    states = py_modrec.get_vehicle_states()
    if not states:
      continue
    
    for st in states:
      f.write(str(frame)+' ')
      f.write(str(st['id'])+' ')
      t = st['translation']
      f.write(str(t[0])+' '+str(t[1])+' '+str(t[2])+' ')
      r = st['rotation']
      f.write(str(r[0])+' '+str(r[1])+' '+str(r[2])+' ')
      f.write(str(st['trans_vel'])+' ')
      f.write(str(st['ang_vel'])+' ')
      params = st['params']
      for p in params:
        f.write(str(p)+' ')
      f.write('\n')

    
    
# sample the residual surface for later plotting
def residual_surface(vehicle,scale,views):
    opened = load_data(vehicle, scale, views)
    py_modrec.set_options([2])
    py_modrec.set_translation((0,0,0))
    py_modrec.set_rotation((0,0,0))
    py_modrec.set_params([])
    model_type = py_modrec.get_vehicle_model()
    
    img_scale = 8.0;
    
    rfile = results_path + 'rsurf_'+vehicle+'_'+model_type+'_s'+str(scale)+'_is'+str(int(img_scale))+'.txt'
    print "Writting output to:", rfile
    f=open(rfile,'w')
    res = py_modrec.evaluate_residual(img_scale,True)
    
    r1 = range(-50,51)
    for i in r1:
      for j in r1:
        print "setting params", [i/100.0,j/100.0]
        py_modrec.set_params([i/100.0,j/100.0])
        res = py_modrec.evaluate_residual(img_scale,False)
        f.write(str(res)+' ')
      f.write('\n')
    f.close()
    
    
# compute edge coverage
def edge_coverage(vehicle,scale,views):
  opened = load_data(vehicle, scale, views)
  py_modrec.set_mesh(geom_path+truth_model[vehicle]+"_poly.obj","poly")
  py_modrec.set_parts(geom_path+truth_model[vehicle]+".parts")
  E = []
  for i in range(40):
    (m1,m2,total) = py_modrec.relative_coverage(float(i)/8.0)
    E.append( (float(i)/8.0, float(m1)/total, float(m2)/total) )
  for i in range(40):
    print E[i][0], E[i][1], E[i][2]

  
def init_modrec(build_type):
  sys.path.append("/projects/lems/bin/"+build_type+"/lib/")
  global py_modrec
  py_modrec = __import__("py_modrec")
  if build_type == "Debug":
    print "Attach Debugger to", os.getpid()
    raw_input("Press Enter")
  print "--loading mesh parts and PCA parameters--" 
  py_modrec.set_vehicle_model("Dodecahedral")
  py_modrec.set_pca(pca_path+"default_dodec.pca")
  py_modrec.set_vehicle_model("Ferryman")
  py_modrec.set_pca(pca_path+"default_ferryman.pca")
  py_modrec.set_vehicle_model("Detailed1")
  py_modrec.set_parts(pca_path+"default.parts")
  py_modrec.set_pca(pca_path+"default1.pca")
  py_modrec.set_vehicle_model("Detailed2")
  py_modrec.set_parts(pca_path+"default.parts")
  py_modrec.set_pca(pca_path+"default2.pca")
  py_modrec.set_vehicle_model("Detailed3")
  py_modrec.set_parts(pca_path+"default.parts")
  py_modrec.set_pca(pca_path+"default3.pca")


# Call the main function if run from the command line
if __name__ == "__main__":
  try:
    opts, args = getopt.getopt(sys.argv[1:], "dglv:s:c:m:n:f:w:i:b:t:")
  except getopt.GetoptError, err:
    # print help info and exit
    print str(err) # will print error message
    usage()
    sys.exit(2)
  
  # Defaults
  build_type = "Release"
  vehicle = "Dodge_Stratus"
  model_type = "Detailed3"
  func = "Convergence"
  video = "westin"
  track_with_silhouette = False
  start_frame = 0
  num_frames = 150
  scale = 3
  num_pc = 5
  options = ['tx','ty','tz','rx','ry','rz']
  #options = ['tx','ty','rz',10]
  views = [0,1,2,3,4,5,6,7]
  # Handle arguments
  for o, a in opts:
    if o == "-d":
      build_type = "Debug"
    elif o == "-g":
      options = ['tx','ty','rz']
    elif o == "-v":
      vehicle = a
    elif o == "-s":
      scale = int(a)
    elif o == "-m":
      model_type = a
    elif o == "-n":
      num_pc = int(a)
    elif o == "-f":
      func = a
    elif o == "-w":
      views = eval(a);
    elif o == "-i":
      video = a
    elif o == "-b":
      start_frame = int(a)
    elif o == "-t":
      num_frames = int(a)
    elif o == "-l":
      track_with_silhouette = True
    else:
      assert False, "unhandled option"
      
      
  init_modrec(build_type)
  
  options.append(num_pc)
  if not py_modrec.set_vehicle_model(model_type):
    print "unable to set model type", model_type
  
  #fit_in_region(vehicle, scale,views)
  if func == "Convergence":
    find_convergence(vehicle,scale,options,views)
  elif func == "Num_PCA":
    find_best_num_params_all(scale,model_type)
  elif func == "SVG":
    make_svg(vehicle,scale,options,views)
  elif func == "Region":
    fit_in_region(vehicle,scale,options,views)
  elif func == "Rand":
    fit_random_pose(vehicle,scale,options,views)
  elif func == "Residual":
    residual_surface(vehicle,scale,views)
  elif func == "Track":
    track_vehicles(vehicle,video,start_frame,num_frames,options,model_type,track_with_silhouette)
else:
  init_modrec("Release")


