
import os, sys, glob, re

src_path = sys.argv[1]

ext = '_body.obj'

make_vehicle_exec = '/projects/lems/bin/debug/contrib/mleotta/cmd/mesh/make_vehicle_mesh'

def make_mesh(pname, outname):
    os.system(make_vehicle_exec+' -i \"'+pname+'\" -o \"'+outname+'\" -f')

for root, dirs, files in os.walk(src_path):
    for f in files:
        if f[-len(ext):] == ext:
            base_name = os.path.join(root,f[:-len(ext)])
            param_name = base_name+'.params'
            out_name = base_name+'_ferryman.obj'
            print "makeing mesh: "+base_name
            make_mesh(param_name, out_name)
 


