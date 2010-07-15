
import os, sys, glob, re

src_path = sys.argv[1]

ext = '_mapped.parts'
init_parts = '/data/research/projects/carshape/blender/vehicles/init.parts'


fit_exec = '/projects/lems/bin/release/contrib/mleotta/cmd/mesh/fit_parts'

def make_mesh(inname, outname):
    os.system(fit_exec+' -i \"'+init_parts+'\" -m \"'+inname+'\" -o \"'+outname+'\"')

for root, dirs, files in os.walk(src_path):
    for f in files:
        if f[-len(ext):] == ext:
            base_name = os.path.join(root,f[:-len(ext)])
            out_name = base_name+'.parts'
            in_name = base_name+ext
            print "fitting parts: "+base_name
            make_mesh(in_name, out_name)



