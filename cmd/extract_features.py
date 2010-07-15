
import os, sys, glob, re

src_path = sys.argv[1]

ext = '_body.obj'


extract_exec = '/projects/lems/bin/release/contrib/mleotta/cmd/mesh/mesh_extract_features'
part_file = '/data/research/projects/carshape/part_names.txt'

def extract_features(inname, basename, outname, svgname):
    os.system(extract_exec+' -i \"'+inname+'\" -b \"'+basename+'\" -p '+part_file+' -o \"'+outname+'\" -s \"'+svgname+'\" -g body')

for root, dirs, files in os.walk(src_path):
    for f in files:
        if f[-len(ext):] == ext:
            base_name = os.path.join(root,f[:-len(ext)])
            out_name = base_name+'_mapped.parts'
            svg_name = base_name+'_parts.svg'
            body_name = base_name+'_body.obj'
            in_name = base_name+'_m3.obj'
            print "fitting mesh: "+base_name
            extract_features(in_name, body_name, out_name, svg_name)
 


