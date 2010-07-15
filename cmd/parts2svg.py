
import os, sys, glob, re

src_path = sys.argv[1]

ext = '.parts'

conv_exec = '/projects/lems/bin/Release/contrib/mleotta/cmd/mesh/parts2svg'

def convert(inname, outname):
    os.system(conv_exec+' -i \"'+inname+'\" -o \"'+outname+'\"')

for root, dirs, files in os.walk(src_path):
    for f in files:
        if f[-len(ext):] == ext:
            base_name = os.path.join(root,f[:-len(ext)])
            out_name = base_name+'.svg'
            in_name = base_name+ext
            print "converting: "+base_name
            convert(in_name, out_name)



