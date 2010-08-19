#!/usr/bin/python

#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

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
 


