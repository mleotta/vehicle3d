#!/usr/bin/python

#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

import os, sys, glob, re

src_path = sys.argv[1]

ext = '_body.obj'


fit_exec = '/projects/lems/bin/release/contrib/mleotta/cmd/mesh/fit_mesh_body'

def make_mesh(inname, basename, outname):
    os.system(fit_exec+' -i \"'+inname+'\" -b \"'+basename+'\" -o \"'+outname+'\" -subdiv 3')

for root, dirs, files in os.walk(src_path):
    for f in files:
        if f[-len(ext):] == ext:
            base_name = os.path.join(root,f[:-len(ext)])
            out_name = base_name+'_m%u.obj'
            body_name = base_name+'_body.obj'
            in_name = base_name+'_m0.obj'
            print "fitting mesh: "+base_name
            make_mesh(in_name, body_name, out_name)
 


