#!/usr/bin/python

#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

import os, sys, glob, re

from config import vehicle_cmd_dir, vehicle_data_dir

src_path = sys.argv[1]

ext = '_pca.obj'


fit_exec = os.path.join(vehicle_cmd_dir, 'fit_mesh_body')
pca_exec = os.path.join(vehicle_cmd_dir, 'pca_fit_mesh_body')
pca_data = os.path.join(vehicle_data_dir, 'pca_mesh.pca')

def fit_mesh(inname, basename, outname):
    os.system(fit_exec+' -i \"'+inname+'\" -b \"'+basename+'\" -o \"'+outname+'\"')

def pca_mesh(inname, outname):
    os.system(pca_exec+' -i \"'+inname+'\" -o \"'+outname+'\" -p '+pca_data)

for root, dirs, files in os.walk(src_path):
    for f in files:
        if f[-len(ext):] == ext:
            base_name = os.path.join(root,f[:-len(ext)])
            out_name = base_name+'_pca.obj'
            body_name = base_name+'_body.obj'
            in_name = base_name+'_pca.obj'
            print "fitting mesh: "+base_name
            pca_mesh(in_name, out_name)
            fit_mesh(out_name, body_name, out_name)
 


