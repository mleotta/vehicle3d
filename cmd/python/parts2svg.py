#!/usr/bin/python

#          Copyright Matthew Leotta 2006 - 2010.
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file ../LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

import os, sys, glob, re

from config import vehicle_cmd_dir

src_path = sys.argv[1]

ext = '.parts'

conv_exec = os.path.join(vehicle_cmd_dir, 'parts2svg')

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



