This software is the implementation accompanying the Ph.D. thesis of Matthew J.
Leotta, "Generic, Deformable Models for 3-D Vehicle Surveillance", Brown
University, May 2010.

For more information please visit the project website:
http://www.lems.brown.edu/~mleotta/project/defvehicle3d.php
This website has links to download the thesis and datasets used in the thesis.


Top level directory structure:

  cmake  CMake modules directory.

  cmd    Command line utilities. These are primarily used for training the
         deformable model. The python subdirectory provides simple scripts to
         run these utilities over batches of files.

  data   The learned data files. This includes PCA data files and a parts file.

  dml    Deformable Model Library. This library contains the all the functions
         and classes used in optimizing the vehicle shape from multiple views
         or video. The python subdirectory builds a module to provide a python
         interface to run experiments.

  gui    Graphical User Interface. This directory contains wxWidgets based GUI
         for running experiments and visual debugging.

  klt    Kanade-Lucas-Tomasi Feature Tracker. This is 3rd party code written by
         Stan Birchfield.  It has its own license, see the klt/README file.

  spl    Stream Processing Library.  This library provides a video stream
         processing framework.

