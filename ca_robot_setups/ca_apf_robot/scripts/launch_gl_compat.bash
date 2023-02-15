#!/bin/bash

gnome-terminal -- /bin/sh -c 'export SVGA_VGPU10=0;roslaunch app_apf_collision_avoidance ur10e_sim.launch gl_compat:=true; exec bash'

gnome-terminal -- /bin/sh -c 'sleep 4;roslaunch app_apf_collision_avoidance urdf_filter.launch; exec bash'
