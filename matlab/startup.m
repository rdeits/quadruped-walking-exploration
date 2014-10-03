path(pathdef)
tbxmanager restorepath
run([getenv('DRC_BASE'), '/software/build/config/drc_control_setup.m'])
javaaddpath('/usr/local/share/java/lcm.jar');
javaaddpath('/home/rdeits/drc/software/build/share/java/lcmtypes_drake.jar');
javaaddpath('/home/rdeits/drc/software/build/share/java/lcmtypes_drc_lcmtypes.jar');

addpath('../build/matlab');
addpath('../drake/systems/robotInterfaces/test');
addpath_drake
addpath_iris
addpath_gurobi