path(pathdef)
tbxmanager restorepath
javaaddpath('/usr/local/share/java/lcm.jar');

addpath('../build/matlab');
addpath('../drake/systems/robotInterfaces/test');
addpath_drake
javaaddpath(fullfile(getDrakePath(), '..', 'build', 'share', 'java', 'lcmtypes_drake.jar'));
addpath_iris
addpath_gurobi