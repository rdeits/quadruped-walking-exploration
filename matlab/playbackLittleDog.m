function playbackLittleDog(sol)

xtraj = littleDogKinematicTrajectoryGenerator(sol, fullfile(getDrakePath(), '..', 'LittleDog'));
dog = LittleDog();
xtraj = xtraj.setOutputFrame(dog.getStateFrame());
vdog = dog.constructVisualizer();
vdog.playback(xtraj, struct('slider', true));

end