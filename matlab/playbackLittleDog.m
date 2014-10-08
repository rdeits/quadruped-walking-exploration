function playbackLittleDog(sol)

xtraj = littleDogKinematicTrajectoryGenerator(sol, '~/locomotion/LittleDog');
dog = LittleDog();
xtraj = xtraj.setOutputFrame(dog.getStateFrame());
vdog = dog.constructVisualizer();
vdog.playback(xtraj, struct('slider', true));

end