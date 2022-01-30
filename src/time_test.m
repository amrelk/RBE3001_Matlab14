model = Model();
kine = Kinematics();

tic;
model.plot_arm([0 0 0]);
toc

tic;
kine.fk3001_inter([0 0 0], 2);
kine.fk3001([0 0 0]);
toc
