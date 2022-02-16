planner = Vel_Planner();
model = Model();

model.plot_arm([0 0 0]);
view([0 1 0]);
while 1
    p = ginput3d(1);
    planner.ik3001_numeric(p, model);
end