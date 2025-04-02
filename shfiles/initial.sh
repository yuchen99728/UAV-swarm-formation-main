roslaunch command position.launch & sleep 10;
roslaunch plan_env grid_map.launch & sleep 5;
roslaunch ipc ipc_real.launch 
wait;