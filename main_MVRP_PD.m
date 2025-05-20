clc
clear all
close all
env_bounds=[0 200 0 200];
numCities=10;
numVehicles=5;
numObstacles=6;
vehicleSpeed=6.5;%modified since example simulation
M=1000;
vehicleCapacity=200;
numProducers=6;
numReceivers=3;
e = [0; 11; 12; 13; 12; 11; 11; 15; 16; 14];
l = [70; 59; 62; 63; 60; 59; 58; 68; 69; 67];%modified since example simulation
[env_bounds,numCities,numVehicles,numObstacles,vehicleSpeed,M,vehicleCapacity,numProducers,numReceivers,e,l]=define_data();
[Obstacles,Cities] = define_environment(env_bounds,numObstacles,numCities);
[Producers,Receivers,isProducer,isReceiver,demand]=location_type(numCities,numProducers,numReceivers); 


%%
 [fig_handle] = extract_producers_receivers_obstacles(env_bounds,numCities,numObstacles,Cities,e,l,isProducer,isReceiver,Obstacles);


fprintf('full\ n')
tic
fprintf('Visibility graph construction time:\n')
tic
[Nodes_coord,Adj_full] = construct_visibility_graph(Obstacles,Cities', fig_handle);
toc
[distanceMatrix,Edge_to_path,ind_red_to_full,travelTimes] = reduce_graph(Adj_full,1:numCities,vehicleSpeed); %reduced graph with first numcityLocations
toc
%%
% ==== OPTIMIZATION PROBLEM ====
tic
[solution,fval,exitflag]=MILP(numCities,numVehicles,distanceMatrix,Producers,Receivers,isReceiver,numProducers,numReceivers,travelTimes,demand,vehicleCapacity,e,l,M);
toc

%%
% ==== VISUALIZE ====
colors = lines(numVehicles);
if exitflag == 1
visualizePaths(Cities,fig_handle,env_bounds,numCities,numVehicles,numObstacles,isProducer,Producers,isReceiver,Receivers,Edge_to_path,Nodes_coord,solution,e,l,colors,Obstacles);
   [arrival_all, load_all] = vehicle_results(numVehicles, solution, isProducer, isReceiver,Producers,Receivers, e, l)
else
    disp('Solver did not converge.');
end
toc
save
