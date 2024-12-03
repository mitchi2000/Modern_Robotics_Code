this is my attempt for the week 2 assignment of course 4 of Modern Robotics 
in this assignment I chose to use PRM as my sampling method.


this code start from the bottom at first i define script director,robot width
the C space,number of nodes number of neighbors and where to start the search and where to end
it, then we start calling the functioons one by one we start by getting the obstacle list
then we use that to create the randomly chosen nodes that are collision free after that we connect the edges
then we filter out these edges so we can get the ones that are collision free


after all that is done we should get .csv files with all the information we need to start
 the search with the least cost and we do that by implementing the assignment we did last time

 finally we get a path.csv file that has the crrect path which we can implement in our
 coppeslim simulation to observe the robot completing it's trajectory