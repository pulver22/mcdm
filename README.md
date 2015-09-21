# MCDM Online Navigation Framework
In this repo is contained the implementation of an online greedy algorithm for navigation, which combined three ctriteria in a single utility function to establish which is the next best pose to reach.
These criteria are the followings:
- Travel distance : distance from the goal
- Information Gain : the amount of new information acquirable from the goal
- Sensing Time : time required for a sensing operation

These criteria are combined using the Choquet Fuzzy Integral as aggregation function, because it allow us to model the sinergy/redundancy relationship existing among them.


### Usage

After downloading the repo, you need to run:

```sh
$  ./mcdm_online_exploration_ros ./../Maps/map_RiccardoFreiburg_1m2.pgm 100 75 5 0 15 180 1 0 1
```
where:
- ./mcdm_online_exploration_ros is the executable file
- ./../Maps/map_RiccardoFreiburg_1m2.pgm is the image file you want to use
- 100 meanse we are using the full resolution of the map, without resizing it
- 75 and 5 are the coordinates of the starting position of the robot
- 0 is the initial orientation of the robot
- 15 is the sensor range
- 180 is the maximum scan angle
- 1 means we want the full coverage ( use 0.9X with a lower resolution)
- 0 is the threshold factor to speed up the exploration (us 0.XX for different values)
- 1 means we use cells of one square meter per size during the navigation
