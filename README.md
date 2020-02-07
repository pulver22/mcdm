# MCDM Online Navigation Framework
In this repo is contained the implementation of an online greedy algorithm for navigation, which combines four criteria in a single utility function to establish which is the next best pose to reach.
These criteria are the followings:
- Travel distance : distance between the current position and the next one
- Information Gain : the amount of new information acquirable from the next pose
- Sensing Time : time required for a sensing operation
- RFIDGain : the amount of radio information present in the map around the next pose

These criteria are combined using the Choquet Fuzzy Integral as aggregation function, because it allows us to model the synergy/redundancy relationship existing among them.


### Usage

After downloading the repo, you need to run:

```sh
$  ./../build/mcdm_online_exploration ./../Images/inbeng_small_correct.pgm 1 72 124 180 26 180 0.99 0 1 ./../config/tag_inbeng_1.yaml 865e6 0 0.5 0.5 0 0 /tmp/result_inbeng.csv /tmp/coverage_mcdm_inbeng.csv /tmp/distance_tag.csv 1 /tmp/accuracy.csv 1
```
where:
- `./../build/mcdm_online_exploration` is the executable file
- `./../Images/inbeng_small_correct.pgm` is the image file you want to use
- `1` means we are using the full resolution of the map, without resizing it
- `72` and `124` are the coordinates of the starting position of the robot
- `180` is the initial orientation of the robot
- `26` is the sensor range expressed in cells
- `180` is the maximum scan angle
- `0.99` means we want the full coverage ( use 0.9X with a lower resolution)
- `0` is the threshold factor for discarding frontiers and speeding up the exploration (us 0.XX for different values)
- `1` means we use cells of one square meter per size during the navigation
- `./../config/tag_inbeng_1.yaml` is the file containing the position of 10 RFID tags
- `865e6` and `0` are the frequency (hz) and trasmitted power (dB) for the RFID scan
- `0.5 0.5 0 0` are the w_info_gain, w_travel_distance, w_sensing_time, w_rfid_gain respectively
- `/tmp/result_inbeng.csv` is the path where to save the final result
- `/tmp/coverage_mcdm_inbeng.csv` is the path where to save increasing coverage results
- `/tmp/distance_tag.csv` is the path where to save the distance from each RFID tags
- `1` use for building the ellipse in the radiation model 
- `/tmp/accuracy.csv` is the path where to save detection accuracy results
- `1` boolean to use mcdm (1) or the weighted_average (0) as aggregation function

### MAPS DETAILS
#### 1) [OREBRO TEKHNIKHUSET](./Images/cor_map_05_00_new1.pgm)
`./mcdm_online_exploration ./../Images/cor_map_05_00_new1.pgm 1 99 99 180 5 180 1 0 1 54 143 865e6 0`
-   Map size: 200x200
-   Resolution: 0.5
-   Robot starting position: (99,99) \[top right corner]
-   RFID tag: (54,143) \[around the bottom left corner, just after the black space]


#### 2) [INB3123](./Images/inb3123_1m4px.pgm)
`./mcdm_online_exploration ./../Images/inb3123_1m4px.pgm 1 300 300 180 12 180 1 0 1 278 87 865e6 0`
-   Map size: 500x500
-   Resolution: 0.025000
-   Robot starting position: (300,300) \[ fifth corridor starting from the top]
-   RFID tag: (278,87) \[at the junction of the second corridor]


#### 3) [INB_ENGINEERING](./Images/inb3123_1m4px.pgm)
`./mcdm_online_exploration ./../Images/inb_eng_1m4px.pgm 1 318 411 180 12 180 1 0 1 174 263 865e6 0`
-   Map size: 500x500
-   Resolution: 0.025000
-   Robot starting position: (318,411) \[at the beginning of the bottom corridor]
-   RFID tag: (174,263) \[inside the small corridor on the left side of the map]


#### 4) INB_ATRIUM
`./mcdm_online_exploration ./../Images/XXX.pgm 1 XX XX 180 5 180 1 0 1 XX XX 865e6 0`
-   Map size:
-   Resolution:
-   Robot starting position: (,) \[]
-   RFID tag: (,) \[]


#### 5) [ILIAD](./Images/ncfm_old.pgm)
`./mcdm_online_exploration ./../Images/ncfm_old.pgm 1 132 345 180 5 180 1 0 1 294 305 865e6 0`
-   Map size: 500x500
-   Resolution: 0.1
-   Robot starting position: (132,345) \[inside the small room on the bottom-left]
-   RFID tag: (294,305) \[on the top of the black square on the right of the map]
