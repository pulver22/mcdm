w_sensing_time=0.33
sum_weights=1.0
max_w=$(echo "($sum_weights - $w_sensing_time)" | bc -l)
increment=0.01
starting_w=0.00
# for w_info_gain in `seq -f "%f" 0.0 0.01 0.66`
# do
#     counter=0
#     start_loop=$(echo "(0.66 - $w_info_gain)" | bc -l)
#     for w_travel_distance in `seq -f "%f" $start_loop -0.01 0.0`
#     do
    
#         echo "-----------------------------------------------------------------"
#         echo "Testing : [$w_info_gain , $w_travel_distance, $w_sensing_time] "
#         ./../build/mcdm_online_exploration ./../Images/inbeng_small_correct.pgm 1 72 124 180 26 180 0.999 0 1 ./../config/tag_inbeng_1.yaml 865e6 0 $w_info_gain $w_travel_distance 0 0 /tmp/result_gs_mcdm.csv /tmp/coverage_gs_mcdm.csv /tmp/distance_gs_mcdm.csv 1 /tmp/accuracy_gs_mcdm.csv &>/dev/null &
#         ((counter++))
        
#         if ((counter==3))
#         then
#             wait
#             echo "================================================================="
#             echo "First three scripts are done!"
#             echo "================================================================="
#             counter=0
#         fi
#     done
# done

w_info_gain=0.0
counter=0
batch_size=10
while (( $(echo "$w_info_gain < $max_w" |bc -l) ));
do
    w_info_gain=$(echo "($starting_w + $counter * $increment)" | bc -l)
    w_travel_distance=$(echo "($max_w - $w_info_gain)" | bc -l)
    echo "-----------------------------------------------------------------"
    echo "Testing : [$w_info_gain , $w_travel_distance, $w_sensing_time] "
    ./../build/mcdm_online_exploration ./../Images/inbeng_small_correct.pgm 1 72 124 180 26 180 0.999 0 1 ./../config/tag_inbeng_1.yaml 865e6 0 $w_info_gain $w_travel_distance 0 0 /tmp/result_gs_mcdm.csv /tmp/coverage_gs_mcdm.csv /tmp/distance_gs_mcdm.csv 1 /tmp/accuracy_gs_mcdm.csv &>/dev/null &
    ((counter++))
    if ((counter%batch_size==0))
    then
        wait
        echo "================================================================="
        num_batch=$(echo "scale=2;($counter/$batch_size)" | bc -l)
        echo "Batch [$num_batch] of scripts is done!"
        echo "================================================================="
        # counter=0
    fi
done