use_mcdm=1
# param_list=(0.0 0.01 0.1 0.5 1.0 5.0 10.0 50.0 100.0)
# param_list=(0.0 0.005 0.01 0.05 0.1 0.5 1.0 2.5 5.0 7.5 10.0 25.0 50.0 75.0 100.0)
# param_list=(0.0 0.0025 0.005 0.0075 0.01 0.025 0.05 0.075 0.1 0.25 0.5 0.75 1.0 2.5 5.0 7.5 10.0 25.0 50.0 75.0 100.0)
# param_list=(0.01 0.1 0.5 1.0 5.0 10.0 50.0 100.0)
# len_param_list=${#param_list[@]}

# List of starting coordinate of the robots
X=(102 105)
Y=(97  78)

# MCDM parameters list
w_info_gain_list=(1 0 0 0.333 0.6 0.428 0.2 0.144 0.2 0.428 0.5 0 0.5)
w_travel_distance_list=(0 1 0 0.333 0.2 0.428 0.6 0.428 0.2 0.144 0.5 0.5 0)
w_sensing_time_list=(0 0 1 0.333 0.2 0.144 0.2 0.428 0.6 0.428 0   0.5 0.5)
w_rfid_gain_list=(0 0 0 0     0   0     0   0     0   0     0   0   0)

len_param_list=${#w_info_gain_list[@]}


counter=0
batch_size=15

total_runs=${#X[@]}
counter_run=0



if (($use_mcdm == 1))
then
    echo "========== MCDM =========="
else
    echo "========== wAVG =========="
fi

while (( $(echo "$counter_run < $total_runs" | bc -l)));
do
    echo "===== RUN $counter_run ====="
    for ((i=0; i<len_param_list; i++))
    do
        
        w_info_gain=${w_info_gain_list[$i]}
        w_sensing_time=${w_sensing_time_list[$i]}
        w_rfid_gain=${w_rfid_gain_list[$i]}
        w_travel_distance=${w_travel_distance_list[$i]}
        # echo "-->[B] IG: $w_info_gain, TD: $w_travel_distance, ST: $w_sensing_time, RFID: $w_rfid_gain"
        # sum_w=$(echo "$w_info_gain + $w_travel_distance + $w_sensing_time" | bc -l)
        # # Normalize the variable in [0, 1]
        # w_info_gain=$(echo "$w_info_gain / $sum_w" | bc -l)
        # w_travel_distanceres=$(echo "$w_travel_distance / $sum_w" | bc -l)
        # w_sensing_time=$(echo "$w_sensing_time / $sum_w" | bc -l)
        # sum_w=$(($w_info_gain + $w_travel_distance))
        # echo "---> SUM: $sum_w"
        # echo "-->[A] IG: $w_info_gain, TD: $w_travel_distance, ST: $w_sensing_time"
        echo "Testing : [$w_info_gain , $w_travel_distance, $w_sensing_time, $w_rfid_gain] "
        ./../build/mcdm_online_exploration ./../Images/inb3123_small.pgm 1 ${X[$counter_run]} ${Y[$counter_run]} 180 6 180 0.999 0 1 ./../config/tag_inb3123_1.yaml 902e6 -10 $w_info_gain $w_travel_distance $w_sensing_time $w_rfid_gain /tmp/result_inb3123_mcdm_r$counter_run.csv /tmp/coverage_inb3123_mcdm_$i_r$counter_run.csv /tmp/distance_inb3123_mcdm_$i_r$counter_run.csv 1 /tmp/accuracy_inb3123_mcdm_r$counter_run.csv $use_mcdm &>/dev/null &
# &>/dev/null
        # ./../build/mcdm_online_exploration ./../Images/mfc_test.pgm 1 72 124 180 25 180 0.99 0 1 ./../config/tag_inbeng_3.yaml 902e6 -10 0 0.33 0.66 0 /tmp/result_inbeng.csv /tmp/coverage_mcdm_inbeng.csv /tmp/distance_tag.csv 1 /tmp/accuracy.csv 1 &>/dev/null &
        ((counter++))
        if ((counter%batch_size==0))
        then
            wait
            # echo "================================================================="
            # num_batch=$(echo "scale=2;($counter/$batch_size)" | bc -l)
            # echo "Batch [$num_batch] of scripts is done!"
            # echo "================================================================="
            # counter=0
        fi
        echo ""  # print new line
    done
    ((counter_run++))
    echo "Increasing the counter"
done
echo "Script done"