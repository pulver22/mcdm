use_mcdm=0
param_list=(0.0 0.01 0.1 0.5 1.0 5.0 10.0 50.0 100.0)
len_param_list=${#param_list[@]}

counter=0
batch_size=4

total_runs=10
counter_run=0

X=(72 79 33 56 54 51 46 76 103 36)
Y=(124 117 45 41 98 109 128 149 150 72)
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
        for ((j = 0; j < len_param_list; j++))
        do
            w_info_gain=${param_list[$i]}
            w_sensing_time=1.0
            w_rfid_gain=0.0
            w_travel_distance=${param_list[$j]}
            echo "-->[B] IG: $w_info_gain, TD: $w_travel_distance, ST: $w_sensing_time"
            sum_w=$(echo "$w_info_gain + $w_travel_distance + $w_sensing_time" | bc -l)
            # Normalise the variable in [0, 1]
            w_info_gain=$(echo "$w_info_gain / $sum_w" | bc -l)
            w_travel_distance=$(echo "$w_travel_distance / $sum_w" | bc -l)
            w_sensing_time=$(echo "$w_sensing_time / $sum_w" | bc -l)
            # sum_w=$(($w_info_gain + $w_travel_distance))
            # echo "---> SUM: $sum_w"
            # echo "-->[A] IG: $w_info_gain, TD: $w_travel_distance, ST: $w_sensing_time"
            echo "Testing : [$w_info_gain , $w_travel_distance, $w_sensing_time] "
            ./../build/mcdm_online_exploration ./../Images/inbeng_small_correct.pgm 1 ${X[$counter_run]} ${Y[$counter_run]} 180 9 180 0.999 0 1 ./../config/tag_inbeng_1.yaml 865e6 0 $w_info_gain $w_travel_distance $w_sensing_time $w_rfid_gain /tmp/result_gs_wAVG_r$counter_run.csv /tmp/coverage_gs_wAVG_r$counter_run.csv /tmp/distance_gs_wAVG_r$counter_run.csv 1 /tmp/accuracy_gs_wAVG_r$counter_run.csv $use_mcdm &>/dev/null &
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
            echo ""
        done

        echo ""  # print new line
    done
    ((counter_run++))
done
