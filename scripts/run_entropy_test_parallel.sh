use_mcdm=1

# List of starting coordinate of the robots
X=(72   79   33)
Y=(124  117  45)

# Path where to save data (create it if it doesn't exist)
out_path=/home/pulver/Desktop/mcdm_ral_experiments/inbeng
[ -d $out_path ] || mkdir -p $out_path

# MCDM parameters list
w_info_gain_list=(      0.1 )
w_travel_distance_list=(0.1 )
w_sensing_time_list=(   0.1 )
w_rfid_gain_list=(      0.6 )
w_battery_status_list=( 0.1 )

len_param_list=${#w_info_gain_list[@]}


counter=0
batch_size=11  # how many experiments to launch in parallel

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
        w_travel_distance=${w_travel_distance_list[$i]}
        w_rfid_gain=${w_rfid_gain_list[$i]}
        w_battery_status=${w_battery_status_list[$i]}
        echo "Testing : [$w_info_gain , $w_travel_distance, $w_sensing_time, $w_rfid_gain, $w_battery_status] "
        ./../build/mcdm_online_exploration ./../Images/inbeng_small.pgm 1 ${X[$counter_run]} ${Y[$counter_run]} 180 12 180 0.99 0 1 ./../config/tag_inbeng_${counter_run}.yaml 902e6 -10 $w_info_gain $w_travel_distance $w_sensing_time $w_rfid_gain $w_battery_status ${out_path}/result_inbeng_mcdm_r$counter_run.csv ${out_path}/coverage_inbeng_mcdm_${i}_r${counter_run}.csv ${out_path}/distance_inbeng_mcdm_$r${counter_run}.csv 1 ${out_path}/accuracy_inbeng_mcdm_r$counter_run.csv $use_mcdm ${out_path}/entropy_$counter_run &>/dev/null &
        ((counter++))
        if ((counter%batch_size==0))
        then
            wait
        fi
        echo ""
    done
    ((counter_run++))
done
echo "Script done"