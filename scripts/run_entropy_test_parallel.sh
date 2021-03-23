use_mcdm=1

# List of starting coordinate of the robots
X=(72   79   33)
Y=(124  117  45)

entropy_list=(0.5 1 2 4)

# Path where to save data (create it if it doesn't exist)
root=/home/pulver/Desktop/nbs_aaai/static/std_
# root=/home/pulver/Desktop/test/std
# [ -d $out_path ] || mkdir -p $out_path

# MCDM parameters list
w_info_gain_list=(      0.1 )
w_travel_distance_list=(0.1 )
w_sensing_time_list=(   0.1 )
w_rfid_gain_list=(      0.6 )
w_battery_status_list=( 0.1 )

len_entropy_list=${#entropy_list[@]}


counter=0  # how many threads are running
batch_size=25  # how many experiments to launch in parallel

total_runs=50


if (($use_mcdm == 1))
then
    echo "========== MCDM =========="
else
    echo "========== wAVG =========="
fi

for ((i=0; i<len_entropy_list; i++))
do
    echo "===== ENTROPY ${entropy_list[$i]} ====="
    counter_run=0
    while (( $(echo "$counter_run < $total_runs" | bc -l)));
    do
        echo "===== RUN $counter_run ====="
        # Create a folder if not existing
        out_path=${root}_${entropy_list[$i]}
        [ -d $out_path ] || mkdir -p $out_path

        w_info_gain=${w_info_gain_list[0]}
        w_sensing_time=${w_sensing_time_list[0]}
        w_travel_distance=${w_travel_distance_list[0]}
        w_rfid_gain=${w_rfid_gain_list[0]}
        w_battery_status=${w_battery_status_list[0]}
        echo "Testing : [$w_info_gain , $w_travel_distance, $w_sensing_time, $w_rfid_gain, $w_battery_status] "
        ./../build/mcdm_online_exploration ./../Images/inbeng_small.pgm 1 ${X[0]} ${Y[0]} 180 12 180 0.99 0 1 ./../config/tag_inbeng_0.yaml 902e6 -10 $w_info_gain $w_travel_distance $w_sensing_time $w_rfid_gain $w_battery_status ${out_path}/result_inbeng_mcdm_r$counter_run.csv ${out_path}/coverage_inbeng_mcdm_${i}_r${counter_run}.csv ${out_path}/distance_inbeng_mcdm_$r${counter_run}.csv 1 ${out_path}/accuracy_inbeng_mcdm_r$counter_run.csv $use_mcdm ${out_path}/entropy_$counter_run ${entropy_list[$i]} &>/dev/null &
        ((counter++))
        if ((counter%batch_size==0))
        then
            wait
        fi
        echo ""
        
        ((counter_run++))
    done
done
echo "Script done"