use_mcdm=1


# Environment to test
envs=(inb3123 inbeng inbatrium ncfm orebro)
# List of starting coordinate of the robots
X=(102 105 106   72   79   33   126 40   28   38  52   116   97  81   54)
Y=(97  78  76    124  117  45   99  125  154  95  136  90    100 116  140)
range=(6         12             9             12             6)

envs=(orebro)
X=(97  81   54)
Y=(100 116  140)
range=6

# MCDM parameters list
w_info_gain_list=(      1  0  0  ) #0  0  0.2  0.6  0.1  0.1  0.1  0.1 )
w_travel_distance_list=(0  1  0  ) #0  0  0.2  0.1  0.6  0.1  0.1  0.1 )
w_sensing_time_list=(   0  0  1  ) #0  0  0.2  0.1  0.1  0.6  0.1  0.1 )
w_rfid_gain_list=(      0  0  0  ) #1  0  0.2  0.1  0.1  0.1  0.6  0.1 )
w_battery_status_list=( 0  0  0  ) #0  1  0.2  0.1  0.1  0.1  0.1  0.6 )

len_param_list=${#w_info_gain_list[@]}


counter=0  # how many threads are running
batch_size=9  # how many experiments to launch in parallel

total_runs=${#X[@]}
total_envs=${#envs[@]}
counter_run=0
counter_env=0

if (($use_mcdm == 1))
then
    echo "========== MCDM =========="
else
    echo "========== wAVG =========="
fi

while (( $(echo "$counter_env < $total_envs" | bc -l)));
do
    echo "===== ENV ${envs[$counter_env]} ====="
    # Path where to save data (create it if it doesn't exist)
    out_path=/home/pulver/Desktop/mcdm_ral_experiments/random_frontier/${envs[$counter_env]}
    [ -d $out_path ] || mkdir -p $out_path

    while (( $(echo "$counter_run < 3" | bc -l)));
    do
        echo "===== RUN $counter_run ====="
        for ((i=0; i<len_param_list; i++))
        do
            
            w_info_gain=${w_info_gain_list[$i]}
            w_sensing_time=${w_sensing_time_list[$i]}
            w_travel_distance=${w_travel_distance_list[$i]}
            w_rfid_gain=${w_rfid_gain_list[$i]}
            w_battery_status=${w_battery_status_list[$i]}
            echo "Trial #: $i "

            ./../build/random_frontier ./../Images/${envs[$counter_env]}_small.pgm 1 ${X[$counter_run * $counter_env]} ${Y[$counter_run * $counter_env]} 180 ${range[$counter_env]} 180 0.99 0 1 ./../config/tag_${envs[$counter_env]}_${counter_run}.yaml 902e6 -10 $w_info_gain $w_travel_distance $w_sensing_time $w_rfid_gain $w_battery_status ${out_path}/result_${envs[$counter_env]}_mcdm_r$counter_run.csv ${out_path}/coverage_${envs[$counter_env]}_mcdm_${i}_r${counter_run}.csv ${out_path}/distance_${envs[$counter_env]}_mcdm_$r${counter_run}.csv 1 ${out_path}/accuracy_${envs[$counter_env]}_mcdm_r${counter_run}.csv $use_mcdm &>/dev/null &
            ((counter++))
            if ((counter%batch_size==0))
            then
                wait
            fi
            echo ""
        done
        ((counter_run++))
    done
    ((counter_env++))
    counter_run=0
done
echo "Script done"