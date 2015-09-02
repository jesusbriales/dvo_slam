#!/bin/bash

# Notes:
# Use full paths for ROS arguments, otherwise it defaults to ~/.ros directory

echo "This is a test"
results_dir=~/results

dataset=/datasets/TUM/benchmark/rgbd_dataset_freiburg1_desk
sampling_ratios=(0.01 0.02 0.03 0.05 0.07 0.10 0.15 0.20 0.30 0.50 1.0)
statistics_size=5

for ratio in ${sampling_ratios[@]}
	do
		current_date=`date +%Y-%m-%d-%H%M%S`
		exp_dir="${results_dir}/${current_date}"
		mkdir $exp_dir
		cd $exp_dir
		for (( exp_idx=1; exp_idx<=$statistics_size; exp_idx++ ))
		do
			current_traj_file="${PWD}/trajectory_${exp_idx}.txt"
			roslaunch dvo_benchmark benchmark.launch \
			dataset:=$dataset \
			sampling_proportion:=$ratio \
			trajectory_file:=$current_traj_file
			
			rosrun rgbd_benchmark_tools evaluate_rpe.py \
			--fixed_delta \
			"${dataset}/groundtruth.txt" $current_traj_file \
			>> mean_translational_error.txt

		done
done