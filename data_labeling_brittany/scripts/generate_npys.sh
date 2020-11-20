#!/bin/bash
if [ $# -eq 2 ]; then
    echo "Starting creation of data..."
    path_rosbags=$1
	path_npys=$2

	cd $path_rosbags
	data=( $( ls . ) )	

	for i in "${data[@]}"
	do
		cd $path_rosbags"/$i"
		bags=( $( ls . | grep .bag ) ) 	

		for j in "${bags[@]}"
		do
			roslaunch data_labeling_brittany data_labeling_brittany.launch rosbag_file:=$path_rosbags"$i/$j" npy_directory:=$path_npys id_label_person:="$i"
		done	

	done
else
    echo "./generate_npys.sh <absolute_path_rosbags> <absolute_path_where_save_npys>"
fi
