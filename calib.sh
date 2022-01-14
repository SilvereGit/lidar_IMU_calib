#!/usr/bin/env bash

bag_path="$HOME/data/dataset_calibration"

outdoor_sync_bag_name=(
#"Court-01.bag"
#"Court-02.bag"
#"Court-03.bag"
#"Court-04.bag"
#"Court-05.bag"
)

indoor_sync_bag_name=(
#"Garage-01.bag"
#"Garage-03.bag"
#"Garage-04.bag"
#"Garage-05.bag"

)

imu_topic_name=(
"/imu"
#"/imu1/data_sync"
#"/imu2/data_sync"
#"/imu3/data_sync"
)

lidar_model="HDL_32E"
#lidar_model="VLP_16"

gyro_weight=28.0
accelerometer_weight=18.5
lidar_weight=10.0

initial_p_LinI=([0.0,0.0,0.0])
initial_q_LtoI=([0.0,0.0,0.0,1.0])

#how many seconds worth of data from the timestamp of first scan do we use to create the lidar odometry map
#after that time map is not updated, data is just matched to the map, only for initial lidar odometry
scan4map=15
timeOffsetPadding=0.015

show_ui=true  #false

bag_start=1
bag_durr=600


bag_count=-1
sync_bag_name=(${outdoor_sync_bag_name[*]} ${indoor_sync_bag_name[*]})
for i in "${!sync_bag_name[@]}"; do
    let bag_count=bag_count+1

    ndtResolution=0.5	# indoor
    if [ $bag_count -lt ${#outdoor_sync_bag_name[*]} ]; then
        ndtResolution=1.0 # outdoor
    fi

    for j in "${!imu_topic_name[@]}"; do
        path_bag="$bag_path/${sync_bag_name[i]}"

        echo "lidar_model:=${lidar_model}"
        echo "topic_imu:=${imu_topic_name[j]}"
        echo "path_bag:=${path_bag}"
        echo "ndtResolution:=${ndtResolution}"
        echo "=============="

        roslaunch li_calib licalib_gui.launch \
                          topic_imu:="${imu_topic_name[j]}" \
                          path_bag:="${path_bag}" \
                          bag_start:="${bag_start}" \
                          bag_durr:="${bag_durr}" \
                          scan4map:="${scan4map}" \
                          lidar_model:="${lidar_model}" \
                          time_offset_padding:="${timeOffsetPadding}"\
                          ndtResolution:="${ndtResolution}" \
                          gyro_weight:="${gyro_weight}" \
                          accelerometer_weight:="${accelerometer_weight}" \
                          lidar_weight:="${lidar_weight}" \
                          initial_p_LinI:="${initial_p_LinI}"\
                          initial_q_LtoI:="${initial_q_LtoI}" \
                          show_ui:="${show_ui}"
    done
done
