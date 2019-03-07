Header header
# from 4e0
uint16 scan_index
#uint8 dsp_timestamp # in ms
#float32 vehicle_speed # in m/s
#float32 vehicle_yaw_rate # in in deg/s
#float32 vehicle_turning_radius # in m
#float32 range # cropped between 0..204.7 m
bool is_comm_error
# from 4e1
#uint16 steering_angle_ack # in deg
bool is_radiating
bool is_failed
bool is_blocked
bool is_overheating
#uint8 max_tracks_ack # 1..64
bool is_raw_data_mode
int8 internal_temperature # in degC
uint16 sw_version
#float32 yaw_rate_bias # deg/s
#float32 veh_spd_comp_factor
# from 4e3
bool is_lr_enabled
bool is_mr_enabled
bool is_sidelobe_blockage
bool is_partial_blockage
#float32 auto_align_angle
