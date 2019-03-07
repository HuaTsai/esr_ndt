Header header
uint8 status # 0: no target 1: new 2: new updated 3: updated 4: coasted 5: merged 6: invalid coasted 7: new coasted
uint8 index # 1..64
float32 range # cropped between 0..204.7 m
float32 range_rate # cropped between -81.92..81.91m/s
float32 range_accl # cropped between -25.6..25.55 m/s^2
float32 azimuth # cropped between -51.2..51.1 deg
float32 lateral_rate # cropped between -8..7.75 m/s
float32 width # cropped between 0..7.5 m
bool is_mr_update
bool is_lr_update
#bool is_oncoming
#bool is_bridge
#bool is_grouping_changed
int8 amplitude # cropped to -10..21dB
