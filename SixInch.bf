# Betaflight / MATEKF405 (MKF4) 4.0.0 Apr 11 2019 / 07:25:14 (9ad2cc844) MSP API: 1.41

# start the command batch
batch start

board_name MATEKF405
manufacturer_id

# name
name Daniel

# resources

# timer

# dma

# mixer

# servo

# servo mix


# feature
feature -SOFTSERIAL
feature GPS
feature LED_STRIP

# beeper

# beacon
beacon RX_LOST
beacon RX_SET

# map

# serial
serial 0 4160 115200 57600 115200 115200
serial 1 0 115200 57600 0 115200
serial 3 2 115200 38400 0 115200

# led

# color

# mode_color

# aux
aux 0 0 0 1975 2025 0 0
aux 1 46 3 1975 2025 0 0
aux 2 13 3 1975 2025 0 0
aux 3 35 1 1975 2025 0 0

# adjrange

# rxrange

# vtx

# rxfail

# display_name

# master
set mag_hardware = NONE
set baro_hardware = NONE
set rssi_channel = 11
set serialrx_provider = IBUS
set dshot_idle_value = 200
set motor_pwm_protocol = DSHOT1200
set beeper_dshot_beacon_tone = 4
set small_angle = 180
set gps_provider = UBLOX
set gps_sbas_mode = EGNOS
set gps_rescue_allow_arming_without_fix = ON
set pid_process_denom = 1
set osd_vbat_pos = 2103
set osd_rssi_pos = 2081
set osd_current_pos = 2135
set osd_gps_speed_pos = 2113
set osd_gps_lon_pos = 2049
set osd_gps_lat_pos = 2065
set gyro_1_sensor_align = CW180

# profile
profile 0

set dterm_lowpass2_type = PT1
set dterm_lowpass2_hz = 200
set feedforward_transition = 30
set iterm_relax_type = GYRO
set i_pitch = 85
set i_roll = 80
set d_roll = 30
set f_yaw = 35
set d_min_roll = 25
set d_min_pitch = 28
set d_min_boost_gain = 30
set d_min_advance = 80

# rateprofile
rateprofile 0

set roll_srate = 80
set pitch_srate = 80
set yaw_srate = 80

# end the command batch
batch end
save
