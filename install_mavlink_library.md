MAVLink Type Enumerations
FIRMWARE_VERSION_TYPE
[Enum] These values define the type of firmware release. These values indicate the first version or release of this type. For example the first alpha release would be 64, the second would be 65.

Value	Field Name	Description
0	FIRMWARE_VERSION_TYPE_DEV	development release
64	FIRMWARE_VERSION_TYPE_ALPHA	alpha release
128	FIRMWARE_VERSION_TYPE_BETA	beta release
192	FIRMWARE_VERSION_TYPE_RC	release candidate
255	FIRMWARE_VERSION_TYPE_OFFICIAL	official stable release
HL_FAILURE_FLAG
[Enum] Flags to report failure cases over the high latency telemtry.

Value	Field Name	Description
1	HL_FAILURE_FLAG_GPS	GPS failure.
2	HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE	Differential pressure sensor failure.
4	HL_FAILURE_FLAG_ABSOLUTE_PRESSURE	Absolute pressure sensor failure.
8	HL_FAILURE_FLAG_3D_ACCEL	Accelerometer sensor failure.
16	HL_FAILURE_FLAG_3D_GYRO	Gyroscope sensor failure.
32	HL_FAILURE_FLAG_3D_MAG	Magnetometer sensor failure.
64	HL_FAILURE_FLAG_TERRAIN	Terrain subsystem failure.
128	HL_FAILURE_FLAG_BATTERY	Battery failure/critical low battery.
256	HL_FAILURE_FLAG_RC_RECEIVER	RC receiver failure/no rc connection.
512	HL_FAILURE_FLAG_OFFBOARD_LINK	Offboard link failure.
1024	HL_FAILURE_FLAG_ENGINE	Engine failure.
2048	HL_FAILURE_FLAG_GEOFENCE	Geofence violation.
4096	HL_FAILURE_FLAG_ESTIMATOR	Estimator failure, for example measurement rejection or large variances.
8192	HL_FAILURE_FLAG_MISSION	Mission failure.
MAV_GOTO
[Enum] Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.

Value	Field Name	Description
0	MAV_GOTO_DO_HOLD	Hold at the current position.
1	MAV_GOTO_DO_CONTINUE	Continue with the next item in mission execution.
2	MAV_GOTO_HOLD_AT_CURRENT_POSITION	Hold at the current position of the system
3	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION	Hold at the position specified in the parameters of the DO_HOLD action
MAV_MODE
[Enum] These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.

Value	Field Name	Description
0	MAV_MODE_PREFLIGHT	System is not ready to fly, booting, calibrating, etc. No flag is set.
80	MAV_MODE_STABILIZE_DISARMED	System is allowed to be active, under assisted RC control.
208	MAV_MODE_STABILIZE_ARMED	System is allowed to be active, under assisted RC control.
64	MAV_MODE_MANUAL_DISARMED	System is allowed to be active, under manual (RC) control, no stabilization
192	MAV_MODE_MANUAL_ARMED	System is allowed to be active, under manual (RC) control, no stabilization
88	MAV_MODE_GUIDED_DISARMED	System is allowed to be active, under autonomous control, manual setpoint
216	MAV_MODE_GUIDED_ARMED	System is allowed to be active, under autonomous control, manual setpoint
92	MAV_MODE_AUTO_DISARMED	System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
220	MAV_MODE_AUTO_ARMED	System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
66	MAV_MODE_TEST_DISARMED	UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
194	MAV_MODE_TEST_ARMED	UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
MAV_SYS_STATUS_SENSOR
[Enum] These encode the sensors whose status is sent as part of the SYS_STATUS message.

Value	Field Name	Description
1	MAV_SYS_STATUS_SENSOR_3D_GYRO	0x01 3D gyro
2	MAV_SYS_STATUS_SENSOR_3D_ACCEL	0x02 3D accelerometer
4	MAV_SYS_STATUS_SENSOR_3D_MAG	0x04 3D magnetometer
8	MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE	0x08 absolute pressure
16	MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE	0x10 differential pressure
32	MAV_SYS_STATUS_SENSOR_GPS	0x20 GPS
64	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW	0x40 optical flow
128	MAV_SYS_STATUS_SENSOR_VISION_POSITION	0x80 computer vision position
256	MAV_SYS_STATUS_SENSOR_LASER_POSITION	0x100 laser based position
512	MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH	0x200 external ground truth (Vicon or Leica)
1024	MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL	0x400 3D angular rate control
2048	MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION	0x800 attitude stabilization
4096	MAV_SYS_STATUS_SENSOR_YAW_POSITION	0x1000 yaw position
8192	MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL	0x2000 z/altitude control
16384	MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL	0x4000 x/y position control
32768	MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS	0x8000 motor outputs / control
65536	MAV_SYS_STATUS_SENSOR_RC_RECEIVER	0x10000 rc receiver
131072	MAV_SYS_STATUS_SENSOR_3D_GYRO2	0x20000 2nd 3D gyro
262144	MAV_SYS_STATUS_SENSOR_3D_ACCEL2	0x40000 2nd 3D accelerometer
524288	MAV_SYS_STATUS_SENSOR_3D_MAG2	0x80000 2nd 3D magnetometer
1048576	MAV_SYS_STATUS_GEOFENCE	0x100000 geofence
2097152	MAV_SYS_STATUS_AHRS	0x200000 AHRS subsystem health
4194304	MAV_SYS_STATUS_TERRAIN	0x400000 Terrain subsystem health
8388608	MAV_SYS_STATUS_REVERSE_MOTOR	0x800000 Motors are reversed
16777216	MAV_SYS_STATUS_LOGGING	0x1000000 Logging
33554432	MAV_SYS_STATUS_SENSOR_BATTERY	0x2000000 Battery
67108864	MAV_SYS_STATUS_SENSOR_PROXIMITY	0x4000000 Proximity
134217728	MAV_SYS_STATUS_SENSOR_SATCOM	0x8000000 Satellite Communication
268435456	MAV_SYS_STATUS_PREARM_CHECK	0x10000000 pre-arm check status. Always healthy when armed
536870912	MAV_SYS_STATUS_OBSTACLE_AVOIDANCE	0x20000000 Avoidance/collision prevention
1073741824	MAV_SYS_STATUS_SENSOR_PROPULSION	0x40000000 propulsion (actuator, esc, motor or propellor)
MAV_FRAME
[Enum] Co-ordinate frames used by MAVLink. Not all frames are supported by all commands, messages, or vehicles. Global frames use the following naming conventions: - `GLOBAL`: Global co-ordinate frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default. The following modifiers may be used with `GLOBAL`: - `RELATIVE_ALT`: Altitude is relative to the vehicle home position rather than MSL - `TERRAIN_ALT`: Altitude is relative to ground level rather than MSL - `INT`: Latitude/longitude (in degrees) are scaled by multiplying by 1E7 Local frames use the following naming conventions: - `LOCAL`: Origin of local frame is fixed relative to earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator ("EKF"). - `BODY`: Origin of local frame travels with the vehicle. NOTE, `BODY` does NOT indicate alignment of frame axis with vehicle attitude. - `OFFSET`: Deprecated synonym for `BODY` (origin travels with the vehicle). Not to be used for new frames. Some deprecated frames do not follow these conventions (e.g. MAV_FRAME_BODY_NED and MAV_FRAME_BODY_OFFSET_NED).

Value	Field Name	Description
0	MAV_FRAME_GLOBAL	Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).
1	MAV_FRAME_LOCAL_NED	NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
2	MAV_FRAME_MISSION	NOT a coordinate frame, indicates a mission command.
3	MAV_FRAME_GLOBAL_RELATIVE_ALT	Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
4	MAV_FRAME_LOCAL_ENU	ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
5	MAV_FRAME_GLOBAL_INT	Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level (MSL).
6	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT	Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude with 0 being at the altitude of the home location.
7	MAV_FRAME_LOCAL_OFFSET_NED	NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
8	MAV_FRAME_BODY_NED
DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).

Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/accelaration values.
9	MAV_FRAME_BODY_OFFSET_NED
DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).

This is the same as MAV_FRAME_BODY_FRD.
10	MAV_FRAME_GLOBAL_TERRAIN_ALT	Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
11	MAV_FRAME_GLOBAL_TERRAIN_ALT_INT	Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
12	MAV_FRAME_BODY_FRD	FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle. The forward axis is aligned to the front of the vehicle in the horizontal plane.
13	MAV_FRAME_RESERVED_13
DEPRECATED: Replaced by (2019-04).

MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).
14	MAV_FRAME_RESERVED_14
DEPRECATED: Replaced by MAV_FRAME_LOCAL_FRD (2019-04).

MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down).
15	MAV_FRAME_RESERVED_15
DEPRECATED: Replaced by MAV_FRAME_LOCAL_FLU (2019-04).

MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up).
16	MAV_FRAME_RESERVED_16
DEPRECATED: Replaced by MAV_FRAME_LOCAL_FRD (2019-04).

MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down).
17	MAV_FRAME_RESERVED_17
DEPRECATED: Replaced by MAV_FRAME_LOCAL_FLU (2019-04).

MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up).
18	MAV_FRAME_RESERVED_18
DEPRECATED: Replaced by MAV_FRAME_LOCAL_FRD (2019-04).

MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down).
19	MAV_FRAME_RESERVED_19
DEPRECATED: Replaced by MAV_FRAME_LOCAL_FLU (2019-04).

MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up).
20	MAV_FRAME_LOCAL_FRD	FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
21	MAV_FRAME_LOCAL_FLU	FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
MAVLINK_DATA_STREAM_TYPE
[Enum]

Value	Field Name	Description
0	MAVLINK_DATA_STREAM_IMG_JPEG	
1	MAVLINK_DATA_STREAM_IMG_BMP	
2	MAVLINK_DATA_STREAM_IMG_RAW8U	
3	MAVLINK_DATA_STREAM_IMG_RAW32U	
4	MAVLINK_DATA_STREAM_IMG_PGM	
5	MAVLINK_DATA_STREAM_IMG_PNG	
FENCE_ACTION
[Enum] Actions following geofence breach.

Value	Field Name	Description
0	FENCE_ACTION_NONE	Disable fenced mode. If used in a plan this would mean the next fence is disabled.
1	FENCE_ACTION_GUIDED	Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT in GUIDED mode. Note: This action is only supported by ArduPlane, and may not be supported in all versions.
2	FENCE_ACTION_REPORT	Report fence breach, but don't take action
3	FENCE_ACTION_GUIDED_THR_PASS	Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT with manual throttle control in GUIDED mode. Note: This action is only supported by ArduPlane, and may not be supported in all versions.
4	FENCE_ACTION_RTL	Return/RTL mode.
5	FENCE_ACTION_HOLD	Hold at current location.
6	FENCE_ACTION_TERMINATE	Termination failsafe. Motors are shut down (some flight stacks may trigger other failsafe actions).
7	FENCE_ACTION_LAND	Land at current location.
FENCE_BREACH
[Enum]

Value	Field Name	Description
0	FENCE_BREACH_NONE	No last fence breach
1	FENCE_BREACH_MINALT	Breached minimum altitude
2	FENCE_BREACH_MAXALT	Breached maximum altitude
3	FENCE_BREACH_BOUNDARY	Breached fence boundary
FENCE_MITIGATE
[Enum] Actions being taken to mitigate/prevent fence breach

Value	Field Name	Description
0	FENCE_MITIGATE_UNKNOWN	Unknown
1	FENCE_MITIGATE_NONE	No actions being taken
2	FENCE_MITIGATE_VEL_LIMIT	Velocity limiting active to prevent breach
MAV_MOUNT_MODE
DEPRECATED: Replaced by GIMBAL_MANAGER_FLAGS (2020-01).

[Enum] Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal messages.

Value	Field Name	Description
0	MAV_MOUNT_MODE_RETRACT	Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
1	MAV_MOUNT_MODE_NEUTRAL	Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
2	MAV_MOUNT_MODE_MAVLINK_TARGETING	Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
3	MAV_MOUNT_MODE_RC_TARGETING	Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
4	MAV_MOUNT_MODE_GPS_POINT	Load neutral position and start to point to Lat,Lon,Alt
5	MAV_MOUNT_MODE_SYSID_TARGET	Gimbal tracks system with specified system ID
6	MAV_MOUNT_MODE_HOME_LOCATION	Gimbal tracks home location
GIMBAL_DEVICE_CAP_FLAGS
[Enum] Gimbal device (low level) capability flags (bitmap)

Value	Field Name	Description
1	GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT	Gimbal device supports a retracted position
2	GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL	Gimbal device supports a horizontal, forward looking position, stabilized
4	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS	Gimbal device supports rotating around roll axis.
8	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW	Gimbal device supports to follow a roll angle relative to the vehicle
16	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK	Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)
32	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS	Gimbal device supports rotating around pitch axis.
64	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW	Gimbal device supports to follow a pitch angle relative to the vehicle
128	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK	Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)
256	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS	Gimbal device supports rotating around yaw axis.
512	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW	Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)
1024	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK	Gimbal device supports locking to an absolute heading (often this is an option available)
2048	GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW	Gimbal device supports yawing/panning infinetely (e.g. using slip disk).
GIMBAL_MANAGER_CAP_FLAGS
[Enum] Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the GIMBAL_DEVICE_CAP_FLAGS which are identical with GIMBAL_DEVICE_FLAGS. However, the gimbal manager does not need to copy the flags from the gimbal but can also enhance the capabilities and thus add flags.

Value	Field Name	Description
1	GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
2	GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
4	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
8	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
16	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
32	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
64	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
128	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
256	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.
512	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.
1024	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK	Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
2048	GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW	Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
65536	GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL	Gimbal manager supports to point to a local position.
131072	GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL	Gimbal manager supports to point to a global latitude, longitude, altitude position.
GIMBAL_DEVICE_FLAGS
[Enum] Flags for gimbal device (lower level) operation.

Value	Field Name	Description
1	GIMBAL_DEVICE_FLAGS_RETRACT	Set to retracted safe position (no stabilization), takes presedence over all other flags.
2	GIMBAL_DEVICE_FLAGS_NEUTRAL	Set to neutral position (horizontal, forward looking, with stabiliziation), takes presedence over all other flags except RETRACT.
4	GIMBAL_DEVICE_FLAGS_ROLL_LOCK	Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.
8	GIMBAL_DEVICE_FLAGS_PITCH_LOCK	Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.
16	GIMBAL_DEVICE_FLAGS_YAW_LOCK	Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).
GIMBAL_MANAGER_FLAGS
[Enum] Flags for high level gimbal manager operation The first 16 bytes are identical to the GIMBAL_DEVICE_FLAGS.

Value	Field Name	Description
1	GIMBAL_MANAGER_FLAGS_RETRACT	Based on GIMBAL_DEVICE_FLAGS_RETRACT
2	GIMBAL_MANAGER_FLAGS_NEUTRAL	Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
4	GIMBAL_MANAGER_FLAGS_ROLL_LOCK	Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
8	GIMBAL_MANAGER_FLAGS_PITCH_LOCK	Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
16	GIMBAL_MANAGER_FLAGS_YAW_LOCK	Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
GIMBAL_DEVICE_ERROR_FLAGS
[Enum] Gimbal device (low level) error flags (bitmap, 0 means no error)

Value	Field Name	Description
1	GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT	Gimbal device is limited by hardware roll limit.
2	GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT	Gimbal device is limited by hardware pitch limit.
4	GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT	Gimbal device is limited by hardware yaw limit.
8	GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR	There is an error with the gimbal encoders.
16	GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR	There is an error with the gimbal power source.
32	GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR	There is an error with the gimbal motor's.
64	GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR	There is an error with the gimbal's software.
128	GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR	There is an error with the gimbal's communication.
256	GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING	Gimbal is currently calibrating.
GRIPPER_ACTIONS
[Enum] Gripper actions.

Value	Field Name	Description
0	GRIPPER_ACTION_RELEASE	Gripper release cargo.
1	GRIPPER_ACTION_GRAB	Gripper grab onto cargo.
WINCH_ACTIONS
[Enum] Winch actions.

Value	Field Name	Description
0	WINCH_RELAXED	Relax winch.
1	WINCH_RELATIVE_LENGTH_CONTROL	Wind or unwind specified length of cable, optionally using specified rate.
2	WINCH_RATE_CONTROL	Wind or unwind cable at specified rate.
UAVCAN_NODE_HEALTH
[Enum] Generalized UAVCAN node health

Value	Field Name	Description
0	UAVCAN_NODE_HEALTH_OK	The node is functioning properly.
1	UAVCAN_NODE_HEALTH_WARNING	A critical parameter went out of range or the node has encountered a minor failure.
2	UAVCAN_NODE_HEALTH_ERROR	The node has encountered a major failure.
3	UAVCAN_NODE_HEALTH_CRITICAL	The node has suffered a fatal malfunction.
UAVCAN_NODE_MODE
[Enum] Generalized UAVCAN node mode

Value	Field Name	Description
0	UAVCAN_NODE_MODE_OPERATIONAL	The node is performing its primary functions.
1	UAVCAN_NODE_MODE_INITIALIZATION	The node is initializing; this mode is entered immediately after startup.
2	UAVCAN_NODE_MODE_MAINTENANCE	The node is under maintenance.
3	UAVCAN_NODE_MODE_SOFTWARE_UPDATE	The node is in the process of updating its software.
7	UAVCAN_NODE_MODE_OFFLINE	The node is no longer available online.
ESC_CONNECTION_TYPE
[Enum] Indicates the ESC connection type.

Value	Field Name	Description
0	ESC_CONNECTION_TYPE_PPM	Traditional PPM ESC.
1	ESC_CONNECTION_TYPE_SERIAL	Serial Bus connected ESC.
2	ESC_CONNECTION_TYPE_ONESHOT	One Shot PPM ESC.
3	ESC_CONNECTION_TYPE_I2C	I2C ESC.
4	ESC_CONNECTION_TYPE_CAN	CAN-Bus ESC.
5	ESC_CONNECTION_TYPE_DSHOT	DShot ESC.
ESC_FAILURE_FLAGS
[Enum] Flags to report ESC failures.

Value	Field Name	Description
0	ESC_FAILURE_NONE	No ESC failure.
1	ESC_FAILURE_OVER_CURRENT	Over current failure.
2	ESC_FAILURE_OVER_VOLTAGE	Over voltage failure.
4	ESC_FAILURE_OVER_TEMPERATURE	Over temperature failure.
8	ESC_FAILURE_OVER_RPM	Over RPM failure.
16	ESC_FAILURE_INCONSISTENT_CMD	Inconsistent command failure i.e. out of bounds.
32	ESC_FAILURE_MOTOR_STUCK	Motor stuck failure.
64	ESC_FAILURE_GENERIC	Generic ESC failure.
STORAGE_STATUS
[Enum] Flags to indicate the status of camera storage.

Value	Field Name	Description
0	STORAGE_STATUS_EMPTY	Storage is missing (no microSD card loaded for example.)
1	STORAGE_STATUS_UNFORMATTED	Storage present but unformatted.
2	STORAGE_STATUS_READY	Storage present and ready.
3	STORAGE_STATUS_NOT_SUPPORTED	Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored.
STORAGE_TYPE
[Enum] Flags to indicate the type of storage.

Value	Field Name	Description
0	STORAGE_TYPE_UNKNOWN	Storage type is not known.
1	STORAGE_TYPE_USB_STICK	Storage type is USB device.
2	STORAGE_TYPE_SD	Storage type is SD card.
3	STORAGE_TYPE_MICROSD	Storage type is microSD card.
4	STORAGE_TYPE_CF	Storage type is CFast.
5	STORAGE_TYPE_CFE	Storage type is CFexpress.
6	STORAGE_TYPE_XQD	Storage type is XQD.
7	STORAGE_TYPE_HD	Storage type is HD mass storage type.
254	STORAGE_TYPE_OTHER	Storage type is other, not listed type.
ORBIT_YAW_BEHAVIOUR
[Enum] Yaw behaviour during orbit flight.

Value	Field Name	Description
0	ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER	Vehicle front points to the center (default).
1	ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING	Vehicle front holds heading when message received.
2	ORBIT_YAW_BEHAVIOUR_UNCONTROLLED	Yaw uncontrolled.
3	ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE	Vehicle front follows flight path (tangential to circle).
4	ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED	Yaw controlled by RC input.
WIFI_CONFIG_AP_RESPONSE
[Enum] Possible responses from a WIFI_CONFIG_AP message.

Value	Field Name	Description
0	WIFI_CONFIG_AP_RESPONSE_UNDEFINED	Undefined response. Likely an indicative of a system that doesn't support this request.
1	WIFI_CONFIG_AP_RESPONSE_ACCEPTED	Changes accepted.
2	WIFI_CONFIG_AP_RESPONSE_REJECTED	Changes rejected.
3	WIFI_CONFIG_AP_RESPONSE_MODE_ERROR	Invalid Mode.
4	WIFI_CONFIG_AP_RESPONSE_SSID_ERROR	Invalid SSID.
5	WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR	Invalid Password.
CELLULAR_CONFIG_RESPONSE
[Enum] Possible responses from a CELLULAR_CONFIG message.

Value	Field Name	Description
0	CELLULAR_CONFIG_RESPONSE_ACCEPTED	Changes accepted.
1	CELLULAR_CONFIG_RESPONSE_APN_ERROR	Invalid APN.
2	CELLULAR_CONFIG_RESPONSE_PIN_ERROR	Invalid PIN.
3	CELLULAR_CONFIG_RESPONSE_REJECTED	Changes rejected.
4	CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED	PUK is required to unblock SIM card.
WIFI_CONFIG_AP_MODE
[Enum] WiFi Mode.

Value	Field Name	Description
0	WIFI_CONFIG_AP_MODE_UNDEFINED	WiFi mode is undefined.
1	WIFI_CONFIG_AP_MODE_AP	WiFi configured as an access point.
2	WIFI_CONFIG_AP_MODE_STATION	WiFi configured as a station connected to an existing local WiFi network.
3	WIFI_CONFIG_AP_MODE_DISABLED	WiFi disabled.
COMP_METADATA_TYPE
[Enum] Supported component metadata types. These are used in the "general" metadata file returned by COMPONENT_INFORMATION to provide information about supported metadata types. The types are not used directly in MAVLink messages.

Value	Field Name	Description
0	COMP_METADATA_TYPE_GENERAL	General information about the component. General metadata includes information about other COMP_METADATA_TYPEs supported by the component. This type must be supported and must be downloadable from vehicle.
1	COMP_METADATA_TYPE_PARAMETER	Parameter meta data.
2	COMP_METADATA_TYPE_COMMANDS	Meta data that specifies which commands and command parameters the vehicle supports. (WIP)
3	COMP_METADATA_TYPE_PERIPHERALS	Meta data that specifies external non-MAVLink peripherals.
4	COMP_METADATA_TYPE_EVENTS	Meta data for the events interface.
MAV_DATA_STREAM
DEPRECATED: Replaced by MESSAGE_INTERVAL (2015-06).

[Enum] A data stream is not a fixed set of messages, but rather a recommendation to the autopilot software. Individual autopilots may or may not obey the recommended messages.

Value	Field Name	Description
0	MAV_DATA_STREAM_ALL	Enable all data streams
1	MAV_DATA_STREAM_RAW_SENSORS	Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
2	MAV_DATA_STREAM_EXTENDED_STATUS	Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
3	MAV_DATA_STREAM_RC_CHANNELS	Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
4	MAV_DATA_STREAM_RAW_CONTROLLER	Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
6	MAV_DATA_STREAM_POSITION	Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
10	MAV_DATA_STREAM_EXTRA1	Dependent on the autopilot
11	MAV_DATA_STREAM_EXTRA2	Dependent on the autopilot
12	MAV_DATA_STREAM_EXTRA3	Dependent on the autopilot
MAV_ROI
DEPRECATED: Replaced by MAV_CMD_DO_SET_ROI_* (2018-01).

[Enum] The ROI (region of interest) for the vehicle. This can be be used by the vehicle for camera/vehicle attitude alignment (see MAV_CMD_NAV_ROI).

Value	Field Name	Description
0	MAV_ROI_NONE	No region of interest.
1	MAV_ROI_WPNEXT	Point toward next waypoint, with optional pitch/roll/yaw offset.
2	MAV_ROI_WPINDEX	Point toward given waypoint.
3	MAV_ROI_LOCATION	Point toward fixed location.
4	MAV_ROI_TARGET	Point toward of given id.
MAV_CMD_ACK
[Enum] ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.

Value	Field Name	Description
0	MAV_CMD_ACK_OK	Command / mission item is ok.
1	MAV_CMD_ACK_ERR_FAIL	Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
2	MAV_CMD_ACK_ERR_ACCESS_DENIED	The system is refusing to accept this command from this source / communication partner.
3	MAV_CMD_ACK_ERR_NOT_SUPPORTED	Command or mission item is not supported, other commands would be accepted.
4	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED	The coordinate frame of this command / mission item is not supported.
5	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE	The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
6	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE	The X or latitude value is out of range.
7	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE	The Y or longitude value is out of range.
8	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE	The Z or altitude value is out of range.
MAV_PARAM_TYPE
[Enum] Specifies the datatype of a MAVLink parameter.

Value	Field Name	Description
1	MAV_PARAM_TYPE_UINT8	8-bit unsigned integer
2	MAV_PARAM_TYPE_INT8	8-bit signed integer
3	MAV_PARAM_TYPE_UINT16	16-bit unsigned integer
4	MAV_PARAM_TYPE_INT16	16-bit signed integer
5	MAV_PARAM_TYPE_UINT32	32-bit unsigned integer
6	MAV_PARAM_TYPE_INT32	32-bit signed integer
7	MAV_PARAM_TYPE_UINT64	64-bit unsigned integer
8	MAV_PARAM_TYPE_INT64	64-bit signed integer
9	MAV_PARAM_TYPE_REAL32	32-bit floating-point
10	MAV_PARAM_TYPE_REAL64	64-bit floating-point
MAV_PARAM_EXT_TYPE
[Enum] Specifies the datatype of a MAVLink extended parameter.

Value	Field Name	Description
1	MAV_PARAM_EXT_TYPE_UINT8	8-bit unsigned integer
2	MAV_PARAM_EXT_TYPE_INT8	8-bit signed integer
3	MAV_PARAM_EXT_TYPE_UINT16	16-bit unsigned integer
4	MAV_PARAM_EXT_TYPE_INT16	16-bit signed integer
5	MAV_PARAM_EXT_TYPE_UINT32	32-bit unsigned integer
6	MAV_PARAM_EXT_TYPE_INT32	32-bit signed integer
7	MAV_PARAM_EXT_TYPE_UINT64	64-bit unsigned integer
8	MAV_PARAM_EXT_TYPE_INT64	64-bit signed integer
9	MAV_PARAM_EXT_TYPE_REAL32	32-bit floating-point
10	MAV_PARAM_EXT_TYPE_REAL64	64-bit floating-point
11	MAV_PARAM_EXT_TYPE_CUSTOM	Custom Type
MAV_RESULT
[Enum] Result from a MAVLink command (MAV_CMD)

Value	Field Name	Description
0	MAV_RESULT_ACCEPTED	Command is valid (is supported and has valid parameters), and was executed.
1	MAV_RESULT_TEMPORARILY_REJECTED	Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work.
2	MAV_RESULT_DENIED	Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work.
3	MAV_RESULT_UNSUPPORTED	Command is not supported (unknown).
4	MAV_RESULT_FAILED	Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc.
5	MAV_RESULT_IN_PROGRESS	Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation.
6	MAV_RESULT_CANCELLED	Command has been cancelled (as a result of receiving a COMMAND_CANCEL message).
MAV_MISSION_RESULT
[Enum] Result of mission operation (in a MISSION_ACK message).

Value	Field Name	Description
0	MAV_MISSION_ACCEPTED	mission accepted OK
1	MAV_MISSION_ERROR	Generic error / not accepting mission commands at all right now.
2	MAV_MISSION_UNSUPPORTED_FRAME	Coordinate frame is not supported.
3	MAV_MISSION_UNSUPPORTED	Command is not supported.
4	MAV_MISSION_NO_SPACE	Mission items exceed storage space.
5	MAV_MISSION_INVALID	One of the parameters has an invalid value.
6	MAV_MISSION_INVALID_PARAM1	param1 has an invalid value.
7	MAV_MISSION_INVALID_PARAM2	param2 has an invalid value.
8	MAV_MISSION_INVALID_PARAM3	param3 has an invalid value.
9	MAV_MISSION_INVALID_PARAM4	param4 has an invalid value.
10	MAV_MISSION_INVALID_PARAM5_X	x / param5 has an invalid value.
11	MAV_MISSION_INVALID_PARAM6_Y	y / param6 has an invalid value.
12	MAV_MISSION_INVALID_PARAM7	z / param7 has an invalid value.
13	MAV_MISSION_INVALID_SEQUENCE	Mission item received out of sequence
14	MAV_MISSION_DENIED	Not accepting any mission commands from this communication partner.
15	MAV_MISSION_OPERATION_CANCELLED	Current mission operation cancelled (e.g. mission upload, mission download).
