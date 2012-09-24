define robotir sensor
(
  size [0.002 0.007 0.004]
  range [0 0.115]
  fov 40
  samples 3
  color_rgba [0 1 0 0.2]
)

define robot_kit_ir ranger
(
  robotir(pose [0.04  -0.02  0  0])
  robotir(pose [0.02  -0.04  0  -90])	
  robotir(pose [ -0.02    -0.04  0  -90])
  robotir(pose [ -0.04   -0.02  0  180])
  robotir(pose [ -0.04  0.02  0  180])
  robotir(pose [ -0.02   0.04  0  90])
  robotir(pose [0.02  0.04  0  90])
  robotir(pose [0.04  0.02  0   0])
  ranger_return 2
  obstacle_return 1
#  ctrl "proximity"
)

define robot_blobfinder blobfinder
( 
  # blobfinder properties
  colors_count 3
  colors [ "IndianRed" "NavyBlue" "gold"]
  image [ 30 30 ]
  range 2.0
  fov 60
  

  # model properties
  size [ 0.01 0.01 0.01 ]
  ranger_return 2
  obstacle_return 1
)

define mainbody block(
  points 10
  point[0] [0.0350 0.0000]
  point[1] [0.0329 0.0120]
  point[2] [0.0268 0.0225]
  point[3] [-0.0268 0.0225]
  point[4] [-0.0329 0.0120]
  point[5] [-0.0350 0.0000]
  point[6] [-0.0329 -0.0120]
  point[7] [-0.0268 -0.0225]
  point[8] [0.0268 -0.0225]
  point[9] [0.0329 -0.0120]
  z [0 0.04]
  # color "gray"
)

# top pcb
define mainpcb block(
  points 18
  point[0] [-0.0000 0.0350]
  point[1] [-0.0120 0.0329]
  point[2] [-0.0225 0.0268]
  point[3] [-0.0303 0.0175]
  point[4] [-0.0345 0.0061]
  point[5] [-0.0345 -0.0061]
  point[6] [-0.0303 -0.0175]
  point[7] [-0.0225 -0.0268]
  point[8] [-0.0120 -0.0329]
  point[9] [0.0000 -0.0350]
  point[10] [0.0120 -0.0329]
  point[11] [0.0225 -0.0268]
  point[12] [0.0303 -0.0175]
  point[13] [0.0345 -0.0061]
  point[14] [0.0345 0.0061]
  point[15] [0.0303 0.0175]
  point[16] [0.0225 0.0268]
  point[17] [0.0120 0.0329]
  z [0.04 0.0416]
  # color "DarkGreen"
)

define dummybody block(
  points 4
  point[0] [0.04 0.04]
  point[1] [-0.04 0.04]
  point[2] [-0.04 -0.04]
  point[3] [0.04 -0.04]
  z [0.0 0.0496]
  # color "green"
)
define sensorboard block(
  points 4
  point[0] [0.04 0.04]
  point[1] [-0.04 0.04]
  point[2] [-0.04 -0.04]
  point[3] [0.04 -0.04]
  z [0.048 0.0496]
  #color "green"
)

define filledblock block(
  points 4
  point[0] [0.036 0.036]
  point[1] [-0.036 0.036]
  point[2] [-0.036 -0.036]
  point[3] [0.036 -0.036]
  z [0.0496 0.054]
  # color "green"
)

define turrent2 block(
  points 11
  point[0] [-0.035 0]
  point[1] [-0.028 -0.018 ]
  point[2] [0.011 -0.012 ]
  point[3] [0.026 -0.021 ]
  point[4] [0.027 -0.019 ]
  point[5] [0.015 -0.008 ]
  point[6] [0.015 0.008 ]
  point[7] [0.027 0.019 ]
  point[8] [0.026 0.021 ]
  point[9] [0.011 0.012 ]
  point[10] [-0.028 0.018 ]
  z [0.0576 0.0592]
  #color "LightGreen"
)


define  robot_base position
(
  
  origin [0 0 0 0]
  mass 1.0
  
  color "yellow"
  drive "diff"
  gui_nose 1
  obstacle_return 1
  ranger_return 2
  blob_return 0
  fiducial_return 1

  localization "gps"
  localization_origin [0 0 0 0]
  
  #some blocks  	
  #mainbody()
  #mainpcb()
  
)

define robot_kit robot_base
(
  size [0.08 0.08 0.05]
  #sensorboard()
  turrent2()
  filledblock()
  dummybody()

  robot_blobfinder(pose [0.05 0 -0.01 0])

  robot_kit_ir(pose [0 0 -0.008 0])
	
  lightsensor(index 0  pose [0.04  -0.02  -0.008 0])
  lightsensor(index 1  pose [0.02  -0.04   -0.008 -90])
  lightsensor(index 2  pose [ -0.02    -0.04  -0.008 -90])
  lightsensor(index 3  pose [-0.04   -0.02   -0.008 180])
  lightsensor(index 4  pose [-0.04  0.02  -0.008 180])
  lightsensor(index 5  pose [-0.02   0.04    -0.008 90])
  lightsensor(index 6  pose [0.02 0.04  -0.008 90])
  lightsensor(index 7  pose [0.04  0.02   -0.008 0])
  blinkenlight(index 0 obstacle_return 0 blob_return 0 ranger_return 2 size [0.001 0.002 0.002] pose [ 0.04  0  -0.008 0] color "red" period 4 dutycycle 0.5 enabled 0)
  blinkenlight(index 1 obstacle_return 0 blob_return 0 ranger_return 2 size [0.001 0.002 0.002] pose [ 0  -0.04  -0.008 -90] color "red" period 4 dutycycle 0.5  enabled 0)
  blinkenlight(index 2 obstacle_return 0 blob_return 0 ranger_return 2 size [0.001 0.002 0.002] pose [ -0.04  0  -0.008 180] color "red" period 4 dutycycle 0.5  enabled 0)
  blinkenlight(index 3 obstacle_return 0 blob_return 0 ranger_return 2 size [0.001 0.002 0.002] pose [ 0  0.04  -0.008 90] color "red" period 4 dutycycle 0.5  enabled 0)
  ircomm(index 0 size [0.001 0.002 0.001] fov 120 pose [ 0.04  0  -0.006 0]  color "white")
  ircomm(index 1 size [0.001 0.002 0.001] fov 120 pose [ 0.0  -0.04  -0.006 -90]  color "white")
  ircomm(index 2 size [0.001 0.002 0.001] fov 120 pose [ -0.04  0.0  -0.006 180]  color "white")
  ircomm(index 3 size [0.001 0.002 0.001] fov 120 pose [ 0.0  0.04  -0.006 90]  color "white")
  connector(name "g0" blob_return 0 ranger_return 2	obstacle_return 0 connector_return 0 size [0.01 0.014 0.016]	pose [0.045 0.000 -0.04 0] 	color "blue" )
  connector(name "g1" blob_return 0 ranger_return 2	obstacle_return 0 connector_return 0 size [0.01 0.014 0.016]	pose [0.0 -0.045 -0.04 -90] color "green")
  connector(name "g2" blob_return 0 ranger_return 2 obstacle_return 0 connector_return 0 size [0.01 0.014 0.016]	pose [-0.045 0.000 -0.04 180] color "gray")
  connector(name "g3" blob_return 0 ranger_return 2 obstacle_return 0 connector_return 0 size [0.01 0.014 0.016]	pose [0.00 0.045 -0.04 90] 	color "blue")

  # body
  connector_return 0

  #
  joules_capacity 8000
  
  ctrl "morphStage"
)

define lightsensor lightdetector
(
  color "red"
  blob_return 0
  obstacle_return 0
  ranger_return 2
  size [0.002  0.007 0.004]
  range 0.25
  fov 180
)

define light model
(
  size [0.002 0.004 0.001]
  color "gray"
  blinkenlight(size [0.001 0.002 0.001] pose [ 0.0015 0 -0.001 0] color "red" period 10 dutycycle 0.5 enabled 1)
)


