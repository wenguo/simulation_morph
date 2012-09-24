define socket model
(
    size [ 0.020 0.20 0.0592 ]

    #shape
    block
    (
        color "IndianRed"
        points 4
        point[0] [0.1 0]
        point[1] [0.1 0.2 ]
        point[2] [-0.1 0.2 ]
        point[3] [-0.1 0 ]
        z [0.0 0.0496]
    )
    block
    (
        color "IndianRed"
        points 4
        point[0] [0.1 0]
        point[1] [0.1 0.2 ]
        point[2] [0.0 0.2 ]
        point[3] [0.0 0 ]
        z [0.0496 0.0592]
    )

    #gripper for docking
    connector(name "g0" color "IndianRed"   obstacle_return 0  connector_return 1 blob_return 0 size [0.01 0.014 0.016]	pose [-0.0105 0.000  -0.04 180])

    #blinken light for robot docking alignment
    blinkenlight(index 0 size [0.001 0.002 0.002] pose [-0.01 0.000 -0.008 180] blob_return 0 color "red" period 4 dutycycle 0.5 enabled 0)

    ircomm(index 2 size [0.001 0.002 0.001] fov 120 pose [ -0.01  0.0  -0.006 180] blob_return 0 color "white")

    # insensible to collision and range sensors
    obstacle_return 0
    ranger_return -1  
    blob_return 1
    # ctrl "power_sockets"
)

define powersocket_on_column model
(
    color "green"
    size [ 0.4 0.4 0.5 ]
    blob_return 1
    obstacle_return 1
    ranger_return 4
    
    socket ( pose [-0.2 0 -0.25 0] )
)

define powersocket_on_base model
(
    size [ 2.0 2.0 0.1 ]
    blob_return 0
    obstacle_return 1
    ranger_return 5
    #base
    block
    (
        color "LightGray"
        points 4
        point[0] [0.5 0.5 ]
        point[1] [-0.5 0.5 ]
        point[2] [-0.5 -0.5 ]
        point[3] [0.5 -0.5 ]
        z [0.0 0.1]
    )

    powersocket_on_column ( pose [0 0 -0.1 0] )
)

define power_socket model
(
    color "green"
    size [ 0.1 0.4 0.5 ]
    #back

    blob_return 0
    obstacle_return 0
    ranger_return -1
    socket ( pose [-0.05 0 -0.25 0] )
)

define barrier model
(
    color "PaleVioletRed1"
    blob_return 1
    obstacle_return 1
    ranger_return 5
)

define wall model
(
    color "DimGray"
    blob_return 1
    obstacle_return 1
    ranger_return 4
)

