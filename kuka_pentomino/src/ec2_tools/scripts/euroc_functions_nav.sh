#!/bin/bash

##################################
# navigation functions
##################################

# ----------------------------------------------------------------------

# miiwa aligend with room long axis
miiwa_max_x="9.5"
miiwa_max_y="12.5"
miiwa_min_x="6.0"
miiwa_min_y="8.3"

# miiwa perpendicular to room long axis
miiwa_max_x="9.0"
miiwa_max_y="12.0"
miiwa_min_x="6.5"
miiwa_min_y="8.0"
miiwa_max_velocity="0.4"

# ----------------------------------------------------------------------

# general module help
_nav_help()
{
    local helpmsg="Available 'nav' functions:\n"
        helpmsg+=" movrel     : move relative platform (under construction)\n"

    WarnMessage "$helpmsg"
}

# ----------------------------------------------------------------------

# wrapper to rosservice call /miiwa/move_relative_platform
mov()
{
    local helpmsg="mov [OPTIONS]\n"
          helpmsg+="OPTIONS:\n"
          helpmsg+="  -h          : display help message and quit\n"
          helpmsg+="  -x num      : relative movement in x direction (default: 0)\n"
          helpmsg+="  -y num      : relative movement in y direction (default: 0)\n"
          helpmsg+="  -[azY] num  : relative angular movement around z axis (default: 0)\n"
          helpmsg+="  -nb         : set blocking mode to false (default: true)\n"
          helpmsg+="  -rad        : angles in radians (default: degrees)\n"
          helpmsg+="  -v num      : set velocity (range: 0.05--0.3; default: 0.1)\n"
          helpmsg+="  -abs        : absolute position in world coordinates (default: platform coordinates)\n"

    local errmsg[1]="missing argument"
    local errmsg[2]="argument is not a number"
    local errmsg[3]="argument is out of range"
    local errmsg[4]="unknown argument"
    local errmsg[5]="in absolute mode all coordinates must be given"
    local errno=0

    # local variables
    local xyz_set=0
    local x=0 y=0 z=0 
    local blocking_mode=true 
    local velocity=0.1
    local in_deg=true 
    local absolute_position=false
    local something_to_do=false

    # process arguments
    while [ $# -ne 0 ]; do
        case "$1" in
            "-h") # display help message and quit
                InfoMessage "$helpmsg"
                return 0
                ;;
            "-x") # relative movement in x direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                x=$(adj_number $2)
                shift 2
                xyz_set=$(($xyz_set | 1))
                something_to_do=true
                ;;
            "-y") # relative movement in y direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                y=$(adj_number $2)
                shift 2
                xyz_set=$(($xyz_set | 2))
                something_to_do=true
                ;;
            "-z"|"-a"|"-Y") # relative angular movement around z axis (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                z=$(adj_number $2)
                shift 2
                xyz_set=$(($xyz_set | 4))
                something_to_do=true
                ;;
            "-nb") # set blocking mode to false (default: true)
                blocking_mode=false
                shift 1
                ;;
            "-v") # set velocity (range: 0.05--0.2; default: 0.1)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 0 $miiwa_max_velocity && errno=3 && break
                velocity=$(adj_number $2)
                shift 2
                ;;
            "-rad") # angle in radians
                in_deg=false
                shift 1
                ;;
            "-abs") # absolute position in world coordinates
                absolute_position=true
                shift 1
                ;;
            *) # error in format
                errno=4; break
                ;;
        esac
    done

    # check if all coordinates are given in absolute position mode 
    if [ $absolute_position == true -a $xyz_set -ne 7 ]; then
        errno=5;
    fi

    # if an error occurred, disply error message and quit
    if [ $errno -ne 0 ]; then
        ErrorMessage "mov: error #$errno: ${errmsg[$errno]}"
        return 1
    fi

    # do nothing if nothing to do
    if [ $something_to_do == "false" ]; then
        WarnMessage "Nothing to be done"
        return
    fi

    # convert angles to rad, if necesary
    test "$in_deg" == "true" && eval z=$(deg_to_rad $z)

    # call appropriated service based on reference frame
    if [ $absolute_position == "false" ]; then
        args="destination_pose: {x: $x, y: $y, theta: $z, cov_xx: 0.0, cov_yy: 0.0, cov_thetatheta: 0.0, cov_xy: 0.0, cov_xtheta: 0.0, cov_ytheta: 0.0}
parameter: {blocking: $blocking_mode, velocity: $velocity, blending: 0.0}"

        InfoMessage "rosservice call /miiwa/move_relative_navigation $args"

        rosservice call /miiwa/move_relative_navigation "$args"
    else
        args="destination_pose: {x: $x, y: $y, theta: $z, cov_xx: 0.0, cov_yy: 0.0, cov_thetatheta: 0.0, cov_xy: 0.0, cov_xtheta: 0.0, cov_ytheta: 0.0}
parameter: {blocking: $blocking_mode, velocity: $velocity, blending: 0.0}"

        InfoMessage "rosservice call /miiwa/move_absolute_navigation $args"

        rosservice call /miiwa/move_absolute_navigation "$args"
    fi  
}

# ----------------------------------------------------------------------

# wrapper to rosservice call /miiwa/move_relative_platform
movrel()
{
    local helpmsg="movrel [OPTIONS]\n"
          helpmsg+="OPTIONS:\n"
          helpmsg+="  -h          : display help message and quit\n"
          helpmsg+="  -x num      : relative movement in x direction (default: 0)\n"
          helpmsg+="  -y num      : relative movement in y direction (default: 0)\n"
          helpmsg+="  -[azY] num  : relative angular movement around z axis (default: 0)\n"
          helpmsg+="  -nb         : set blocking mode to false (default: true)\n"
          helpmsg+="  -rad        : angles in radians (default: degrees)\n"
          helpmsg+="  -v num      : set velocity (range: 0.05--0.3; default: 0.1)\n"

    local errmsg[1]="missing argument"
    local errmsg[2]="argument is not a number"
    local errmsg[3]="argument is out of range"
    local errmsg[4]="unknown argument"
    local errno=0

    # local variables
    local x=0 y=0 z=0 blocking_mode=true velocity=0.1 in_deg=true 
    local something_to_do=false

    # process arguments
    while [ $# -ne 0 ]; do
        case "$1" in
            "-h") # display help message and quit
                InfoMessage "$helpmsg"
                return 0
                ;;
            "-x") # relative movement in x direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                x=$(adj_number $2)
                shift 2
                something_to_do=true
                ;;
            "-y") # relative movement in y direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                y=$(adj_number $2)
                shift 2
                something_to_do=true
                ;;
            "-z"|"-a"|"-Y") # relative angular movement around z axis (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                z=$(adj_number $2)
                shift 2
                something_to_do=true
                ;;
            "-nb") # set blocking mode to false (default: true)
                blocking_mode=false
                shift 1
                ;;
            "-v") # set velocity (range: 0.05--0.2; default: 0.1)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 0 $miiwa_max_velocity && errno=3 && break
                velocity=$(adj_number $2)
                shift 2
                ;;
            "-rad") # angle in radians
                in_deg=false
                shift 1
                ;;
            *) # error in format
                errno=4; break
                ;;
        esac
    done

    # if an error occurred, disply error message and quit
    if [ $errno -ne 0 ]; then
        ErrorMessage "movrel: error #$errno: ${errmsg[$errno]}"
        return 1
    fi

    # do nothing if nothing to do
    if [ $something_to_do == "false" ]; then
        WarnMessage "Nothing to be done"
        return
    fi

    # convert angles to rad, if necesary
    test "$in_deg" == "true" && eval z=$(deg_to_rad $z)

    args="rel_pose: {x: $x, y: $y, theta: $z, cov_xx: 0.0, cov_yy: 0.0, cov_thetatheta: 0.0, cov_xy: 0.0, cov_xtheta: 0.0, cov_ytheta: 0.0}
parameter: {blocking: $blocking_mode, velocity: $velocity, blending: 0.0}"

    InfoMessage "rosservice call /miiwa/move_relative_platform $args"

    rosservice call /miiwa/move_relative_platform "$args"
}

# ----------------------------------------------------------------------

# wrapper to get_navigation_pose service call
getpose()
{
    # call the service to a variable
    local a=$(rosservice call /miiwa/get_navigation_pose "{}")
    miiwa_cur_x=$(echo $a | cut -d\  -f3)
    miiwa_cur_y=$(echo $a | cut -d\  -f5)
    miiwa_cur_theta_rad=$(echo $a | cut -d\  -f7)
    miiwa_cur_theta_deg=$(rad_to_deg $miiwa_cur_theta_rad)

    InfoMessage "Current pose: x: $miiwa_cur_x, y: $miiwa_cur_y theta: $miiwa_cur_theta_deg"
}

# ----------------------------------------------------------------------

