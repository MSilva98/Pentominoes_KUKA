#!/bin/bash

##################################
# arm manipulation
##################################

# ----------------------------------------------------------------------

# general module help
_arm_help()
{
    local helpmsg="Available 'arm' functions:\n"
        #helpmsg+="  j_        : show/set arm joints (previous version)\n"
        helpmsg+="  j          : show/set arm joints (under test)\n"
        helpmsg+="  jreset     : set all arm joints to 0\n"
        helpmsg+="  g          : show/set gripper position (under test)\n"
        helpmsg+="  gopen      : open the gripper\n"
        helpmsg+="  gclose     : close the gripper\n"
        helpmsg+="  tcpmove    : move relative tcp (only in real mode)\n"
        helpmsg+="  tcpmoveimp : move relative tcp impedance (only in real mode)\n"

    WarnMessage "$helpmsg"
}

# ----------------------------------------------------------------------

# NOT TO BE CALLED DIRECTLY
# synopsis: __arm_save_tags__ «filename»
__arm_save_tags__()
{
    test $# -eq 1 -a -f $1 || return 1

    for t in ${!__jtag__[@]}
    do
        echo "jtag:$t:${__jtag__[$t]}" >> $1
    done
}

# ----------------------------------------------------------------------

# NOT TO BE CALLED DIRECTLY
# synopsis: __arm_load_tags__ «filename»
__arm_load_tags__()
{
    test $# -eq 1 -a -f $1 || return 1

    egrep '^jtag:' "$fname" | \
    while read p; 
    do
        key=$(echo $p | cut -d: -f2)
        value=$(echo $p | cut -d: -f3)
        __jtag__[$key]="$value"
        echo "jtag (\"$key\", ${__jtag__[$key]}) loaded"
    done
}

# ----------------------------------------------------------------------

# get current positions of arm joints
_jget()
{
    local a i v 

    # get joint positions from ROS topic
    a="$(rostopic echo -n 1 /miiwa/joint_state | grep '^positions' | tr -d 'positions:[] ')"

    # abort if fail
    test -z $a && return 1

    # extract joint positions
    i=0
    for v in $(echo "$a" | awk 'BEGIN {FS=","; OFS=" "} {print $1,$2,$3,$4,$5,$6,$7;}')
    do
        i=$(($i + 1))
        jcur[$i]=$(rad_to_deg $v)
    done
}

# ----------------------------------------------------------------------

# Function: get and show current values of arm joints
# Parameters: none
jshow()
{
    _jget || return 1
    echo "current joint positions: ${jcur[*]}"
}

# ----------------------------------------------------------------------

# Function: set arm joints to given positions
# Parameters: jset [-d(eg)] angle1 angle2 ... angle7
jset()
{
    local to_convert=false
    case "$1" in
        "-d"|"-deg") # angles are given in degrees
            to_convert=true
            shift 1
            ;;
    esac

    # check number of parameters
    if [ $# -ne 7 ]; then
        ErrorMessage "Wrong number of parameters"
        return 1;
    fi

    # fill in jval[] with angles, converting to radians if necessary
    i=0;
    for v in $*; do
        if ! is_number $v; then
            ErrorMessage "$v is not a number"
            return 1
        fi
        i=$(($i + 1))
        if [ $to_convert == true ]; then
            jval[$i]=$(deg_to_rad $v)
        else
            jval[$i]=$(adj_number $v)
        fi
    done

    #echo "applying ${jval[*]}"

    # apply
    rosservice call /miiwa/move_joints "desired_joint_positions:
  values: [ ${jval[1]} , ${jval[2]} , ${jval[3]} , ${jval[4]} , ${jval[5]} , ${jval[6]} , ${jval[7]} ]
parameter:
  blocking: true
  velocity: 0.1
  blending: 0.0"

    test $? -ne 0 && return 1

    # update current position if applying succeed
    # InfoMessage "Waiting 1 second to have confidence on current values"
    # sleep 1
    _jshow
}

# ----------------------------------------------------------------------

# Function: set arm joints to given positions
# Parameters: _jset angle1 angle2 ... angle7
_jset()
{
    if [ $# -ne 7 ]; then
        ErrorMessage "Wrong parameters"
        return 1;
    fi

    # convert to radians
    i=0;
    for v in $*; do
        i=$(($i + 1))
        jval[$i]=$(deg_to_rad $v)
    done

    #echo "applying ${jval[*]}"

    # apply
    rosservice call /miiwa/move_joints "desired_joint_positions:
  values: [ ${jval[1]} , ${jval[2]} , ${jval[3]} , ${jval[4]} , ${jval[5]} , ${jval[6]} , ${jval[7]} ]
parameter:
  blocking: true
  velocity: 0.1
  blending: 0.0"

    test $? -ne 0 && return 1

    # update current position if applying succeed
    # InfoMessage "Waiting 1 second to have confidence on current values"
    # sleep 1
    _jshow
}

# ----------------------------------------------------------------------

# Function: set all arm joints to reset position
# Parameters: none
jreset()
{
    j -1 0 -2 0 -3 0 -4 0 -5 0 -6 0 -7 0 $*
}

# ----------------------------------------------------------------------

# moving arm joints
j()
{
    # min/max values;
    # v7 = min(v,vmax)
    # vmax1to6 = min(v,vmax,vmaxhard)
    local vmax="1.0"   vmin="0.1"  vmaxhard="0.4"  # in m/s

    # help message 
    local helpmsg="j [OPTIONS]\n"
            helpmsg+="OPTIONS\n"
            helpmsg+="              : show current positions, if no arguments are given\n"
            helpmsg+="  -h          : this help\n"
            helpmsg+="  -[1-7] num  : set joint angle\n"
            helpmsg+="  -nb         : set non blocking mode (default: blocking)\n"
            helpmsg+="  -rad        : angles in radians (default: degrees) (TO BE IMPLEMENTED)\n"
            helpmsg+="  -v num      : set velocity (default: 0.1; range: [${vmin},${vmax}]\n"
            helpmsg+="  -saveas tag : save current/applied position as tag\n"
            helpmsg+="  -tag tag    : set joint angles to tag values\n"

    # error messages
    local errmsg[1]="missing argument"
    local errmsg[2]="argument is not a number"
    local errmsg[3]="argument is out of range"
    local errmsg[4]="unknown argument"
    local errmsg[5]="option still not implemented"
    local errmsg[6]="tag does not exist"
    local errmsg[7]="tag and normal movement at the same time is not allowed"
    local errno=0

    # joint limits
    local jlim
    jlim[1]=170
    jlim[2]=120
    jlim[3]=170
    jlim[4]=120
    jlim[5]=170
    jlim[6]=120
    jlim[7]=175

    # what to do 
    # bit 1 - normal joint movement
    # bit 2 - apply tag
    # bit 3 - save as tag
    # bits 1 and 2 can not be on at the same time
    local what_to_do=0

    # local variables
    local velocity=0.1 blocking_mode=true in_rad=false
    local jn
    local tag

    # get current joint positions
    _jget
    for i in $(seq 1 7) ; do
        eval jnew[$i]="${jcur[$i]}"
    done
    local jindices=""

    # process arguments
    while [ $# -gt 0 ]; do
        case "$1" in 
            "-h") # display synopsis and quit
                WarnMessage "$helpmsg"
                return
                ;;
            "-"[1-7]) # requested joint
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                jn=$((0 - $1))
                ! on_range $2 -${jlim[$jn]} ${jlim[$jn]} && errno=3 && break
                test $(($what_to_do & 2)) -eq 2 && errno=7 && break
                eval jnew[$jn]=$(adj_number $2)
                jindices+=" $jn"
                what_to_do=$(($what_to_do | 1))
                shift 2
                ;;
            "-v") # requested velocity
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 $vmin $vmax && errno=3 && break
                velocity=$(adj_number $2)
                shift 2
                ;;
            "-nb") # set blocking mode to false (default: true)
                blocking_mode=false
                #errno=5 && break
                shift 1
                ;;
            "-rad") # angle in radians
                in_rad=true
                #errno=5 && break
                shift 1
                ;;
            "-tag") # apply tag values
                test $# -eq 1 && errno=1 && break
                test $(($what_to_do & 1)) -eq 1 && errno=7 && break
                ! test -v __jtag__["$2"] && errno=6 && break
                what_to_do=$(($what_to_do | 2))
                jn=1
                for jv in ${__jtag__["$2"]}
                do
                    jnew[$jn]=$jv
                    jn=$(($jn + 1))
                done
                shift 2
                ;;
            "-saveas") # save as tag
                test $# -eq 1 && errno=1 && break
                tag="$2"
                what_to_do=$(($what_to_do | 4))
                shift 2
                ;;
            *) # other situations (unknown argument)
                errno=4
                break
                ;;
        esac
    done

    # if an error occurred, disply error message and quit
    if [ $errno -ne 0 ]; then
        ErrorMessage "j: ${errmsg[$errno]}"
        return 1
    fi

    # complete joint values to applied, in degrees, converting from rad to deg, if necessary
    if [ $(($what_to_do & 1)) -eq 1 ]; then # case normal
        for jn in $jindices
        do
            if [ $in_rad == true ]; then
                jnew[$jn]=$(rad_to_deg ${jnew[$jn]})
            fi
        done
    fi

    # truncate velocity, depending on changed joints
    if [ $(echo "$velocity > $vmaxhard" | bc -l) -eq 1 ]; then
        for jn in 1 2 3 4 5 6
        do
            if ! are_equal ${jnew[$jn]} ${jcur[$jn]} 0.5; then 
                velocity=$vmaxhard
                WarnMessage "velocity truncated to $velocity"
                break
            fi
        done
    fi

    # convert angles to radians
    for jn in 1 2 3 4 5 6 7
    do
        jnew[$jn]=$(deg_to_rad ${jnew[$jn]})
    done

    local ret=0

    # apply, if required
    if [ $(($what_to_do & 3)) -ne 0 ]; then
        rosservice call /miiwa/move_joints "desired_joint_positions:
  values: [ ${jnew[1]} , ${jnew[2]} , ${jnew[3]} , ${jnew[4]} , ${jnew[5]} , ${jnew[6]} , ${jnew[7]} ]
parameter:
  blocking: ${blocking_mode}
  velocity: ${velocity}
  blending: 0.0"

        ret=$?
    fi

    jshow

    # save tag, if required
    if [ $(($what_to_do & 4)) -ne 0 ]; then
        #declare -A __jtag__
        __jtag__["$tag"]="${jcur[@]}"
        echo "keys: ${!__jtag__[@]}"
    fi

    return $ret

}

# ----------------------------------------------------------------------

##################################
# gripper manipulation
##################################

# ----------------------------------------------------------------------

# Function: close the gripper
# Parameters: 
gclose()
{
    local force

    if [ $# -eq 0 ]; then
        force=80
    else
        force=$1
    fi

    rosservice call /gripper/grasp "direction: 1
velocity: 0.2
force: $force"
}

# ----------------------------------------------------------------------

# Function: open the gripper
# Parameters: 
gopen()
{
    rosservice call /gripper/release "direction: 0
velocity: 0.2"
}

# ----------------------------------------------------------------------

# Function: move the gripper
g()
{
    local helpmsg="g [OPTIONS]\n"
            helpmsg+="OPTIONS\n"
            helpmsg+="           : show current position, if no arguments are given\n"
            helpmsg+="  -h       : this help\n"
            helpmsg+="  -m num   : set position (default: 0.007; range: [0.007,0.10]; in m)\n"
            helpmsg+="  -p num   : set position (default: 0.7; range: [0.7,10]; in cm)\n"
            helpmsg+="  -v num   : set velocity (default: 0.1; range: [0.1,0.2]\n"
            helpmsg+="  -f force : set force (default: 80; range: [10,80]\n"

    local errmsg[1]="missing argument"
    local errmsg[2]="argument is not a number"
    local errmsg[3]="argument is out of range"
    local errmsg[4]="unknown argument"
    local errno=0

    # if no arguments are given, show gripper position
    if [ $# -eq 0 ]; then
        _gshow
        return 
    fi

    # min/max values for position, velocity and force
    local pmax="11.0"  pmin="0.7"   # in cm
    local mmax="0.11"  mmin="0.007"   # in m
    local vmax="0.2"   vmin="0.1"   # in m/s
    local fmax="80.0"  fmin="10.0"  # 

    # default values (position in cm)
    local position="$mmin" velocity="0.1" force="80.0"

    while [ $# -gt 0 ]; do
        case "$1" in 
            "-h") # display synopsis and quit
                WarnMessage "$helpmsg"
                return
                ;;
            "-m") # requested position in meters
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 $mmin $mmax && errno=3 && break
                position=$(adj_number $2)
                shift 2
                ;;
            "-p") # requested position in cm
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 $pmin $pmax && errno=3 && break
                position=$(adj_number $(echo "$2 / 100.0" | bc -l))
                shift 2
                ;;
            "-v") # requested velocity
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 $vmin $vmax && errno=3 && break
                velocity=$(adj_number $2)
                shift 2
                ;;
            "-f") # requested force
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 $fmin $fmax && errno=3 && break
                force=$(adj_number $2)
                shift 2
                ;;
            *) # other situations
                errno=4
                break
                ;;
        esac
    done

    # if an error occurred, disply error message and quit
    if [ $errno -ne 0 ]; then
        ErrorMessage "g: ${errmsg[$errno]}"
        return 1
    fi

    args="position: $position
velocity: $velocity
force: $force"

    InfoMessage "Applying:\n$args"
    
    rosservice call /gripper/move "$args"
    return $?
}

# ----------------------------------------------------------------------

# get current gripper position
_gget()
{
    local a
    a=$(rosservice call /gripper/get_position | grep "position")
    test $? -ne 0 && return 1

    gcur=$(echo "$a" | awk '{printf "%.2g", $2*100}')
}

# ----------------------------------------------------------------------

# get and show current gripper position
_gshow()
{
    _gget
    echo "current gripper position: $gcur cm"
}

# ----------------------------------------------------------------------

##################################
# tcp manipulation
##################################

# ----------------------------------------------------------------------

# wrapper to rosservice call /miiwa/move_relative_tcp
tcpmove()
{
    local x=0 y=0 z=0 
    local a=0 b=0 c=0 
    local blocking_mode=true velocity=0.1 in_deg=true errno=0
    local errmsg[1]="missing argument for at least one of the options"
    local errmsg[2]="given argument is not a number"
    local errmsg[3]="unknown argument"
    local vmin=0.05 vmax=0.2
    local args

    # process arguments
    while [ $# -ne 0 ]; do
        case "$1" in
            "-h") # display help message and quit
                helpmsg="tcpmove [OPTIONS]\n"
                helpmsg+="OPTIONS:\n"
                helpmsg+="  -h        : display help message and quit\n"
                helpmsg+="  -x num    : relative movement in x direction (default: 0)\n"
                helpmsg+="  -y num    : relative movement in y direction (default: 0)\n"
                helpmsg+="  -z num    : relative movement in z direction (default: 0)\n"
                helpmsg+="  -a num    : relative angular movement around z direction (default: 0)\n"
                helpmsg+="  -b num    : relative angular movement around y direction (default: 0)\n"
                helpmsg+="  -c num    : relative angular movement around x direction (default: 0)\n"
                helpmsg+="  -nb       : set blocking mode to false (default: true)\n"
                helpmsg+="  -rad      : angles in radians (default: degrees)\n"
                helpmsg+="  -v num    : set velocity (range: 0.05--0.2; default: 0.1)\n"
                WarnMessage "$helpmsg"
                return 0
                ;;
            "-x") # relative movement in x direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                x=$(adj_number $2)
                shift 2
                ;;
            "-y") # relative movement in y direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                y=$(adj_number $2)
                shift 2
                ;;
            "-z") # relative movement in z direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                z=$(adj_number $2)
                shift 2
                ;;
            "-a") # relative angular movement around z direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                a=$(adj_number $2)
                shift 2
                ;;
            "-b") # relative angular movement around z direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                b=$(adj_number $2)
                shift 2
                ;;
            "-c") # relative angular movement around z direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                c=$(adj_number $2)
                shift 2
                ;;
            "-nb") # set blocking mode to false (default: true)
                blocking_mode=false
                shift 1
                ;;
            "-v") # set velocity (range: 0.05--0.2; default: 0.1)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                velocity=$(adj_number $2)
                shift 2
                ;;
            "-rad") # angle in radians
                in_deg=false
                shift 1
                ;;
            *) # error in format, unexpected argument
                errno=3; break
                ;;
        esac
    done

    # if an error occurred, disply error message and quit
    if [ $errno -ne 0 ]; then
        ErrorMessage "tcpmove: ${errmsg[$errno]}"
        return 1
    fi

    # check if velocity is on range
    on_range $velocity $vmin $vmax
    if [ $? -ne 0 ]; then
        ErrorMessage "tcpmove: velocity out of range"
        return 1
    fi

    # convert angles to rad, case it is necessary
    if [ $in_deg == true ]; then
        a=$(deg_to_rad $a)
        b=$(deg_to_rad $b)
        c=$(deg_to_rad $c)
    fi

    args="transformation: {x: $x, y: $y, z: $z, a: $a, b: $b, c: $c}
parameter: {blocking: $blocking_mode, velocity: $velocity, blending: 0.0}"

    InfoMessage "applying: $args"

    rosservice call /miiwa/move_relative_tcp "$args"
    ret=$?

    [ $ret -eq 0 ] && jshow
    return $ret
}

# ----------------------------------------------------------------------

# wrapper to rosservice call /miiwa/move_relative_impedance_tcp
tcpmoveimp()
{
    local x=0 y=0 z=0 
    local a=0 b=0 c=0 
    local sx=1000 sy=1000 sz=1000
    local sa=300 sb=300 sc=300
    local imp_mode=false
    local blocking_mode=true velocity=0.1 in_deg=true errno=0
    local errmsg[1]="missing argument for at least one of the options"
    local errmsg[2]="given argument is not a number"
    local errmsg[3]="unknown argument"
    local vmin=0.05 vmax=0.2
    local args cmd

    # process arguments
    while [ $# -ne 0 ]; do
        case "$1" in
            "-h") # display help message and quit
                helpmsg="tcpmoveimp [OPTIONS]\n"
                helpmsg+="OPTIONS:\n"
                helpmsg+="  -h        : display help message and quit\n"
                helpmsg+="  -x num    : relative movement in x direction (default: 0)\n"
                helpmsg+="  -y num    : relative movement in y direction (default: 0)\n"
                helpmsg+="  -z num    : relative movement in z direction (default: 0)\n"
                helpmsg+="  -a num    : relative angular movement around z direction (default: 0)\n"
                helpmsg+="  -b num    : relative angular movement around y direction (default: 0)\n"
                helpmsg+="  -c num    : relative angular movement around x direction (default: 0)\n"
                helpmsg+="  -imp      : set impedance mode on, even if no stiffness is given (default: false)\n"
                helpmsg+="  -sx num   : stiffness in x direction (default: 1000)\n"
                helpmsg+="  -sy num   : stiffness in y direction (default: 1000)\n"
                helpmsg+="  -sz num   : stiffness in z direction (default: 1000)\n"
                helpmsg+="  -sa num   : stiffness in a angular direction (default: 300)\n"
                helpmsg+="  -sb num   : stiffness in b angular direction (default: 300)\n"
                helpmsg+="  -sc num   : stiffness in c angular direction (default: 300)\n"
                helpmsg+="  -nb       : set blocking mode to false (default: true)\n"
                helpmsg+="  -rad      : angles in radians (default: degrees)\n"
                helpmsg+="  -v num    : set velocity (range: 0.05--0.2; default: 0.1)\n"
                WarnMessage "$helpmsg"
                return 0
                ;;
            "-x") # relative movement in x direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                x=$(adj_number $2)
                shift 2
                ;;
            "-y") # relative movement in y direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                y=$(adj_number $2)
                shift 2
                ;;
            "-z") # relative movement in z direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                z=$(adj_number $2)
                shift 2
                ;;
            "-a") # relative angular movement around z direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                a=$(adj_number $2)
                shift 2
                ;;
            "-b") # relative angular movement around z direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                b=$(adj_number $2)
                shift 2
                ;;
            "-c") # relative angular movement around z direction (default: 0)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                c=$(adj_number $2)
                shift 2
                ;;
            "-sx") # stiffness in z direction (default: 1000)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                sx=$(adj_number $2)
                shift 2
                imp_mode=true
                ;;
            "-sy") # stiffness in z direction (default: 1000)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                sy=$(adj_number $2)
                shift 2
                imp_mode=true
                ;;
            "-sz") # stiffness in z direction (default: 1000)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                sz=$(adj_number $2)
                shift 2
                imp_mode=true
                ;;
            "-sa") # stiffness in a angular direction (default: 300)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                sa=$(adj_number $2)
                shift 2
                imp_mode=true
                ;;
            "-sb") # stiffness in a angular direction (default: 300)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                sb=$(adj_number $2)
                shift 2
                imp_mode=true
                ;;
            "-sc") # stiffness in a angular direction (default: 300)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                sc=$(adj_number $2)
                shift 2
                imp_mode=true
                ;;
            "-imp") # set impedance mode to true (default: false)
                imp_mode=true
                shift 1
                ;;
            "-nb") # set blocking mode to false (default: true)
                blocking_mode=false
                shift 1
                ;;
            "-v") # set velocity (range: 0.05--0.2; default: 0.1)
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                velocity=$(adj_number $2)
                shift 2
                ;;
            "-rad") # angle in radians
                in_deg=false
                shift 1
                ;;
            *) # error in format, unexpected argument
                errno=3; break
                ;;
        esac
    done

    # if an error occurred, disply error message and quit
    if [ $errno -ne 0 ]; then
        ErrorMessage "tcpmove: ${errmsg[$errno]}"
        return 1
    fi

    # check if velocity is on range
    on_range $velocity $vmin $vmax
    if [ $? -ne 0 ]; then
        ErrorMessage "tcpmove: velocity out of range"
        return 1
    fi

    # convert angles to rad, case it is necessary
    if [ $in_deg == true ]; then
        a=$(deg_to_rad $a)
        b=$(deg_to_rad $b)
        c=$(deg_to_rad $c)
    fi

    # service called depends on arguments
    # if stiffness is given, impedance service is called
    if [[ $imp_mode == true ]]; then
        cmd="rosservice call /miiwa/move_relative_impedance_tcp"
        args="transformation: {x: $x, y: $y, z: $z, a: $a, b: $b, c: $c}
stiffness: {x: $sx, y: $sy, z: $sz, a: $sa, b: $sb, c: $sc}
parameter: {blocking: $blocking_mode, velocity: $velocity, blending: 0.0}"
    else
        cmd="rosservice call /miiwa/move_relative_tcp"
        args="transformation: {x: $x, y: $y, z: $z, a: $a, b: $b, c: $c}
parameter: {blocking: $blocking_mode, velocity: $velocity, blending: 0.0}"
    fi

    InfoMessage "applying: $cmd $args"
    $cmd "$args"
    ret=$?
    [ $ret -eq 0 ] && jshow
    return $ret
}

# ----------------------------------------------------------------------

unset jcur __jtag__
declare -A __jtag__

# ----------------------------------------------------------------------

##################################
# initial setup
##################################

# limit values for joint positions


