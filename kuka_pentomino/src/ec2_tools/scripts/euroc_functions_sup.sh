#!/bin/bash

#. ../../devel/setup.bash

# ----------------------------------------------------------------------

# general module help
_sup_help()
{
    local helpmsg="Available 'sup' functions:\n"
            helpmsg+="  mode          : show/change real/simulated mode\n"
            helpmsg+="  deg_to_rad    : convert degrees to radians\n"
            helpmsg+="  rad_to_deg    : convert radians to degress\n"
            helpmsg+="  is_number     : check if argument is a real number\n"
            helpmsg+="  adj_number    : convert to a real number accepted by ROS\n"
            helpmsg+="  on_range      : check if number is in range \n"

    WarnMessage "$helpmsg"
}

# ---------------------------------------------------------------------

# Display error message
ErrorMessage()
{
    echo -e "\e[01;31m${1}\e[0m" 1>&2 ;
}

# ---------------------------------------------------------------------

# Display info message
InfoMessage()
{
    echo -e "\e[01;34m${1}\e[0m" ;
}

# ---------------------------------------------------------------------

# Display warn message
WarnMessage()
{
    echo -e "\e[01;33m${1}\e[0m" ;
}

# ---------------------------------------------------------------------

# Function: check if the given argument is a number
# Returns:
#   0 - success
#   1 - argument not numeric
is_number()
{
#    test "$1" -eq "$1" 2>/dev/null && return 0
#    return 1
    echo "$1" | awk '{exit($1!=$1+0);}'
}

# ---------------------------------------------------------------------

# Function: check if given value is in the given range
# Returns:
#   0 - on success
#   1 - out of range
#   2 - argument not numeric OR invalid number of arguments
# Synopsis: on_range value lower_limit upper_limit
on_range()
{
    # check if arguments are numeric
    for i in $*; do
        is_number $i || return 2
    done

    # check number of arguments
    test $# -ne 3 && return 2

    # do 
    echo "$1" "$2" "$3" | awk '{exit($1<$2 || $1>$3);}'
}

# ---------------------------------------------------------------------

# Function: check if two float numbers are equal, possible within a tolerance
# Returns:
#   0 - are equal
#   1 - are different
#   2 - argument not numeric OR invalid number of arguments
# Synopsis: are_equal value1 value2 [tolerance]
are_equal()
{
    # check if arguments are numeric
    for i in $*; do
        is_number $i || return 2
    done
    
    # check arguments
    local tolerance=0
    case $# in
        2) ;;
        3) tolerance=$3 ;;
        *) return 2 ;;
    esac

    # do
    local exp="a=$1; b=$2; t=$tolerance; if (a>b) (a-b)>t else (b-a) > t"
    return $(echo "$exp" | bc -l)
}

# ---------------------------------------------------------------------

# Function: adj_to_range value to given range
# Returns:
#   0 - on success
#   2 - argument not numeric
# Synopsis: adj_to_range value lower_limit upper_limit
adj_to_range()
{
    test $# -ne 3 && return 1
    for i in $*; do
        is_number $i || return 2
    done
    echo "$1" "$2" "$3" | awk '{
            x = $1;
            if ($1 < $2) x = $2;
            else if ($1 > $3) x = $3;
            printf "%g", x;
            exit(0);
        }'
}

# ---------------------------------------------------------------------

# Function: adjust float value to observe ROS rules
adj_number()
{
    echo $1 | awk '{printf "%g", $1}'
}

# ---------------------------------------------------------------------

deg_to_rad()
{
    echo $1 | awk '{printf "%g", $0*atan2(1,1)/45.0}'
}

# ---------------------------------------------------------------------

rad_to_deg()
{
    echo $1 | awk '{printf "%.2f", $0*45.0/atan2(1,1)}'
}

# ---------------------------------------------------------------------

# change/show to real/simulated platform
mode()
{
    local helpmsg="mode [OPTIONS]\n"
        helpmsg+="OPTIONS:\n"
        helpmsg+="      : with no option show current mode\n"
        helpmsg+="  -h  : this help\n"
        helpmsg+="  -r1 : set miiwa 1 mode\n"
        helpmsg+="  -r2 : set miiwa 2 mode\n"
        helpmsg+="  -s  : set simulated mode\n"

    case $# in 
        0) # show if no arguments are given
            InfoMessage "mode = $__euroc_mode__"
            return
            ;;
        2) # error
            ErrorMessage "Too many arguments"
            return 1
            ;;
    esac

    case $1 in 
        "-s") # sim mode
            __euroc_mode__=simulated
            export ROS_MASTER_URI="http://localhost:11311"
            ;;
        "-r1") # real mode
            __euroc_mode__=miiwa_1
            export ROS_MASTER_URI="http://192.168.128.212:11311"
            ;;
        "-r2") # real mode
            __euroc_mode__=miiwa_2
            export ROS_MASTER_URI="http://192.168.128.214:11311"
            ;;
        "-h") # show help message
            WarnMessage "$helpmsg"
            return
            ;;
        *) # invalid mode
            ErrorMessage "Invalid argument"
            return 1
            ;;
    esac

    InfoMessage "$__euroc_mode__ mode set"
}

# ---------------------------------------------------------------------

mode -s
