#!/bin/bash

# ----------------------------------------------------------------------



# ----------------------------------------------------------------------

euroc_help()
{
    local helpmsg="Give an argument to get help\n"
        helpmsg+="  arm         : euroc_help arm\n"
        helpmsg+="  navigation  : euroc_help nav\n"
        helpmsg+="  pan&tilt    : euroc_help pt\n"
        helpmsg+="  support     : euroc_help sup\n"

    if [ $# -eq 0 ]; then
        WarnMessage "$helpmsg"
        return
    fi

    case $1 in
        "arm")  _arm_help;;
        "nav")  _nav_help;;
        "pt")  _pt_help;;
        "sup")  _sup_help;;
        *) # bad argument(s)
            ErrorMessage "euroc_help: bad argument(s)"
            return 1
            ;;
    esac
}

# ----------------------------------------------------------------------

# Function: save tags in default or given file
# Synopsis: euroc_save [-f «filename» ]
euroc_save()
{
    homedir=$(cd ; pwd)
    fname="$homedir/.euroc_functions/tags"

    # check arguments
    while [ $# -ne 0 ]
    do
        case $1 in
            "-f") # set output file
                test $# -eq 1 && return 1
                fname="$2"
                shift 2
                ;;
            *) # wrong value
                return 1
                ;;
        esac
    done

    mkdir -p "$(dirname "$fname")"
    echo -n "" > "$fname"

    # call local savers
    __arm_save_tags__ $fname
}

# ----------------------------------------------------------------------

# Function: load tags in default or given file
# Synopsis: euroc_load [-f «filename» ]
euroc_load()
{
    homedir=$(cd ; pwd)
    fname="$homedir/.euroc_functions/tags"

    # check arguments
    while [ $# -ne 0 ]
    do
        case $1 in
            "-f") # set output file
                test $# -eq 1 && return 1
                fname="$2"
                shift 2
                ;;
            *) # wrong value
                return 1
                ;;
        esac
    done

    test ! -f "$fname" && return 1

    # call local loaders
    __arm_load_tags__ $fname
}

# ----------------------------------------------------------------------

##################################
# initial setup
##################################

# ----------------------------------------------------------------------
set -- $(hostname -I)
export ROS_IP=$1
export ROS_HOSTNAME=$1

for suffix in sup arm nav pt wld ; do
    echo "sourcing euroc_functions_${suffix}.sh"
    source "$(rospack find ec2_tools)/scripts/euroc_functions_${suffix}.sh"
done

alias euroc_resource='source "$(rospack find ec2_tools)/scripts/euroc_functions.sh"'

# ----------------------------------------------------------------------


