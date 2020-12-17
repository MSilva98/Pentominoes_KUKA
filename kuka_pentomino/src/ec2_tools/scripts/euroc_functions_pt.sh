#!/bin/bash

##################################
# pan&tilt manipulation
##################################

# ----------------------------------------------------------------------

# general module help
_pt_help()
{
    local helpmsg="Available 'pan&tilt' functions:\n"
        helpmsg+=" pt       : show/set pan&tilt\n"
        helpmsg+=" _ptset   : show/set pan&tilt (deprecated)\n"
        helpmsg+=" _pan     : show/set pan (deprecated)\n"
        helpmsg+=" _tilt    : show/set tilt (deprecated)\n"

    WarnMessage "$helpmsg"
}

# ----------------------------------------------------------------------

# change pan&tilt of mast cameras
# show current values if no arguments are given
_ptset()
{
    local args p t p2 t2

    if [ $# -eq 0 ]; then
        if [ -v _pt_pan_cur -a -v _pt_tilt_cur ]; then
            echo "pan = $_pt_pan_cur, tilt = $_pt_tilt_cur"
            return
        else
            ErrorMessage "pan and/or tilt never set"
            return 1
        fi
    fi

    if test $# -ne 2 || ! is_number "$1" || ! is_number "$2" ; then
        ErrorMessage "Missing or non numeric argument(s)"
        return 1
    fi

    p=$1; t=$2

    # check value limits
    if ! on_range $p -$_pt_pan_lim $_pt_pan_lim ; then
        WarnMessage "Requested pan out of range:"
        p=$(adj_to_range $p -$_pt_pan_lim $_pt_pan_lim)
        WarnMessage "  truncated to $p"
    fi
    if ! on_range $t -$_pt_tilt_lim $_pt_tilt_lim ; then
        WarnMessage "Requested tilt out of range:"
        t=$(adj_to_range $t -$_pt_tilt_lim $_pt_tilt_lim)
        WarnMessage "  truncated to $t"
    fi

    p2=$(deg_to_rad $p)
    t2=$(deg_to_rad $t)

    args="pan: $p2
tilt: $t2"
    rosservice call /pan_tilt/move_pan_tilt "$args" || return $?

    _pt_pan_cur=$p; _pt_tilt_cur=$t;
}

# ----------------------------------------------------------------------

# change pan
# show current value if no arguments are given
_pan()
{
    if [ $# -eq 0 ]; then
        if [ -v _pt_pan_cur ]; then
            echo "pan = $_pt_pan_cur"
            return
        else
            ErrorMessage "pan never set"
            return 1
        fi
    fi

    test $# -eq 1 || return 1
    test -v _pt_tilt_cur || _pt_tilt_cur=0
    _ptset $1 $_pt_tilt_cur
}

# ----------------------------------------------------------------------

# change tilt
# show current value if no arguments are given
_tilt()
{
    if [ $# -eq 0 ]; then
        if [ -v _pt_tilt_cur ]; then
            echo "tilt = $_pt_tilt_cur"
            return
        else
            ErrorMessage "tilt never set"
            return 1
        fi
    fi

    test $# -eq 1 || return 1
    test -v _pt_pan_cur || _pt_pan_cur=0
    _ptset $_pt_pan_cur $1
}

# ----------------------------------------------------------------------

# show/set pan&tilt
pt()
{
    # help message
    local helpmsg="\n"
        helpmsg+="           : without arguments show current pan&tilt\n"
        helpmsg+="  -h       : this help\n"
        helpmsg+="  -p angle : set pan (does not check limits in radians)\n"
        helpmsg+="  -t angle : set tilt (does not check limits in radians)\n"
        helpmsg+="  -r       : angles in radians (default: degress)\n"
        helpmsg+="  -xon     : turn xtion projection on\n"
        helpmsg+="  -xoff    : turn xtion projection off\n"

    # error messages
    local errno=0
    local errmsg[1]="missing argument"
    local errmsg[2]="value not a number"
    local errmsg[3]="value out of range"
    local errmsg[4]="unknown value for pan or tilt"
    local errmsg[5]="bad option or extra argument"
    local errmsg[6]="xon and xoff can not be set simultaneously"

    # what to do
    # bit 0 - deg=0/rad=1 ; MASK = 0x01
    # bit 1 - pan  ; MASK = 0x02
    # bit 2 - tilt ; MASK = 0x04
    # bit 3 - xon  ; MASK = 0x08
    # bit 4 - xoff ; MASK = 0x10
    local what_to_do=0

    local in_rad=false
    local p t p2 t2 args

    # check arguments
    while [ $# -ne 0 ]; do
        case $1 in
            "-h") # show help
                WarnMessage "$helpmsg"
                return
                ;;
            "-r") # angles in radians
                what_to_do=$(($what_to_do | 1))
                in_rad=true
                shift 1
                ;;
            "-p") # set pan
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 -$_pt_pan_lim $_pt_pan_lim && errno=3 && break
                p=$(adj_number $2)
                what_to_do=$(($what_to_do | 2))
                shift 2
                ;;
            "-t") # set tilt
                test $# -eq 1 && errno=1 && break
                ! is_number "$2" && errno=2 && break
                ! on_range $2 -$_pt_tilt_lim $_pt_tilt_lim && errno=3 && break
                t=$(adj_number $2)
                what_to_do=$(($what_to_do | 4))
                shift 2
                ;;
            "-xon") # set xtion projection on
                test $(($what_to_do & 0x10)) -ne 0 && errno=6 && break
                what_to_do=$(($what_to_do | 0x08))
                shift 1
                ;;
            "-xoff") # set xtion projection off
                test $(($what_to_do & 0x08)) -ne 0 && errno=6 && break
                what_to_do=$(($what_to_do | 0x10))
                shift 1
                ;;
            *) # unknown value or option
                errno=5; break
                ;;
        esac
    done

    # check if values for both pan and tilt are valid, if an action involving them exists
    test $(($what_to_do & 0x06)) -ne 0 -a $_pt_not_set == true -a $(($what_to_do & 0x06)) -ne 6 && errno=4

    # check if an error exists
    if [ $errno -ne 0 ]; then
        ErrorMessage "pt: ${errmsg[$errno]}"
        return 1
    fi

    # show or calculate pan & tilt to apply
    case $(($what_to_do & 7)) in
        0) # show pan&tilt in degrees
            ;;
        1) # show pan&tilt in radians
            echo "pan = $(deg_to_rad $_pt_pan_cur), tilt = $(deg_to_rad $_pt_tilt_cur)"
            return
            ;;
        2) # only pan (in degrees), keep tilt
            t=$_pt_tilt_cur
            ;;
        3) # only pan (in radians), convert it to degrees and keep tilt
            p=$(rad_to_deg $p)
            t=$_pt_tilt_cur
            ;;
        4) # only tilt (in degrees), keep pan
            p=$_pt_pan_cur
            ;;
        5) # only tilt (in radians), convert it to degrees and keep pan
            t=$(rad_to_deg $t)
            p=$_pt_pan_cur
            ;;
        6) # pan&tilt (in degrees), do nothing
            ;;
        7) # pan&tilt (in radians), convert both to degrees
            p=$(rad_to_deg $p)
            t=$(rad_to_deg $t)
            ;;
    esac

    local args

    # apply xtion action, if required
    local ret1=0
    if [ $(($what_to_do & 0x18)) -ne 0 ]; then
        if [ $__euroc_mode__ == "simulated" ]; then
            WarnMessage "xtion action does not exist in \"simulted\" mode; ignoring"
        else
            case $(($what_to_do & 0x18)) in
                "8") #
                    rosservice call /xtion_projector/turn_projector_on "{}"
                    ret1=$?
                    ;;
                "16") #
                    rosservice call /xtion_projector/turn_projector_off "{}"
                    ret1=$?
                    ;;
            esac
        fi
    fi

    # apply pan & tilt, if required
    local ret2=0
    if [ $(($what_to_do & 0x06)) -ne 0 ] ; then
        args="pan: $(deg_to_rad $p)
tilt: $(deg_to_rad $t)"
        rosservice call /pan_tilt/move_pan_tilt "$args"
        ret2=$?

        if [ $ret2 -eq 0 ]; then
            _pt_pan_cur=$p; _pt_tilt_cur=$t; _pt_not_set=false
            echo "pan = $_pt_pan_cur, tilt = $_pt_tilt_cur"
        fi
    fi

    return $(($ret1 | $ret2))
}

# ----------------------------------------------------------------------

##################################
# initial setup
##################################

# ----------------------------------------------------------------------

_pt_pan_lim=180;
_pt_tilt_lim=120;

if ! [ -v _pt_not_set ]; then
    _pt_not_set=true
    _pt_pan_cur=nan
    _pt_tilt_cur=nan
fi

