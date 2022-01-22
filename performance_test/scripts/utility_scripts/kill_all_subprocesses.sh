#!/bin/bash


## trap interrupt signal from keyboard (ctrl + c) to kill all subprocesses
## trapping HUP is not necessary, but prints a nicer output message
intexit() {
    # Kill all subprocesses (all processes in the current process group)
    kill -HUP -$$
}

hupexit() {
    # HUP'd (probably by intexit)
    echo
    echo "Interrupted"
    exit
}

trap hupexit HUP
trap intexit INT