#!/bin/bash

# use getopt to parse the command-line arguments
while getopts "c:p:r:h:f:" opt; do
  case ${opt} in
    c) CELLROWS=$OPTARG
       CELLCOLS=$OPTARG;;
    p) POS=$OPTARG;;
    r) ROBNAME=$OPTARG;;
    h) HOST=$OPTARG;;
    f) MAPFNAME=$OPTARG
       PATHFNAME=$OPTARG;;
    \?) echo "Invalid option -$OPTARG" >&2
        exit 1;;
  esac
done

python3 mainRob.py --pos $POS --robname $ROBNAME --host $HOST --filename $MAPFNAME