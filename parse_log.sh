#!/bin/bash
#extract all lat and lon as csv from logfile
LOGFILE="log-follow.log"
grep lat $LOGFILE | grep -o "[0-9]*\.[0-9]*" | tr '\n' ',' | sed 's/[0-9]\.[0-9]\{2,3\}\,/\n/g'
