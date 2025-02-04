#!/bin/bash
#
#script to delete old log files associated with old roscore
#
# Thu 27 Apr 2023
# Use: bash delete_old_logs.bash
#

cd ~/.ros/log/
out=$(ls -lR latest)
filename=${out##*/}
echo $filename
for x in *
do
    if [ "$x" == $filename ] || [ "$x" == "latest" ]
    then
        echo "keeping only latest log files"
    else
        rm -rf $x;
    fi
done;
