#!/bin/bash
(./astra-goaldetect > output_pipe &)
#PID1=$!
(nc -l -p 8222 < output_pipe &)
#PID2=$!
echo "Running commands as $PID1 and $PID2"
read -p "Press any key to stop running..." -n1 -s
killall nc
killall ./astra-goaldetect
