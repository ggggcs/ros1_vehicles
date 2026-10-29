#! /bin/bash

result=$(pidof calibration)

start_node()
{
  source install/setup.bash
  if [ "$result" == "" ]; then
    ./build/calibration/calibration
  fi
}

stop_node()
{
  kill -9 $result
}

if [ "$1" == "start" ]; then
  start_node
elif [ "$1" == "stop" ]; then
  stop_node
fi

exit 0
