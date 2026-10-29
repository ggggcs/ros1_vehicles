#! /bin/bash 

# left 20%
test_left_20()
{
  cansend can1 609#5A01D00700000000
  cansend can1 609#A40100FF00000000
}
#test_left_20

# left and right 100%
left_and_right_80()
{
  while [ ! -f /tmp/stop_can ]; do
#    cansend can1 609#5A01401F00000000 && cansend can1 609#A40100FF00000000
#    cansend can1 609#5B01401F00000000 && cansend can1 609#A50100FF00000000
    cansend can1 609#5A01701700000000 && cansend can1 609#A40100FF00000000
    cansend can1 609#5B01701700000000 && cansend can1 609#A50100FF00000000
    sleep 0.05
  done
}
#left_and_right_80

test_mk_mid()
{
  while [ true ]; do
    cansend can0 18C4D2D0#0400000000001014
    sleep 0.01
    cansend can0 18C4D2D0#0400000000002024
    sleep 0.01
    cansend can0 18C4D2D0#0400000000003034
  done
}
test_mk_mid
