#! /bin/bash

ping -w 1 192.168.9.51 >/dev/null # 4d毫米波
if [ $? -eq 0 ]; then
  echo ""
  echo "[4D毫米波] is ok"
else
  echo "[4D毫米波] is not ok"
fi
echo ""

ping -w 1 192.168.1.100 >/dev/null # 双目红外相机 
if [ $? -eq 0 ]; then
  echo ""
  echo "[双目红外相机] is ok"
else
  echo "[双目红外相机] is not ok"
fi
echo ""

ping -w 1 192.168.2.200 >/dev/null # 激光
if [ $? -eq 0 ]; then
  echo ""
  echo "[激光] is ok"
else
  echo "[激光] not ok"
fi
echo ""

ping -w 1 192.168.9.151 >/dev/null # 组合导航
if [ $? -eq 0 ]; then
  echo ""
  echo "[组合导航] is ok"
else
  echo "[组合导航] not ok"
fi
echo ""

ping -w 1 192.168.117.117 >/dev/null # pano camera(4 cameras)
if [ $? -eq 0 ]; then
  echo ""
  echo "[4目环视] is ok"
else
  echo "[4目环视] not ok"
fi
echo ""
