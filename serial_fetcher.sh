#!/bin/bash
udevadm info --query=property --name=$1 | grep ID_SERIAL
