#/bin/bash

I=can1
ip l set $I down
ip l set $I type can bitrate 500000
ip l set $I type can tq 50 prop-seg 17 phase-seg1 17 phase-seg2 5 sjw 1
ip l set $I txqueuelen 1000
ip l set $I up
ip -det -stat link show $I

