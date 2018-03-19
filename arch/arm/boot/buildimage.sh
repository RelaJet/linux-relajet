#!/bin/bash
mkimage -A arm -O linux -T kernel -C none -a 0x1008000 -e 0x1008000 -n 'Linux-3.2.7' -d $1 $2