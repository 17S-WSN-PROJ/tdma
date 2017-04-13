#!/bin/bash

make
set -m
for f in /dev/tty.usbserial*
do
      make program -e "PROGRAMMING_PORT=$f" &
done

# Wait for all parallel jobs to finish
while [ 1 ]; do fg 2> /dev/null; [ $? == 1 ] && break; done
