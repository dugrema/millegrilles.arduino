#!/bin/bash

AVR_PATH=/usr/share/arduino-1.8.5/hardware/tools/avr
AVRDUDE=$AVR_PATH/bin/avrdude
AVR_CONF=$AVR_PATH/etc/avrdude.conf
FILE=$2
PORT=$1

$AVRDUDE -C$AVR_CONF -v \
         -patmega328p -carduino \
         -P$PORT -b57600 \
         -D -Uflash:w:$FILE:i

