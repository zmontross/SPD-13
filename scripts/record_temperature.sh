#!/bin/bash

set -e


OUTDIR="/home/speedy/sysinfo_logs/temperature"

if [ -n $1 ]; then
	MAX=$1
else
	MAX=10
fi

PERIOD=5 # seconds

TIMESTAMP=$(date "+%s")

FILENAME="${OUTDIR}/temperature_${TIMESTAMP}.txt"


echo "Starting $(basename $0), ${MAX} samples, File '$(basename $FILENAME)'."

touch $FILENAME

echo 'XXyyy, XX Upper digits, yyy Lower digits' >> $FILENAME


samples=0
while [ $samples -le $MAX ]; do
	samples=$((samples+1))
	echo $(cat /sys/devices/virtual/thermal/thermal_zone0/temp) >> $FILENAME
	sleep $PERIOD
done

echo "Finished $(basename $0). See '$(basename $FILENAME)'."
