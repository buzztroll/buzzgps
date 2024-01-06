#!/bin/bash

set -u

cd $(dirname $0)

PIPENAME=$(pwd)/fakepipe

mkfifo ${PIPENAME}

./buzzgps-fake ${PIPENAME} 1 &
pgm_pid=$!

function cleanup {
    kill ${pgm_pid}
    wait ${pgm_pid}
}

trap cleanup EXIT

while read -r line; do
    echo ${line} >> ${PIPENAME}
    sleep 0.5
done < sample.txt