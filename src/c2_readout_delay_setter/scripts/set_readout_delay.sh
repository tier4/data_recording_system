#!/bin/bash

delay_in_ms=${1:-10}  # default: 10ms delay
bus=${2:-42}          # default: bus #42
ONE_H=0.0125          # 1H = 0.0125[ms] on IMX490

# Calculate the number of line to be set in HEX
delay_in_line=$(echo "${delay_in_ms} / ${ONE_H}" | bc |  xargs printf '%06x\n')

function write_to () {
    target_reg="0x${1}"
    value="0x${2}"
    regs=(0x33 0x47 0x15 0x0 0x0 0x0 0xe0 0x0 0x80 0x1 0x0 0x0 0x0 0x34 0x0 0x0 0x0
         ${target_reg} 0x1 0x0 0x0 ${value} 0x0 0x0 0x0 0x2 0x1)

    check_sum=0
    for i in ${regs[@]}; do
        ((check_sum+=i))
    done
    check_sum=$((check_sum & 0xff))
    check_sum="0x$(printf '%x\n' ${check_sum})"

    i2ctransfer -f -y ${bus} w28@0x6d $(echo "${regs[@]}" ${check_sum}) r6
}

# Write to EX_VRESET_VDLY, which is 3-byte length register, byte by byte
write_to "fd" $(echo ${delay_in_line:4:2})
sleep 0.1
write_to "fe" $(echo ${delay_in_line:2:2})
sleep 0.1
write_to "ff" $(echo ${delay_in_line:0:2})
