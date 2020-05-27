#!/bin/bash
make ARCH=arm64 ratta_sn078_defconfig
make ARCH=arm64 ratta-sn078.img -j8
