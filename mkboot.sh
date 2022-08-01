#!/bin/bash

rm boot.img zboot.img
scripts/mkbootimg --kernel arch/arm64/boot/Image --ramdisk ramdisk.img --second resource.img -o boot.img
if [ $? -eq 0 ];then
  echo '  Image:  boot.img (with Image resource.img) is ready'
fi
scripts/mkbootimg --kernel arch/arm64/boot/Image.lz4 --ramdisk ramdisk.img --second resource.img -o zboot.img
if [ $? -eq 0 ];then
  echo '  Image:  zboot.img (with Image.lz4 resource.img) is ready'
fi
