#!/bin/bash

CONFIGS=("px30_ht_eink" "px30_eink_6" "px30_eink_103h")
CONFIG=$2
COUNT=${#CONFIGS[*]}
PROJECTNAME=""
function display_project(){
    echo "****************************************"
    echo Please select one project:
    for((i=0;i<COUNT;i++));  
    do   
        echo $(expr $i + 1).${CONFIGS[$i]};  
    done  
    echo "****************************************"
    read confignum
    if [[ $confignum -gt $COUNT ]] || [[ $confignum -eq "0" ]]
    then
        echo -e "\033[31m error:project not exist \033[0m"
        exit 1
    else
        CONFIG=${CONFIGS[$confignum -1]}
    fi
    echo -e "\033[32m config is $CONFIG \033[0m" 

}

function check_project()
{
    projecttemp=$1
    for((i=0;i<COUNT;i++));  
    do   
        if [[ $projecttemp == ${CONFIGS[$i]} ]]
        then
            CONFIG=$projecttemp
            return
        fi
    done
    echo -e "\033[31m error:$projecttemp not exist \033[0m" 
    exit 1
}

function build(){
    #check_project $1
    UPPERCASE=$(echo $CONFIG | tr '[a-z]' '[A-Z]') 
    echo -e "\033[32m start to build $CONFIG \033[0m" 
    make ARCH=arm64 ${CONFIG}_defconfig
    CONFIG_FILE=.config
    #==20190107: make use Don't debug at release build.
    sed -i '/CONFIG_DEBUG_BUILD*/d' $CONFIG_FILE
    if [ "$DEBUG_BUILD" != "false"  ]; then
        DEBUG_SET=`grep CONFIG_DEBUG_BUILD $CONFIG_FILE | grep "^[^#]" `
        if [ -z "$DEBUG_SET"  ];  then
            echo "CONFIG_DEBUG_BUILD=y" >> $CONFIG_FILE
    #       grep CONFIG_DEBUG_BUILD $CONFIG_FILE
        fi
    else
        echo "# CONFIG_DEBUG_BUILD is not set" >> $CONFIG_FILE
    fi

    if [ "${CONFIG}" = "super103s" ]
      then
      make ARCH=arm64 ${CONFIG}.img super78.img super78lp.img -j8
      if [ $? -ne 0 ]; then
        echo -e "\033[40;31m build kernel error super103s \033[0m"
        return 1
      fi
    else
      make ARCH=arm64 ${CONFIG}.img -j8
      if [ $? -ne 0 ]; then
        echo -e "\033[40;31m build kernel error \033[0m"
        return 1
      fi
    fi
    if [ -f boot.img ]; then
        echo  -e "\033[32m  remove boot.img which is useless. \033[0m"
        rm boot.img
    fi

    if [ -f zboot.img ]; then
        echo -e "\033[32m  remove zboot.img which is useless. \033[0m"
        rm zboot.img
    fi

    if [ "${CONFIG}" = "super103s" ]
      then
      rm resource.img
      scripts/mkmultidtb.py SUPER-DTB
      ./mkboot.sh
    fi

    #copyfile  #only for ourself
}

function menuconfig()
{
	if [ "${CONFIG}" = "" ]; then
    display_project
	fi 
	
    echo "copy ${CONFIG}_defconfig to .config"
    cp arch/arm64/configs/${CONFIG}_defconfig .config
    make menuconfig
    saveconfig
}

function saveconfig()
{
    #display_project
    cp .config arch/arm64/configs/${CONFIG}_defconfig
    echo "perss Y/y to build,other to exit"
    read BUILD
    case "$BUILD" in
    Y)
        build $CONFIG
        ;;
    y)
        build $CONFIG
        ;;
    *)
        echo "not build"
        exit 0
        ;;
    esac 
}

function backupimg()
{
    cp resource.img resource_${CONFIG}.img
    cp kernel.img kernel_${CONFIG}.img
}

echo "Parm1=$1,Param2=$2,CONFIG=$CONFIG"
if [ "$1" = "" ]
  then   
  display_project
  build $CONFIG
  backupimg
  elif [ "$1" = "menuconfig" ]
  then
  menuconfig
  elif [ "$1" = "saveconfig" ]
  then
  saveconfig
  backupimg
  else
  CONFIG=$1
  build $1
  if [ $? -ne 0 ]; then
    exit 1
  fi
  backupimg
fi

exit 0
