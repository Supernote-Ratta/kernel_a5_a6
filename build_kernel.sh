#!/bin/bash

WORKDIR="`pwd`"
OUTDIR=$WORKDIR/out
CCSTR="arm-poky-linux-gnueabi-gcc  -march=armv7-a -mfpu=neon  -mfloat-abi=hard -mcpu=cortex-a9 --sysroot=/opt/fsl-imx6-x11/4.1.15-2.1.0/sysroots/cortexa9hf-neon-poky-linux-gnueabi"


set_env()
{
	source /opt/fsl-imx6-x11/4.1.15-2.1.0/environment-setup-cortexa9hf-neon-poky-linux-gnueabi
}

check_env()
{
	if [ "$CC" != "$CCSTR" ];then
		set_env
		if [ "$CC" != "$CCSTR" ];then
			echo "$CC"
			echo "$CCSTR"
			echo "Please set your cross compile environment first."
			exit
		fi
	fi

	if [ ! -d $OUTDIR ];then
		mkdir $OUTDIR
	fi
}

make_config()
{
	if [ ! -e $OUTDIR/.config ];then
		echo "Make config"
		make O="$OUTDIR" ARCH=arm ratta_manufacture_defconfig
	fi
	LatestTag=$(git describe --tags `git rev-list --tags --max-count=1`)
	if [ "X"$LatestTag != "X" ];then
		echo "-"$LatestTag > $WORKDIR/.scmversion
	else
		rm $WORKDIR/.scmversion
	fi
}

copy_target()
{
	cp $OUTDIR/arch/arm/boot/zImage $OUTDIR/
	cp $OUTDIR/arch/arm/boot/dts/ratta-sn100.dtb $OUTDIR/
	cp $OUTDIR/arch/arm/boot/dts/ratta-sn078.dtb $OUTDIR/
}

do_make()
{
	check_env
	make_config
	echo "Make zImage and dtb"
	make O="$OUTDIR"
	copy_target
}

make_clean()
{
	rm -rf $OUTDIR
}

usage()
{
	echo "$1 [clean]"
	echo "examples:"
	echo "[1]:$1 will make kernel and epdc drawline driver together."
	echo "[2]:$1 clean will remove output directory."
}

case "$1" in
	"")
		do_make
		;;
	"clean")
		make_clean
		;;
	*)
		usage $0
		exit
		;;
esac
