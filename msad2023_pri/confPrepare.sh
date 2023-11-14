#!/usr/bin/env bash
usage_exit() {
        echo "Usage: $0 -s" 1>&2
        echo "  -s suffix of configuration files" 1>&2
        echo "" 1>&2
        exit 1
}

TGTDIR="/home/pi/etrobo/RasPike/sdk/workspace/msad2023_pri"
TESTPROFDIR="${TGTDIR}/work/test_scripts"


while getopts s:h OPT
do
    case $OPT in
        s)  SUFFIX=$OPTARG
            ;;
        h)  usage_exit
            ;;
        \?) usage_exit
            ;;
    esac
done


shift $((OPTIND - 1))

# check the validitity of argument
ls ${TESTPROFDIR}/*.${SUFFIX} > /dev/null 2>&1
if [ $? -ne 0 ]; then
    usage_exit
fi
echo "replacing configuration files with $SUFFIX counterparts in ${TESTPROFDIR}"
cp ${TESTPROFDIR}/tr_block.h.${SUFFIX}           ${TGTDIR}/tr_block.h
cp ${TESTPROFDIR}/tr_block_profile.txt.${SUFFIX} ${TGTDIR}/tr_block_profile.txt
cp ${TESTPROFDIR}/tr_run.h.${SUFFIX}             ${TGTDIR}/tr_run.h
cp ${TESTPROFDIR}/tr_run_profile.txt.${SUFFIX}   ${TGTDIR}/tr_run_profile.txt
cp ${TESTPROFDIR}/global_profile.txt.${SUFFIX}   ${TGTDIR}/global_profile.txt
