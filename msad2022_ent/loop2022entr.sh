#!/usr/bin/env bash

##ltloop.sh
trap 'killbackground > /dev/null 2>&1' EXIT

killbackground(){
    local jobs=`jobs -p`
    for job in $jobs; do
        killpstree $job
    done
}

killpstree(){
    local children=`pgrep -P $1`
    for child in $children; do
        killpstree $child
    done
    kill $1
}

usage_exit() {
        echo "  maxtime make開始から走行打ち切りまでの実時間" 1>&2
        echo "" 1>&2
        echo "  ログは${DSTDIR}に格納されています" 1>&2
        exit 1
}

SRCDIR=${ETROBO_HRP3_WORKSPACE}/msad2022_ent
DSTDIR=${ETROBO_HRP3_WORKSPACE}/msad2022_ent/work

MAKELOG="makelog"
LOOPLOG="lplog"
LOGEXT="txt"
CSVEXT="csv"
DT=`date "+%y%m%d%H%M%S"`

#実行ディレクトリはETROBOのroot
cd $ETROBO_ROOT

#パラメータ設定
#パラメータ数が2を超える場合はエラー
if [ $# -gt 2 ];then
    echo " loop2022ent.sh parm1 parm2" 1>&2
    echo "" 1>&2
    echo " parm1:MAXTIME  parm2:loopcnt" 1>&2
#パラメータ数が2の場合は打ち切り時間と繰返し回数をセット
    elif [ $# -eq 2 ];then
    MAXTIME=$1
    LOOPCNT=$2
#パラメータ数が1の場合は打ち切り時間のみセット
        elif [ $# -eq 1 ];then
        MAXTIME=$1
        LOOPCNT=5
#それ以外(パラメータ数0)は初期値をセット
        else
            MAXTIME=30
            LOOPCNT=5
fi

# 右：最初に1回だけmake
make right app=msad2022_ent sim 2>&1 | tee ${DSTDIR}/${MAKELOG}_${DT}.${LOGEXT}
echo "MAXTIME="$MAXTIME
echo "LOOPCNT="$LOOPCNT
sleep 3
# 処理ループ
for ((ll = 1; ll <= LOOPCNT; ll++)) {
# シミュレータ起動と初期位置設定
#    sim ctl pos 2 0 -15.5 0
#    sleep 1
# 光源値設定
#    curl -X POST -H "Content-Type: application/json" -d "{\"EnvLightIntensityLevel\":$ll,\"EnvLightRotation\":$rot,\"LSpotLight\":$lsp,\"RSpotLight\":"0"}" http://localhost:54000
#    curl -X POST -H "Content-Type: application/json" -d "{\"EnvLightIntensityLevel\":"0",\"EnvLightRotation\":"0",\"LSpotLight\":"0",\"RSpotLight\":"0"}" http://172.27.160.1:54000
#    sleep 3
# アプリを実行しプロセスIDを記録
    asp right msad2022_ent &
    sleep 2
#    PID=`asp check l`
    PID=`asp check`
    echo $PID
    sleep 2
# シミュレータ PREPAREモード
    sim right ctl prepare
    sleep 4
# シミュレータ GOモード
    sim right ctl go &
# 処理打ち切り時間を越えたらアプリプロセスの動作確認、プロセスが居たら殺す
    sleep $MAXTIME
    CNT=0
    CNT=`ps -ef | grep $PID | wc -l`
    if [ $CNT -ne 0 ]; then
        kill $PID 2>&1 > /dev/null
    fi
    wait $PID 2>&1 > /dev/null
# 終了処理
    echo "stop"
    sleep 1
    PRTCNT=$(printf "%04d" ${ll})
    sim right ctl end 2>&1 | tee ${DSTDIR}/lpr_${PRTCNT}.${CSVEXT}
    asp stop
}

#集計
awk -f ${SRCDIR}/lpprtr2022.awk ${DSTDIR}/lpr_*.${CSVEXT} > ${DSTDIR}/${DT}.${CSVEXT}
cd ${DSTDIR}
tar -zcvf ${DT}.tar.gz lpr_*.csv
rm  lpr_*.csv
cd ../
