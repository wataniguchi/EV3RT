BEGIN {
    FS = ","
}
{
#右コース用
#ファイル名から連番を抽出
    FNAME = substr(FILENAME,index(FILENAME,".csv")-4,4)
    printf "%s%c",FNAME,","
#配列にデータをセット
    FLG = 0
    for(i = 1 ; i <= NF ; i++) {
#        print "$i="$i
        split( $i , word , ":")
#       print "word(1)"word[1]"word(2)"word[2]
        gsub("\"","",word[1])
#        print "word(1)"word[1]"word(2)"word[2]
        if (word[1] == "rightMeasurement") {
            FLG = 1
        }
        if (FLG == 1) {
            Set[word[1]] = word[2]
        }
    }
#データを出力
##計測値
    if (Set["MEASUREMENT_TIME"] == 0) {
        printf "%s","0,"
    }
    else {
         printf "%s%c%s%c",substr(Set["MEASUREMENT_TIME"],1,2),".",substr(Set["MEASUREMENT_TIME"],3,3),","
    }
##走行値
    if (Set["RUN_TIME"] == 0) {
        printf "%s","0,"
    }
    else {
         printf "%s%c%s%c",substr(Set["RUN_TIME"],1,2),".",substr(Set["RUN_TIME"],3,3),","
    }
##ゲート1通過
    printf "%s%c",Set["GATE1"],","
##ゲート2通過
    printf "%s%c",Set["GATE2"],","
##ゴール通過
    printf "%s%c",Set["GOAL"],","
##エントリーボーナス値
    printf "%s%c",Set["ENTRY_BONUS"],"\n"
}
