#define TR_SLALOM_R \
    .leaf<StopNow>()  

#define TR_SLALOM_L \
    .composite<BrainTree::MemSequence>()\
        /* ライントレースから引継ぎして、直前の青線まで走る */\
        .composite<BrainTree::ParallelSequence>(1,2)\
            .leaf<IsDistanceEarned>(200)\
            .leaf<TraceLine>(40, GS_TARGET, P_CONST, I_CONST, 0.08D, 0.0, TS_CENTER)\
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
            /*.leaf<SetArmPosition>(0, 40)*/\
            .composite<BrainTree::MemSequence>()\
                .leaf<IsColorDetected>(CL_BLACK)\
                .leaf<IsColorDetected>(CL_BLUE)\
            .end()\
            .leaf<TraceLine>(40, GS_TARGET, P_CONST, I_CONST, 0.04D, 0.0, TS_OPPOSITE)\
        .end()\
        /*台にのる　勢いが必要*/\
        /*青検知の後、台乗上前にギリギリまでトレース*/\
        .composite<BrainTree::ParallelSequence>(1,2)\
            .leaf<IsDistanceEarned>(360)\
            .leaf<TraceLine>(45, GS_TARGET, P_CONST, I_CONST, 0.04D, 0.0, TS_OPPOSITE)\
        .end()\
        .composite<BrainTree::ParallelSequence>(1,2)\
        /*段差ストップ 3.0 sec*/\
            .leaf<IsTimeEarned>(1000000) \
            .leaf<TraceLine>(35, GS_TARGET, P_CONST, I_CONST, 0.04D, 0.0, TS_OPPOSITE)\
        .end()\
        /*遠藤追加（疑似台形駆動）*/\
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsTimeEarned>(3000000) \
            .leaf<TraceLine>(25, GS_TARGET, P_CONST, I_CONST, 0.04D, 0.0, TS_OPPOSITE)\
        .end()\
        .composite<BrainTree::ParallelSequence>(1,2)\
            .leaf<IsDistanceEarned>(150)\
            .leaf<TraceLine>(80, GS_TARGET,  P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE)\
        .end()\
        /*スタブ用ライントレース１*/\
        .composite<BrainTree::ParallelSequence>(1,2)\
            .leaf<IsDistanceEarned>(1050)\
            .leaf<TraceLine>(40, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE)\
        .end()\
        /*スタブ用回転*/\
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsTimeEarned>(700000) \
            .leaf<RunAsInstructed>(-50, 50, 0.0)\
        .end()  \
        /*スタブ用黒検知したらライントレース*/\
        .composite<BrainTree::ParallelSequence>(1,2)\
            .leaf<IsColorDetected>(CL_BLACK)\
            .leaf<RunAsInstructed>(30, 40, 0.0)\
        .end()\
        .composite<BrainTree::ParallelSequence>(1,2)\
            .leaf<IsDistanceEarned>(600)\
            .leaf<TraceLine>(40, GS_TARGET, P_CONST, I_CONST, 0.04D, 0.0, TS_NORMAL)\
        .end()\
    .end()