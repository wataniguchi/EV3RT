#define TR_BLOCK_R \
  .leaf<StopNow>()  

#define TR_BLOCK_L \
  .composite<BrainTree::MemSequence>()\
    /* ライントレースから引継ぎして、直前の青線まで走る*/\
    .composite<BrainTree::ParallelSequence>(2,3)\
        .leaf<SetArmPosition>(0, 40)\
        .composite<BrainTree::MemSequence>()\
            .leaf<IsColorDetected>(CL_BLACK)\
            .leaf<IsColorDetected>(CL_BLUE)\
        .end()\
        .leaf<TraceLine>(35, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE)\
    .end()\
    /* 台にのる　勢いが必要 */\
    /*青検知の後、台乗上前にギリギリまでトレース*/\
    .composite<BrainTree::ParallelSequence>(1,2)\
        .leaf<IsDistanceEarned>(360)\
        .leaf<TraceLine>(45, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE)\
    .end()\
    /*段差ストップ 3.0 sec*/\
    .composite<BrainTree::ParallelSequence>(1,2)\
        .leaf<IsTimeEarned>(3000000) \
        .leaf<TraceLine>(25, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_CENTER)\
    .end()\
    .composite<BrainTree::ParallelSequence>(1,2)\
        .leaf<IsDistanceEarned>(150)\
        .leaf<TraceLine>(80, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_CENTER)\
    .end()\
    /*スタブ用ライントレース１*/\
    .composite<BrainTree::ParallelSequence>(1,2)\
        .leaf<IsDistanceEarned>(1100)\
        .leaf<TraceLine>(45, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_CENTER)\
    .end()\
    /*スタブ用回転*/ \
      .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsTimeEarned>(700000) \
      .leaf<RunAsInstructed>(-50, 50, 0.0)\
    .end()  \
    /*スタブ用黒検知したらライントレース*/\
    .composite<BrainTree::ParallelSequence>(1,2)\
        .leaf<IsColorDetected>(CL_BLACK)\
        .leaf<RunAsInstructed>(30, 30, 0.0)\
    .end()\
    .composite<BrainTree::ParallelSequence>(1,2)\
        .leaf<IsDistanceEarned>(1000)\
        .leaf<TraceLine>(45, 47, P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE)\
    .end()\
    /*停止テスト*/\
    .composite<BrainTree::ParallelSequence>(1,2)\
        .leaf<IsDistanceEarned>(1000000)\
        .leaf<TraceLine>(0, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_CENTER)\
    .end() \
  .end()