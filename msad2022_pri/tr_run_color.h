#define TR_RUN_COLOR \
    .composite<BrainTree::ParallelSequence>(2,3) \
        .leaf<IsTouchOn>() \
        .leaf<SetArmPosition>(10, 40) \
        .composite<BrainTree::MemSequence>() \
/* GATE1を通過後ラインの交差地点地点直前まで */ \
            .leaf<IsBackOn>() \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsColorDetected>(CL_JETBLACK_YMNK) /* JETBLACKを検知 */ \
                .leaf<IsDistanceEarned>(prof->getValueAsNum("DIST1")) \
                /* .leaf<IsTimeEarned>(prof->getValueAsNum("TIME1")) */ \
                /* leaf<StopNow>() */ \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED1"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), \
                prof->getValueAsNum("srewrate1"), TS_OPPOSITE) /* ライントレース1,右のライン検知 */ \
            .end() \
/* ラインの交差地点直前から検知するまで減速 */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsColorDetected>(CL_JETBLACK_YMNK) /* JETBLACKを検知 */ \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME1")) /* 18秒 */ \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED1a"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), \
                prof->getValueAsNum("srewrate1"), TS_OPPOSITE) /* ライントレース1,右のライン検知 */ \
            .end() \
/* 交差地点後にしばらく直進 */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                /* .leaf<IsTimeEarned>(18000000)  */ /* 18秒 */ \
                /* .leaf<StopNow>() */ \
                /* .leaf<IsTimeEarned>(prof->getValueAsNum("TIME1a")) */ \
                .leaf<IsDistanceEarned>(prof->getValueAsNum("DIST1a")) \
                .leaf<RunAsInstructed>(prof->getValueAsNum("POWER_L1a"), \
                prof->getValueAsNum("POWER_R1a"), 0.0) \
            .end() \
/* ゆるやかに右カーブ */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME1aa")) \
                .leaf<RunAsInstructed>(prof->getValueAsNum("POWER_L1aa"), \
                prof->getValueAsNum("POWER_R1aa"), prof->getValueAsNum("srewrate1aa")) \
            .end() \
/* ライン検知するまでさらに緩やかに右カーブ */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsColorDetected>(CL_BLACK) \
                .leaf<RunAsInstructed>(65,40, 0.0) \
            .end() \
/* ライン検知後にトレースを補正するために2秒速度を落とす */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME2")) \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED2"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), 0.0, TS_NORMAL) /* ライントレース2,左のライン検知 */ \
            .end() \
/* ゲート2,3通過後にラインの交差点直前まで */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsColorDetected>(CL_JETBLACK_YMNK) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME2a")) \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED2a"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), 0.0, TS_NORMAL) /* ライントレース2a,左のライン検知 */ \
            .end() \
/* ラインの交差点検知まで */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsColorDetected>(CL_JETBLACK_YMNK) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME2a")) \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED2aa"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), 0.0, TS_NORMAL) /* ライントレース2aa,左のライン検知 */ \
            .end() \
/* ライン交差点検知後に緩やかに左カーブ */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME3")) \
                .leaf<RunAsInstructed>(prof->getValueAsNum("POWER_L3"), \
                prof->getValueAsNum("POWER_R3"), 0.0) \
            .end() \
/* ライン検知するまで緩やかに右カーブ */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME3a")) \
                .leaf<IsColorDetected>(CL_BLACK) \
                .leaf<RunAsInstructed>(prof->getValueAsNum("POWER_L4"), \
                prof->getValueAsNum("POWER_R4"), 0.0) \
            .end() \
/* ライン検知後にトレースを補正するために1.9秒速度を落とす */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME2")) \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED2"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), 0.0, TS_OPPOSITE) /* ライントレース2,右のライン検知 */ \
            .end() \
/* 2回カーブまでライントレース */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME4")) \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED4"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), 0.0, TS_OPPOSITE) /* ライントレース4,右のライン検知 */ \
            .end() \
/* 最終カーブまでライントレース */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .leaf<IsTimeEarned>(prof->getValueAsNum("TIME5")) \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED5"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), \
                prof->getValueAsNum("srewrate3"), TS_OPPOSITE) /* ライントレース5,右のライン検知 */ \
            .end() \
/* スラロームに引き渡すまでライントレース */ \
            .composite<BrainTree::ParallelSequence>(1,2) \
                .composite<BrainTree::MemSequence>() \
                    .leaf<IsColorDetected>(CL_BLACK) \
                    .leaf<IsColorDetected>(CL_BLUE) \
                .end() \
                .leaf<TraceLine>(prof->getValueAsNum("SPEED6"), \
                prof->getValueAsNum("GS_TARGET1"), prof->getValueAsNum("P_CONST1"), \
                prof->getValueAsNum("I_CONST1"), \
                prof->getValueAsNum("D_CONST1"), 0.0, TS_OPPOSITE) /* ファイナルライントレース,右のライン検知 */ \
            .end() \
        .end() \
    .end()