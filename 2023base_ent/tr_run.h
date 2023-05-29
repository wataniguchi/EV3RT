#define TR_RUN_R \
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsBackOn>() \
            .composite<BrainTree::MemSequence>() \
                .leaf<IsColorDetected>(CL_BLACK) \
                .leaf<IsColorDetected>(CL_BLUE) \
            .end() \
            .leaf<TraceLine>(SPEED_NORM, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
        .end()

#define TR_RUN_L \
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsBackOn>() \
            .composite<BrainTree::MemSequence>() \
                .leaf<IsColorDetected>(CL_BLACK) \
                .leaf<IsColorDetected>(CL_BLUE) \
            .end() \
            .leaf<TraceLine>(SPEED_NORM, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
        .end()
