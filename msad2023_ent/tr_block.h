/*右コース*/
#define TR_BLOCK_R \
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsBackOn>() \
            .composite<BrainTree::MemSequence>() \
                .composite<BrainTree::ParallelSequence>(1,3) \
                    .leaf<IsColorDetected>(CL_RED) \
                    .leaf<IsTimeEarned>(12000000) \
                    .leaf<TraceLine>(20, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(7000000) \
                    .leaf<RunAsInstructed>(10, 10, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RunAsInstructed>(-2, -2, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(6500000) \
                    .leaf<RotateEV3>(2,10,0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsSonarOn>(168) \
                    .leaf<RunAsInstructed>(15, 15, 0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RotateEV3>(0, 0, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,3) \
                    .leaf<IsColorDetected>(CL_RED) \
                    .leaf<IsTimeEarned>(10000000) \
                    .leaf<RunAsInstructed>(-15, -15, 0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RunAsInstructed>(-4, -4, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,3) \
                    .leaf<IsColorDetected>(CL_BLACK) \
                    .leaf<IsTimeEarned>(2300000) \
                    .leaf<RotateEV3>(-180, 20, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RunAsInstructed>(0, 0, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLUE) \
                    .leaf<TraceLine>(25, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
                .end() \
                \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLACK) \
                    .leaf<TraceLine>(30, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE) \
                .end() \
                \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLUE) \
                    .leaf<TraceLine>(50, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE) \
                .end() \
            .end() \
        .end() 

/*左コース*/
#define TR_BLOCK_L \
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsBackOn>() \
            .composite<BrainTree::MemSequence>() \
                .composite<BrainTree::ParallelSequence>(1,3) \
                    .leaf<IsColorDetected>(CL_RED) \
                    .leaf<IsTimeEarned>(12000000) \
                    .leaf<TraceLine>(20, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(7000000) \
                    .leaf<RunAsInstructed>(10, 10, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RunAsInstructed>(-2, -2, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(6500000) \
                    .leaf<RotateEV3>(-2,10,0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsSonarOn>(168) \
                    .leaf<RunAsInstructed>(15, 15, 0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RotateEV3>(0, 0, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,3) \
                    .leaf<IsColorDetected>(CL_RED) \
                    .leaf<IsTimeEarned>(10000000) \
                    .leaf<RunAsInstructed>(-15, -15, 0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RunAsInstructed>(-4, -4, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,3) \
                    .leaf<IsColorDetected>(CL_BLACK) \
                    .leaf<IsTimeEarned>(2300000) \
                    .leaf<RotateEV3>(180, 20, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RunAsInstructed>(0, 0, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLUE) \
                    .leaf<TraceLine>(25, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLACK) \
                    .leaf<TraceLine>(30, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE) \
                .end() \
                \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLUE) \
                    .leaf<TraceLine>(50, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_OPPOSITE) \
                .end() \
            .end() \
        .end() 
