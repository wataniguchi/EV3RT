/*右コース*/
#define TR_RUN_R \
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsBackOn>() \
            .composite<BrainTree::MemSequence>() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(7200000) \
                    .leaf<RunAsInstructed>(100, 100, 0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1800000) \
                    .leaf<RunAsInstructed>(100, 65, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1200000) \
                    .leaf<RunAsInstructed>(100, 73, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1700000) \
                    .leaf<RunAsInstructed>(100, 100, 0.0) \
                .end() \                
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(700000) \
                    .leaf<RunAsInstructed>(78, 100, 0.0) \
                .end() \  
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1200000) \
                    .leaf<RunAsInstructed>(100, 45, 0.5) \
                .end() \                              
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLUE) \
                    .leaf<TraceLine>(77, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
                .end() \
            .end() \
        .end()

/*左コース*/
#define TR_RUN_L \
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsBackOn>() \
            .composite<BrainTree::MemSequence>() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(7200000) \
                    .leaf<RunAsInstructed>(90, 90, 0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1800000) \
                    .leaf<RunAsInstructed>(90, 65, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(2700000) \
                    .leaf<RunAsInstructed>(85, 82, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1100000) \
                    .leaf<RunAsInstructed>(85, 85, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1600000) \
                    .leaf<RunAsInstructed>(90, 60, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLUE) \
                    .leaf<TraceLine>(77, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
                .end() \
            .end() \
        .end()