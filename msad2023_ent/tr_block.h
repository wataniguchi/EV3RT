#define TR_BLOCK_R \
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsBackOn>() \
            .composite<BrainTree::MemSequence>() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_RED) \
                    .leaf<TraceLine>(20, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
                .end() \
/* Reset Arm Pos. \
                .leaf<SetArmPosition>(ARM_INITIAL_ANGLE, ARM_SHIFT_PWM) \
*/ \
                .composite<BrainTree::ParallelSequence>(1,3) \
                    .leaf<IsSonarOn>(300) \
                    .leaf<IsTimeEarned>(7290000) \
                    .leaf<RunAsInstructed>(20, 20, 0.5) \
                .end() \
            .end() \
        .end()

#define TR_BLOCK_L \
        .composite<BrainTree::ParallelSequence>(1,2) \
            .leaf<IsBackOn>() \
            .composite<BrainTree::MemSequence>() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_RED) \
                    .leaf<IsTimeEarned>(500000) \                   
                    .leaf<TraceLine>(20, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RunAsInstructed>(15, 15, 0.0) \
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
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_RED) \
                    .leaf<RunAsInstructed>(-15, -15, 0.5) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsTimeEarned>(1300000) \
                    .leaf<RunAsInstructed>(-4, -4, 0.0) \
                .end() \
                .composite<BrainTree::ParallelSequence>(1,2) \
                    .leaf<IsColorDetected>(CL_BLACK) \
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
