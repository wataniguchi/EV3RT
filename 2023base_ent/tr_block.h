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
