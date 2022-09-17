#define TR_BLOCK_G_R \
    .leaf<StopNow>()  
#define TR_BLOCK_G_L \
    .composite<BrainTree::MemSequence>()\
        .composite<BrainTree::ParallelSequence>(1,3)\
            .leaf<TraceLine>(40,\
                                GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL)  \
            .leaf<IsColorDetected>(CL_BLUE) \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* 後ろ向き走行。狙いは黒線。*/\
            .leaf<IsTimeEarned>(900000) \
            .leaf<RunAsInstructed>(-30,\
                                    -80,\
                                    0.0)   \   
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* 後ろ向き走行。狙いは黒線。*/\
            .leaf<IsTimeEarned>(1700000) \
            .leaf<RunAsInstructed>(-50,\
                                    -50,\
                                    0.0)  \    
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* 後ろ向き走行。狙いは黒線。*/\
            .leaf<IsTimeEarned>(4000000) \
            .leaf<RunAsInstructed>(-40,\
                                    -40,\
                                    0.0)   \     
            .leaf<IsColorDetected>(CL_BLACK)  \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* 黒線検知後、ライントレース準備*/\
            .leaf<IsTimeEarned>(900000) \
            .leaf<RunAsInstructed>(-30,\
                                    60,\
                                    0.0)   \  
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
            .leaf<TraceLine>(45,\
                                GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL)  \
        /* 黒線検知後、ライントレース準備*/\
            .leaf<IsTimeEarned>(750000) \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
            .leaf<TraceLine>(45, \
                                GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL)  \
        /*グレー検知までライントレース*/\
            .leaf<IsColorDetected>(CL_GRAY)  \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
            .composite<BrainTree::MemSequence>()\
            /*グレー検知までライントレース*/\
                .leaf<IsColorDetected>(CL_GRAY)  \  
            /*グレー検知までライントレース  */\
                .leaf<IsColorDetected>(CL_WHITE)  \
            /*グレー検知までライントレース  */\
                .leaf<IsColorDetected>(CL_GRAY)   \
            .end()\
            /* break after 10 seconds */\
            .leaf<IsTimeEarned>(1000000) \
            .leaf<RunAsInstructed>(45,\
            /*グレー検知後、丸穴あき部分があるため少し前進*/\
                                    45,0.0)      \ 
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* break after 10 seconds */\
            .leaf<IsTimeEarned>(555000) \
            .leaf<RunAsInstructed>(-55,\
            /*左に旋回。ライントレース準備。*/\
                                    55,0.0) \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /*少し前進。ライントレース準備。*/\
            .leaf<IsTimeEarned>(1000000) \
            .leaf<RunAsInstructed>(50,\
                                    52,0.0)  \   
                .leaf<IsColorDetected>(CL_BLACK)\
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
            .leaf<IsTimeEarned>(5000000)\
            .leaf<TraceLine>(48, \
                                GS_TARGET, P_CONST, I_CONST, 0.04D, 0.0, TS_OPPOSITE)  \
            /*純粋な青検知までライントレース*/\
            .leaf<IsColorDetected>(CL_BLUE2)  \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* break after 10 seconds */\
            .leaf<IsTimeEarned>(300000) \
            .leaf<RunAsInstructed>(52,\
        /*青検知後は大きく右に旋回  */\
                                    -52,0.0)    \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* break after 10 seconds */\
            .leaf<IsTimeEarned>(1000000) \
            .leaf<RunAsInstructed>(45,\
        /*前進。次の青検知を目指す。*/\
                                    60,0.0) \  
        .end() \
        .composite<BrainTree::ParallelSequence>(1,3)\
            .leaf<IsTimeEarned>(5000000)\
            .leaf<RunAsInstructed>(70,\
                                    70,0.0)  \ 
        /*前進。次の青検知を目指す。*/\
            .leaf<IsColorDetected>(CL_BLUE2)  \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* 青検知後、大きく右旋回。向きを整える。*/\    
            .leaf<IsTimeEarned>(500000) \
            .leaf<RunAsInstructed>(60,\
                                    -60,0.0)     \    
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
        /* 青検知後、大きく右旋回。向きを整える。*/\
            .leaf<IsColorDetected>(CL_BLUE2)  \           
            .leaf<IsTimeEarned>(250000) \
            .leaf<RunAsInstructed>(60,\
                                    -60,0.0)     \    
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
            .leaf<IsColorDetected>(CL_WHITE)  \
            .leaf<TraceLine>(48, \
                                GS_TARGET, P_CONST, I_CONST, 0.04D, 0.0, TS_OPPOSITE)  \
        .end()\
        .composite<BrainTree::ParallelSequence>(1,3)\
            .leaf<IsTimeEarned>(1500000)\
            .leaf<RunAsInstructed>(50,\
        /*目的の色検知まで前進*/\
                                    50,0.0)  \
        .end() \
        .composite<BrainTree::ParallelSequence>(1,3)\
            .leaf<IsTimeEarned>(5000000)\
            .leaf<RunAsInstructed>(50,\
                                    50,0.0) \
            .leaf<IsColorDetected>(CL_GREEN)  \
        .end()\
        .leaf<StopNow>()\
        /* wait 3 seconds */\
        .leaf<IsTimeEarned>(30000000) \
        .leaf<SetArmPosition>(10, 40)\
    .end()