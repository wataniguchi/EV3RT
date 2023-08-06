#define TR_BLOCK_R \
  /* BLOCK_R: make a turn, down the arm, and capture Treasure Block */ \
  .composite<BrainTree::MemSequence>() \
    /* section R0: stop */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsTimeEarned>(prof->getValueAsNum("BLOCK_R0_TIME")) \
      .leaf<RunAsInstructed>(prof->getValueAsNum("BLOCK_R0_PWML"), \
	  prof->getValueAsNum("BLOCK_R0_PWMR"), 0.0) \
    .end() \
    /* section R1: rotate and arm down */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsTimeEarned>(prof->getValueAsNum("BLOCK_R1_TIME")) \
      .leaf<RunAsInstructed>(prof->getValueAsNum("BLOCK_R1_PWML"), \
	  prof->getValueAsNum("BLOCK_R1_PWMR"), 0.0) \
    .end() \
    /* section R2: stop */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsTimeEarned>(prof->getValueAsNum("BLOCK_R2_TIME")) \
      .leaf<RunAsInstructed>(prof->getValueAsNum("BLOCK_R2_PWML"), \
	  prof->getValueAsNum("BLOCK_R2_PWMR"), 0.0) \
    .end() \
\
    /* section R3: go */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsColorDetected>(CL_BLUE) \
      .leaf<RunAsInstructed>(prof->getValueAsNum("BLOCK_R3_PWML"), \
    prof->getValueAsNum("BLOCK_R3_PWMR"), 0.0) \
    .end() \
    \
    /* section R11: pass the fork beyond LAP Gate */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .composite<BrainTree::MemSequence>() \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R11_DIST")) \
        .leaf<IsColorDetected>(CL_GREEN) \
      .end() \
      .leaf<TraceLine>(prof->getValueAsNum("RUN_R11_SPEED"), 47, \
     prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
     prof->getValueAsNum("RUN_Rx_GS_MIN"), \
     prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
        TS_NORMAL) \
    .end() \
\
  /* end */ \
    .leaf<StopNow>() \
    .leaf<ArmUpDownFull>(AD_DOWN) \
  .end()
\
\
#define TR_BLOCK_L \
  /* BLOCK_L: make a turn, down the arm, and capture Treasure Block */ \
  .composite<BrainTree::MemSequence>() \
    /* section L1: rotate and arm down */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsTimeEarned>(prof->getValueAsNum("BLOCK_L1_TIME")) \
      .leaf<RunAsInstructed>(prof->getValueAsNum("BLOCK_L1_PWML"), \
	  prof->getValueAsNum("BLOCK_L1_PWMR"), 0.0) \
    .end() \
    .leaf<StopNow>() \
    .leaf<ArmUpDownFull>(AD_DOWN) \
  .end()
