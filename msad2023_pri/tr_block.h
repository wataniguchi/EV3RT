#define TR_BLOCK_R \
  .leaf<StopNow>()

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
    .leaf<IsFoundBlock>(prof->getValueAsNum("BLOCK_Lx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Lx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Lx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Lx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Lx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Lx_BGR_MAX_DEC"))	\
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(1200) \
      .leaf<ApproachBlock>(prof->getValueAsNum("BLOCK_Lx_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Lx_PID_CONST"), \
	    prof->getValueAsNum("BLOCK_Lx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Lx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Lx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Lx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Lx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Lx_BGR_MAX_DEC"))	\
    .end() \
  .end()
