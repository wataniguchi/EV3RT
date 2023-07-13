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
    .leaf<SetArmPosition>(prof->getValueAsNum("BLOCK_L1_ARMDEG"),	\
	  prof->getValueAsNum("BLOCK_L1_PWMARM")) \
    .leaf<StopNow>() \
  .end()
