#define TR_BLOCK_R \
  .leaf<StopNow>()

#define TR_BLOCK_L \
  /* BLOCK_L: make a turn, down the arm, and capture Treasure Block */ \
  .composite<BrainTree::MemSequence>() \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_L1_ROTDEG"), \
	  prof->getValueAsNum("BLOCK_L1_SPEED"), 0.0) \
    .leaf<SetArmPosition>(prof->getValueAsNum("BLOCK_L1_ARMDEG"), \
	  prof->getValueAsNum("BLOCK_L1_ARMRPM")) \
    .leaf<StopNow>() \
  .end()
