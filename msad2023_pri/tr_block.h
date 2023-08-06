#define TR_BLOCK_R \
  /* BLOCK_R: make a turn, down the arm, and capture Treasure Block */ \
  .composite<BrainTree::MemSequence>() \
    /* section R1: rotate and arm down */ \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_R1_DEGREE"), \
		     prof->getValueAsNum("BLOCK_R1_SPEED"), 0.0) \
    .leaf<ArmUpDownFull>(AD_DOWN) \
    .leaf<IsFoundBlock>(prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Rx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_DEC"))	\
    /* section R2: capture */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(1202) \
      .leaf<ApproachBlock>(prof->getValueAsNum("BLOCK_R2_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_PID_CONST"), \
	    prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Rx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_DEC"))	\
    .end() \
  .end()

#define TR_BLOCK_L \
  .leaf<StopNow>()
