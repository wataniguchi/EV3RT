#define TR_BLOCK1_R \
  /* BLOCK1: make a turn, down the arm, and try to find the Treasure Block at the initial position */ \
  .composite<BrainTree::MemSequence>() \
    /* section R101: rotate and arm down */ \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_R101_DEGREE"), \
		     prof->getValueAsNum("BLOCK_R101_SPEED"), 0.0) \
    .leaf<ArmUpDownFull>(AD_DOWN) \
    .leaf<IsFoundBlock>(prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Rx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_DEC"))	\
  .end()

#define TR_BLOCK2_R		       \
  /* BLOCK2: go closer the first red Block Circle and scan the area until finding the Treasure Block  */ \
  .composite<BrainTree::MemSequence>() \
    /* section R201: approach the line */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R201_DIST")) \
      .leaf<RunAsInstructed>(prof->getValueAsNum("BLOCK_R201_PWML"), \
			 prof->getValueAsNum("BLOCK_R201_PWMR"), 0.0) \
    .end() \
    .leaf<StopNow>() \
    /* section R202: scan the area until finding the Treasure Block */ \
    .leaf<ScanBlock>(prof->getValueAsNum("BLOCK_R202_MAX_ROTATE"), \
            prof->getValueAsNum("BLOCK_R202_DEGREE"), \
	    prof->getValueAsNum("BLOCK_R202_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_PID_CONST"),	\
	    prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Rx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_DEC"))	\
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK3_R \
  /* BLOCK3: capture Treasure Block and drag the block to the goal */ \
  .composite<BrainTree::MemSequence>() \
    /* section R301: capture */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R301_DIST")) \
      .leaf<ApproachBlock>(prof->getValueAsNum("BLOCK_R301_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_PID_CONST"), \
	    prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Rx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_DEC"))	\
    .end() \
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK1_L \
  .composite<BrainTree::MemSequence>() \
    .leaf<SetGuideAngle>() \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_L101_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_L101_OFFSET"), \
	    prof->getValueAsNum("BLOCK_L101_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Lx_PID_CONST")) \
    .end() \
    .leaf<StopNow>() \
  .end()
/* .leaf<StopNow>() */

#define TR_BLOCK2_L \
  .leaf<StopNow>()

#define TR_BLOCK3_L				\
  .leaf<StopNow>()
