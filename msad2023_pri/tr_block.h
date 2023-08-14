#define TR_BLOCK1_R \
  /* BLOCK1: make a turn, down the arm, and try to find the Treasure Block at the initial position */ \
  .composite<BrainTree::MemSequence>() \
    /* section R101: face toward the area and arm down */ \
    .leaf<SetGuideAngle>() \
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
  /* BLOCK2: place between the red Block Circle and Blue Circle,
             then scan the area until finding the Treasure Block  */ \
  .composite<BrainTree::MemSequence>() \
    /* section R201: move further toward the area */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R201_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R201_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R201_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_ANGPID_CONST")) \
    .end() \
    /* section R202: move parallel to the course line */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R202_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R202_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R202_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_ANGPID_CONST")) \
    .end() \
    .leaf<StopNow>() \
    /* section R203: scan the area until finding the Treasure Block */ \
    .leaf<ScanBlock>(prof->getValueAsNum("BLOCK_R203_MAX_ROTATE"), \
            prof->getValueAsNum("BLOCK_R203_DEGREE"), \
	    prof->getValueAsNum("BLOCK_R203_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_PID_CONST"),	\
	    prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Rx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_DEC"))	\
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK3_R		       \
  /* BLOCK3: move again into the lattice area,
             then scan the area until finding the Treasure Block  */ \
  .composite<BrainTree::MemSequence>() \
    /* section R301: move further into the area */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R301_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R301_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R301_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_ANGPID_CONST")) \
    .end() \
    /* section R302: face parallel to the course line */ \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_R302_DEGREE"), \
		     prof->getValueAsNum("BLOCK_R302_SPEED"), 0.0) \
    /* section R303: scan the area until finding the Treasure Block */ \
    .leaf<ScanBlock>(prof->getValueAsNum("BLOCK_R303_MAX_ROTATE"), \
            prof->getValueAsNum("BLOCK_R303_DEGREE"), \
	    prof->getValueAsNum("BLOCK_R303_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_PID_CONST"),	\
	    prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Rx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_DEC"))	\
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK4_R \
  /* BLOCK4: capture Treasure Block and drag the block to the goal */ \
  .composite<BrainTree::MemSequence>() \
    /* section R401: capture */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R401_DIST")) \
      .leaf<ApproachBlock>(prof->getValueAsNum("BLOCK_R401_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_PID_CONST"), \
	    prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
            prof->getValueAsNum("BLOCK_Rx_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_Rx_BGR_MAX_DEC"))	\
    .end() \
    /* section R402: move toward the line to the goal */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R402_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R402_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R402_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_Rx_ANGPID_CONST")) \
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

#define TR_BLOCK4_L				\
  .leaf<StopNow>()
