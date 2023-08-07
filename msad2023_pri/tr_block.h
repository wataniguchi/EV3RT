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
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R2_DIST")) \
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

#define TR_BLOCK2_R \
  .composite<BrainTree::MemSequence>() \
    /* section R21: approach the line */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R21_DIST")) \
      .leaf<RunAsInstructed>(prof->getValueAsNum("BLOCK_R21_PWML"), \
			 prof->getValueAsNum("BLOCK_R21_PWMR"), 0.0) \
    .end() \
    /* section R22: trace the line until the first red circle */ \
    .composite<BrainTree::ParallelSequence>(1,3) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R22_DIST")) \
      .leaf<IsColorDetected>(CL_RED) \
      .leaf<TraceLineCam>(prof->getValueAsNum("BLOCK_R22_SPEED"), \
	      prof->getValueAsNumVec("BLOCK_Rx_PID_CONST"), \
	      prof->getValueAsNum("BLOCK_Rx_GS_MIN"), \
	      prof->getValueAsNum("BLOCK_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("BLOCK_R22_TS", gEnumPairs)) \
    .end() \
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK_L \
  .leaf<StopNow>()

#define TR_BLOCK2_L \
  .leaf<StopNow>()
