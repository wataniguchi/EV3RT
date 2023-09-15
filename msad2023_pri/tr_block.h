#define TR_BLOCK1_R \
  /* BLOCK1: make a turn, down the arm, and try to find the Treasure Block at the initial position */ \
  .composite<BrainTree::MemSequence>() \
    /* section R101: set guide angle and location */ \
    .leaf<SetGuideAngle>() \
    .leaf<SetGuideLocation>() \
    /* section R102: face toward the area and arm down */ \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_R102_DEGREE"), \
		     prof->getValueAsNum("BLOCK_R102_SPEED"), 0.0) \
    .leaf<ArmUpDownFull>(AD_DOWN) \
    .leaf<IsFoundBlock>(prof->getValueAsNum("BLOCK_GS_MIN"), \
            prof->getValueAsNum("BLOCK_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_BGR_MAX_DEC"))	\
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
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R202: move parallel to the course line */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R202_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R202_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R202_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    .leaf<StopNow>() \
    /* section R203: scan the area until finding the Treasure Block */ \
    .leaf<ScanBlock>(prof->getValueAsNum("BLOCK_R203_MAX_ROTATE"), \
            prof->getValueAsNum("BLOCK_R203_DEGREE"), \
	    prof->getValueAsNum("BLOCK_R203_SPEED"), \
	    prof->getValueAsNum("BLOCK_GS_MIN"), \
            prof->getValueAsNum("BLOCK_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_BGR_MAX_DEC"))	\
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
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R302: face parallel to the course line */ \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_R302_DEGREE"), \
		     prof->getValueAsNum("BLOCK_R302_SPEED"), 0.0) \
    /* section R303: scan the area until finding the Treasure Block */ \
    .leaf<ScanBlock>(prof->getValueAsNum("BLOCK_R303_MAX_ROTATE"), \
            prof->getValueAsNum("BLOCK_R303_DEGREE"), \
	    prof->getValueAsNum("BLOCK_R303_SPEED"), \
	    prof->getValueAsNum("BLOCK_GS_MIN"), \
            prof->getValueAsNum("BLOCK_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_BGR_MAX_DEC"))	\
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK4_R \
  /* BLOCK4: capture Treasure Block and see if the current position requires addtional move */ \
  .composite<BrainTree::MemSequence>() \
    /* section R401: capture the block */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R401_DIST")) \
      .leaf<ApproachBlock>(prof->getValueAsNum("BLOCK_R401_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_PID_CONST"), \
	    prof->getValueAsNum("BLOCK_GS_MIN"), \
            prof->getValueAsNum("BLOCK_GS_MAX"), \
            prof->getValueAsNumVec("BLOCK_BGR_MIN_TRE"), \
	    prof->getValueAsNumVec("BLOCK_BGR_MAX_TRE"), \
            prof->getValueAsNumVec("BLOCK_BGR_MIN_DEC"), \
            prof->getValueAsNumVec("BLOCK_BGR_MAX_DEC"))	\
    .end() \
    .leaf<StopNow>() \
    /* section R402: check Ydiff */ \
    .leaf<IsYdiffFromGuideLocationLarger>(prof->getValueAsNum("BLOCK_R402_YDIFF")) \
  .end()

#define TR_BLOCK5_R \
  /* BLOCK5: move away from Goal paralle to the course line */ \
  .composite<BrainTree::MemSequence>() \
    /* section R501: move */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R501_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R501_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R501_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
  .end()

#define TR_BLOCK6_R						      \
  /* BLOCK6: move to the course line and to Goal */ \
  .composite<BrainTree::MemSequence>() \
    /* section R601: move toward the course line until getting out of the lattice area */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsXdiffFromGuideLocationLarger>(prof->getValueAsNum("BLOCK_R601_XDIFF")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R601_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R601_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    .leaf<StopNow>() \
    /* section R602: move toward the course line */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R602_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R602_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R602_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R603: move further until touching to the course line */ \
    .composite<BrainTree::ParallelSequence>(1,3) \
      .leaf<IsColorDetected>(CL_BLACK) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R603_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R603_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R603_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R604: move along the course line to help TraceLine to find the line */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsColorDetected>(CL_BLUE) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R604_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R604_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R604_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R606: reach to Goal */ \
    /*.composite<BrainTree::ParallelSequence>(1,3) */ \
    /*  .leaf<IsColorDetected>(CL_BLUE) */ \
    /*  .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R606_DIST"))  */ \
    /*  .leaf<TraceLineCamWithBlockInArm>(prof->getValueAsNum("BLOCK_R606_SPEED"), */\
    /*        prof->getValueAsNumVec("BLOCK_TRACECAM_PID_CONST"),  */ \
    /*        prof->getValueAsNum("BLOCK_TRACECAM_GS_MIN"),    */ \
    /*       prof->getValueAsNum("BLOCK_TRACECAM_GS_MAX"), 0.0,  */ \
    /*        (TraceSide)prof->getValueAsIntFromEnum("BLOCK_R606_TS", gEnumPairs)) */ \
    /*.end() */\
    /* section R607: get into Goal!!! */ \
    /*.composite<BrainTree::ParallelSequence>(1,2)  */\
    /*  .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R607_DIST")) */\
    /*  .leaf<RunAsInstructed>(prof->getValueAsNum("BLOCK_R607_PWML"), */\
	  /*  prof->getValueAsNum("BLOCK_R607_PWMR"), 0.0) */\
    /*.end() */ \ 
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK1_L \
  .composite<BrainTree::MemSequence>() \
    .leaf<SetGuideAngle>() \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_L101_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_L101_OFFSET"), \
	    prof->getValueAsNum("BLOCK_L101_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
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

#define TR_BLOCK5_L				\
  .leaf<StopNow>()

#define TR_BLOCK6_L				\
  .leaf<StopNow>()
