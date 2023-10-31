#define TR_BLOCK1_R \
  /* BLOCK1: move closer to the block area and determine if Column 0 is clear */ \
  .composite<BrainTree::MemSequence>() \
    /* section R101: set guide angle and location */ \
    .leaf<SetGuideAngle>() \
    .leaf<SetGuideLocation>() \
    /* section R102: face to the block area */ \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_R102_DEGREE"), \
		     prof->getValueAsNum("BLOCK_R102_SPEED"), 0.0) \
    .leaf<ArmUpDownFull>(AD_DOWN) \
    /* section R103: move closer to the block area */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R103_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R103_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R103_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R104: make a complete stop */ \
    .leaf<StopNow>() \
    .leaf<IsTimeEarned>(prof->getValueAsNum("BLOCK_R104_TIME")) \
    /* TEMPORARY - Failer to force state transition to ST_BLOCK2 */ \
    .decorator<BrainTree::Failer>() \
      .leaf<IsTimeEarned>(prof->getValueAsNum("BLOCK_R104_TIME")) \
    .end() \
  .end()

#define TR_BLOCK2_R \
  /* BLOCK2: traverse the area from Column 0 to push out two Decoy blocks, and then carry Treasure block to Goal */ \
  .composite<BrainTree::MemSequence>() \
    /* section R201: traverse the area */ \
    .leaf<SetVLineColumn>(1) \
    .leaf<TraverseVLine>(prof->getValueAsNum("BLOCK_R201_SPEED"), \
	  prof->getValueAsNum("BLOCK_TARGET_R"), \
	  prof->getValueAsNumVec("BLOCK_PIDSEN_CONST"), \
	  prof->getValueAsNumVec("BLOCK_PIDCAM_CONST"), \
	  prof->getValueAsNum("BLOCK_GS_MIN"), \
          prof->getValueAsNum("BLOCK_GS_MAX"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_TRE"), \
	  prof->getValueAsNumVec("BLOCK_BGR_MAX_TRE"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_DEC"), \
	  prof->getValueAsNumVec("BLOCK_BGR_MAX_DEC"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_LIN"), \
          prof->getValueAsNumVec("BLOCK_BGR_MAX_LIN"), \
          (TraceSide)prof->getValueAsIntFromEnum("BLOCK_R201_TS", gEnumPairs)) \
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK3_R \
  .leaf<StopNow>()

#define TR_BLOCK4_R \
  .leaf<StopNow>()

#define TR_BLOCK5_R \
  .leaf<StopNow>()

#define TR_BLOCK6_R \
  .leaf<StopNow>()

#define TR_BLOCK1_L \
  .leaf<StopNow>()

#define TR_BLOCK2_L \
  .leaf<StopNow>()

#define TR_BLOCK3_L \
  .leaf<StopNow>()

#define TR_BLOCK4_L \
  .leaf<StopNow>()

#define TR_BLOCK5_L \
  .leaf<StopNow>()

#define TR_BLOCK6_L \
  .leaf<StopNow>()
