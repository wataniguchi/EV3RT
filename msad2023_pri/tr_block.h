#define TR_BLOCK1_R \
  /* BLOCK1: move closer to the block area and determine if Column 0 is clear */ \
  .composite<BrainTree::MemSequence>() \
    /* section R101: set guide angle and location */ \
    .leaf<SetGuideAngle>() \
    .leaf<SetGuideLocation>() \
    /* section R102: position in the center of Red circle */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R102_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R102_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R102_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R103: face to the block area */ \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_R103_DEGREE"), \
		     prof->getValueAsNum("BLOCK_R103_SPEED"), 0.0) \
    .leaf<ArmUpDownFull>(AD_DOWN) \
    /* section R104: move closer to the block area */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R104_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R104_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R104_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R105: make a complete stop */ \
    .leaf<StopNow>() \
    .leaf<IsTimeEarned>(prof->getValueAsNum("BLOCK_R105_TIME")) \
    /* section R106: see if two blocks are on VLine Column 1 */ \
    .leaf<TestNumBlocksOnVLine>(2, prof->getValueAsNum("BLOCK_GS_MIN"), \
          prof->getValueAsNum("BLOCK_GS_MAX"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_TRE"), \
	  prof->getValueAsNumVec("BLOCK_BGR_MAX_TRE"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_DEC"), \
	  prof->getValueAsNumVec("BLOCK_BGR_MAX_DEC"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_LIN"), \
          prof->getValueAsNumVec("BLOCK_BGR_MAX_LIN")) \
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
