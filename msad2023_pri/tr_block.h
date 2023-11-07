#define TR_BLOCK1_R \
  /* BLOCK1: move closer to the block area and determine if VLine Column 1 is clear */ \
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
    /* section R106: see if NO blocks are on VLine Column 1 */ \
    .leaf<TestNumBlocksOnVLine>(0, prof->getValueAsNum("BLOCK_GS_MIN"), \
          prof->getValueAsNum("BLOCK_GS_MAX"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_TRE"), \
	  prof->getValueAsNumVec("BLOCK_BGR_MAX_TRE"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_DEC"), \
	  prof->getValueAsNumVec("BLOCK_BGR_MAX_DEC"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_LIN"), \
          prof->getValueAsNumVec("BLOCK_BGR_MAX_LIN")) \
  .end()

#define TR_BLOCK2_R \
  /* BLOCK2: traverse the area from Column 1 to push out two Decoy blocks, and then carry Treasure block to Goal */ \
  .composite<BrainTree::MemSequence>() \
    /* section R201: traverse the area */ \
    .leaf<SetVLineColumn>(1, false) \
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
  /* BLOCK3: determine if VLine Column 1 has ONE block */ \
  .composite<BrainTree::MemSequence>() \
    /* section R301: see if ONE block is on VLine Column 1 */ \
    .leaf<TestNumBlocksOnVLine>(1, prof->getValueAsNum("BLOCK_GS_MIN"), \
          prof->getValueAsNum("BLOCK_GS_MAX"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_TRE"), \
	  prof->getValueAsNumVec("BLOCK_BGR_MAX_TRE"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_DEC"), \
	  prof->getValueAsNumVec("BLOCK_BGR_MAX_DEC"), \
          prof->getValueAsNumVec("BLOCK_BGR_MIN_LIN"), \
          prof->getValueAsNumVec("BLOCK_BGR_MAX_LIN")) \
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK4_R \
  /* BLOCK4: traverse the area from Column 1 while taking care of the block on the column */ \
  .composite<BrainTree::MemSequence>() \
    /* section R401: traverse the area */ \
    .leaf<SetVLineColumn>(1, true) \
    .leaf<StopNow>() \
  .end()

#define TR_BLOCK5_R \
  /* BLOCK5: move to VLine Column 4 and traverse the area from Column 4 */ \
  .composite<BrainTree::MemSequence>() \
    /* section R501: move further to the block area */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R501_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R501_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R501_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R502: move to VLine Column 4 */ \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("BLOCK_R502_DIST")) \
      .leaf<RunPerGuideAngle>(prof->getValueAsNum("BLOCK_R502_OFFSET"), \
	    prof->getValueAsNum("BLOCK_R502_SPEED"), \
	    prof->getValueAsNumVec("BLOCK_ANG_PID_CONST")) \
    .end() \
    /* section R503: face to the block area */ \
    .leaf<RotateEV3>(prof->getValueAsNum("BLOCK_R503_DEGREE"), \
		     prof->getValueAsNum("BLOCK_R503_SPEED"), 0.0) \
    /* section R504: traverse the area */ \
    .leaf<SetVLineColumn>(4, false) \
    .leaf<TraverseVLine>(prof->getValueAsNum("BLOCK_R504_SPEED"), \
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
          (TraceSide)prof->getValueAsIntFromEnum("BLOCK_R504_TS", gEnumPairs)) \
    .leaf<StopNow>() \
  .end()

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
