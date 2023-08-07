#define TR_RUN_R \
  /* RUN_R: test bed for block challenge */ \
  .composite<BrainTree::MemSequence>() \
    .leaf<ArmUpDownFull>(AD_DOWN) \
    .composite<BrainTree::ParallelSequence>(1,3) \
      .leaf<IsColorDetected>(CL_RED) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R1_DIST")) \
      .composite<BrainTree::MemSequence>() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R1_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"),    \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R1_TS", gEnumPairs)) \
      .end() \
    .end() \
    .leaf<StopNow>() \
  .end()

#define TR_RUN_L \
  /* RUN_L: test bed for block challenge */ \
  .composite<BrainTree::MemSequence>() \
    .leaf<ArmUpDownFull>(AD_DOWN) \
    .composite<BrainTree::ParallelSequence>(1,3) \
      .leaf<IsColorDetected>(CL_RED) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L1_DIST")) \
      .composite<BrainTree::MemSequence>() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L1_SPEED"), \
	      prof->getValueAsNumVec("RUN_Lx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Lx_GS_MIN"),    \
	      prof->getValueAsNum("RUN_Lx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L1_TS", gEnumPairs)) \
      .end() \
    .end() \
    .leaf<StopNow>() \
  .end()
