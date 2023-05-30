#define TR_RUN_R \
  /* RUN_R: color sensor line trace until RUN_R_DIST */	\
  .composite<BrainTree::ParallelSequence>(1,2) \
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R_DIST")) \
    .composite<BrainTree::MemSequence>() \
      .leaf<TraceLine>(prof->getValueAsNum("RUN_R1_SPEED"), \
                       prof->getValueAsNum("RUN_Rx_R_TARGET"), \
                       prof->getValueAsNumVec("RUN_Rx_PID_CONST"), 0.0, \
                       (TraceSide)prof->getValueAsIntFromEnum("RUN_R1_TS", gEnumPairs)) \
    .end() \
  .end()

#define TR_RUN_L \
  /* RUN_L: camera line trace until RUN_L_DIST */ \
  .composite<BrainTree::ParallelSequence>(1,2) \
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L_DIST")) \
    .composite<BrainTree::MemSequence>() \
      .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L1_SPEED"), \
			  prof->getValueAsNumVec("RUN_Lx_PID_CONST"), \
			  prof->getValueAsNum("RUN_Lx_GS_MIN"), \
			  prof->getValueAsNum("RUN_Lx_GS_MAX"), 0.0, \
                          (TraceSide)prof->getValueAsIntFromEnum("RUN_L1_TS", gEnumPairs)) \
   .end() \
  .end()
