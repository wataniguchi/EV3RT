#define TR_RUN_R \
  /* RUN_R: camera line trace until RUN_R_DIST */ \
  .composite<BrainTree::ParallelSequence>(1,2) \
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R_DIST")) \
    .composite<BrainTree::MemSequence>() \
      /* section R1: trace NORMAL side and run fast until the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsJunction>(JST_JOINED) \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R1_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, TS_NORMAL) \
      .end() \
      /* section R2: switch the trace side, slow down tropezoidally and \
                     run until the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsJunction>(JST_FORKED) \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R2_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.5, TS_OPPOSITE) \
      .end() \
      /* section R3: run faster tropezidally until the intersection between loops */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsJunction>(JST_JOINED) \
          .leaf<IsJunction>(JST_JOINING) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R3_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.5, TS_NORMAL) \
      .end() \
      /* section R4: trace OPPOSITE side and get into the larger loop */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsJunction>(JST_FORKED) \
          .leaf<IsJunction>(JST_FORKED) \
          .leaf<IsJunction>(JST_JOINED) \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R4_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, TS_OPPOSITE) \
      .end() \
      /* section R5: switch the trace side and get into the smaller loop */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsJunction>(JST_FORKED) \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R5_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, TS_NORMAL) \
      .end() \
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
