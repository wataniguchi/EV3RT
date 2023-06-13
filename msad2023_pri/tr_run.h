#define TR_RUN_R \
  /* RUN_R: camera line trace until RUN_R_DIST */ \
  .composite<BrainTree::ParallelSequence>(1,2) \
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R_DIST")) \
    .composite<BrainTree::MemSequence>() \
      /* section R1: trace NORMAL side and run fast until the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R1_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R1_TS", gEnumPairs)) \
      .end() \
      /* section R2: slow down tropezoidally and \
                     run before the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R2_DIST")) \
          .leaf<IsJunction>(JST_FORKING) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R2_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.5, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R2_TS", gEnumPairs)) \
      .end() \
      /* section R3: run faster tropezidally until the join before CP1 */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R3_DIST")) \
          .leaf<IsJunction>(JST_JOINING) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R3_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R3_TS", gEnumPairs)) \
      .end() \
      /* section R4: run until the intersection between loops */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R4_DIST")) \
          .leaf<IsJunction>(JST_JOINING) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R4_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R4_TS", gEnumPairs)) \
      .end() \
      /* section R5: switch trace side and get into the larger loop */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R5_DIST1")) \
          .leaf<IsJunction>(JST_JOINED) \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R5_DIST2")) \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R5_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R5_TS", gEnumPairs)) \
      .end() \
      /* section R6: switch the trace side and get into the smaller loop */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R6_DIST")) \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R6_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R6_TS", gEnumPairs)) \
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
