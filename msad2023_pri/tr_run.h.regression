#define TR_RUN_R \
  /* RUN_R: camera line trace until RUN_R_DIST */ \
  .composite<BrainTree::ParallelSequence>(1,2) \
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R_DIST")) \
    .composite<BrainTree::MemSequence>() \
      /* section R0: run straight for 0.5 sec */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsTimeEarned>(500000) \
        .leaf<RunAsInstructed>(prof->getValueAsNum("RUN_R1_SPEED"), \
	      prof->getValueAsNum("RUN_R1_SPEED"), 0.0) \
      .end() \
      /* section R1: to the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R1_DIST")) \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R1_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R1_TS", gEnumPairs)) \
      .end() \
      /* section R2: to the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R2_DIST")) \
          .leaf<IsJunction>(JST_FORKED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R2_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R2_TS", gEnumPairs)) \
      .end() \
      /* section R3: while passing the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R3_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R3_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R3_TS", gEnumPairs)) \
      .end() \
      /* section R4: while passing the join before CP1 */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R4_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R4_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R4_TS", gEnumPairs)) \
      .end() \
      /* section R5: to the intersection between loops */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R5_DIST")) \
          .leaf<IsJunction>(JST_JOINING) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R5_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R5_TS", gEnumPairs)) \
      .end() \
      /* section R6: go along the larger loop */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R6_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R6_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R6_TS", gEnumPairs)) \
      .end() \
      /* section R7: to the intersection */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R7_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R7_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R7_TS", gEnumPairs)) \
      .end() \
      /* section R8: go along the smaller loop to the intersection */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R8_DIST")) \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R8_SPEED"), \
	      prof->getValueAsNumVec("RUN_R8_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R8_TS", gEnumPairs)) \
      .end() \
      /* section R9: run from the intersection to the fork getting out of the large loop by distance */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R9_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R9_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R9_TS", gEnumPairs)) \
      .end() \
      /* section R10: run just a little bit */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R10_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R10_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R10_TS", gEnumPairs)) \
      .end() \
      /* section R11: run the main course toward the fork beyond LAP Gate by distance */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R11_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R11_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R11_TS", gEnumPairs)) \
      .end() \
      /* section R12: pass the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R12_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R12_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R12_TS", gEnumPairs)) \
      .end() \
    .end() \
  .end()

#define TR_RUN_L \
  /* RUN_L: camera line trace until Blue is detected by color sensor */ \
  .composite<BrainTree::MemSequence>() \
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
