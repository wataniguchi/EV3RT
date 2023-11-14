#define TR_RUN_R \
  /* RUN_R: camera line trace until RUN_R_DIST */ \
  .composite<BrainTree::ParallelSequence>(1,2) \
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R_DIST")) \
    .composite<BrainTree::MemSequence>() \
      /* section RA: to the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_RA_DIST")) \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_RA_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_RA_TS", gEnumPairs)) \
      .end() \
      /* section RB: to the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_RB_DIST")) \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_RB_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_RB_TS", gEnumPairs)) \
      .end() \
      /* section R1: to the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R1_DIST")) \
/*          .leaf<IsJunction>(JST_JOINED)  */ \
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
	      prof->getValueAsNumVec("RUN_Ry_PID_CONST"), \
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
          .leaf<IsJunction>(JST_JOINING) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R7_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R7_TS", gEnumPairs)) \
      .end() \
      /* section R8: go along the smaller loop to the main course */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R8_DIST")) \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R8_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ry_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R8_TS", gEnumPairs)) \
      .end() \
      /* section R9: run the main course to the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R9_DIST")) \
          .leaf<IsJunction>(JST_FORKED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R9_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R9_TS", gEnumPairs)) \
      .end() \
    /* section R10A: pass the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R10A_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R10_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rv_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R10A_TS", gEnumPairs)) \
      .end() \
    /* section R10B: pass the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R10B_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R10_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rz_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R10B_TS", gEnumPairs)) \
      .end() \
      /* section R10: pass the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R10_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R10_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rv_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R10_TS", gEnumPairs)) \
      .end() \
      /* section R11: pass the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R11_DIST")) \
          .leaf<IsColorDetected>(CL_RED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R11_SPEED"), \
	      prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_R11_TS", gEnumPairs)) \
      .end() \ 
    .end() \
  .end()

#define TR_RUN_L \
  /* RUN_L: camera line trace until RUN_L_DIST */ \
  .composite<BrainTree::ParallelSequence>(1,2) \
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L_DIST")) \
    .composite<BrainTree::MemSequence>() \
      /* section RA: to the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_LA_DIST")) \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_LA_SPEED"), \
	      prof->getValueAsNumVec("RUN_Lx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_LA_TS", gEnumPairs)) \
      .end() \
      /* section LB: to the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_LB1_DIST")) \
        .leaf<RunAsInstructed>((prof->getValueAsNum("RUN_LB1_PWL")), \
        (prof->getValueAsNum("RUN_LB1_PWR")), \
        (prof->getValueAsNum("RUN_LB1_SR"))) \
      .end() \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_LB2_DIST")) \
        .leaf<RunAsInstructed>((prof->getValueAsNum("RUN_LB2_PWL")), \
        (prof->getValueAsNum("RUN_LB2_PWR")), \
        (prof->getValueAsNum("RUN_LB2_SR"))) \
      .end() \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_LB3_DIST")) \
        .leaf<RunAsInstructed>((prof->getValueAsNum("RUN_LB3_PWL")), \
        (prof->getValueAsNum("RUN_LB3_PWR")), \
        (prof->getValueAsNum("RUN_LB3_SR"))) \
      .end() \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_LB4_DIST")) \
        .leaf<RunAsInstructed>((prof->getValueAsNum("RUN_LB4_PWL")), \
        (prof->getValueAsNum("RUN_LB4_PWR")), \
        (prof->getValueAsNum("RUN_LB4_SR"))) \
      .end() \
      /* section L1: to the first join */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L1_DIST")) \
/*          .leaf<IsJunction>(JST_JOINED)  */ \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L1_SPEED"), \
	      prof->getValueAsNumVec("RUN_Lx_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L1_TS", gEnumPairs)) \
      .end() \
      /* section L2: to the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L21_DIST")) \
        .leaf<RunAsInstructed>((prof->getValueAsNum("RUN_L21_PWL")), \
        (prof->getValueAsNum("RUN_L21_PWR")), \
        (prof->getValueAsNum("RUN_L21_SR"))) \
      .end() \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L22_DIST")) \
        .leaf<RunAsInstructed>((prof->getValueAsNum("RUN_L22_PWL")), \
        (prof->getValueAsNum("RUN_L22_PWR")), \
        (prof->getValueAsNum("RUN_L22_SR"))) \
      .end() \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L23_DIST")) \
        .leaf<RunAsInstructed>((prof->getValueAsNum("RUN_L23_PWL")), \
        (prof->getValueAsNum("RUN_L23_PWR")), \
        (prof->getValueAsNum("RUN_L23_SR"))) \
      .end() \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L24_DIST")) \
        .leaf<RunAsInstructed>((prof->getValueAsNum("RUN_L24_PWL")), \
        (prof->getValueAsNum("RUN_L24_PWR")), \
        (prof->getValueAsNum("RUN_L24_SR"))) \
      .end() \
      /* section L3: while passing the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L3_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L3_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L3_TS", gEnumPairs)) \
      .end() \
      /* section L4: while passing the join before CP1 */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L4_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L4_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L4_TS", gEnumPairs)) \
      .end() \
      /* section L5: to the intersection between loops */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L5_DIST")) \
          .leaf<IsJunction>(JST_JOINING) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L5_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L5_TS", gEnumPairs)) \
      .end() \
      /* section L6: go along the larger loop */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L6_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L6_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L6_TS", gEnumPairs)) \
      .end() \
      /* section L7: to the intersection */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L7_DIST")) \
          .leaf<IsJunction>(JST_JOINING) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L7_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L7_TS", gEnumPairs)) \
      .end() \
      /* section L8: go along the smaller loop to the main course */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L8_DIST")) \
          .leaf<IsJunction>(JST_JOINED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L8_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L8_TS", gEnumPairs)) \
      .end() \
      /* section L9: run the main course to the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L9_DIST")) \
          .leaf<IsJunction>(JST_FORKED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L9_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L9_TS", gEnumPairs)) \
      .end() \
      /* section L10: pass the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L10_DIST")) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L10_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L10_TS", gEnumPairs)) \
      .end() \
      /* section L11: pass the fork beyond LAP Gate */ \
      .composite<BrainTree::ParallelSequence>(1,2) \
        .composite<BrainTree::MemSequence>() \
          .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_L11_DIST")) \
          .leaf<IsColorDetected>(CL_RED) \
        .end() \
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN_L11_SPEED"), \
	      prof->getValueAsNumVec("RUN_Ly_PID_CONST"), \
	      prof->getValueAsNum("RUN_Rx_GS_MIN"), \
	      prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
              (TraceSide)prof->getValueAsIntFromEnum("RUN_L11_TS", gEnumPairs)) \
      .end() \
    .end() \
  .end()