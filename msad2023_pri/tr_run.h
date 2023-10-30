#define TR_RUN_R \
  /* RUN_R: test bed for block challenge */ \
  .composite<BrainTree::MemSequence>() \
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R1_DIST")) \
      .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R1_SPEED"), \
	    prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	    prof->getValueAsNum("RUN_Rx_GS_MIN"),    \
	    prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
            (TraceSide)prof->getValueAsIntFromEnum("RUN_R1_TS", gEnumPairs)) \
    .end() \
    .leaf<SetPlotterDegree>(prof->getValueAsNum("RUN_Rx_DEGREE")) \
    .composite<BrainTree::ParallelSequence>(1,3) \
      .leaf<IsColorDetected>(CL_RED) \
      .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN_R2_DIST")) \
      .leaf<TraceLineCam>(prof->getValueAsNum("RUN_R2_SPEED"), \
	    prof->getValueAsNumVec("RUN_Rx_PID_CONST"), \
	    prof->getValueAsNum("RUN_Rx_GS_MIN"),    \
	    prof->getValueAsNum("RUN_Rx_GS_MAX"), 0.0, \
            (TraceSide)prof->getValueAsIntFromEnum("RUN_R2_TS", gEnumPairs)) \
    .end() \
    .leaf<StopNow>() \
    .leaf<IsTimeEarned>(1000000) \
  .end()

#define TR_RUN_L \
  .leaf<StopNow>()
