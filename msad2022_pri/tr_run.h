#define TR_RUN \
  /* RUN: until detecting BLUE while executing MemSequence in parallel */	\
  .composite<BrainTree::ParallelSequence>(1,2)				\
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUNx_DIST")) /* temp stopper */ \
    .composite<BrainTree::MemSequence>()			\
      /* RUN1: until passing Gate 3 */	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN1_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_OPPOSITE) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN1_DIST"))	      	\
      .end()								\
      /* RUN2: until the intersection between Gate 3 and 4 */	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN2_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_NORMAL) \
        .leaf<IsJunction>(JST_JOINED)	      	\
      .end()								\
      /* RUN3: until passing the intersection */	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<RunAsInstructed>(prof->getValueAsNum("RUN3_PWML"),	      	\
			       prof->getValueAsNum("RUN3_PWMR"),0.0)	\
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN3_DIST"))	      	\
      .end()		      	\
      /* RUN4: before the corner around Gate 4 */	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN4_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_OPPOSITE) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN4_DIST"))	      	\
      .end()		      	\
      /* RUN5: until passing Gate 4 */	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN5_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_OPPOSITE) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN5_DIST"))	      	\
      .end()		      	\
      /* RUN6: the rest until detecting BLUE */	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN6_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_OPPOSITE) \
        .leaf<IsDistanceEarned>(prof->getValueAsNum("RUN6_DIST"))	      	\
      .end()		      	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN7_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_NORMAL) \
        .leaf<IsColorDetected>(CL_BLUE2)					\
        .leaf<IsColorDetected>(CL_BLUE)					\
        /*.leaf<SetArmPosition>(0, 40)           */\
      .end()								\
      /* RUN7: Turn and move forward.  */								\
      .composite<BrainTree::MemSequence>()								\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN5_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_OPPOSITE) \
        .leaf<IsDistanceEarned>(100)    	\
      .end()		      	\
        .composite<BrainTree::ParallelSequence>(1,2)								\
            .leaf<IsTimeEarned>(prof->getValueAsNum("RUN7_TIME"))								\
            .leaf<RunAsInstructed>(60, -60, 0.0)								\
        .end()  \
      .end()								  \
    .end()     								\
  .end()