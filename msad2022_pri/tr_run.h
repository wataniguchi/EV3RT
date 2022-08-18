#define TR_RUN_R \
  .composite<BrainTree::ParallelSequence>(1,2)		   \
    .leaf<TraceLine>(prof->getValueAsNum("SPEED"),	       \
		     prof->getValueAsNum("GS_TARGET"),			\
		     prof->getValueAsNum("P_CONST"),			\
		     prof->getValueAsNum("I_CONST"),			\
		     prof->getValueAsNum("D_CONST"), 0.0, TS_NORMAL)	\
    .leaf<IsDistanceEarned>(2000)						\
  .end()

#define TR_RUN_L \
  .composite<BrainTree::ParallelSequence>(1,3)	\
    .leaf<IsBackOn>()				\
    .leaf<IsDistanceEarned>(1000)			 \
    .composite<BrainTree::MemSequence>()			 \
      .leaf<IsColorDetected>(CL_BLACK)				 \
      .leaf<IsColorDetected>(CL_BLUE)				 \
    .end()								\
    .leaf<TraceLine>(SPEED_NORM, GS_TARGET, P_CONST, I_CONST, D_CONST, 0.0, TS_NORMAL) \
  .end()
