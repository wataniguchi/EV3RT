#define TR_RUN_R \
  .composite<BrainTree::MemSequence>()			\
  /*							\
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<TraceLine>(prof->getValueAsNum("SPEED"),		\
                       prof->getValueAsNum("GS_TARGET"),				\
                       prof->getValueAsNum("P_CONST"),				\
                       prof->getValueAsNum("I_CONST"),					\
                       prof->getValueAsNum("D_CONST"), 0.0,					\
                       (TraceSide)prof->getValueAsNum("TR_TS"))				\
      .leaf<IsDistanceEarned>(prof->getValueAsNum("TR_DIST"))		\
    .end()								\
  */									\
    .composite<BrainTree::ParallelSequence>(1,2)				\
      .leaf<TraceLineCam>(prof->getValueAsNum("CAM_SPEED"),			\
			  prof->getValueAsNum("CAM_P_CONST"),		\
			  prof->getValueAsNum("CAM_I_CONST"),		\
			  prof->getValueAsNum("CAM_D_CONST"),		\
			  prof->getValueAsNum("CAM_GS_MIN"),		\
			  prof->getValueAsNum("CAM_GS_MAX"), 0.0,	\
			  (TraceSide)prof->getValueAsNum("CAM_TS"))	\
      .leaf<IsDistanceEarned>(prof->getValueAsNum("TR_DIST"))		\
    .end()								\
  .end()

#define TR_RUN_L \
  .composite<BrainTree::MemSequence>()			\
  /*							\
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<TraceLine>(prof->getValueAsNum("SPEED"),		\
                       prof->getValueAsNum("GS_TARGET"),				\
                       prof->getValueAsNum("P_CONST"),				\
                       prof->getValueAsNum("I_CONST"),					\
                       prof->getValueAsNum("D_CONST"), 0.0,					\
                       (TraceSide)prof->getValueAsNum("TR_TS"))				\
      .leaf<IsDistanceEarned>(prof->getValueAsNum("TR_DIST"))		\
    .end()								\
  */									\
    .composite<BrainTree::ParallelSequence>(1,2)				\
      .leaf<TraceLineCam>(prof->getValueAsNum("CAM_SPEED"),			\
			  prof->getValueAsNum("CAM_P_CONST"),		\
			  prof->getValueAsNum("CAM_I_CONST"),		\
			  prof->getValueAsNum("CAM_D_CONST"),		\
			  prof->getValueAsNum("CAM_GS_MIN"),		\
			  prof->getValueAsNum("CAM_GS_MAX"), 0.0,	\
			  (TraceSide)prof->getValueAsNum("CAM_TS"))	\
      .leaf<IsDistanceEarned>(prof->getValueAsNum("TR_DIST"))		\
    .end()								\
  .end()
