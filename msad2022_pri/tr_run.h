#define TR_RUN_L \
  /* RUN: a regression test case to ensure sensor trace works */	\
  .composite<BrainTree::MemSequence>()			\
    .composite<BrainTree::ParallelSequence>(1,2) \
      .leaf<TraceLine>(prof->getValueAsNum("SPEED"),		\
                       prof->getValueAsNum("GS_TARGET"),				\
                       prof->getValueAsNum("P_CONST"),				\
                       prof->getValueAsNum("I_CONST"),					\
                       prof->getValueAsNum("D_CONST"), 0.0,					\
                       (TraceSide)prof->getValueAsNum("TR_TS"))				\
      .leaf<IsDistanceEarned>(prof->getValueAsNum("TR_DIST"))		\
    .end()								\
  .end()

#define TR_RUN_R \
  /* RUN: MemSequenceを順次実行しながら青ラインを検知したら終了する */	\
  .composite<BrainTree::ParallelSequence>(1,3)				\
    .leaf<IsColorDetected>(CL_BLUE)				 \
    .leaf<IsDistanceEarned>(prof->getValueAsNum("RUNx_DIST")) /* temp stopper */ \
    .composite<BrainTree::MemSequence>()			\
      /* RUN1: 1番目の分岐までTS_NORMAL側をトレースする */	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN1_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_OPPOSITE) \
         .leaf<IsJunction>(JST_JOINED)						\
      .end()								\
      /* RUN2: 2番目の分岐までTS_NORMAL側をトレースする */	\
      .composite<BrainTree::ParallelSequence>(1,2)				\
        .leaf<TraceLineCam>(prof->getValueAsNum("RUN2_SPEED"),			\
			    prof->getValueAsNum("RUNx_P_CONST"),	\
			    prof->getValueAsNum("RUNx_I_CONST"),	\
			    prof->getValueAsNum("RUNx_D_CONST"),	\
			    prof->getValueAsNum("RUNx_GS_MIN"),		\
			    prof->getValueAsNum("RUNx_GS_MAX"), 0.0, TS_NORMAL) \
         .leaf<IsJunction>(JST_FORKED)						\
      .end()								\
    .end()     								\
  .end()
