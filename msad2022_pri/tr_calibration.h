#define TR_CALIBRATION \
  .composite<BrainTree::MemSequence>()					\
    .composite<BrainTree::ParallelSequence>(0,2)				\
       .leaf<IsTouchOn>()		       				\
    .end()								\
    .leaf<ResetClock>()				\
  .end()
