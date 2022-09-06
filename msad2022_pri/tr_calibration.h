#define TR_CALIBRATION \
  .composite<BrainTree::MemSequence>()					\
    .composite<BrainTree::ParallelSequence>(1,2)				\
       .leaf<IsBackOn>()		       				\
    .end()								\
    .leaf<ResetClock>()				\
  .end()
