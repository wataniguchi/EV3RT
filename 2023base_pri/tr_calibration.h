#define TR_CALIBRATION \
  .composite<BrainTree::MemSequence>()					\
    .decorator<BrainTree::UntilSuccess>()				\
      .leaf<IsTouchOn>()		       				\
    .end()								\
    .leaf<ResetClock>()				\
  .end()
