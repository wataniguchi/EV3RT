#define TR_CALIBRATION \
  .composite<BrainTree::MemSequence>()					\
  /* temp fix 2022/6/20 W.Taniguchi, as no touch sensor available on RasPike \
     .decorator<BrainTree::UntilSuccess>()				\
       .leaf<IsTouchOn>()		       				\
     .end()								\
  */ \
    .leaf<ResetClock>()				\
  .end()
