#define TR_CALIBRATION \
  .composite<BrainTree::MemSequence>() \
  /*.decorator<BrainTree::UntilSuccess>() \
      .leaf<IsTouchOn>() \
      .end() */ \
    .leaf<IsTimeEarned>(1000000) /* test only - wait one sec for filter */ \
    .leaf<ResetClock>()	\
  .end()
