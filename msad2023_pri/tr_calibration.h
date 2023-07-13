#define TR_CALIBRATION \
  .composite<BrainTree::MemSequence>() \
    .leaf<IsTimeEarned>(1000000) /* wait one sec for filling the filter pipeline */ \
    .leaf<ResetArm>() /* reset arm */ \
    .leaf<SetArmPosition>(30,30) /* then down */ \
    /* wait until Touch Button is pressed */ \
  /*.decorator<BrainTree::UntilSuccess>() \
      .leaf<IsTouchOn>() \
    .end() */ \
    .leaf<ResetClock>()	\
  .end()
