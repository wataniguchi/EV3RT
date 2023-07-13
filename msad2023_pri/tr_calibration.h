#define TR_CALIBRATION \
  .composite<BrainTree::MemSequence>() \
    .leaf<IsTimeEarned>(1000000) /* wait one sec for filling the filter pipeline */ \
    .leaf<SetArmPosition>(-15,30) /* lift up the arm... */ \
    .leaf<ResetArm>() /* and mark arm position as zero */ \
    /* wait until Touch Button is pressed */ \
  /*.decorator<BrainTree::UntilSuccess>() \
      .leaf<IsTouchOn>() \
    .end() */ \
    .leaf<ResetClock>()	\
  .end()
