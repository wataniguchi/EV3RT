#define TR_BLOCK_R \
  .leaf<StopNow>()

#define TR_BLOCK_L \
  .composite<BrainTree::MemSequence>()		\
    .leaf<StopNow>()					  \
    .leaf<IsTimeEarned>(3000000) /* wait 3 seconds */	  \
    .composite<BrainTree::ParallelSequence>(1,3)			   \
      .leaf<IsTimeEarned>(10000000) /* break after 10 seconds */		\
      .leaf<RunAsInstructed>(-50,-25,0.5)					\
    .end()								\
    .leaf<StopNow>()							\
  .end()
