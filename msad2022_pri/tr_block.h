#define TR_BLOCK_R \
  .leaf<StopNow>()

#define TR_BLOCK_L \
  .composite<BrainTree::MemSequence>()		\
    .leaf<StopNow>()
/*					  \
    .leaf<IsTimeEarned>(3000000)	  \
    .composite<BrainTree::ParallelSequence>(1,3)			   \
      .leaf<IsTimeEarned>(10000000)		\
      .leaf<RunAsInstructed>(50,25,0.5)					\
    .end()								\
    .leaf<StopNow>()							\
*/
  .end()
