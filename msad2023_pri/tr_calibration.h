#define TR_CALIBRATION \
  .composite<BrainTree::MemSequence>() \
    .leaf<IsTimeEarned>(1000000) /* wait one sec for filling the filter pipeline */ \
    .leaf<ArmUpDownFull>(AD_UP) /* raise arm to the full */ \
    .leaf<ResetArm>() /* reset arm angle to zero */ \
    .leaf<SetArmPosition>(prof->getValueAsNum("CAL_x_ARM_ANGLE"),ARM_SHIFT_PWM) /* then down */ \
    .leaf<SetMotorAdjustmentFactors>(prof->getValueAsNumVec("CAL_x_MOTOR_ADJ_FACTORS")) \
    /* wait until Touch Button is pressed */ \
    .decorator<BrainTree::UntilSuccess>()  \
      .leaf<IsEnterOn>() \
      .end() 		 \
    .leaf<ResetClock>()	\
  .end()
