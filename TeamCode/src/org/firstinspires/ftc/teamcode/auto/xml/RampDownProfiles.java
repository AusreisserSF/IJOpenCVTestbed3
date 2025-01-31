package org.firstinspires.ftc.teamcode.auto.xml;

import java.util.EnumMap;

public class RampDownProfiles {
    public enum MovementKey {
        UPWARD_RAMPDOWN_TEST, DOWNWARD_RAMPDOWN_TEST,
        START_EXTEND_TO_HIGH_BASKET_PRELOAD,
        START_EXTEND_TO_HIGH_CHAMBER_NOREQ
    }

    private final EnumMap<MovementKey, RampDownProfile> rampDownProfiles;


    public RampDownProfiles(EnumMap<MovementKey, RampDownProfile> pRampDownProfiles) {
        rampDownProfiles = pRampDownProfiles;
    }

    public RampDownProfile getRampDownProfile(MovementKey pMovementKey) {
        return rampDownProfiles.get(pMovementKey);
    }

    public static class RampDownProfile {
        public enum RampDownDirection {ASCENDING, DESCENDING}

        // The RampDownDirection specifies whether the ramp-down should be
        // performed when the motor is moving from its current click count
        // to a higher click count or a lower click count. Including this
        // value here avoids our having to infer the direction of the ramp-
        // down, which is easy in the case of a single motor but not so easy
        // in the case of dual motors where one motor's click count may be
        // above the target and the other below.
        public final RampDownDirection rampDownDirection;

        public final double initialVelocity;
        public final double finalVelocity;
        public final double rampDownFactor; // apply to Math.abs(target clicks -  current clicks)

        public RampDownProfile(RampDownDirection pRampDownDirection,
                               double pInitialVelocity, double pFinalVelocity, double pRampDownFactor) {

            rampDownDirection = pRampDownDirection;

            // With run to position velocity is always positive; the position determines the direction.
            initialVelocity = Math.abs(pInitialVelocity);
            finalVelocity = Math.abs(pFinalVelocity);
            rampDownFactor = pRampDownFactor;
        }

    }
}