package frc.robot.subsystems.indicator;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.drivers.led.AddressableLEDPattern;
import frc.robot.drivers.led.patterns.BlinkingPattern;
import frc.robot.drivers.led.patterns.RainbowPattern;
import frc.robot.drivers.led.patterns.SolidColorPattern;
import org.littletonrobotics.junction.AutoLog;

public interface IndicatorIO {
    /**
     * Returns alliance color.
     *
     * @return Current alliance color
     */
    default Color allianceColor() {
//        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
//            case Blue -> Color.kBlue;
//            case Red -> Color.kRed;
//        };
        return Color.kBlue;
    }

    /**
     * Updates the set of loggable inputs.
     */
    void updateInputs(IndicatorIOInputs inputs);

    /**
     * Set current pattern.
     */
    void setPattern(Patterns pattern);

    /**
     * Stops and starts indicator.
     */
    void reset();

    /**
     * All available patterns.
     */
    enum Patterns {
        NORMAL(new SolidColorPattern(Color.kBlue)),
        INDEXED(new SolidColorPattern(Color.kGreen)),
        SPEAKER_AIMING(new BlinkingPattern(Color.kRed, 0.1)),
        FERRY_AIMING(new BlinkingPattern(Color.kYellow, 0.1)),
        INDEX_FINISHING(new BlinkingPattern(Color.kGreen, 0.05)),
        INDEXING(new BlinkingPattern(Color.kBlue, 0.2)),
        SHOOT_FINISH(new BlinkingPattern(Color.kBlue, 0.1)),
        //RAINBOW WILL CAUSE ERROR, DO NOT USE
        //TODO: fix it
        SHOULD_AMPLIFY(new RainbowPattern()),
        CAN_CLIMB(new BlinkingPattern(Color.kPurple, 0.2)),
        CLIMBING(new BlinkingPattern(Color.kViolet, 0.7)),
        RESET_ODOM(new BlinkingPattern(Color.kWhite, 0.25));

        public final AddressableLEDPattern pattern;

        Patterns(AddressableLEDPattern color) {
            this.pattern = color;
        }
    }

    @AutoLog
    class IndicatorIOInputs {
        public Patterns currentPattern;
    }
}
