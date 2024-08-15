package frc.robot.subsystems.indicator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indicator.IndicatorIO.Patterns;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class IndicatorSubsystem extends SubsystemBase {
    private final IndicatorIO io;
    private final IndicatorIOInputsAutoLogged inputs = new IndicatorIOInputsAutoLogged();
    private Patterns currentPattern = Patterns.NORMAL;
    @Getter
    private Patterns lastPattern = Patterns.NORMAL;

    private final Timer timer = new Timer();

    public IndicatorSubsystem(IndicatorIO io) {
        this.io = io;
    }

    public void setPattern(Patterns pattern) {
        if (pattern == currentPattern) {
            io.setPattern(currentPattern);
            return;
        }
        lastPattern = currentPattern;
        currentPattern = pattern;
        io.setPattern(pattern);
        switch (pattern) {
            case FINISH_SHOOT, FINISH_INDEX -> timer.restart();
            default -> {
            }
        }
    }

    @Override
    public void periodic() {
        switch (currentPattern) {
            case FINISH_SHOOT, FINISH_INDEX -> resetLed();
            default -> {
            }
        }
        io.updateInputs(inputs);
        Logger.processInputs("Indicator", inputs);
    }

    private void resetLed() {
        if (!timer.hasElapsed(2)) return;
        switch (currentPattern) {
            case FINISH_SHOOT -> setPattern(Patterns.NORMAL);
            case FINISH_INDEX -> setPattern(Patterns.INDEX);
        }
    }

    public void reset() {
        this.io.reset();
    }

    public void resetToLastPattern() {
        setPattern(lastPattern);
    }
}
