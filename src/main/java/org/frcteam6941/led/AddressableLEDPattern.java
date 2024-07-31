package org.frcteam6941.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface AddressableLEDPattern {
    default void setLEDs(AddressableLEDBuffer buffer) {
        
    }

    default boolean isAnimated() {
        return false;
    }
}
