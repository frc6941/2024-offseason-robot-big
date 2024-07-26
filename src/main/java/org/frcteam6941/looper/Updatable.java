package org.frcteam6941.looper;

public interface Updatable {
    default void read(double time, double dt) {}

    default void update(double time, double dt) {}

    default void write(double time, double dt) {}

    default void telemetry() {}

    default void start() {}

    default void stop() {}

    default void simulate(double time, double dt) {
        read(time, dt);
    }
}
