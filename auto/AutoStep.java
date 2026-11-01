package org.firstinspires.ftc.teamcode.auto;

/**
 * One unit of an autonomous routine.
 *
 * The main loop calls {@link #begin()} once, then {@link #update(double)} every
 * iteration until it returns true, then {@link #end()}. Keeping steps this small
 * makes an auto program readable as a list of intents instead of a switch
 * statement with a dozen magic integers.
 */
public interface AutoStep {

    String name();

    void begin();

    /** @return true when the step is finished. */
    boolean update(double dtSeconds);

    void end();
}
