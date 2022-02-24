package org.firstinspires.ftc.teamcode;

/**
 * An errand - something the robot will do.
 * 
 * @author Camden D. Bartlo
 * @version 2 - 24 - 22
 */
public interface Errand
{
    /**
     * What 'subsystems' the Errand utilizes. There are five
     *  subsystems, each repressented by a '0' or '1' depending
     *  on whether or not the 'subsystem' utilizes it.
     *
     * First digit is he Wheels, then the Arm, then the Turntable,
     *  then the Intake and lastly the Ducky.
     */
    public static final int reqs = 0b11111;
    
    /**
     * Calculates and returns the 'inverse' of the Errand.
     *  In other, better words - the Errand that'd 'undo' 
     *  this Errand.
     * 
     * @return the 'inverse' of this Errand
     */
    public Errand invert();
}