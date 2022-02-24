/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="John: TeleOp", group="John")

public class JohnTele extends OpMode {

    JohnMap robot = new JohnMap();

    // Variables
    RevBlinkinLedDriver.BlinkinPattern pattern;
    double speed, spinSpeed, angle, lastHeading, armSpeed, turntableSpeed, intakeSpeed;
    
    // Constants
    static final double ARM_GOVERNOR = 1.00,
                        TURNTABLE_GOVERNOR = 1.00,
                        SLOWER_TURNTABLE_GOVERNOR = 0.80;
    
    static final int LEVEL_3 = 900,
                     SHARED = 400;
    
    @Override
    public void init()
    {
        robot.init( hardwareMap );

        robot.resetEncoders();
        robot.patternBlinkin( RevBlinkinLedDriver.BlinkinPattern.BLACK );
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop()
    {
        // -------------------------------- CALCULATING --------------------------------
        
        /*
         *   Motors
         */
        
        // Calculating speed...
        speed = Math.hypot( -gamepad1.left_stick_y, gamepad1.left_stick_x ) * Math.abs( Math.hypot( -gamepad1.left_stick_y, gamepad1.left_stick_x ) );
        
        if( robot.lowGear )
            speed /= 2;
        
        // Displaying speed...
        telemetry.addData( "speed", "%1.2f", speed );
        
        // Calculating spinSpeed...
        spinSpeed = -gamepad1.right_trigger - -gamepad1.left_trigger;
        
        if( robot.lowGear )
            spinSpeed /= 2;
        
        // Displaying spinSpeed...
        telemetry.addData( "spinSpeed", "%1.2f", spinSpeed );
        
        // Calculating angle...
        angle = Math.toDegrees( Math.atan2( -gamepad1.left_stick_y, gamepad1.left_stick_x ) );
        
        // Displaying angle...
        telemetry.addData( "angle", "%3.2f", angle );
        
        // Calculating armSpeed...
        armSpeed = ( -gamepad2.right_stick_y * ARM_GOVERNOR ) * Math.abs( -gamepad2.right_stick_y * ARM_GOVERNOR );
        
        // Displaying armSpeed...
        telemetry.addData( "armSpeed", armSpeed );
        
        /*
         *  Servos
         */
        
        // Turntable
        // Calculating turntableSpeed...
        turntableSpeed = gamepad2.right_stick_x;

        //Displaying turntableSpeed...
        telemetry.addData( "turntableSpeed", turntableSpeed );
        
        // Intake
        // Calculating intakeSpeed...
        intakeSpeed = -gamepad2.right_trigger - -gamepad2.left_trigger;
        
        // Displaying intakeSpeed...
        telemetry.addData( "intakeSpeed", intakeSpeed );
        
        // -------------------------------- BINDING --------------------------------
        
        // Centering...
        if( gamepad1.a )
            robot.robotCentric = !robot.robotCentric;
        
        telemetry.addData( "robotCentric", robot.robotCentric );
        
        // Gearing...
        if( gamepad1.b )
            robot.lowGear = !robot.lowGear;

        telemetry.addData( "lowGear", robot.lowGear );
        
        // Moving...
        if( speed > 0 && spinSpeed != 0 )
            robot.sauce( speed, spinSpeed, angle, robot.getHeading() - lastHeading );
        else if( speed > 0 && spinSpeed == 0 )
        {
            lastHeading = robot.getHeading();
            robot.move( speed, angle );
        }
        else if( speed == 0 && spinSpeed != 0 )
        {
            lastHeading = robot.getHeading();
            robot.spin( spinSpeed );
        }
        else
        {
            lastHeading = robot.getHeading();
            robot.stopWheels();
        }
        
        if( gamepad2.right_stick_y != 0 )
            robot.liftArm( ( armSpeed ) * Math.abs( armSpeed ) );
        else
            robot.stopArm();
            
        if( gamepad2.b && robot.getArmEncoderPosition() < LEVEL_3 ) 
            robot.liftArm( ARM_GOVERNOR );
        
        if( gamepad2.a && robot.getArmEncoderPosition() < SHARED )
            robot.liftArm( ARM_GOVERNOR );
        
        if( gamepad2.x )
            robot.resetArmEncoderPosition();
        
        /*
         *  Servos
         */
        
        // Ducky
        if( gamepad2.right_bumper )
            robot.spinDuckyRight();
        else if( gamepad2.left_bumper )
            robot.spinDuckyLeft();
        else
            robot.stopDucky();
        
        // Turntable
        if( turntableSpeed != 0 )
            robot.turnTurntable( turntableSpeed * Math.abs( turntableSpeed ) );
        else
            robot.stopTurntable();
        
        // Intake
        if( intakeSpeed != 0 )
            robot.intake( intakeSpeed );
        else
            robot.stopIntake();
    }

    @Override
    public void stop()
    {
        robot.stopRobot();
    }
}