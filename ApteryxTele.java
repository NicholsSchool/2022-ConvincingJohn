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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import java.util.Map;
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

@TeleOp(name="Apteryx: TeleOp", group="Apteryx")

public class ApteryxTele extends OpMode{

    ApteryxMap robot = new ApteryxMap();

    // Variables
    RevBlinkinLedDriver.BlinkinPattern pattern;
    double speed, spinSpeed, angle;
    float lastHeading;

    @Override
    public void init() {
        
        robot.init( hardwareMap );
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        
        
        /*
         *   SPEED
         */
        
        // Calculating speed, r, magnitude of vector... r = sqrt( x^2 + y^2 ) ...
        speed = Math.sqrt( Math.pow( gamepad1.left_stick_x, 2 ) + Math.pow( gamepad1.left_stick_y, 2 ) );
        
        // Displaying speed...
        telemetry.addData( "speed", "%1.2f", speed );
        
        
        /*
         *   SPIN SPEED
         */
        
        // Calculating spinSpeed...
        spinSpeed = 0 + gamepad1.right_trigger - gamepad1.left_trigger;
        
        // Displaying spinSpeed...
        telemetry.addData( "spinSpeed", "%1.2f", spinSpeed );
        
        
        /* 
         *   ANGLE
         */
        
        // Calculating angle, Theta, direction of vector... Theta = angle = arctan( y / x ) ...
        angle = ( 180 / Math.PI ) * Math.atan( -gamepad1.left_stick_y / gamepad1.left_stick_x );
        
        // Accounting for arctan's silly Range...
        if( gamepad1.left_stick_x < 0 )
            angle += 180;
        if( angle < 0 )
            angle += 360;
        
        // Displaying angle...
        telemetry.addData( "angle", "%3.2f", angle );
        
        
        /*
         *   ROBOT
         */
         
        telemetry.addData( "Pattern", pattern );
         
        // Blinkin Patterning
        if( gamepad1.a )
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.fromNumber( 1 + new java.util.Random().nextInt( 100 ) ) );
        if( gamepad1.dpad_down )
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.BLACK );

        
        // Movement. Moving, Spinning, and Saucing.
        if( speed > 0 ) {
            if( spinSpeed != 0 )
                robot.sauce( speed, spinSpeed, angle, robot.getHeading() - lastHeading );
            else {
                lastHeading = robot.getHeading();
                robot.move( speed, angle );
            } 
        }
        else if( spinSpeed != 0 ) {
            if( speed > 0 )
                robot.sauce( speed, spinSpeed, angle, robot.getHeading() - lastHeading );
            else {
                lastHeading = robot.getHeading();
                robot.spin( spinSpeed );
            }
        } 
        else
            stop();
        
        
        // Front-shifting and s'more Blinkin Patterning
        if( gamepad1.x ) {
            robot.setFront( 'x' );
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
        if( gamepad1.y ) {
            robot.setFront( 'y' );
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        if( gamepad1.b ) {
            robot.setFront( 'b' );
            robot.blinkin.setPattern( RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        
        
        // Robot 'Anchoring'
        if( gamepad1.left_stick_button )
            robot.anchorless = !robot.anchorless;
        
        telemetry.addData( "ANCHORED", !robot.anchorless );
    }

    @Override
    public void stop() { robot.stop(); }
}
