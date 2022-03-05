package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous( name = "John: Auto - RD", group = "John" )
public class JohnAutoRD extends LinearOpMode
{
    public DcMotor rearMotor, rightMotor, leftMotor, arm;
    public DcMotorSimple duck, intake;
    public OpenCvCamera webcam;

                        // Robot constants
    public final double ROBOT_CIRC = 13.7 * Math.PI,
                        TICKS_PER_ROBOT_CM = 15,
                        CM_PER_INCH = 2.54,
                        TICKS_PER_ROBOT_INCH = TICKS_PER_ROBOT_CM * CM_PER_INCH,
                        
                        // Wheel constants
                        WHEEL_CIRC = 5.5 * Math.PI,
                        TICKS_PER_WHEEL_REV = 20.0 * 28.0,
                        
                        // Oddballs
                        WHEEL_REVS_PER_ROBOT_REV = ROBOT_CIRC / WHEEL_CIRC,
                        TICKS_PER_ROBOT_REV = TICKS_PER_WHEEL_REV * WHEEL_REVS_PER_ROBOT_REV,
                        
                        // vs
                        SPEED = 0.30,
                        APPLY_SPEED = 0.03,
                        SPIN_SPEED = 0.30,
                        ARM_SPEED = 0.30,
                        DUCK_SPEED = 0.60,
                        INTAKE_SPEED = 0.50,
                        OUTTAKE_SPEED = -0.50;
                    
                        // hs
                        public final int LEVEL_1 = 500,
                                         LEVEL_2 = 1000,
                                         LEVEL_3 = 1500,
                                         ZENITH = 1000,
                                         EPSILON = 20;
                    
    public final Scalar CAP_UPPER_BOUND = new Scalar( 75, 255, 255 ), // HSV v
                        CAP_LOWER_BOUND = new Scalar( 70, 150, 150 );

    public int level = -1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Motors
        rearMotor = hardwareMap.get( DcMotor.class, "Motor3" );
        rightMotor = hardwareMap.get( DcMotor.class, "Motor2" );
        leftMotor = hardwareMap.get( DcMotor.class, "Motor1" );
        
        arm = hardwareMap.get( DcMotor.class, "Arm" );
        
        rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        
        arm.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        
        rearMotor.setDirection( DcMotor.Direction.REVERSE );
        rightMotor.setDirection( DcMotor.Direction.REVERSE );
        leftMotor.setDirection( DcMotor.Direction.REVERSE );

        arm.setDirection( DcMotor.Direction.FORWARD );
        
        rearMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        rightMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        leftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        
        arm.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        
        // Servos
        duck = hardwareMap.get( DcMotorSimple.class, "Duck" );
        intake = hardwareMap.get( DcMotorSimple.class, "Intake" );

        duck.setDirection( DcMotorSimple.Direction.REVERSE );
        intake.setDirection( DcMotorSimple.Direction.REVERSE );

        // Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier( "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName() );

        webcam = OpenCvCameraFactory.getInstance().createWebcam( hardwareMap.get( WebcamName.class, "cam" ), cameraMonitorViewId );

        webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming( 1280, 720, OpenCvCameraRotation.UPRIGHT );
            }
            @Override
            public void onError(int errorCode) {}
        } );

        webcam.setPipeline( new CapPipeline() );
        
        resetArmEncoder();
        
        waitForStart();
        
        if( opModeIsActive() )
        {
            switch( level )
            {
                case 1:
                    doSync( new Go( 31, 120 ), new LiftArmTo( LEVEL_1 ) );
                    doNow( new OuttakeFor( 2 ) );
                    doSync( new Go( 28, 300 ), new WaitTo( 1, new BringArmIn() ) );
                    doNow( new Go( 24, 0 ) );
                    break;
                case 3:
                    doSync( new Go( 33, 120 ), new LiftArmTo( LEVEL_3 ) );
                    doNow( new OuttakeFor( 2 ) );
                    doSync( new Go( 30, 300 ), new WaitTo( 1, new BringArmIn() ) );
                    doNow( new Go( 24, 0 ) );
                    break;
                default:
                    doSync( new Go( 32, 120 ), new LiftArmTo( LEVEL_2 ) );
                    doNow( new OuttakeFor( 2 ) );
                    doSync( new Go( 29, 300 ), new WaitTo( 1, new BringArmIn() ) );
                    doNow( new Go( 24, 0 ) );
            }
        }
    }

    public void doNow( Thread toDo ) throws InterruptedException
    {
        toDo.start();

        toDo.join();
    }

    public void doSync( Thread ... toDos ) throws InterruptedException
    {
        for( Thread toDo : toDos )
            toDo.start();

        for( Thread toDo : toDos )
            toDo.join();
    }

    public void resetWheelEncoders()
    {
        rearMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
        leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODERS );
    }

    public void resetArmEncoder()
    {
        arm.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        arm.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
    }
    
    class WaitTo extends Thread
    {
        private int time;
        private Thread thread;

        public WaitTo( int t, Thread th ) 
        {
            time = t;
            thread = th;
        }

        public void run()
        {
            long startTime = System.currentTimeMillis() / 1000;

            while( opModeIsActive() && ( System.currentTimeMillis() / 1000 ) - startTime < time ) {}

            thread.run();
        }
    }

    class Go extends Thread
    {
        int dist, angle;

        public Go( int d, int a )
        {
            dist = d;
            angle = a;
        }

        public void run()
        {
            resetWheelEncoders();
            
            int rearTarget = ( int )( ( TICKS_PER_ROBOT_INCH * dist ) * Math.cos( Math.toRadians( 180 + angle ) ) );
            int rightTarget = ( int )( ( TICKS_PER_ROBOT_INCH * dist ) * Math.cos( Math.toRadians( 60 + angle ) ) );
            int leftTarget = ( int )( ( TICKS_PER_ROBOT_INCH * dist ) * Math.cos( Math.toRadians( 300 + angle ) ) );
            
            rearMotor.setTargetPosition( rearTarget );
            rightMotor.setTargetPosition( rightTarget );
            leftMotor.setTargetPosition( leftTarget );
            
            rearMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            rightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            leftMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            
            rearMotor.setPower( SPEED * Math.cos( Math.toRadians( 180 + angle ) ) );
            rightMotor.setPower( SPEED * Math.cos( Math.toRadians( 60 + angle ) ) );
            leftMotor.setPower( SPEED * Math.cos( Math.toRadians( 300 + angle ) ) );
            
            while( opModeIsActive() && ( rightMotor.isBusy() && leftMotor.isBusy() ) ) {}
            
            rearMotor.setPower( 0 );
            rightMotor.setPower( 0 );
            leftMotor.setPower( 0 );
        }
    }
    
    class Apply extends Thread 
    {
        int angle;
        int time;
        
        public Apply( int a, int t ) 
        {
            angle = a;
            time = t;
        }
        
        public void run() 
        {
            resetWheelEncoders();
            
            rearMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
            rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
            leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
            
            long startTime = System.currentTimeMillis() / 1000;
            
            rearMotor.setPower( APPLY_SPEED * Math.cos( Math.toRadians( 180 + angle ) ) );
            rightMotor.setPower( APPLY_SPEED * Math.cos( Math.toRadians( 60 + angle ) ) );
            leftMotor.setPower( APPLY_SPEED * Math.cos( Math.toRadians( 300 + angle ) ) );
                
            while( opModeIsActive() && ( System.currentTimeMillis() / 1000 ) - startTime < time ) {}
            
            rearMotor.setPower( 0 );
            rightMotor.setPower( 0 );
            leftMotor.setPower( 0 );
        }
    }

    class Turn extends Thread
    {
        int degrees;

        public Turn( int d )
        {
            degrees = d;
        }

        public void run()
        {
            resetWheelEncoders();

            int target = ( -1 ) * ( int )( TICKS_PER_ROBOT_REV * degrees / 360 );

            rearMotor.setTargetPosition( target );
            rightMotor.setTargetPosition( target );
            leftMotor.setTargetPosition( target );

            rearMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            rightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
            leftMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

            rearMotor.setPower( SPIN_SPEED );
            rightMotor.setPower( SPIN_SPEED );
            leftMotor.setPower( SPIN_SPEED );

            while( opModeIsActive() && ( rearMotor.isBusy() || rightMotor.isBusy() || leftMotor.isBusy() ) ) {}

            rearMotor.setPower( 0 );
            rightMotor.setPower( 0 );
            leftMotor.setPower( 0 );
        }
    }

    class LiftArmTo extends Thread
    {
        int position;

        public LiftArmTo( int p )
        {
            position = p;
        }

        public void run()
        {
            while( opModeIsActive() && Math.abs( position - arm.getCurrentPosition() ) > EPSILON ) 
            {
                arm.setPower( position - arm.getCurrentPosition() > 0? ARM_SPEED : -ARM_SPEED );
            }
            
            arm.setPower( 0 );
        }
    }

    class BringArmIn extends Thread
    {
        public void run()
        {
            new LiftArmTo( 0 ).run();
            resetArmEncoder();
        }
    }

    class DuckFor extends Thread
    {
        int time;

        public DuckFor( int t )
        {
            time = t;
        }

        public void run()
        {
            int startTime = ( int )( System.currentTimeMillis() / 1000.0 );

            duck.setPower( DUCK_SPEED );

            while( opModeIsActive() && ( int )( System.currentTimeMillis() / 1000.0 ) - startTime < time  ) {}

            duck.setPower( 0 );
        }
    }

    class IntakeFor extends Thread
    {
        int time;

        public IntakeFor( int t )
        {
            time = t;
        }

        public void run()
        {
            int startTime = ( int )( System.currentTimeMillis() / 1000.0 );

            intake.setPower( INTAKE_SPEED );

            while( opModeIsActive() && ( int )( System.currentTimeMillis() / 1000.0 ) - startTime < time  ) {}

            intake.setPower( 0 );
        }
    }

    class OuttakeFor extends Thread
    {
        int time;

        public OuttakeFor( int t )
        {
            time = t;
        }

        public void run()
        {
            int startTime = ( int )( System.currentTimeMillis() / 1000.0 );

            intake.setPower( OUTTAKE_SPEED );

            while( opModeIsActive() && ( int )( System.currentTimeMillis() / 1000.0 ) - startTime < time  ) {}

            intake.setPower( 0 );
        }
    }

    class CapPipeline extends OpenCvPipeline
    {
        Mat mat = new Mat();
        
        final Rect RECT_1 = new Rect( new Point( 150, 650 ), new Point( 350, 250 ) );
        final Rect RECT_2 = new Rect( new Point( 550, 650 ), new Point( 750, 250 ) );
        final Rect RECT_3 = new Rect( new Point( 1000, 650 ), new Point( 1200, 250 ) );
            
        @Override
        public Mat processFrame( Mat input ) 
        {
            Imgproc.cvtColor( input, mat, Imgproc.COLOR_RGB2HSV );
            
            Core.inRange( mat, CAP_LOWER_BOUND, CAP_UPPER_BOUND, mat );
            
            Mat loc1 = mat.submat( RECT_1 );
            Mat loc2 = mat.submat( RECT_2 );
            Mat loc3 = mat.submat( RECT_3 );
            
            double percent1 = Core.sumElems( loc1 ).val[0] / RECT_1.area() / 255;
            double percent2 = Core.sumElems( loc2 ).val[0] / RECT_2.area() / 255;
            double percent3 = Core.sumElems( loc3 ).val[0] / RECT_3.area() / 255;
            
            loc1.release();
            loc2.release();
            loc3.release();
            
            telemetry.addData( "loc 1 raw value", (int) Core.sumElems( loc1 ).val[ 0 ] );
            telemetry.addData( "loc 2 raw value", (int) Core.sumElems( loc2 ).val[ 0 ] );
            telemetry.addData( "loc 3 raw value", (int) Core.sumElems( loc3 ).val[ 0 ] );
            
            telemetry.addData( "loc 1 percentage", Math.round( percent1 * 100 ) + "%" );
            telemetry.addData( "loc 2 percentage", Math.round( percent2 * 100 ) + "%");
            telemetry.addData( "loc 3 percentage", Math.round( percent3 * 100 ) + "%");
            
            if( percent1 > Math.max( percent2, percent3 ) ) 
                level = 1;
            
            if( percent2 > Math.max( percent1, percent3 ) ) 
                level = 2;
            
            if( percent3 > Math.max( percent1, percent2 ) ) 
                level = 3;
            
            telemetry.addData( "level", level );
            
            telemetry.update();
            
            Imgproc.cvtColor( mat, mat, Imgproc.COLOR_GRAY2RGB );
            
            final Scalar NOT_HERE = new Scalar( 255, 0, 0 );
            final Scalar HERE = new Scalar( 0, 128, 0 );
            
            Imgproc.rectangle( mat, RECT_1, level == 1? HERE : NOT_HERE );
            Imgproc.rectangle( mat, RECT_2, level == 2? HERE : NOT_HERE );
            Imgproc.rectangle( mat, RECT_3, level == 3? HERE : NOT_HERE );
            
            return mat;
        }
    }
}