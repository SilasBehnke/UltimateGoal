package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.OutputStream;


public abstract class Functions {

    private DcMotor FLM;
    private DcMotor FRM;
    private DcMotor BLM;
    private DcMotor BRM;
    private DcMotorEx FlyMotor;
    private DcMotorEx FlyMotor2;

    private DcMotor IntakeMotor;
    private DcMotor TransferMotor;

    private DistanceSensor TheDS;
    private DistanceSensor TheSideDS;
    private DistanceSensor FDS;
    private DistanceSensor BDS;
    private DistanceSensor LDS;
    /// private DistanceSensor RDS;

    private Servo wobble1;
    private Servo wobble2;
    private Servo wobble3;
    private Servo wobble4;
    private Servo intakeDropper;
    private Servo RingBlocker;


    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation(); //sets the last angle to whatever the robot last had. This is just to avoid errors


    private double globalAngle; //the number of degrees the robot has turned

    public abstract boolean CanMove();

    public abstract void AddToTelemetry(String Tag, String Message);

    public abstract void UpdateTelemetry();

    public double NormPower = 0.7;
    public Move MoveToZone;


    public void Init(HardwareMap hardwareMap) {

        FLM = hardwareMap.get(DcMotor.class, "FLM"); //get the motors from the config
        FRM = hardwareMap.get(DcMotor.class, "FRM");
        BLM = hardwareMap.get(DcMotor.class, "BLM");
        BRM = hardwareMap.get(DcMotor.class, "BRM");
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        TransferMotor = hardwareMap.get(DcMotor.class, "Transfer");


        RingBlocker = hardwareMap.get(Servo.class, "RingBlocker");


        FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        FlyMotor = hardwareMap.get(DcMotorEx.class, "Fly");
        FlyMotor2 = hardwareMap.get(DcMotorEx.class, "Fly2");
        FlyMotor2.setDirection(DcMotorEx.Direction.REVERSE);


        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we set the power to 0 we want the motors to stop
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //if we don't set it they will be in neutral and friction will slow it
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLM.setDirection(DcMotor.Direction.REVERSE); //reverse the motors
        BLM.setDirection(DcMotor.Direction.REVERSE);

        /*wobble1 = hardwareMap.get(Servo.class, "wobble1");
        wobble2 = hardwareMap.get(Servo.class, "wobble2");
        wobble3 = hardwareMap.get(Servo.class, "wobble3");
        wobble4 = hardwareMap.get(Servo.class, "wobble4");
        Flywheel = hardwareMap.get(Servo.class, "Flywheel");
        RingBlocker = hardwareMap.get(Servo.class, "RingBlocker");
        intakeDropper = hardwareMap.get(Servo.class, "Flywheel");*/


        imu = hardwareMap.get(BNO055IMU.class, "imu"); //gets the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //makes parameters for imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters); //initalizes the imu
        while (CanMove() && !imu.isGyroCalibrated()) {
            if (!CanMove()) return;
        }

        ResetAngle();
    }


    double MaxPower = .7;
    double MinPower = -MaxPower;

    public void Pathing(HardwareMap hwmp){
        SampleMecanumDrive bot = new SampleMecanumDrive(hwmp);
        Trajectory traj1 = bot.trajectoryBuilder(new Pose2d(50,-52))
                .forward(84)
                .build();
        Trajectory traj2 = bot.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-50,-30),0)
                .build();

    }


    public double CurrentEncoderPosition(){
        double AverageEncoder = FRM.getCurrentPosition() + FRM.getCurrentPosition() + BLM.getCurrentPosition() + BRM.getCurrentPosition();
        AverageEncoder /= 4;
        return AverageEncoder;
    }

    public void EncoderPID(double distance, double correction) {
        double desiredTicks = InchesToTicks(distance + correction);
        double motorPos = CurrentEncoderPosition();
        double currentPosition = motorPos;
        double previousPosition = motorPos;
        double motorPower, kP = 5;
        double error = desiredTicks - motorPos;
        double MaxError = error;
        while (CanMove() && Math.abs(error) > 10) {
            motorPos = CurrentEncoderPosition();
            previousPosition = currentPosition;
            currentPosition = motorPos;

            error = desiredTicks - motorPos;
            motorPower = (kP * error) / MaxError;
            if (motorPower > MaxPower) motorPower = MaxPower;
            else if (motorPower < MinPower) motorPower = MinPower;
            AddToTelemetry("error", String.valueOf(error));
            AddToTelemetry("current", String.valueOf(currentPosition));
            AddToTelemetry("Desired", String.valueOf(desiredTicks + previousPosition));
            AddToTelemetry("Speed", String.valueOf(motorPower));

            UpdateTelemetry();
            DriveTicks(motorPower);
        }
    }


    protected double InchesToTicks(double inches) {
        double tick = 55;
        double TicksNeeded = inches * tick;
        return TicksNeeded;
    }


    public void CheckEncoders() {
        while (CanMove()) {
            double outputFL = FLM.getCurrentPosition();
            double outputFR = FRM.getCurrentPosition();
            double outputBL = BLM.getCurrentPosition();
            double outputBR = BRM.getCurrentPosition();


            AddToTelemetry("OutputFL", String.valueOf(outputFL));
            AddToTelemetry("OutputFR", String.valueOf(outputFR));
            AddToTelemetry("OutputBL", String.valueOf(outputBL));
            AddToTelemetry("OutputBR", String.valueOf(outputBR));

            UpdateTelemetry();
        }
    }



    private void ResetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //sets lastAngles to current angles

        globalAngle = 0; //global angle is set to 0
    }

    public void Drive(double forward, double sideways, double rotation) { //make a function to drive
        double correction = 0; //default correction
        if (rotation == 0) {
            if (forward != 0) {
                correction = CheckDirection(Math.abs(forward)); //if there isn't any rotation then use correction
            } else if (sideways != 0) {
                correction = CheckDirection(Math.abs(sideways)); //if there isn't any rotation then use correction
            }
        }

        FRM.setPower(-(forward + sideways + rotation) + correction);
        FLM.setPower(-(forward - sideways - rotation) - correction);
        BRM.setPower((forward - sideways + rotation) + correction);
        BLM.setPower((forward + sideways - rotation) - correction);
    }

    public void DriveTicks(double forward) { //make a function to drive
        double correction = 0; //default correction
        correction = CheckDirection(Math.abs(forward)); //if there isn't any rotation then use correction
        FRM.setPower(forward);
        FLM.setPower(forward);
        BRM.setPower(forward);
        BLM.setPower(forward);
    }

    private double GetAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES); //gets the angle

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle; //deltaAngle is the current angle minus the last angle it got

        if (deltaAngle < -180) //switches it to use 0 to 360 instead of -180 to 180
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle; //adds the deltaAngle to the globalAngle

        lastAngles = angles; //lastAngle is the angles

        return globalAngle; //returns the amount turned
    }

    private double CheckDirection(double Speed) {
        double correction;

        double angle = GetAngle();  //get the total amount the angle has changed since last reset

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        double CorrectionSensitivity = 0.10 * Speed;

        correction = correction * CorrectionSensitivity;

        return correction;
    }

    public Move PathToZone(int pathChoice) {
        switch (pathChoice) {
            case 0: //zone A
                MoveToZone = new Move(84, 12, NormPower, Move.Direction.Forward);
            case 1: // Zone B
                MoveToZone = new Move(108, 36, NormPower, Move.Direction.Forward);
            case 2: //Zone C
                MoveToZone = new Move(132, 12, NormPower, Move.Direction.Forward);
            default:
                MoveToZone = new Move(84, 12, NormPower, Move.Direction.Forward);


        }
        return null;
    }
    public void FlyPower(double tickssec){
        double error1, error2, P1 = 1,P2 = 1;
        error1 = tickssec - FlyMotor.getVelocity();
        error2 = tickssec - FlyMotor2.getVelocity();
        FlyMotor.setVelocity((P1*error1)+tickssec);
        FlyMotor2.setVelocity((P2*error2)+tickssec);

    }
    public void TeleOp(Gamepad gamepad1) {
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double forward;
        double strafe;
        double turn;
        boolean intake = false, ramp = false;
        //Intake Code
        boolean previousPressIntake = false, previousPressFly = false, previousPressPush = false, previousPressRingBlock;
        boolean isIntaking = false, isFly = false, isPush = false;
        int stoneLevel = 0;
        double Fly1velocity1;
        double Fly2velocity;
        while (CanMove()) {
            AddToTelemetry("RingBlocker", String.valueOf(RingBlocker.getPosition()));
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.right_stick_x;
            turn = -gamepad1.left_stick_x;
            AddToTelemetry("Intake", String.valueOf(gamepad1.b));
            AddToTelemetry("Flywheels", String.valueOf(gamepad1.y));

            //Intake
           // IntakeMotor.setPower(gamepad1.b == true ? 1 : 0);
            if (gamepad1.b) {
                if (previousPressIntake != gamepad1.b) {
                    if (!isIntaking) {
                        IntakeMotor.setPower(-1);
                        TransferMotor.setPower(-1);
                        isIntaking = true;
                    } else {
                        IntakeMotor.setPower(0);
                        TransferMotor.setPower(0);
                        isIntaking = false;
                    }
                    previousPressIntake = true;
                }
            } else {
                previousPressIntake = false;
            }
            if (gamepad1.b) {
                if (previousPressIntake != gamepad1.b) {
                    if (!isIntaking) {
                        IntakeMotor.setPower(-1);
                        TransferMotor.setPower(-1);
                        isIntaking = true;
                    } else {
                        IntakeMotor.setPower(0);
                        TransferMotor.setPower(0);
                        isIntaking = false;
                    }
                    previousPressIntake = true;
                }
            } else {
                previousPressIntake = false;
            }

            //FLywheels
            if (gamepad1.y) {
                if (previousPressFly != gamepad1.y) {
                    if (!isFly) {
                        FlyMotor2.setPower(1);
                        FlyMotor.setPower(1);
                        isFly = true;
                    } else {
                        FlyMotor2.setPower(0);
                        FlyMotor.setPower(0);
                        isFly = false;
                    }
                    previousPressFly = true;
                }
            } else {
                previousPressFly = false;
            }
            //RingFlicks
            if (gamepad1.right_bumper) {
                if (previousPressPush != gamepad1.right_bumper) {
                    if (!isPush) {
                        RingBlocker.setPosition(.5);
                        isPush = true;
                    } else {
                        RingBlocker.setPosition(.9);
                        isPush = false;
                    }
                    previousPressPush = true;
                }
            } else {
                previousPressPush = false;
            }


            AddToTelemetry("isIntaking", String.valueOf(isIntaking));
            AddToTelemetry("stone", String.valueOf(stoneLevel));


            double FLMpower = (forward + strafe - turn);// * NormPower;
            double FRMpower = (forward - strafe + turn);// * NormPower;
            double BLMpower = (forward - strafe - turn);// * NormPower;
            double BRMpower = (forward + strafe + turn);// * NormPower;
            AddToTelemetry("FLM", String.valueOf(FLMpower));
            AddToTelemetry("FRM", String.valueOf(FRMpower));
            AddToTelemetry("BLM", String.valueOf(BLMpower));
            AddToTelemetry("BRM", String.valueOf(BRMpower));


            FLM.setPower(FLMpower); //set the power to the motor
            FRM.setPower(FRMpower);
            BLM.setPower(BLMpower);
            BRM.setPower(BRMpower);
            UpdateTelemetry();

        }
    }

    enum Randomization {
        A,
        B,
        C;
    }

    Randomization randomization;

    public void CheckRandomization(double rings) {
        double stack = rings;
        if (stack == 0) {
            randomization = Randomization.A;
        }
        if (stack == 1) {
            randomization = Randomization.B;
        }
        if (stack == 4) {
            randomization = Randomization.C;
        } else {
            randomization = Randomization.A;
        }
        AddToTelemetry("Randomization", String.valueOf(randomization));
        UpdateTelemetry();
    }

    public void GoToZone() {
        if (randomization == Randomization.A) {
            Reverse(3, 0);
            EncoderPID(-84, 0);
        }
        if (randomization == Randomization.B) {
            Reverse(3, 0);
            EncoderPID(-84, 12);
        }
        if (randomization == Randomization.C) {
            Reverse(3, 0);
            EncoderPID(-84, 24);
        }
    }

    public void ParkOnTape() {
 Reverse(84,0);    }

    private void Rotate(double degrees) {
        double desiredTickDifference = degrees * TickToDegreesRatio();
        double TickDifference = 0;
        double leftTicksStart = FLM.getCurrentPosition();
        double rightTicksStart = FRM.getCurrentPosition();
        double leftTicks = leftTicksStart;
        double rightTicks = rightTicksStart;
        double error = degrees;
        while (CanMove() && Math.abs(error) > degrees / (degrees * 10) && degrees != 0) {
            TickDifference = leftTicks - rightTicks;
            // error =
        }

    }

    private double TickToDegreesRatio() {

        return 100;
    }

    public void Reverse(double distance, double correction) {
        double motorPos = FLM.getCurrentPosition();
        double desiredTicks = InchesToTicks(distance + correction);
        double currentPosition = motorPos;
        double previousPosition = motorPos;
        while (CanMove() && (currentPosition < (desiredTicks + previousPosition))) {
            motorPos = Math.abs(FLM.getCurrentPosition());
            currentPosition = motorPos;
            DriveTicks(-.4);
            AddToTelemetry("encoder:", String.valueOf(currentPosition));
            AddToTelemetry("Desired Position:", String.valueOf(desiredTicks + previousPosition));
            UpdateTelemetry();
        }

        DriveTicks(0);

    }
}
