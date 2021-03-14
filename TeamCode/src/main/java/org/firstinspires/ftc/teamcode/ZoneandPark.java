//Randomization A 
//Move Wobble
//Park on Launch Line

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;

//This auto was made by SiB on 12/2/2020

@Autonomous(name = "ZoneandPark")

public class ZoneandPark extends LinearOpMode {

    private UltimateGoalAuto UGA= new UltimateGoalAuto(telemetry);
    private Function Function = new Function();


    public class Function extends Functions {
        @Override
        public boolean CanMove() {
            return opModeIsActive() && !isStopRequested();
        }
        @Override
        public void AddToTelemetry(String Tag, String message) {
            telemetry.addData(Tag, message);
        }

        @Override
        public void UpdateTelemetry() {
            telemetry.update();
        }


}
    public void runOpMode() //when you press init
    {
        Function.Init(hardwareMap);
        RingDetectorV3 RingDetect = new RingDetectorV3("red",hardwareMap,telemetry,1,1,1);
        RingDetect.init();
        double rings;


        // Tell the driver tha,t initialization is complete.
        telemetry.addData("Status", "Initialized");

        telemetry.update();
        waitForStart(); //waits for t6he start button


        //Move to Zone
        rings = RingDetect.getRingPosition();
        Function.CheckRandomization(rings);
        Function.ParkOnTape();

        sleep(5000);
        //Function.ParkOnTape();
        //sleep(5000);
        //Function.CheckEncoders();
        stop();


    }
}
