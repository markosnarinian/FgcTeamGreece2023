/*
 * ----IMPORTANT----
 * DO NOT DISTRIBUTE THIS SOFTWARE IN ANY WAY, PRIVATELY OR PUBLICLY,
 * WITHOUT EXPLICIT PERMISSION FROM THE DEVELOPER, MARKOS NARINIAN (manarinian@gmail.com)
 * ----IMPORTANT----
 *
 * This program ("AdvancedPovDriveNarinian") and "ToggleButtonMgrNarinian" were developed by
 * Markos Narinian (manarinian@gmail.com), one of the five members that represented the
 * Greek National Robotics Team (GNRT/FGC Team Greece) in the 2023 FIRST Global Challenge in Singapore.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.text.DecimalFormat;
import java.util.ArrayList;

@TeleOp(name = "TeamGreeceAdvancedPovDriveNarinian")
public class TeamGreeceAdvancedPovDriveNarinian extends LinearOpMode {
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {
        // ----CONFIGURATION: CRITICAL - BEGINNING----
        final double MANUAL_OVERRIDE_BUTTON_PRESS_DURATION = 8; // BUTTON PRESS DURATION TO ACTIVATE MANUAL OVERRIDE
        // ----CONFIGURATION: CRITICAL - END----

        // ----CONFIGURATION - BEGINNING----
        final int toggleMgrButtonBlockTime = 150; // Button block time passed to ToggleMgrNarinian constructor

        final double joystickDeadzone = 0.06; // Gamepad joystick deadzone
        final double triggerDeadzone = 0.2; // Gamepad trigger deadzone

        final double cfgDriveMotorPwrIncrementCoefficient = 1; // Rate of acceleration for the drivetrain motors

        // Drive motor power gains - multiplied with the joystick inputs to calculate the power to set
        final double cfgMotorPowerGainNormal = 0.6; // Motor power gain -> normal mode
        final double cfgMotorPowerGainHigh = 1; // Motor power gain -> turbo mode
        final double cfgMotorPowerGainPrecision = 0.2; // Motor power gain -> precision mode

        final double cfgSteeringGainNormal = 0.4; // Steering gain -> normal mode
        final double cfgSteeringGainHigh = 1; // Steering gain -> turbo mode
        final double cfgSteeringGainPrecision = 0.2; // Steering gain -> precision mode

        final double cfgIntakeMotorPower = 1; // Intake motor power when it is ON or in REVERSE

        final int[] cfgLeftLiftLevels = {0, 600, 1465}; // Left lift preset positions in encoder ticks
        final int[] cfgRightLiftLevels = {0, 600, 1465}; // Right lift preset positions in encoder ticks
        final int cfgLiftTargetTolerance = 10;
        final double cfgLiftTravelVelocityPercentage = 1; // Velocity for high speed lift extension/retraction
        final int cfgLiftRetractedPosTolerance = 60; // Tolerance in the criteria to regard a lift as fully retracted

        final int cfgHydrogenDoorTicksOpen = 5; // Hydrogen door spool optical encoder ticks when door is open
        // Minimum lift height to open hydrogen door and lower lift position limit when the hydrogen door is open
        final int cfgHydrogenDoorOpenLiftFloor = 500; // The lowest position that the lifts are allowed to reach while the hydrogen door is open/position above which the hydrogen door gets unlocked
        final double cfgHydrogenDoorServoPower = 1; // Power to apply to hydrogen door servo
        final double cfgOxygenDoorServoPower = 1; // Power to apply to oxygen door servo

        // Hook lift motor power gain - multiplied with the trigger input to calculate the power to set
        final double hookLiftMotorPowerGain = 1;
        final double cfgClimbMotorPower = 1; // Power to apply to the climb motor

        double cfgCogDependentReductionCoefficientDrivePwrGain = 0.0025; // CGDPAR - Power gain reduction coefficient
        double cfgCogDependentReductionCoefficientPwrIncrement = 0.005; // CGDPAR - Power increment reduction coefficient
        // ----CONFIGURATION - END----

        // ----CONSTANTS - BEGINNING----
        final int cstCorehexTpsNoLoad = 600; // Encoder ticks per second at max rpm (free speed) - core hex motor
        final int cstHdhexTpsNoLoad = 2800; // Encoder ticks per second at max rpm (free speed) - hd hex motor
        final int cstCorehexTicksPerRevolution = 288;
        final int cstHdhexTicksPerRevolution = 28;
        // ----CONSTANTS - END----

        // ----CONTROLS - BEGINNING----
        // The names of all variables that hold control inputs start with the "ctl" prefix
        boolean CTL_MANUAL_OVERRIDE;

        double ctlJsThrottleAxis; // Drivetrain throttle axis
        double ctlJsSteeringAxis; // Drivetrain steering axis
        boolean ctlTurboModeEnable; // Turbo mode: hold to enable
        boolean ctlPrecisionModeToggle; // Precision mode: hold to reduce power gain
        double driveMotorPwrIncrementCoefficient;

        boolean ctlExtendLift; // Lift extension button
        boolean ctlRetractLift; // Lift retraction button

        boolean ctlIntakeToggle; // Intake toggle button
        boolean ctlIntakeReverse; // Intake reverse button

        boolean ctlOpenOxygenDoor; // Oxygen door open button
        boolean ctlCloseOxygenDoor; // Oxygen door close button
        boolean ctlOpenHydrogenDoor; // Hydrogen door open button
        boolean ctlCloseHydrogenDoor; // Hydrogen door close button

        boolean ctlClimb; // Hook lift extension/retraction button
        boolean ctlDescend; // Hook lift emergency stop button

        double ctlHookLiftExtend; // Hook lift extension axis
        double ctlHookLiftRetract; // Hook lift retraction axis

        // ----CONTROLS - END----

        boolean SAFE_MODE_ACTIVATED = false; // SAFE MODE ACTIVATION STATUS
        boolean MONITOR_CTL_MANUAL_OVERRIDE = false; // START MONITORING MANUAL OVERRIDE BUTTON STATE
        double RUNTIME_AT_MANUAL_OVERRIDE_BTN_PRESS = 0;

        final ElapsedTime runtime = new ElapsedTime(); // Initialize ElapsedTime timer

        // Motor current consumption
        double currentTotal;

        DecimalFormat df = new DecimalFormat("#.#");

        double runtimeItrStart; // Runtime at while loop iteration start
        double itrEtNst; // Duration of current iteration (instantaneous)
        double itrEtSum = 0; // Sum of iteration durations
        int itrCount = 0; // Count of iterations

        boolean precisionModeEnabled = false; // Is precision mode enabled?
        boolean precisionModeDisableOverride; // Override precision mode control

        double intakeMotorPowerToSet = 0; // Intake motor power target
        String intakeMotorStateHrf = "OFF";

        double liftTravelVelocity = cfgLiftTravelVelocityPercentage * cstCorehexTpsNoLoad;
        int liftTargetLvlIndexNo = 0; // Index number of lift target level from the rightLiftLevels array (initial)

        int hydrogenDoorTicks = -1; // Hydrogen door ticks counted since execution of the program
        boolean cmdOpenHydrogenDoor = false; // Hydrogen door commanded (flagged) to open
        boolean cmdCloseHydrogenDoor = false; // Hydrogen door commanded (flagged) to close
        String hydrogenStateHrf = "STATIC";

        String hookLiftMotorStateHrf; // Hook lift motor state in human readable format (hrf)
        String climbMotorStateHrf; // Climb motor state in human readable format (hrf)

        boolean conditionLiftRetracted; // Whether or not the right lift is fully retracted

        boolean protectiveOverrideLiftFloorEnforced; // Whether the lift floor is enforced due to protect the hydrogen door from crashing on the intake
        boolean protectiveOverrideH2Blocked = false; // Whether the hydrogen door is blocked to prevent damaging it during lift extension

        double motorPowerGainCogDependent; // Throttle and joystick axis multiplier

        ToggleMgrNarinian intakeToggleMgr = new ToggleMgrNarinian(toggleMgrButtonBlockTime);
        ToggleMgrNarinian liftLvlSelectorMgr = new ToggleMgrNarinian(toggleMgrButtonBlockTime);
        ToggleMgrNarinian openHydrogenDoorMgr = new ToggleMgrNarinian(toggleMgrButtonBlockTime);
        ToggleMgrNarinian closeHydrogenDoorMgr = new ToggleMgrNarinian(toggleMgrButtonBlockTime);
        ToggleMgrNarinian hydrogenMagneticEncoderMgr = new ToggleMgrNarinian(toggleMgrButtonBlockTime);
        ToggleMgrNarinian precisionModeToggleMgr = new ToggleMgrNarinian(toggleMgrButtonBlockTime);

        // Initialization of variables that hold motor, servo and sensor objects
        DcMotorEx leftMotor = hardwareMap.get(DcMotorEx.class, "left"); // Left drive motor
        DcMotorEx rightMotor = hardwareMap.get(DcMotorEx.class, "right"); // Right drive motor
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "in"); // Intake motor
        DcMotorEx leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLift"); // Left lift motor
        DcMotorEx rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLift"); // Right lift motor
        DcMotorEx hookLiftMotor = hardwareMap.get(DcMotorEx.class, "hookLift"); // Hook lift motor
        DcMotorEx climbMotor = hardwareMap.get(DcMotorEx.class, "climbMotor"); // Climb motor
        CRServo oxygenServo = hardwareMap.get(CRServo.class, "oxygenServo"); // Servo actuator for the oxygen release door
        CRServo hydrogenServo = hardwareMap.get(CRServo.class, "hydrogenServo"); // Servo actuator for the hydrogen release door
        TouchSensor hydrogenMagneticSensor = hardwareMap.get(TouchSensor.class, "hydrogenMagnetic");

        // ----SET MOTOR DIRECTIONS, RUN-MODES - BEGINNING----
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hookLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        climbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hydrogenServo.setDirection(DcMotorSimple.Direction.FORWARD);
        oxygenServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // Uncomment to setPower to drivetrain motors
        leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Uncomment to setVelocity to drivetrain motors
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hookLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // ---- SET MOTOR DIRECTIONS, RUN-MODES - END----

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                runtimeItrStart = runtime.milliseconds(); // Runtime in milliseconds at while loop iteration start

                // ----CONTROL MAPPING - BEGINNING----
                // Center joystick deadzone to account for defective/drifting joystick potentiometers
                // was used on some controls

                CTL_MANUAL_OVERRIDE = gamepad1.touchpad || gamepad2.touchpad;

                // Joystick throttle axis, joystick steering axis
                ctlJsThrottleAxis = Math.abs(gamepad1.left_stick_y) < joystickDeadzone ? 0 : -gamepad1.left_stick_y;
                ctlJsSteeringAxis = Math.abs(gamepad1.right_stick_x) < joystickDeadzone ? 0 : gamepad1.right_stick_x;
                ctlTurboModeEnable = gamepad1.left_bumper; // Turbo mode button (enable while pressed)
                ctlPrecisionModeToggle = gamepad1.cross; // Precision mode toggle button

                ctlIntakeToggle = gamepad1.right_bumper; // Intake toggle button
                ctlIntakeReverse = gamepad1.square; // Intake reverse (enable while pressed)

                ctlClimb = gamepad1.triangle; // Climb motor: climb (while pressed)
                ctlDescend = gamepad1.circle; // Climb motor: descend (while pressed)

                ctlHookLiftExtend = gamepad1.right_trigger < triggerDeadzone ? 0 : gamepad1.right_trigger;
                ctlHookLiftRetract = gamepad1.left_trigger < triggerDeadzone ? 0 : gamepad1.left_trigger;

                ctlExtendLift = gamepad2.dpad_up; // Extend lif one level
                ctlRetractLift = gamepad2.dpad_down; // Retract lif tone level

                ctlOpenOxygenDoor = gamepad2.cross; // Open oxygen door (while pressed)
                ctlCloseOxygenDoor = gamepad2.circle; // Close oxygen door (while pressed)
                ctlOpenHydrogenDoor = gamepad2.square; // Run H2 door to position using DIY magnetic encoder
                ctlCloseHydrogenDoor = gamepad2.triangle; // Run H2 door to position using DIY magnetic encoder
                // ----CONTROL MAPPING - END----

                // Check if the right lift (and therefore the ball storage bucket is fully retracted)
                conditionLiftRetracted = liftTargetLvlIndexNo == 0 && rightLiftMotor.getCurrentPosition() <= cfgLiftRetractedPosTolerance;

                /* CGDPAR - Reduce drive motor power gain and acceleration proportionally
                 * to the height of the lift storage bucket (the center of gravity)
                 */
                if (conditionLiftRetracted) { // If lift retracted, assign default values to the variables (do nothing)
                    motorPowerGainCogDependent = 1;
                    driveMotorPwrIncrementCoefficient = cfgDriveMotorPwrIncrementCoefficient;
                    precisionModeDisableOverride = false;
                } else { // If lift not retracted, set motor power gain limit, reduce acceleration & disable turbo, precision modes
                    motorPowerGainCogDependent = 1 / (rightLiftMotor.getCurrentPosition()
                            * cfgCogDependentReductionCoefficientDrivePwrGain);
                    driveMotorPwrIncrementCoefficient = cfgDriveMotorPwrIncrementCoefficient / (rightLiftMotor.getCurrentPosition()
                            * cfgCogDependentReductionCoefficientPwrIncrement);
                    ctlTurboModeEnable = false;
                    precisionModeDisableOverride = true;
                }
                telemetry.addData("CGDPAR PWR GAIN COEFFICIENT : ", df.format(motorPowerGainCogDependent));
                telemetry.addData("CGDPAR PWR INCREMENT COEFFICIENT : ", df.format(driveMotorPwrIncrementCoefficient));

                // Toggle precision mode
                if (precisionModeToggleMgr.buttonPress(ctlPrecisionModeToggle, runtime.milliseconds())) {
                    precisionModeEnabled = !precisionModeEnabled;
                }

                // Normal/Turbo/Precision drive mode
                if (ctlTurboModeEnable) { // If turbo mode button is pressed
                    ctlJsThrottleAxis *= cfgMotorPowerGainHigh;
                    ctlJsSteeringAxis *= cfgSteeringGainHigh;
                } else if (precisionModeEnabled && !precisionModeDisableOverride) { // If precision mode is enabled
                    ctlJsThrottleAxis *= cfgMotorPowerGainPrecision;
                    ctlJsSteeringAxis *= cfgSteeringGainPrecision;
                } else { // If turbo and precision modes are disabled
                    ctlJsThrottleAxis *= cfgMotorPowerGainNormal * motorPowerGainCogDependent;
                    ctlJsSteeringAxis *= cfgSteeringGainNormal * motorPowerGainCogDependent;
                }
                telemetry.addData("TURBO MODE : ", ctlTurboModeEnable ? "ENABLED" : "DISABLED");
                telemetry.addData("PRECISION MODE : ", precisionModeEnabled ? "ENABLED" : "DISABLED");

                /*
                 * Calculating and setting motor power/velocity

                 * Motor power formula:
                 * Motor current power/velocity setting + (target power/velocity setting - motor current power/velocity setting)
                 *         * drivetrain motor power increment coefficient

                 *  When setting motor velocity the program converts the control input to a velocity for a specific motor
                 *  by multiplying it with a constant for that specific motor. In this case, "hdhexEncoderTicksPerSecAtMaxRpm".
                 */

                // Uncomment to set power - DO NOT FORGET TO LET THE MOTOR MODE DEFAULT TO RUN_WITHOUT_ENCODER (COMMENT THE LINES IN INITIALIZATION)
                leftMotor.setPower(leftMotor.getPower()
                        + ((Range.clip(ctlJsThrottleAxis + ctlJsSteeringAxis, -1.0, 1.0) - leftMotor.getPower())
                        * driveMotorPwrIncrementCoefficient));
                rightMotor.setPower(rightMotor.getPower()
                        + ((Range.clip(ctlJsThrottleAxis - ctlJsSteeringAxis, -1.0, 1.0) - rightMotor.getPower())
                        * driveMotorPwrIncrementCoefficient));

                // Uncomment to set velocity - DO NOT FORGET TO SET MOTOR MODE TO RUN_WITH_ENCODER (UNCOMMENT THE LINES IN INITIALIZATION)
                //leftMotor.setVelocity(leftMotor.getVelocity() + (Range.clip(ctlJsThrottleAxis + ctlJsSteeringAxis, -1.0, 1.0)
                //        * cstHdhexTpsNoLoad - leftMotor.getVelocity()) * driveMotorPwrIncrementCoefficient);
                //rightMotor.setVelocity(rightMotor.getVelocity() + (Range.clip(ctlJsThrottleAxis - ctlJsSteeringAxis, -1.0, 1.0)
                //        * cstHdhexTpsNoLoad - rightMotor.getVelocity()) * driveMotorPwrIncrementCoefficient);

                telemetry.addLine("DRV IN : ") // Control input
                        .addData("L", df.format(Range.clip(ctlJsThrottleAxis + ctlJsSteeringAxis, -1.0, 1.0)))
                        .addData("R", df.format(Range.clip(ctlJsThrottleAxis - ctlJsSteeringAxis, -1.0, 1.0)));
                telemetry.addLine("DRV VEL : ") // Drive motor velocity
                        .addData("L VEL", leftMotor.getVelocity())
                        .addData("R VEL", rightMotor.getVelocity());
                telemetry.addLine("DRV PWR : ") // Drive motor power
                        .addData("L PWR", leftMotor.getPower())
                        .addData("R PWR", rightMotor.getPower());

                // Intake control
                // One button toggles the intake motor, the other button runs intake motor in reverse while held down
                if (intakeToggleMgr.buttonPress(ctlIntakeToggle, runtime.milliseconds())) {
                    // Invert motor state
                    if (intakeMotorPowerToSet == 0) {
                        intakeMotorPowerToSet = cfgIntakeMotorPower;
                        intakeMotorStateHrf = "ON";
                    } else {
                        intakeMotorPowerToSet = 0;
                        intakeMotorStateHrf = "OFF";
                    }
                }
                // If reverse button is pressed, reverse the intake
                intakeMotor.setPower(conditionLiftRetracted ? (ctlIntakeReverse ? -cfgIntakeMotorPower : intakeMotorPowerToSet) : 0);

                telemetry.addLine("INTAKE : ") // Intake motor state, velocity and power
                        .addData("STATE", ctlIntakeReverse ? "REVERSE" : intakeMotorStateHrf)
                        .addData("VEL", intakeMotor.getVelocity())
                        .addData("PWR", intakeMotor.getPower());

                // Left and right lift control

                // If lift up one level button is pressed
                if (liftLvlSelectorMgr.buttonPress(ctlExtendLift, runtime.milliseconds()) &&
                        liftTargetLvlIndexNo < cfgRightLiftLevels.length - 1) {
                    liftTargetLvlIndexNo += 1; // Increase lift target position by one level
                // If lift down one level button is pressed
                } else if (liftLvlSelectorMgr.buttonPress(ctlRetractLift, runtime.milliseconds())
                        && liftTargetLvlIndexNo > 0) {
                    liftTargetLvlIndexNo -= 1; // Reduce lift target position by one level
                }
                // If the ball storage bucket's target position is below the hydrogenDoorOpenLiftFloor,
                // forcefully set the lift target position to that of the lift floor.
                // Else, set the driver selected target position.


                if (cfgLeftLiftLevels[liftTargetLvlIndexNo] < cfgHydrogenDoorOpenLiftFloor
                        && (hydrogenDoorTicks > 0 || cmdOpenHydrogenDoor)) {
                    leftLiftMotor.setTargetPosition(cfgHydrogenDoorOpenLiftFloor);
                    leftLiftMotor.setTargetPositionTolerance(cfgLiftTargetTolerance);

                    rightLiftMotor.setTargetPosition(cfgHydrogenDoorOpenLiftFloor);
                    rightLiftMotor.setTargetPositionTolerance(cfgLiftTargetTolerance);

                    protectiveOverrideLiftFloorEnforced = true; // Passed to telemetry.addData()
                } else {
                    leftLiftMotor.setTargetPosition(cfgLeftLiftLevels[liftTargetLvlIndexNo]);
                    leftLiftMotor.setTargetPositionTolerance(cfgLiftTargetTolerance);

                    rightLiftMotor.setTargetPosition(cfgRightLiftLevels[liftTargetLvlIndexNo]);
                    rightLiftMotor.setTargetPositionTolerance(cfgLiftTargetTolerance);

                    protectiveOverrideLiftFloorEnforced = false; // Passed to telemetry.addData()
                }

                // Run motors to the target position set above, at the velocity specified in the configuration
                leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setVelocity(liftTravelVelocity);
                rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightLiftMotor.setVelocity(liftTravelVelocity);

                telemetry.addLine("LIFTS : ")
                        .addData("T LVL", liftTargetLvlIndexNo); // Lift target level
                telemetry.addLine("LIFTS : ") // Lift target positions
                        .addData("L tPOS", leftLiftMotor.getTargetPosition())
                        .addData("R tPOS", rightLiftMotor.getTargetPosition());
                telemetry.addLine("LIFTS : ") // Lift current positions
                        .addData("L POS", leftLiftMotor.getCurrentPosition())
                        .addData("R POS", rightLiftMotor.getCurrentPosition());
                telemetry.addLine("LIFTS : ") // Lifts busy reaching target position
                        .addData("L BUSY", leftLiftMotor.isBusy())
                        .addData("R BUSY", rightLiftMotor.isBusy());
                telemetry.addLine("LIFTS : ") // Lift velocities
                        .addData("L VEL", leftLiftMotor.getVelocity())
                        .addData("R VEL", rightLiftMotor.getVelocity());
                telemetry.addLine("LIFTS : ") // Lift power
                        .addData("L PWR", leftLiftMotor.getPower())
                        .addData("R PWR", rightLiftMotor.getPower());
                telemetry.addData("LIFT FLOOR ENFORCED (PROTECTIVE OVERRIDE)", protectiveOverrideLiftFloorEnforced);

                // Oxygen door control
                /*
                 * While open button is pressed: open door
                 * While close button is pressed: close door
                 * Else, set motor power to 0
                 */
                if (ctlOpenOxygenDoor) {
                    oxygenServo.setPower(-cfgOxygenDoorServoPower);
                telemetry.addData("O2 DR ST", "OPENING");
                } else if (ctlCloseOxygenDoor) {
                    oxygenServo.setPower(cfgOxygenDoorServoPower);
                    telemetry.addData("O2 DR ST", "CLOSING");
                } else {
                    oxygenServo.setPower(0);
                    telemetry.addData("O2 DR ST", "STATIC");
                }

                // Hydrogen door control (with magnetic encoder)
                /*
                 * If door open button is pressed:
                 *     If the door is already commanded to open:
                 *         Command the door to stop opening
                 *     Else (if door is not already commanded to open
                 *     and is therefore either static or closing):
                 *         Command the door to start opening
                 *         Command the door to stop closing
                 * Else if door close button is pressed:
                 *     If the door is already commanded to close:
                 *         Command the door to stop closing
                 *     Else (if door is not already commanded to open
                 *     and is therefore either static or opening):
                 *         Command the door to stop opening
                 *         Command the door to start closing
                 */
                if (openHydrogenDoorMgr.buttonPress(ctlOpenHydrogenDoor,
                        runtime.milliseconds())) {
                    if (cmdOpenHydrogenDoor) {
                        cmdOpenHydrogenDoor = false;
                        hydrogenStateHrf = "STATIC";
                    } else {
                        cmdOpenHydrogenDoor = true;
                        cmdCloseHydrogenDoor = false;
                        hydrogenStateHrf = "OPENING";
                    }
                } else if (closeHydrogenDoorMgr.buttonPress(ctlCloseHydrogenDoor, runtime.milliseconds())) {
                    if (cmdCloseHydrogenDoor) {
                        cmdCloseHydrogenDoor = false;
                        hydrogenStateHrf = "STATIC";
                    } else {
                        cmdOpenHydrogenDoor = false;
                        cmdCloseHydrogenDoor = true;
                        hydrogenStateHrf = "CLOSING";
                    }
                }

                if (hydrogenMagneticEncoderMgr.buttonPress(hydrogenMagneticSensor.isPressed(), runtime.milliseconds())) {
                    hydrogenDoorTicks += (cmdCloseHydrogenDoor ? -1 : 1);
                }

                if (cmdOpenHydrogenDoor && hydrogenDoorTicks < cfgHydrogenDoorTicksOpen) {
                    if (rightLiftMotor.getCurrentPosition() >= cfgHydrogenDoorOpenLiftFloor) {
                        hydrogenServo.setPower(-cfgHydrogenDoorServoPower);
                        protectiveOverrideH2Blocked = false;
                    } else {
                        protectiveOverrideH2Blocked = true;
                    }
                } else if (cmdCloseHydrogenDoor && hydrogenDoorTicks > 0) {
                    hydrogenServo.setPower(cfgHydrogenDoorServoPower);
                } else {
                    hydrogenServo.setPower(0);
                }
                telemetry.addLine("H2 DOOR : ")
                        .addData("STATE", hydrogenStateHrf)
                        .addData("TICKS", hydrogenDoorTicks)
                        .addData("H2 BLOCKED - PROTECTIVE OVERRIDE", protectiveOverrideH2Blocked);

                // Hook lift control
                /*
                 * While extension button is pressed: extend lift
                 * While retraction button is pressed: retract lift
                 * Else, set motor power to 0
                 */
                if (ctlHookLiftExtend > 0) {
                    hookLiftMotorStateHrf = "EXTENDING";
                    hookLiftMotor.setPower(ctlHookLiftExtend * hookLiftMotorPowerGain);
                } else if (ctlHookLiftRetract > 0) {
                    hookLiftMotor.setPower(-ctlHookLiftRetract * hookLiftMotorPowerGain);
                    hookLiftMotorStateHrf = "RETRACTING";
                } else {
                    hookLiftMotor.setPower(0);
                    hookLiftMotorStateHrf = "STATIC";
                }
                telemetry.addLine("HOOK LIFT : ")
                        .addData("STATE", hookLiftMotorStateHrf)
                        .addData("PWR", hookLiftMotor.getPower());

                // Climb motor control
                /*
                 * While climb button is pressed: climb
                 * While descend button is pressed: descend
                 * Else, set motor power to 0
                 */
                if (ctlClimb) {
                    climbMotorStateHrf = "CLIMBING";
                    climbMotor.setPower(cfgClimbMotorPower);
                } else if (ctlDescend) {
                    climbMotor.setPower(-cfgClimbMotorPower);
                    climbMotorStateHrf = "DESCENDING";
                } else {
                    climbMotor.setPower(0);
                    climbMotorStateHrf = "STATIC";
                }
                telemetry.addLine("CLIMB : ")
                        .addData("STATE", climbMotorStateHrf)
                        .addData("PWR", climbMotor.getPower());

                // Get and display the current consumption of each DC motor in Amperes
                ArrayList<Double> currentMeasurements = new ArrayList<>();

                currentMeasurements.add(leftMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(rightMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(intakeMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(leftLiftMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(rightLiftMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(leftLiftMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(leftLiftMotor.getCurrent(CurrentUnit.AMPS));

                currentTotal = 0;
                for (double i : currentMeasurements) {
                    currentTotal += i;
                }

                telemetry.addLine("I DRV : ")
                        .addData("L", df.format(currentMeasurements.get(0)))
                        .addData("R", df.format(currentMeasurements.get(1)));
                telemetry.addData("I INTAKE", df.format(currentMeasurements.get(2)));
                telemetry.addLine("I LIFT : ")
                        .addData("L", df.format(currentMeasurements.get(3)))
                        .addData("R", df.format(currentMeasurements.get(4)))
                        .addData("TOTAL", df.format(currentMeasurements.get(3)
                                + currentMeasurements.get(4)));
                telemetry.addData("I TOTAL", df.format(currentTotal));

                // Calculate and display instantaneous and average while loop
                // iteration execution time on the driver hub via telemetry

                // 1) Increase while loop iteration count by one
                // 2) Calculate instantaneous execution time of the while loop's code
                // 3) Add the current instantaneous execution time to the total
                //    runtime since the OpMode was started
                itrCount += 1; //
                itrEtNst = runtime.milliseconds() - runtimeItrStart;
                itrEtSum += itrEtNst; //

                // 1) Display instantaneous while loop iteration duration on the driver hub
                // 2) Display average while loop iteration duration on the driver hub
                telemetry.addLine("IET : ")
                        .addData("NST", df.format(itrEtNst))
                        .addData("AVG", df.format((itrEtSum) / itrCount));

                // MANUAL OVERRIDE / SAFE MODE ACTIVATION
                // If manual override button is held down for a set period of time
                // (MANUAL_OVERRIDE_BUTTON_PRESS_DURATION), break from current
                // while loop and transition into SAFE MODE
                if (MONITOR_CTL_MANUAL_OVERRIDE) {
                    if (CTL_MANUAL_OVERRIDE && runtime.milliseconds()
                            >= RUNTIME_AT_MANUAL_OVERRIDE_BTN_PRESS + MANUAL_OVERRIDE_BUTTON_PRESS_DURATION) {
                        telemetry.clearAll();
                        SAFE_MODE_ACTIVATED = true;
                        break;
                    } else if (!CTL_MANUAL_OVERRIDE) {
                        MONITOR_CTL_MANUAL_OVERRIDE = false;
                    }
                } else if (CTL_MANUAL_OVERRIDE) {
                    MONITOR_CTL_MANUAL_OVERRIDE = true;
                    RUNTIME_AT_MANUAL_OVERRIDE_BTN_PRESS = runtime.milliseconds();
                }

                telemetry.update();
            }

            // ----STABLE SAFE MODE CODE - RUNS AFTER MANUAL OVERRIDE----

            double ctlExtendLiftAxis_SFMD; // Lift extension analog control axis (only used in safe mode)
            double ctlRetractLiftAxis_SFMD; // Lift retraction analog control axis (only used in safe mode)

            itrEtSum = 0; // Sum of iteration durations
            itrCount = 0; // Count of iterations

            precisionModeEnabled = false; // Is precision mode enabled?

            // Set all motors to run in the RUN_WITHOUT_ENCODER mode
            leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hookLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (opModeIsActive() && SAFE_MODE_ACTIVATED) {
                /*
                 * Here goes the safe mode code
                 */

                telemetry.addData("----SAFE MODE----", "");

                runtimeItrStart = runtime.milliseconds(); // Runtime in milliseconds at while loop iteration start

                // ----CONTROL MAPPING - BEGINNING----
                // Center joystick deadzone to account for defective/drifting joystick potentiometers
                // was used on some controls

                ctlJsThrottleAxis = Math.abs(gamepad1.left_stick_y) < joystickDeadzone ? 0 : -gamepad1.left_stick_y;
                ctlJsSteeringAxis = Math.abs(gamepad1.right_stick_x) < joystickDeadzone ? 0 : gamepad1.right_stick_x;
                ctlTurboModeEnable = gamepad1.left_bumper;
                ctlPrecisionModeToggle = gamepad1.cross;

                ctlIntakeToggle = gamepad1.right_bumper; // Intake toggle button
                ctlIntakeReverse = gamepad1.square; // Intake reverse (while pressed)

                ctlClimb = gamepad1.triangle; // Climb motor: climb (while pressed)
                ctlDescend = gamepad1.circle; // Climb motor: descend (while pressed)

                // BBall storage compartment lift controls
                ctlExtendLiftAxis_SFMD = gamepad2.right_trigger < triggerDeadzone ? 0 : gamepad1.right_trigger;
                ctlRetractLiftAxis_SFMD = gamepad2.left_trigger < triggerDeadzone ? 0 : gamepad1.left_trigger;

                // Hook lift controls
                ctlHookLiftExtend = gamepad1.right_trigger < triggerDeadzone ? 0 : gamepad1.right_trigger;
                ctlHookLiftRetract = gamepad1.left_trigger < triggerDeadzone ? 0 : gamepad1.left_trigger;

                // Oxygen and hydrogen door controls (H2, O2 doors)
                ctlOpenOxygenDoor = gamepad2.cross;
                ctlCloseOxygenDoor = gamepad2.circle;
                ctlOpenHydrogenDoor = gamepad2.square;
                ctlCloseHydrogenDoor = gamepad2.triangle;
                // ----CONTROL MAPPING - END----

                // Toggle precision mode
                if (precisionModeToggleMgr.buttonPress(ctlPrecisionModeToggle, runtime.milliseconds())) {
                    precisionModeEnabled = !precisionModeEnabled;
                }

                // Normal/Turbo/Precision drive mode
                if (ctlTurboModeEnable) { // If turbo mode button is pressed
                    ctlJsThrottleAxis *= cfgMotorPowerGainHigh;
                    ctlJsSteeringAxis *= cfgSteeringGainHigh;
                } else if (precisionModeEnabled) { // If precision mode is enabled
                    ctlJsThrottleAxis *= cfgMotorPowerGainPrecision;
                    ctlJsSteeringAxis *= cfgSteeringGainPrecision;
                } else { // If turbo and precision modes are disabled
                    ctlJsThrottleAxis *= cfgMotorPowerGainNormal;
                    ctlJsSteeringAxis *= cfgSteeringGainNormal;
                }
                telemetry.addData("TURBO MODE : ", ctlTurboModeEnable ? "ENABLED" : "DISABLED");
                telemetry.addData("PRECISION MODE : ", precisionModeEnabled ? "ENABLED" : "DISABLED");

                /*
                 * Calculating and setting motor power/velocity

                 * Motor power formula:
                 * Motor current power/velocity setting + (target power/velocity setting - motor current power/velocity setting)
                 *         * drivetrain motor power increment coefficient

                 *  When setting motor velocity the program converts the control input to a velocity for a specific motor
                 *  by multiplying it with a constant for that specific motor. In this case, "hdhexEncoderTicksPerSecAtMaxRpm".
                 */

                leftMotor.setPower(leftMotor.getPower() + ((Range.clip(ctlJsThrottleAxis + ctlJsSteeringAxis, -1.0, 1.0) - leftMotor.getPower())
                        * cfgDriveMotorPwrIncrementCoefficient));
                rightMotor.setPower(rightMotor.getPower() + ((Range.clip(ctlJsThrottleAxis - ctlJsSteeringAxis, -1.0, 1.0) - rightMotor.getPower())
                        * cfgDriveMotorPwrIncrementCoefficient));

                telemetry.addLine("DRV PWR : ")
                        .addData("L PWR", leftMotor.getPower())
                        .addData("R PWR", rightMotor.getPower());

                // Intake control
                // One button toggles the intake motor, the other button runs intake motor in reverse while held down
                if (intakeToggleMgr.buttonPress(ctlIntakeToggle, runtime.milliseconds())) {
                    // Invert motor state
                    if (intakeMotorPowerToSet == 0) {
                        intakeMotorPowerToSet = cfgIntakeMotorPower;
                        intakeMotorStateHrf = "ON";
                    } else {
                        intakeMotorPowerToSet = 0;
                        intakeMotorStateHrf = "OFF";
                    }
                }
                // If reverse button is pressed, reverse the intake
                intakeMotor.setPower(ctlIntakeReverse ? -cfgIntakeMotorPower : intakeMotorPowerToSet);

                telemetry.addLine("INTAKE : ") // Intake motor state, velocity and power
                        .addData("STATE", ctlIntakeReverse ? "REVERSE" : intakeMotorStateHrf)
                        .addData("PWR", intakeMotor.getPower());

                // Left and right lift control
                /*
                 * While left trigger is depressed: extend lift
                 * While right trigger is depressed: retract lift
                 * Else, set lift motor power to 0
                 */
                if (ctlExtendLiftAxis_SFMD > 0) {
                    leftLiftMotor.setPower(ctlExtendLiftAxis_SFMD);
                    rightLiftMotor.setPower(ctlExtendLiftAxis_SFMD);
                } else if (ctlRetractLiftAxis_SFMD > 0) {
                    leftLiftMotor.setPower(-ctlRetractLiftAxis_SFMD);
                    rightLiftMotor.setPower(-ctlRetractLiftAxis_SFMD);
                }
                telemetry.addLine("LIFTS : ") // Left and right lift power
                        .addData("L PWR", leftLiftMotor.getPower())
                        .addData("R PWR", rightLiftMotor.getPower());

                // Oxygen door control
                /*
                 * While open button is pressed: open door
                 * While close button is pressed: close door
                 * Else, set motor power to 0
                 */
                if (ctlOpenOxygenDoor) {
                    oxygenServo.setPower(-cfgOxygenDoorServoPower);
                    telemetry.addData("O2 DR ST", "OPENING");
                } else if (ctlCloseOxygenDoor) {
                    oxygenServo.setPower(cfgOxygenDoorServoPower);
                    telemetry.addData("O2 DR ST", "CLOSING");
                } else {
                    oxygenServo.setPower(0);
                    telemetry.addData("O2 DR ST", "STATIC");
                }

                // Hydrogen door control
                /*
                 * While open button is pressed: open door
                 * While close button is pressed: close door
                 * Else, set motor power to 0
                 */
                if (ctlOpenHydrogenDoor) {
                    hydrogenServo.setPower(-cfgHydrogenDoorServoPower);
                    telemetry.addData("H2 DR ST", "OPENING");
                } else if (ctlCloseHydrogenDoor) {
                    hydrogenServo.setPower(cfgHydrogenDoorServoPower);
                    telemetry.addData("H2 DR ST", "CLOSING");
                } else {
                    hydrogenServo.setPower(0);
                    telemetry.addData("H2 DR ST", "STATIC");
                }

                // Hook lift control
                /*
                 * While extension button is pressed: extend lift
                 * While retraction button is pressed: retract lift
                 * Else, set motor power to 0
                 */
                if (ctlHookLiftExtend > 0) {
                    hookLiftMotorStateHrf = "EXTENDING";
                    hookLiftMotor.setPower(ctlHookLiftExtend);
                } else if (ctlHookLiftRetract > 0) {
                    hookLiftMotor.setPower(-ctlHookLiftRetract);
                    hookLiftMotorStateHrf = "RETRACTING";
                } else {
                    hookLiftMotor.setVelocity(0);
                    hookLiftMotorStateHrf = "STATIC";
                }
                telemetry.addLine("HOOK LIFT : ")
                        .addData("STATE", hookLiftMotorStateHrf)
                        .addData("PWR", hookLiftMotor.getPower());

                // Climb motor control
                /*
                 * While climb button is pressed: climb
                 * While descend button is pressed: descend
                 * Else, set motor power to 0
                 */
                if (ctlClimb) {
                    climbMotorStateHrf = "CLIMBING";
                    climbMotor.setPower(cfgClimbMotorPower);
                } else if (ctlDescend) {
                    climbMotor.setPower(-cfgClimbMotorPower);
                    climbMotorStateHrf = "DESCENDING";
                } else {
                    climbMotor.setPower(0);
                    climbMotorStateHrf = "STATIC";
                }
                telemetry.addLine("CLIMB : ")
                        .addData("STATE", climbMotorStateHrf)
                        .addData("PWR", climbMotor.getPower());

                // Get and display the current consumption of each DC motor in Amperes
                ArrayList<Double> currentMeasurements = new ArrayList<>();

                currentMeasurements.add(leftMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(rightMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(intakeMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(leftLiftMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(rightLiftMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(leftLiftMotor.getCurrent(CurrentUnit.AMPS));
                currentMeasurements.add(leftLiftMotor.getCurrent(CurrentUnit.AMPS));

                currentTotal = 0;
                for (double i : currentMeasurements) {
                    currentTotal += i;
                }

                telemetry.addLine("I DRV : ")
                        .addData("L", df.format(currentMeasurements.get(0)))
                        .addData("R", df.format(currentMeasurements.get(1)));
                telemetry.addData("I INTAKE", df.format(currentMeasurements.get(2)));
                telemetry.addLine("I LIFT : ")
                        .addData("L", df.format(currentMeasurements.get(3)))
                        .addData("R", df.format(currentMeasurements.get(4)))
                        .addData("TOTAL", df.format(currentMeasurements.get(3)
                                + currentMeasurements.get(4)));
                telemetry.addData("I TOTAL", df.format(currentTotal));

                // Calculate and display instantaneous and average while loop
                // iteration execution time on the driver hub via telemetry

                // 1) Increase while loop iteration count by one
                // 2) Calculate instantaneous execution time of the while loop's code
                // 3) Add the current instantaneous execution time to the total
                //    runtime since the OpMode was started
                itrCount += 1; //
                itrEtNst = runtime.milliseconds() - runtimeItrStart;
                itrEtSum += itrEtNst; //

                // 1) Display instantaneous while loop iteration duration on the driver hub
                // 2) Display average while loop iteration duration on the driver hub
                telemetry.addLine("IET : ")
                        .addData("NST", df.format(itrEtNst))
                        .addData("AVG", df.format((itrEtSum) / itrCount));

                telemetry.update();
            }
        }
    }
}
