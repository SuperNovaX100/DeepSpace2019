/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.autonomous.*;
import frc.inputs.GameController;
import frc.loops.Looper;
import frc.subsystem.*;
import frc.subsystem.test.CargoTest;
import frc.subsystem.test.DriveTest;
import frc.subsystem.test.GamepadTest;
import frc.subsystem.test.SubsystemTest;
import frc.utils.DriveSignal;

import java.util.Arrays;
import java.util.LinkedHashMap;

import static frc.utils.ShuffleboardConstants.ROBOT_MAIN_SHUFFLEBOARD;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final String kDefaultAuto = "Test";
    private static final String LvlTwoRightCloseRKT = "Lvl 2 Right to Close Rocket";
    private static final String LvlTwoRightCloseRKT2 = "Lvl 2 Right to Close Rocket Hatch 2";
    private static final String LvlTwoRightCloseRKT3 = "Lvl 2 Right to Close Rocket Hatch 3";
    private static final String Lvl2RightFarRKT = "Lvl 2 Right to Far Rocket";
    private static final String Lvl2RightFarRKT2 = "Lvl 2 Right to Far Rocket Hatch 2";
    private static final String Lvl2RightFarRKT3 = "Lvl 2 Right to Far Rocket Hatch 3";
    private static final String Lvl2LeftFarRKT = "Lvl 2 Left to Far Rocket";
    private static final String Lvl2LeftFarRKT2 = "Lvl 2 Left to Far Rocket Hatch 2";
    private static final String Lvl2LeftFarRKT3 = "Lvl 2 Left to Far Rocket Hatch 3";
    private static final String Lvl2LeftCloseRKT = "Lvl 2 Left to Close Rocket";
    private static final String Lvl2LeftCloseRKT2 = "Lvl 2 Left to Close Rocket Hatch 2";
    private static final String Lvl2LeftCloseRKT3 = "Lvl 2 Left to Close Rocket Hatch 3";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final SendableChooser<String> testChooser = new SendableChooser<>();
    private final SubsystemManager subsystemManager = new SubsystemManager(Arrays.asList(
            DriveTrain.getInstance(),
            Cargo.getInstance(),
            Elevator.getInstance(),
            Hatch.getInstance(),
            Jacks.getInstance()
    ));
    private final LinkedHashMap<String, Test> tests = new LinkedHashMap<>();
    private final Looper enabledLooper = new Looper();
    private final Looper disabledLooper = new Looper();
    private final GameController gameController = GameController.getInstance();
    private final DriveTrain drive = DriveTrain.getInstance();
    private final Cargo cargo = Cargo.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final Hatch hatch = Hatch.getInstance();
    private final Jacks jacks = Jacks.getInstance();
    private final Vision vision = Vision.getInstance();
    private String m_autoSelected;
    private SubsystemTest subsystemTest;
    //public File trajectoryFile = Pathfinder.readFromCSV(EncodeTest.pf1.csv);
    private AutoBase autonomous = null;

    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Autoline", kDefaultAuto);
        m_chooser.addOption(LvlTwoRightCloseRKT, LvlTwoRightCloseRKT);
        m_chooser.addOption("Lvl 2 Right to Close Rocket Hatch 2", LvlTwoRightCloseRKT2);
        m_chooser.addOption("Lvl 2 Right to Close Rocket Hatch 3", LvlTwoRightCloseRKT3);
        m_chooser.addOption("Lvl 2 Right to Far Rocket", Lvl2RightFarRKT);
        m_chooser.addOption("Lvl 2 Right to Far Rocket Hatch 2", Lvl2RightFarRKT2);
        m_chooser.addOption("Lvl 2 Right to Far Rocket Hatch 3", Lvl2RightFarRKT3);
        m_chooser.addOption(Lvl2LeftCloseRKT, Lvl2LeftCloseRKT);
        m_chooser.addOption("Lvl 2 Left to Close Rocket Hatch 2", Lvl2LeftCloseRKT2);
        m_chooser.addOption("Lvl 2 Left to Close Rocket Hatch 3", Lvl2LeftCloseRKT3);
        m_chooser.addOption("Lvl 2 Left to Far Rocket", Lvl2LeftFarRKT);
        m_chooser.addOption("Lvl 2 Left to Far Rocket Hatch 2", Lvl2LeftFarRKT2);
        m_chooser.addOption("Lvl 2 Left to Far Rocket Hatch 3", Lvl2LeftFarRKT3);
        Shuffleboard.getTab("Auton").add("Auto choices", m_chooser);
        tests.put(Test.DEFAULT_TEST.getOption(), Test.DEFAULT_TEST);
        testChooser.setDefaultOption(Test.DEFAULT_TEST.getOption(), Test.DEFAULT_TEST.getOption());

        for (Test test : Test.values()) {
            if (test != Test.DEFAULT_TEST) {
                tests.put(test.getOption(), test);
            }
        }

        for (String testKey : tests.keySet()) {
            testChooser.addOption(testKey, testKey);
        }

        subsystemManager.registerEnabledLoops(enabledLooper);
        subsystemManager.registerDisabledLoop(disabledLooper);
    }

    @Override
    public void robotPeriodic() {
//        drive.outputTelemetry();
    }

    @Override
    public void disabledInit() {
        enabledLooper.stop();
        disabledLooper.start();
        gameController.disabled();
    }

    @Override
    public void disabledPeriodic() {
        gameController.disabledPeriodic();
    }

    @Override
    public void autonomousInit() {
        teleopInit();
//        disabledLooper.stop();
//        enabledLooper.start();
//        m_autoSelected = m_chooser.getSelected();
//        System.out.println("Auto selected: " + m_autoSelected);
//        switch (m_autoSelected) {
//            case LvlTwoRightCloseRKT:
//                autonomous = new Lvl2RightCloseRKT();
//                break;
//            case LvlTwoRightCloseRKT2:
//                autonomous = new Lvl2RightCloseRKT2();
//                break;
//            case LvlTwoRightCloseRKT3:
//                autonomous = new Lvl2RightCloseRKT3();
//                break;
//            case Lvl2RightFarRKT:
//                autonomous = new Lvl2RightFarRKT();
//                break;
//            case Lvl2RightFarRKT2:
//                autonomous = new Lvl2RightFarRKT2();
//                break;
//            case Lvl2RightFarRKT3:
//                autonomous = new Lvl2RightFarRKT3();
//                break;
//            case Lvl2LeftCloseRKT:
//                autonomous = new Lvl2LeftCloseRKT();
//                break;
//            case Lvl2LeftCloseRKT2:
//                autonomous = new Lvl2LeftCloseRKT2();
//                break;
//            case Lvl2LeftCloseRKT3:
//                autonomous = new Lvl2LeftCloseRKT3();
//                break;
//            case Lvl2LeftFarRKT:
//                autonomous = new Lvl2LeftFarRKT();
//                break;
//            case Lvl2LeftFarRKT2:
//                autonomous = new Lvl2LeftFarRKT2();
//                break;
//            case Lvl2LeftFarRKT3:
//                autonomous = new Lvl2LeftFarRKT3();
//                break;
//            case kDefaultAuto:
//                autonomous = new Autoline();
//                break;
//            default:
//                autonomous = null;
//                break;
//        }
    }

    @Override
    public void autonomousPeriodic() {
//        if (autonomous != null) {
//            autonomous.periodic(Timer.getFPGATimestamp());
//        }
        teleopPeriodic();
    }

    @Override
    public void teleopInit() {
        disabledLooper.stop();
        enabledLooper.start();
    }

    @Override
    public void teleopPeriodic() {
        final double t0 = Timer.getFPGATimestamp();
        DriveSignal driveSignal = gameController.getDriveSignal();
        drive.setOpenLoop(driveSignal);
        final double tDrive = Timer.getFPGATimestamp();

        if (gameController.placeHatch()) {
            hatch.setHatchPlace();
        } else if (gameController.pullHatch()) {
            hatch.setHatchPull();
        } else {
            hatch.setOpenLoop(gameController.hatchManual());
        }
        final double tHatch = Timer.getFPGATimestamp();

        cargo.intakeTilt(gameController.intakeTilt());
        cargo.setDesiredState(gameController.getDesiredCargoIntakeState());
// TODO remove
//        if (gameController.cargoIntake()) {
//            cargo.setDesiredState(IntakeState.INTAKE);
//        } else if (gameController.cargoIntakeLeft()) {
//            cargo.setDesiredState(IntakeState.INTAKE_LEFT);
//        } else if (gameController.cargoIntakeRight()) {
//            cargo.setDesiredState(IntakeState.INTAKE_RIGHT);
//        } else if (gameController.cargoOuttakeLeft()) {
//            cargo.setDesiredState(IntakeState.OUTTAKE_LEFT);
//        } else if (gameController.cargoOuttakeRight()) {
//            cargo.setDesiredState(IntakeState.OUTTAKE_RIGHT);
//        } else if (gameController.cargoOuttakeFront()) {
//            cargo.setDesiredState(IntakeState.OUTTAKE_FRONT);
//        } else {
//            cargo.setDesiredState(IntakeState.STOPPED);
//        }
        final double tCargo = Timer.getFPGATimestamp();

        if (!gameController.manualJackWheelOverride()) {
            jacks.setWheels(gameController.runJackWheels());
        } else {
            jacks.setWheels(DriveSignal.NEUTRAL);
        }

        if (gameController.liftAllJacks()) {
            jacks.lift();
        } else if (gameController.retractAllJacks()) {
            jacks.retract();
        } else if (gameController.initializeHabClimbingLevel3()) {
            jacks.beginHabClimbLevel3();
        } else if(gameController.initializeHabClimbingLevel2()) {
            jacks.beginHabClimbLevel2();
        } else if (gameController.zeroJacks()) {
            jacks.beginZeroing();
        }
        final double tJacks = Timer.getFPGATimestamp();

        gameController.update();
        final double tController = Timer.getFPGATimestamp();

        if (gameController.setElevatorPositionLowHatch()) {
            elevator.pidToPosition(ElevatorPosition.HATCH_LOW);
        } else if (gameController.setElevatorPositionMidHatch()) {
            elevator.pidToPosition(ElevatorPosition.HATCH_MID);
        } else if (gameController.setElevatorPositionHighHatch()) {
            elevator.pidToPosition(ElevatorPosition.HATCH_HIGH);
        } else if (gameController.setElevatorPositionLowCargo()) {
            elevator.pidToPosition(ElevatorPosition.CARGO_LOW);
        } else if (gameController.setElevatorPositionMidCargo()) {
            elevator.pidToPosition(ElevatorPosition.CARGO_MID);
        } else if (gameController.setElevatorPositionHighCargo()) {
            elevator.pidToPosition(ElevatorPosition.CARGO_HIGH);
        } else {
            elevator.manualMovement(gameController.elevateManual());
        }
        final double tElevator = Timer.getFPGATimestamp();

//        elevator.outputTelemetry();
//        hatch.outputTelemetry();
//        jacks.outputTelemetry();
//        drive.outputTelemetry();
        final double tOutput = Timer.getFPGATimestamp();

        ROBOT_MAIN_SHUFFLEBOARD.putNumber("TeleopPeriodicTimes/Drive", tDrive - t0);
        ROBOT_MAIN_SHUFFLEBOARD.putNumber("TeleopPeriodicTimes/Hatch", tHatch - tDrive);
        ROBOT_MAIN_SHUFFLEBOARD.putNumber("TeleopPeriodicTimes/Cargo", tCargo - tHatch);
        ROBOT_MAIN_SHUFFLEBOARD.putNumber("TeleopPeriodicTimes/Jacks", tJacks - tCargo);
        ROBOT_MAIN_SHUFFLEBOARD.putNumber("TeleopPeriodicTimes/Controller", tController - tJacks);
        ROBOT_MAIN_SHUFFLEBOARD.putNumber("TeleopPeriodicTimes/Elevator", tElevator - tController);
        ROBOT_MAIN_SHUFFLEBOARD.putNumber("TeleopPeriodicTimes/Output", tOutput - tElevator);
        ROBOT_MAIN_SHUFFLEBOARD.putNumber("TeleopPeriodicTimes/TOTAL", tOutput - t0);
    }

    // TODO add more tests
    // TODO automate test validation
    @Override
    public void testInit() {
        teleopInit();
//        Test testSelected = tests.get(testChooser.getSelected());
//        disabledLooper.stop();
//        enabledLooper.stop();
//        switch (testSelected) {
//            case DEFAULT_TEST:
//                subsystemTest = null;
//                break;
//            case GAMEPAD_TEST:
//                subsystemTest = new GamepadTest();
//                break;
//            case DRIVE_TEST:
//                subsystemTest = new DriveTest();
//                break;
//            case CARGO_TEST:
//                subsystemTest = new CargoTest();
//                break;
//            case HATCH_MECHANISM_TEST:
//                subsystemTest = null;
//                break;
//            case ELEVATOR_TEST:
//                subsystemTest = null;
//                break;
//            case JACKS_TEST:
//                subsystemTest = null;
//                break;
//        }
    }

    @Override
    public void testPeriodic() {
        teleopPeriodic();
        subsystemManager.outputTelemetry();
//        if (subsystemTest != null) {
//            subsystemTest.periodic(Timer.getFPGATimestamp());
//        }
    }

    /**
     * Test cases that can be executed in the test mode to confirm functionality of various subsystems.
     */
    public enum Test {
        /**
         * Default, does not run any test.
         */
        DEFAULT_TEST("None"),
        /**
         * Chooses the {@link GamepadTest} as the test to run to confirm full functionality of the
         * {@link GameController}.
         */
        GAMEPAD_TEST("Gamepad Automated Test"),
        /**
         * Chooses the {@link DriveTest} as the test to run to confirm full functionality of the {@link Drive} subsystem
         */
        DRIVE_TEST("Drive Automated Test"),
        /**
         * Chooses the {@link CargoTest} as the test to run to confirm full functionality of the {@link Cargo} subsystem.
         */
        CARGO_TEST("Cargo Automated Test"),
        /**
         * TODO, will run an automated test on the {@link Hatch}.
         */
        HATCH_MECHANISM_TEST("Hatch Automated Test"),
        /**
         * TODO, will run an automated test on the {@link Elevator}.
         */
        ELEVATOR_TEST("Elevator Automated Test"),

        /**
         * TODO, will run an automated test on the {@link Jacks}.
         */
        JACKS_TEST("Test");

        private final String option;

        /**
         * Constructor.
         *
         * @param option the option to put on the dashboard chooser's list.
         */
        Test(String option) {
            this.option = option;
        }

        /**
         * Returns the option to put on the dashboard chooser's list.
         *
         * @return the option to put on the dashboard chooser's list.
         */
        String getOption() {
            return option;
        }
    }

}