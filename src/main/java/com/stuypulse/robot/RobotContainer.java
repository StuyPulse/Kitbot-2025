package com.stuypulse.robot;

import com.stuypulse.robot.commands.AutoPilot;
import com.stuypulse.robot.commands.SeedFieldRelative;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.L1.Bottom5PieceL1;
import com.stuypulse.robot.commands.auton.L1.Top5PieceL1;
import com.stuypulse.robot.commands.auton.tests.Mobility;
import com.stuypulse.robot.commands.auton.tests.SquareTest;
import com.stuypulse.robot.commands.dropper.DropperDrop;
import com.stuypulse.robot.commands.dropper.DropperReverse;
import com.stuypulse.robot.commands.dropper.DropperShootSequence;
import com.stuypulse.robot.commands.dropper.DropperStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwervePIDToPose;
import com.stuypulse.robot.commands.swerve.SwervePathFind;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.dropper.Dropper;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final SwerveDrive swerve = SwerveDrive.getInstance();
    public final Dropper dropper = Dropper.getInstance();
    public final AprilTagVision vision = AprilTagVision.getInstance();
    public final Odometry odometry = Odometry.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        swerve.configureAutoBuilder();        
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        driver.getDPadUp().onTrue(new SeedFieldRelative());

        // align to closest coral and then shoot automatically
        driver.getRightTriggerButton()
            .whileTrue(new SwervePIDToPose(() -> Field.getClosestBranch().getTargetPose())
                .andThen(new DropperShootSequence()))
            .onFalse(new DropperStop());

        // manual shoot
        driver.getRightBumper()
            .onTrue(new DropperDrop())
            .onFalse(new DropperStop());

        driver.getLeftTriggerButton()
            .onTrue(new DropperReverse())
            .onFalse(new DropperStop());

        // align to nearest algae
        driver.getRightButton()
            .whileTrue(new SwervePIDToPose(() -> Field.getClosestAlgaeBranch().getTargetPose()));
        
        driver.getTopButton()
            .whileTrue(new AutoPilot().repeatedly());
        
        driver.getLeftButton().whileTrue(SwervePathFind.toNearestCoralStation().andThen(new SwervePIDToPose(() -> Field.getTargetPoseForCDCoralStation())));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        AutonConfig TOP_5_L1_BLUE = new AutonConfig("Top 5 L1", Top5PieceL1::new,
        "Blue Top to 1L", "Blue 1L to HP", "Blue HP to 6R", "Blue 6R to HP", "Blue HP to 6R", "Blue 6R to HP", "Blue HP to 6L", "Blue 6L to HP", "Blue HP to 6L");
        AutonConfig TOP_5_L1_RED = new AutonConfig("Top 5 L1", Top5PieceL1::new,
        "Red Top to 1L", "Red 1L to HP", "Red HP to 6R", "Red 6R to HP", "Red HP to 6R", "Red 6R to HP", "Red HP to 6L", "Red 6L to HP", "Red HP to 6L");

        AutonConfig BOTTOM_5_L1_BLUE = new AutonConfig("Bottom 5 L1", Bottom5PieceL1::new,
        "Blue Bottom to 3R", "Blue 3R to HP", "Blue HP to 4L", "Blue 4L to HP", "Blue HP to 4L", "Blue 4L to HP", "Blue HP to 4R", "Blue 4R to HP", "Blue HP to 4R");
        AutonConfig BOTTOM_5_L1_RED = new AutonConfig("Bottom 5 L1", Bottom5PieceL1::new,
        "Red Bottom to 3R", "Red 3R to HP", "Red HP to 4L", "Red 4L to HP", "Red HP to 4L", "Red 4L to HP", "Red HP to 4R", "Red 4R to HP", "Red HP to 4R");

        TOP_5_L1_BLUE.registerDefaultBlue(autonChooser);
        TOP_5_L1_RED.registerDefaultRed(autonChooser);

        BOTTOM_5_L1_BLUE.registerBlue(autonChooser);
        BOTTOM_5_L1_RED.registerRed(autonChooser);

        // Tests

        AutonConfig SQUARE_TEST = new AutonConfig("Square Test", SquareTest::new,
        "Square Top", "Square Right", "Square Bottom", "Square Left");
        AutonConfig TURN_SQUARE_TEST = new AutonConfig("Turn Square Test", SquareTest::new,
        "Turn Square Top", "Turn Square Right", "Turn Square Bottom", "Turn Square Left");
        AutonConfig MOBILITY_BLUE = new AutonConfig("Mobility", Mobility::new, 
        "Blue Mobility");
        AutonConfig MOBILITY_RED = new AutonConfig("Mobility", Mobility::new,
        "Red Mobility");

        SQUARE_TEST.registerBlue(autonChooser);
        TURN_SQUARE_TEST.registerBlue(autonChooser);
        MOBILITY_BLUE.registerBlue(autonChooser);
        MOBILITY_RED.registerRed(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
