package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import team2679.atlantiskit.tunables.SendableType;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesTable;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;
import frc.robot.subsystems.swerve.Swerve;

import static frc.robot.subsystems.swerve.SwerveContants.DriverController.*;
import static frc.robot.subsystems.swerve.SwerveContants.*;

public class SwerveDriverController extends TunableCommand {
    private final Swerve swerve;
    private TunablesTable tuneablesTable = new TunablesTable(SendableType.LIST);

    private DoubleSupplier sidewaysSupplier;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier rotationsSupplier;
    private BooleanSupplier isFieldRelative;
    private BooleanSupplier isSensetiveMode;

    private DoubleHolder maxAngularVelocityRPS = tuneablesTable.addNumber("Max Angular Velocity RPS",
            DRIVER_MAX_ANGULAR_VELOCITY_RPS);
    private SendableChooser<Double> velocityMultiplierChooser = new SendableChooser<>();

    private final SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(DRIVER_ACCELERATION_LIMIT_MPS);
    private final SlewRateLimiter sidewaysSlewRateLimiter = new SlewRateLimiter(DRIVER_ACCELERATION_LIMIT_MPS);
    private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(DRIVER_ANGULAR_ACCELERATION_LIMIT_RPS);


    public SwerveDriverController(Swerve swerve, DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationsSupplier, BooleanSupplier isFieldRelative, BooleanSupplier isSensetiveMode) {
        this.swerve = swerve;
        addRequirements(swerve);

        this.sidewaysSupplier = sidewaysSupplier;
        this.forwardSupplier = forwardSupplier;
        this.rotationsSupplier = rotationsSupplier;
        this.isFieldRelative = isFieldRelative;
        this.isSensetiveMode = isSensetiveMode;

        velocityMultiplierChooser.setDefaultOption("REGULAR (100%)", 1.0);
        velocityMultiplierChooser.addOption("CHILD (50%)", 0.5);
        velocityMultiplierChooser.addOption("BABY (30%)", 0.3);
        velocityMultiplierChooser.addOption("EGG (10%)", 0.1);

        tuneablesTable.addChild("velocity chooser", velocityMultiplierChooser);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double velocityMultiplier = velocityMultiplierChooser.getSelected();

        double precentageForward = forwardSupplier.getAsDouble() * velocityMultiplier;
        double precentageSideways = sidewaysSupplier.getAsDouble() * velocityMultiplier;
        double precentageRotation = rotationsSupplier.getAsDouble() * velocityMultiplier;

        if (isSensetiveMode.getAsBoolean()) {
            precentageForward *= SENSETIVE_TRANSLATION_MULTIPLIER;
            precentageSideways *= SENSETIVE_TRANSLATION_MULTIPLIER;
            precentageRotation *= SENSETIVE_ROTATION_MULTIPLIER;
        }

        swerve.drive(
                forwardSlewRateLimiter.calculate(precentageForward * MAX_MODULE_VELOCITY_MPS),
                sidewaysSlewRateLimiter.calculate(precentageSideways * MAX_MODULE_VELOCITY_MPS),
                rotationSlewRateLimiter.calculate(precentageRotation * maxAngularVelocityRPS.get()),
                isFieldRelative.getAsBoolean(),
                false, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        tuneablesTable.initTunable(builder);
    }
}
