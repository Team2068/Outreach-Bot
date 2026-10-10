package frc.robot.utility;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Swerve.swerveState;

public class AutomatedController {
    public final CommandXboxController controller;
    public final SendableChooser<Runnable> selector = new SendableChooser<Runnable>();
    public int mode = 0;
    public GenericHID rumble = new GenericHID(0);

    IO io;

    public AutomatedController(int port, IO io) {
        this.io = io;

        selector.setDefaultOption("Automated", () -> {
            mode = 0;
        });
        selector.addOption("Manual", () -> {
            mode = 1;
        });
        selector.addOption("Characterise", () -> {
            mode = 2;
        });
        selector.addOption("Debug", () -> {
            mode = 3;
        });

        selector.onChange((x) -> {
            x.run();
        });

        controller = new CommandXboxController(port);

        controller.rightStick()
                .onTrue(new InstantCommand(() -> io.chassis.field_oritented = !io.chassis.field_oritented));
        controller.leftStick().debounce(2).onTrue(new InstantCommand(io.chassis::resetAngle));
        configure();

    }

    public BooleanSupplier mode(int targetMode) {
        return () -> {
            return mode == targetMode;
        };
    }

    public BooleanSupplier automated() {
        return mode(0);
    }

    public BooleanSupplier manual() {
        return mode(1);
    }

    public BooleanSupplier characterise() {
        return mode(2);
    }

    public BooleanSupplier debug() {
        return mode(3);
    }

    public void switchMode() {
        mode = (mode + 1) % 3;
    }

    public void configure() {
        controller.start().and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::switchMode));
        configureAutomated();
        configureManual();
        configureCharacterisaton();
        configureDebug();

    }

    void configureAutomated() {
        // CHASSIS CONTROL
        controller.back().onTrue(Util.Do(io.chassis::resetAngle, io.chassis));
        controller.povDown().and(automated())
                .onTrue(Util.Do(() -> io.chassis.field_oritented = !io.chassis.field_oritented));

        // STATE CONTROLLERS AND AIMBOT
        controller.b().and(automated()).onTrue(Util.Do(() -> io.chassis.currentState = swerveState.SCORING));
        controller.y().and(automated()).onTrue(Util.Do(() -> io.chassis.currentState = swerveState.PASSING));
        controller.x().and(automated()).onTrue(Util.Do(() -> io.chassis.currentState = swerveState.PASSING));
    }

    public void configureManual() {
        controller.back().and(manual()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        controller.leftBumper().and(manual()).onTrue(Util.Do(()->io.elevator.setSpeed(-0.2), io.elevator)).onFalse(Util.Do(io.elevator::stop));
        controller.rightBumper().and(manual()).onTrue(Util.Do(()->io.elevator.setSpeed(0.2), io.elevator)).onFalse(Util.Do(io.elevator::stop));
    }

    void configureCharacterisaton() {
        controller.x().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kForward));
        controller.a().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kReverse));
        controller.y().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kForward));
        controller.b().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kReverse));

        controller.povUp().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.quasistatic(Direction.kForward));
        controller.povDown().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.quasistatic(Direction.kReverse));
        controller.povRight().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kForward));
        controller.povLeft().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kReverse));
    }

    void configureDebug() {
        controller.back().and(debug()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        controller.povDown().and(debug()).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and(debug()).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and(debug()).and(() -> {
            return !io.chassis.active;
        }).onTrue(new InstantCommand(io.chassis::zeroAbsolute));
    }
}