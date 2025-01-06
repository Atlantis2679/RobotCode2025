package frc.lib.tuneables.extensions;

import java.util.function.Function;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneablesTable;

public abstract class TuneableCommand extends Command implements Tuneable {
    public Tuneable fullTuneable() {
        return (builder) -> {
            builder.setSendableType(SendableType.LIST);
            builder.addChild("run button", (Sendable) this);
            initTuneable(builder);
        };
    };

    public TuneableCommand extendTuneable(Tuneable tuneable) {
        return wrap(this, (builder) -> {
            this.initTuneable(builder);
            tuneable.initTuneable(builder);
        });
    }

    public static TuneableCommand wrap(Function<TuneablesTable, Command> commandFactory, SendableType sendableType) {
        return new TuneableWrapperCommand(commandFactory, sendableType);
    }

    public static TuneableCommand wrap(Function<TuneablesTable, Command> commandFactory) {
        return new TuneableWrapperCommand(commandFactory);
    }

    public static TuneableCommand wrap(Command command, Tuneable tuneable) {
        return new TuneableWrapperCommand(command, tuneable);
    }
}
