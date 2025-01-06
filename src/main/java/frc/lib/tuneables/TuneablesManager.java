package frc.lib.tuneables;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.BiConsumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.tuneables.sendableproperties.TuneableProperty;

public class TuneablesManager {
    private static Queue<TuneableItem> newTuneablesQueue = new LinkedList<>();
    private static boolean isEnabled = false;

    public static void add(String key, Tuneable tuneable, BiConsumer<String, Sendable> sendablePublisher) {
        // does not instantly publish even when enabled to avoid publishing from inside
        // initTuneable addChild() call, beacause doing that leads to
        // ConcurrentModificationException with shuffleboard.
        newTuneablesQueue.add(new TuneableItem(key, tuneable, sendablePublisher));
    }

    public static void add(String key, Tuneable tuneable) {
        add(key, tuneable, SmartDashboard::putData);
    }

    public static void add(String key, Sendable sendable, BiConsumer<String, Sendable> sendablePublisher) {
        newTuneablesQueue.add(new TuneableItem(key, sendable::initSendable, sendablePublisher));
    }

    public static void add(String key, Sendable sendable) {
        add(key, sendable, SmartDashboard::putData);
    }

    public static void enable() {
        if (!isEnabled) {
            isEnabled = true;
            update();
        }
    }

    private static void publishTuneable(
            String name,
            Tuneable tuneable,
            BiConsumer<String, Sendable> sendablePublisher) {
        sendablePublisher.accept(name, (builder) -> {
            tuneable.initTuneable(new TuneableBuilder(builder, name, sendablePublisher));
        });
    }

    public static void update() {
        if (isEnabled) {
            TuneableProperty.updateAll();

            while (!newTuneablesQueue.isEmpty()) {
                TuneableItem item = newTuneablesQueue.poll();
                publishTuneable(item.key, item.tuneable, item.sendablePublisher);
            }
        }
    }

    public static boolean isEnabled() {
        return isEnabled;
    }

    private static class TuneableItem {
        public final String key;
        public final Tuneable tuneable;
        public final BiConsumer<String, Sendable> sendablePublisher;

        public TuneableItem(String key, Tuneable tuneable, BiConsumer<String, Sendable> sendablePublisher) {
            this.key = key;
            this.tuneable = tuneable;
            this.sendablePublisher = sendablePublisher;
        }
    }
}
