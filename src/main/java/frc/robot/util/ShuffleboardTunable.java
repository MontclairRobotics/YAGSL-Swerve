package frc.robot.util;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public abstract class ShuffleboardTunable<T> {
    protected ArrayList<Consumer<T>> whenUpdaters;

    protected GenericEntry ent;
    protected T value;

    protected ShuffleboardTunable(T defaultValue, String name) {
        this.value = defaultValue;
        // this.name = name;
        SimpleWidget widget = Shuffleboard.getTab("Tuning").add(name, defaultValue);
        ent = widget.getEntry();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable ntTable = inst.getTable("Shuffleboard").getSubTable("Auto");

        if (DriverStation.getMatchType() == MatchType.None || DriverStation.isTest()) {
            ntTable.addListener(
                name,
                EnumSet.of(Kind.kValueAll),
                (table, key, event) -> {
                    this.value = getValue();
                    for (Consumer<T> whenUpdate : whenUpdaters)
                    whenUpdate.accept(value);
                });
        }
    }

    public void whenUpdate(Consumer<T> func) {
        whenUpdaters.add(func);
    }

    public abstract T getValue();

    public T get() {
        return value;
    }
    
    public static ShuffleboardTunable<Double> of(double val, String name) {

        return new ShuffleboardTunable<Double>(val, name) {
            @Override
            public Double getValue() {
                return ent.getDouble(value);
            }
        };
    }

    public static ShuffleboardTunable<Boolean> of(boolean val, String name) {

        return new ShuffleboardTunable<Boolean>(val, name) {
            @Override
            public Boolean getValue() {
                return ent.getBoolean(value);
            }
        };
    }
}