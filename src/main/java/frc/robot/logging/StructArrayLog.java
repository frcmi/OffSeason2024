package frc.robot.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.TelemetryConstants;

public class StructArrayLog<T> extends LogEntry<T[]> {
    private final String logName;
    private final Struct<T> metadata;

    private StructArrayLogEntry<T> datalogEntry;
    private StructArrayPublisher<T> networkTablesEntry;

    public StructArrayLog(String name, Struct<T> struct) {
        logName = TelemetryConstants.kTabPrefix + "/" + name;
        metadata = struct;

        datalogEntry = null;
        networkTablesEntry = null;
    }

    @Override
    protected void enableDatalog() {
        try {
            datalogEntry = StructArrayLogEntry.create(DataLogManager.getLog(), logName, metadata);
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected void enableNetwork() {
        try {
            networkTablesEntry = NetworkTableInstance.getDefault().getStructArrayTopic(logName, metadata).publish();
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected boolean isDatalogEnabled() { return datalogEntry != null; }

    @Override
    protected boolean isNetworkEnabled() { return networkTablesEntry != null; }

    @Override
    protected void sendData(T[] data) {
        if (datalogEntry != null) {
            datalogEntry.append(data);
        }

        if (networkTablesEntry != null) {
            networkTablesEntry.set(data);
        }
    }

}
