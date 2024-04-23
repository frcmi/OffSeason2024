package frc.robot.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.TelemetryConstants;

public class StructLog<T> extends LogEntry<T> {
    private final String logName;
    private final Struct<T> metadata;

    private StructLogEntry<T> datalogEntry;
    private StructPublisher<T> networkTablesEntry;

    public StructLog(String name, Struct<T> struct) {
        logName = TelemetryConstants.kTabPrefix + "/" + name;
        metadata = struct;

        datalogEntry = null;
        networkTablesEntry = null;
    }

    @Override
    protected void enableDatalog() {
        try {
            datalogEntry = StructLogEntry.create(DataLogManager.getLog(), logName, metadata);
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected void enableNetwork() {
        try {
            networkTablesEntry = NetworkTableInstance.getDefault().getStructTopic(logName, metadata).publish();
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected boolean isDatalogEnabled() { return datalogEntry != null; }

    @Override
    protected boolean isNetworkEnabled() { return networkTablesEntry != null; }

    @Override
    protected void sendData(T data) {
        if (datalogEntry != null) {
            datalogEntry.append(data);
        }

        if (networkTablesEntry != null) {
            networkTablesEntry.set(data);
        }
    }
}
