package frc.robot.logging;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.TelemetryConstants;

public class BooleanLog extends LogEntry<Boolean> {
    private final String logName;

    private BooleanLogEntry datalogEntry;
    private BooleanPublisher networkTablesEntry;

    public BooleanLog(String name) {
        logName = TelemetryConstants.kTabPrefix + "/" + name;

        datalogEntry = null;
        networkTablesEntry = null;
    }

    @Override
    protected void enableDatalog() {
        try {
            datalogEntry = new BooleanLogEntry(DataLogManager.getLog(), logName);
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected void enableNetwork() {
        try {
            networkTablesEntry = NetworkTableInstance.getDefault().getBooleanTopic(logName).publish();
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected boolean isDatalogEnabled() { return datalogEntry != null; }

    @Override
    protected boolean isNetworkEnabled() { return networkTablesEntry != null; }

    @Override
    protected void sendData(Boolean data) {
        if (datalogEntry != null) {
            datalogEntry.append(data);
        }

        if (networkTablesEntry != null) {
            networkTablesEntry.set(data);
        }
    }

}
