package frc.robot.logging;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.TelemetryConstants;

public class DoubleLog extends LogEntry<Double> {
    private final String logName;

    private DoubleLogEntry datalogEntry;
    private DoublePublisher networkTablesEntry;

    public DoubleLog(String name) {
        logName = TelemetryConstants.kTabPrefix + "/" + name;

        datalogEntry = null;
        networkTablesEntry = null;
    }

    @Override
    protected void enableDatalog() {
        try {
            datalogEntry = new DoubleLogEntry(DataLogManager.getLog(), logName);
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected void enableNetwork() {
        try {
            networkTablesEntry = NetworkTableInstance.getDefault().getDoubleTopic(logName).publish();
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected boolean isDatalogEnabled() { return datalogEntry != null; }

    @Override
    protected boolean isNetworkEnabled() { return networkTablesEntry != null; }

    @Override
    protected void sendData(Double data) {
        if (datalogEntry != null) {
            datalogEntry.append(data);
        }

        if (networkTablesEntry != null) {
            networkTablesEntry.set(data);
        }
    }
}
