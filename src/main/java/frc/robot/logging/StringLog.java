package frc.robot.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.TelemetryConstants;

public class StringLog extends LogEntry<String> {
    private final String logName;

    private StringLogEntry datalogEntry;
    private StringPublisher networkTablesEntry;

    public StringLog(String name) {
        logName = TelemetryConstants.kTabPrefix + "/" + name;

        datalogEntry = null;
        networkTablesEntry = null;
    }

    @Override
    protected void enableDatalog() {
        try {
            datalogEntry = new StringLogEntry(DataLogManager.getLog(), logName);
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected void enableNetwork() {
        try {
            networkTablesEntry = NetworkTableInstance.getDefault().getStringTopic(logName).publish();
        } catch (Throwable error) {
            // nothing
        }
    }

    @Override
    protected boolean isDatalogEnabled() { return datalogEntry != null; }

    @Override
    protected boolean isNetworkEnabled() { return networkTablesEntry != null; }

    @Override
    protected void sendData(String data) {
        if (datalogEntry != null) {
            datalogEntry.append(data);
        }

        if (networkTablesEntry != null) {
            networkTablesEntry.set(data);
        }
    }

}
