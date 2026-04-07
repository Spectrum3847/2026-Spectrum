# Elastic Dashboard

Elastic Dashboard is a custom dashboard solution for FRC robots, often used in conjunction with NetworkTables for real-time data visualization and control.

## NetworkTables

NetworkTables is the primary way for the robot code and external applications (like dashboards) to communicate. It provides a key-value store that is synchronized between the robot and any connected clients.

*   **Real-time Data**: Elastic Dashboard can read and display various sensor readings, motor outputs, and other robot states published to NetworkTables.
*   **Control Inputs**: It can also be used to send commands or modify robot parameters (e.g., PID constants, autonomous routine selection) back to the robot code.

## Setting Up Elastic

To set up and use the Elastic Dashboard:

1.  **Install Elastic Dashboard**: Elastic is included in the WPILib installer.
2.  **Connect to Robot**: Configure the Elastic Dashboard to connect to your robot's NetworkTables server (typically on `roborio-<team-number>-frc.local` or a specific IP address).
3.  **Layout Configuration**: Create or load a dashboard layout (`elastic-layout.json`) to define what data to display and how to visualize it (e.g., graphs, gauges, buttons).

## Using SmartDashboard

While Elastic provides a custom solution, it often interacts with or complements the standard WPILib `SmartDashboard`.

*   **Integration**: Data published to `SmartDashboard` by the robot code can often be accessed and displayed within Elastic Dashboard.
*   **Default Display**: `SmartDashboard` provides a simple, built-in way to display basic telemetry and toggle settings.
