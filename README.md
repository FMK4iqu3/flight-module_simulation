# tec_simulation 
Spacecraft flight software, among other tasks, processes input data from multiple sensors, executes control loops, and sends commands to several actuators. The control loop may be a function provided by another team like Guidance, Navigation, and Control (GNC). lThe task is to write a simulation for a simplified chain with concurrently executing components, consisting of:

Input components

IMU component, generating timestamped attitude rate data at 100Hz.
GNSS component, generating timestamped position data at 20Hz.
Processing component


A loop running at 50Hz, reading sensor data and producing filtered outputs. It should


Provide attitude rate output based on the average of all available IMU measurements in one tick. Outdated data should be ignored. Raise flag if there is no valid input data available in one tick.
Provide position output based on the last available GNSS measurement. Raise flag if the last position data is older than 1s.
FDIR component. A component which
raises an alarm if the processing component has no valid output.
raises an alarm if a sensor component is not providing any output for three consecutive nominal measurement intervals.
In reality, the sensor components would read sensor data from real hardware. Here, you can use a simple function generator to produce valid but varying outputs. Add some noise so that different sensors produce slightly different measurements.

Task

Identify a suitable architecture for parallel execution of the components and messaging between them.
Create a simulation consisting of three IMU components, two GNSS components, and one processing and FDIR component.
The sensor components should support fault injection for testing
Create a simple drawing / sketch of your setup and the interactions.
Provide at least three different runs of your simulation, where each creates a logfile of the filtered data and the status of the alarms, covering the following cases
Nominal run for 10s
Failure case where all IMUs drop out one after each other

Failure case where GNSS data drops out for 500ms

Hints
The focus of the task is software architecture, interaction of components, code quality, and how it is executed.

If you are unclear about some instructions, go ahead with your best guess and explain your reasoning in comments.

The solution should be object oriented.

It is up to you how to inject the failures.

You do not have to provide the generated logfiles or binaries.

You may provide a plot of the outputs.

Your submission should allow an easy execution of all three simulation cases.

Constraints

Use modern C++.

You may only use the standard library, no further dependencies allowed (except a test framework if you want).

Your code should come with instructions on how to execute it, it should run under Linux. Use any common build system if necessary.