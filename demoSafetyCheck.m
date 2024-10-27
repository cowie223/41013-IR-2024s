% Initialize the serial port connection
function arduinoDevice = initArduino(port, baudRate)
    arduinoDevice = serialport(port, baudRate);
    configureTerminator(arduinoDevice, "LF"); % Set line ending to match Arduino
end

% Initialize the Arduino connection
arduinoDevice = initArduino("COM3", 9600);

% Main MATLAB code - could be anything
for i = 1:100
    disp("Running other MATLAB operations...");
    pause(0.1); % Simulate other work

    % Query safeOut on demand
    safeOutValue = SafetyCall(arduinoDevice);
    disp(['safeOut: ', num2str(safeOutValue)]);
end

% Close connection when done
clear arduinoDevice;
