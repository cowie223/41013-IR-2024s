function safeOutValue = SafetyCall(arduinoDevice)
    % Function to query the attached arduino device, and return the defined
    % value. This is based on code attached, which is loaded into the
    % Arduino chip via the IDE.

    writeline(arduinoDevice, "getSafeOut");                                 % Request safeOut value to arduino. This correlates with a request in the arduino code.
    data = readline(arduinoDevice);                                         % Read the response.
    safeOutValue = str2double(data);                                        % Convert to a number and return.
end