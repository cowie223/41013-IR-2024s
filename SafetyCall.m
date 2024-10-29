function safeOutValue = SafetyCall(arduinoDevice)
    writeline(arduinoDevice, "getSafeOut"); % Request safeOut value
    data = readline(arduinoDevice);         % Read the response
    safeOutValue = str2double(data);        % Convert to a number
end