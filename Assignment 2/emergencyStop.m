function [] = emergencyStop(app)
value = app.status; %status off=0, on=1,stop=3
message = false;
while value == 3
    if ~message
        message = true;
        disp("Emergency stop pressed")
    end
    pause(1)
    value = app.status;
end
end