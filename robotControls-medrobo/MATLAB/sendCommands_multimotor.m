%% Send Commands (Multimotor) Helper Function
% inputs cell array of messages & arduinoSerial object
% outputs to arduino

% params setup: cell array contains angles in degrees in order of motor

%% function definition
function [] = sendCommands_multimotor(msg, ard)
    % clear any leftover data in the input buffer
    flush(ard);

    % wait for "Enter theta 0:"
    while true
        if ard.NumBytesAvailable > 0
            read = readline(ard);
%             disp(read);
            if contains(read, "Enter theta 0:")
                break;
            end
        end
        pause(0.01);
    end

    % send theta values
    for i = 1:numel(msg)
        % wait for the prompt for each theta
        while true
            if ard.NumBytesAvailable > 0
                read = readline(ard);
%                 disp(read);
                if contains(read, sprintf("Enter theta %d:", i-1))
                    break;
                end
            end
            pause(0.01);
        end

        % send the theta value
%         fprintf("Sending theta %d: %.3f\n", i-1, msg{i});
        writeline(ard, sprintf('%.3f', msg{i}));

        % wait for confirmation
        while true
            if ard.NumBytesAvailable > 0
                read = readline(ard);
%                 disp(read);
                if contains(read, sprintf("Received theta %d:", i-1))
                    break;
                end
            end
            pause(0.01);
        end
    end

    fprintf("All joint values sent successfully.\n");
end