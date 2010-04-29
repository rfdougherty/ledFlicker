function Arduino


%% Define variables
FunctionLetter  = 'f';
Ch1Freq         = 50;
Ch1Ph           = 0; % 0to1,(0-2pi)
Ch1Amp          = 1; % 0to1
Ch1Mean         = 0.5; % currently Mean value should be 0.5, otherwise stimuli don't have harmonic function.

Duration        = 3; % sec / Stimulus duration 
WinFDur         = 0.2;  % sec / Window function duraion for start and end of stimuli (half of Cosine window)

Ch123sameFlag   = true;

if Ch123sameFlag == true;
    Ch2Freq         = Ch1Freq;
    Ch2Ph           = Ch1Ph;
    Ch2Amp          = Ch1Amp;
    Ch2Mean         = Ch1Mean;
    Ch3Freq         = Ch1Freq;
    Ch3Ph           = Ch1Ph;
    Ch3Amp          = Ch1Amp;
    Ch3Mean         = Ch1Mean;
else
    if ~exist('Ch2Freq','var')
        Ch2Freq         = 10;
        Ch2Ph           = 1 / 3;
        Ch2Amp          = 1;
        Ch2Mean         = 0.5;
    end
    if ~exist('Ch3Freq','var')
        Ch3Freq         = 10;
        Ch3Ph           = 2 / 3;
        Ch3Amp          = 1;
        Ch3Mean         = 0.5;
    end
end

 Command = sprintf('%s,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g;'...
    ,FunctionLetter,Ch1Freq,Ch1Ph,Ch1Amp,Ch1Mean...
    ,Ch2Freq,Ch2Ph,Ch2Amp,Ch2Mean...
    ,Duration,WinFDur);


%% Comfirm Stimuli
SampRatio = 2000;

if Ch1Freq < 10
    OneSec = 1:(SampRatio+1);
elseif Ch1Freq >= 10
    OneSec = 1:(SampRatio+1) / 10;
end

NumTime1 = 0 : Duration * SampRatio;
ph1 = 2*pi .* NumTime1 * Ch1Freq  / SampRatio + 2*pi*Ch1Ph;
tmp     = sin(ph1) ./ 2;

% Cosine window function
if WinFDur > 0
    NumAtten = round(SampRatio * WinFDur);
    windFunc = 0.5 * cos([pi:pi/(NumAtten-1):2*pi]) + 0.5;
    tmp(1:NumAtten) = tmp(1:NumAtten) .* windFunc;
    tmp(end-NumAtten+1:end) = tmp(end-NumAtten+1:end) .* fliplr(windFunc); 
end

Amp1 = tmp .* Ch1Amp + Ch1Mean;

% draw channel2 signal
figure(1); hold on
Time1 = 0:1/SampRatio:Duration;
plot(Time1, Amp1,'b'); 

figure(2); hold on
plot(Time1(OneSec), Amp1(OneSec),'b')

if Ch123sameFlag == false;
    % channel 2
    NumTime2 = 0 : Duration * SampRatio;
    ph2 = 2*pi .* NumTime2 * Ch2Freq / SampRatio + 2*pi*Ch2Ph;
    tmp     = sin(ph2) ./ 2;
    
    % Cosine window function
        if WinFDur > 0
            tmp(1:NumAtten) = tmp(1:NumAtten) .* windFunc;
            tmp(end-NumAtten+1:end) = tmp(end-NumAtten+1:end) .* fliplr(windFunc); 
        end
        
    Amp2 = tmp .* Ch2Amp + Ch2Mean;
    Time2 = 0:1/SampRatio:Duration;
    
    % draw channel2 signal
    figure(1);plot(Time2, Amp2,'r'); 
    figure(2);plot(Time2(OneSec), Amp2(OneSec),'r')
    
    % channel 3
    NumTime3 = 0 : Duration * SampRatio;
    ph3 = 2*pi .* NumTime3 * Ch3Freq / SampRatio + 2*pi*Ch3Ph;
    tmp     = sin(ph3) ./ 2;
    
    % Cosine window function
        if WinFDur > 0
            tmp(1:NumAtten) = tmp(1:NumAtten) .* windFunc;
            tmp(end-NumAtten+1:end) = tmp(end-NumAtten+1:end) .* fliplr(windFunc); 
        end
        
    Amp3 = tmp .* Ch3Amp + Ch3Mean;
    Time3 = 0:1/SampRatio:Duration;
    
    % draw channel3 signal
    figure(1);plot(Time3, Amp3,'g'); 
    figure(2);plot(Time3(OneSec), Amp3(OneSec),'g')
    
end

%% Run Arduino

% get serial variable
s1 = serial('/dev/tty.usbserial-A900ae4s','BaudRate',57600);

% open the serial port
fopen(s1);

% run the stimuli
fprintf(s1, Command)

% close the serial port
fclose(s1);
% memo for debugging
% tmp = fgetl(s1);% you can get some information in the device (Ardino)
% !sudo dmesg ; % you can check whether the Mac has usbserial port or not.
% !ls /dev/tty*; % you can check whether the Mac has usbserial port or not.