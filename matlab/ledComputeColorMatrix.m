
% Make sure the vistadisp exptTools2 are in our path
addpath(genpath('~/svn/vistadisp/exptTools2'));


% Compute the rgb2lms transform
load stockman
sensors = stockman;
% Load the raw RGB spectra (should load vars spectraRawRed,
% spectraRawGreen, spectraRawBLue).
load spectraRaw;
spectra(:,1)           = interpPR650(spectraRawRed);
spectra(:,2)           = interpPR650(spectraRawGreen);
[spectra(:,3), lambda] = interpPR650(spectraRawBlue);
% figure; plot(lambda,spectra(:,1),'r',lambda,spectra(:,2),'g',lambda,spectra(:,3),'b')

rgb2lms = sensors'*spectra;
lms2rgb = inv(rgb2lms);
backRGB.dir = [1 1 1]';
backRGB.scale = 0.5;
  
stimLMS.dir = [0 0 1];
stimLMS.scale = 1;
[stimLMS, stimRGB] = findMaxConeScale(rgb2lms,stimLMS,backRGB);

stimLMS.scale = stimLMS.maxScale*sin(linspace(0,2*pi*3,100));
stimRGB = cone2RGB(rgb2lms, stimLMS, backRGB)


[lmsContrast lmsBack]= RGB2ConeContrast(rgb2lms,stimRGB,backRGB)

figure;plot((stimRGB.dir*stimRGB.scale)');
