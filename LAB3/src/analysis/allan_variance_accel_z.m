% Sampling frequency for IMU should be 40 Hz
Fs = 40;

% Load csv file for all variance data from the below file path
data = readtable('/home/rj/lab3_ws/src/data/allan_data.csv');

% Extract the sensor data from csv file
field.IMU.orientation_x = data{:, 8};
field.IMU.orientation_y = data{:, 9};
field.IMU.orientation_z = data{:, 10};
field.IMU.orientation_w = data{:, 11};
field.IMU.linear_acc_x = data{:, 33};
field.IMU.linear_acc_y = data{:, 34};
field.IMU.linear_acc_z = data{:, 35};
field.IMU.gyro_x = data{:, 21};
field.IMU.gyro_y = data{:, 22};
field.IMU.gyro_z = data{:, 23};
field.IMU.magnetic_field_x = data{:, 48};
field.IMU.magnetic_field_y = data{:, 49};
field.IMU.magnetic_field_z = data{:, 50};

% Assigning linear acceleration from the csv file. 
linear_accz = field.IMU.linear_acc_z;
string = 'Linear Acceleration - Z';

% Time step
t = 1 / Fs;

% Compute the cumulative sum
theta = cumsum(linear_accz, 1) * t;

% Parameters for the Allan variance analysis
maxNumber = 100;
L = size(theta, 1);
maxM = 2^floor(log2(L / 2));
m = logspace(log10(1), log10(maxM), maxNumber).';
m = ceil(m);
m = unique(m);

% Time intervals
tau = m * t;

% Initialize arrays for Allan variance and Allan deviation
avar = zeros(numel(m), 1);

% Calculate Allan variance
for i = 1:numel(m)
    mi = m(i);
    avar(i) = sum( ...
        (theta(1 + 2 * mi : L) - 2 * theta(1 + mi : L - mi) + theta(1 : L - 2 * mi)).^2, 1);
end

% Normalize using below formula
avar = avar ./ (2 * tau.^2 .* (L - 2 * m));

% Compute the allan deviation
adev = sqrt(avar);

% Plot Allan deviation
figure
loglog(tau, adev)
title('Allan Deviation Plot for Z');
ylabel('Linear Acceleration in Z (m / s^2)')
xlabel('Time (s)')


% Calculate other parameters and plot them
%{
slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
b = logadev(i) - slope * logtau(i);
logN = slope * log(1) + b;
N = 10^logN;
tauN = 1;
lineN = N ./ sqrt(tau);
figure
loglog(tau, adev, tau, lineN, '--', tauN, N, 'o');
title('Angle Random Walk for X');
%}

slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
b = logadev(i) - slope * logtau(i);
logK = slope * log10(3) + b;
K = 10^logK;
tauK = 3;
lineK = K * sqrt(tau / 3);
figure
loglog(tau, adev, tau, lineK, '--', tauK, K, 'o');
title('Rate Random Walk');

slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));
b = logadev(i) - slope * logtau(i);
scfB = sqrt(2 * log(2) / pi);
logB = b - log10(scfB);
B = 10^logB;
tauB = tau(i);
lineB = B * scfB * ones(size(tau));
figure
loglog(tau, adev, tau, lineB, '--', tauB, scfB * B, 'o');
title('Bias Instability for X');

tauParams = [tauK, tauB];
params = [K, scfB * B];
figure
loglog(tau, adev, tau, [lineK, lineB], '--', ...
tauParams, params, 'o');
title('Noise Parameters for Z');
ylabel('Linear Acceleration in X (m / s^2)')
xlabel('Time (s)')