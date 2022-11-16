close all
clear
clc

ts = 1e-2; % s

%% Graficación

t = out.time1;
rpm = double(squeeze(out.rpm1));
ref = double(squeeze(out.reference1));


plot(t, rpm);
hold on
title('Visualización del control');
ylabel('RPM')
grid on

plot(t, ref)
xlabel('Time [s]')

legend('Encoder', 'Referencia')

hold off
%%
