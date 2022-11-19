outputmin = 10;
outputmax = 90;
pmax = 25;
pmin = 0; 

outputmin = 16777216 * (outputmin / 100.0) + 0.5;
outputmax = 16777216 * (outputmax / 100.0) + 0.5;


%%
t = out.time;
psi = out.psi.Data;
pressure = zeros(length(t));
for i = 1:length(t)
    pressure(i) = psi(:,:,i);
end

ref = zeros(1, length(t));

for i = 1:600
    ref(i) = 11.2914;
end 
for i = 601:length(t)
    ref(i) = 12.93;
end 

figure
pressure = pressure(:,1).';
plot(t/10, pressure);
hold on
plot(t/10, ref);
title('Respuesta a la señal escalón');
ylabel('Pressure [psi]')
xlabel('Time [s]')
grid on

hold off

%%
plot(abs(fft(pressure)));