close all
clear
clc

s = serialport("COM8", 115200);
flush(s)
configureTerminator(s,'CR')

% Variables del muestreo
fs = 128;           % Frecuencia de muestreo [Hz]
N = 128;            % Número de muestras             
f = fs * ((0:N/2-1)/N);

count = 1;

while(1)
    if count == 1
        string=readline(s);
        numeros = str2num(string);
        numerosSide = numeros(1:N/2);
        count = count + 1;
    elseif count == 2
        string1=readline(s);
        numeros1=str2num(string1);
        numeros1Side = numeros1(1:N/2);
        count = count + 1;
    elseif count == 3
        string2=readline(s);
        numeros2=str2num(string2);
        numeros2Side = numeros2(1:N/2);
        count = count + 1;
    else
        stem(f, numerosSide)
        hold on
        stem(f, numeros1Side)
        stem(f, numeros2Side)
        grid on;
        xlabel("Frecuencia [Hz]");
        ylabel("Amplitud");
        legend("X", "Y", "Z");
        hold off
        count = 1;
    end
end