function sensorsret = drawsensors(sensors, l, n)
    sensorspace = linspace(-l/2,l/2,n);
    sensorx = sensorspace*cos(sensors(3)) + sensors(1);
    sensory = sensorspace*sin(sensors(3)) + sensors(2);
    sensorsret = [sensorx' sensory'];
end