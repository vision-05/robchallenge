% All units in cm

car = [2, 2, pi/4];
carlen = 25;
carwidth = 15;
sensors = [10,15,20];
velocity = 3;
r = 10

swid = 3.7;
sn = 9;
kp = 0.06;
kd = 0.0075;
ki = 0.004;
lenhor = carlen*cos(car(3));
lenvert = carlen*sin(car(3));

pathx = [linspace(2,42,100) linspace(42,82,100) linspace(82,122,100) linspace(122,162,100)];
pathy = [linspace(2,42,100) linspace(42,42,100) linspace(42,2,100) linspace(2,42,100)];

figure(1);
drawcar(car,carlen,carwidth);
xlabel("X Position (cm)");
ylabel("Y Position (cm)");
ia = car(3);
preverrors = zeros(3,9);
t = 100;
tstep = 0.3;
for i = 1:tstep:t
    car(1) = car(1) + tstep*velocity*cos(car(3));
    car(2) = car(2) + tstep*velocity*sin(car(3));
    plot(pathx,pathy,'b--');
    hold on;
    drawcar(car,carlen,carwidth);
    

    % error for single center sensor. include sensor width limit for max
    % error
    % error with front and single sensor
    % figure out error calculation from sensors
    sensormid = [car(1) car(2) car(3) - pi/2];
    sm = drawsensors(sensormid,swid,sn);
    sensorfront = [car(1) + lenhor*cos(car(3))/2, car(2) + lenvert*sin(car(3))/2, car(3) - pi/2];
    sf = drawsensors(sensorfront,swid,sn);
    sensorrear = [car(1) - lenhor*cos(car(3))/2, car(2) - lenvert*sin(car(3))/2, car(3) - pi/2];
    sr = drawsensors(sensorrear,swid,sn);

    hold on;
    plot(sm(:,1),sm(:,2), 'g-');
    plot(sf(:,1),sf(:,2), 'g-');
    plot(sr(:,1),sr(:,2), 'g-');
    hold off;
    pause(0.001)

    %for each array of sensors find the closest sensor to the line
    readings = zeros(3,sn);
    for i = 1:sn
        distv = sm(i,:) - [pathx' pathy'];
        normv = vecnorm(distv'); %we need to find distance along the line, not from vec
        [val, idx] = min(normv); %for ith sensor, closest point is [pathx' pathy'] idx, use this to find error
        if val < swid/(sn*2) + swid/sn
            readings(2,i) = 1;
        else
            readings(2,i) = 0;
        end
        distv = sf(i,:) - [pathx' pathy'];
        normv = vecnorm(distv'); %we need to find distance along the line, not from vec
        [val, idx] = min(normv); %for ith sensor, closest point is [pathx' pathy'] idx, use this to find error
        if val < swid/(sn*2) + swid/sn
            readings(1,i) = 1;
        else
            readings(1,i) = 0;
        end
        distv = sr(i,:) - [pathx' pathy'];
        normv = vecnorm(distv'); %we need to find distance along the line, not from vec
        [val, idx] = min(normv); %for ith sensor, closest point is [pathx' pathy'] idx, use this to find error
        if val < swid/(sn*2) + swid/sn
            readings(3,i) = 1;
        else
            readings(3,i) = 0;
        end
        %sensor reads 0 if on line (if this distance less than sensor
        %tolerance
    end
    errors = readings.*[-8:2:8; -4:1:4; -2:0.5:2]
    differrors = (errors - preverrors)/tstep;
    

    sz = size(errors);
    steering_error = zeros(1,sz(1));
    dsteering_error = zeros(1,sz(1));
    isteering_error = zeros(1,sz(1));
    for i = 1:sz(1)
        steering_error(i) = sum(errors(i,:))
        dsteering_error(i) = sum(differrors(i,:))
        isteering_error(i) = sum(errors(i,:)*tstep + preverrors(i,:)*tstep)
    end

    preverrors = errors;

    imse = mean(isteering_error)
    dmse = mean(dsteering_error)
    mse = mean(steering_error)

    dtheta = -kp*mse + kd*dmse + ki*imse

    car(3) = car(3) + dtheta

    % translation from steering + linear speed to individual wheel speeds using
    % inverse kinematics

    omega = dtheta/tstep;

    pl = velocity/r - (carwidth/2)*(omega/r)
    pr = velocity/r + (carwidth/2)*(omega/r)

    % todo: this is for a reduction to 1 wheel diff drive. For 3 wheels we
    % must superpose wheel motion on each side
end