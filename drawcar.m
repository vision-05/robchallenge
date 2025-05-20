function pixels = drawcar(car, len, width)
    pathx = -200:1:200;
    pathy = -200:1:200;
    lenhor = len*cos(car(3));
    lenvert = len*sin(car(3));
    carlinex = linspace(car(1)-lenhor/2,car(1)+lenhor/2, 100);
    carliney = linspace(car(2)-lenvert/2,car(2)+lenvert/2, 100);
    pixels = [carlinex;carliney]';
    plot(pathx,0);
    hold on;
    plot(0,pathy);
    hold on;
    plot(pixels(:,1),pixels(:,2), 'b-');
    hold on;
    plot(pixels(:,1)+width*sin(car(3))/2,pixels(:,2)-width*cos(car(3))/2, 'r-');
    hold on;
    plot(pixels(:,1)-width*sin(car(3))/2,pixels(:,2)+width*cos(car(3))/2, 'r-');
    hold off;
end