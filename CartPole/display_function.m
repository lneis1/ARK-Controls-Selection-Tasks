function display_function(state,desired,params,time)

dt = time(2) - time(1);

for k = 1 : length(time)
    print(state(k,:), params);
    pause(dt);
end

tol = 0.1;

if (abs(state(length(time),3)) < tol) & (abs(state(length(time):1)-desired(1)) < tol)
    disp('Target reached and Pole is Balanced.');
elseif(abs(state(k,3)) < tol)
    disp('Pole is Balanced. Target not reached.');
else
    disp('System not tuned properly.');
end

function print(state_t, params)
    pole_1 = [state_t(1), 2];

    pole_2(1) = pole_1(1) + params.L*sin(state_t(3));
    pole_2(2) = pole_1(2) + params.L*cos(state_t(3));

    pole = [pole_1;pole_2];

    plot([pole(1,1) + 1.5,pole(1,1) - 1.5],[pole(1,2),pole(1,2)],'LineWidth',20,'Color','b');
    hold on;

    plot([-4 12],[1.5 1.5],'Color',[0 0 0]);
    hold on;

    plot(pole(:,1),pole(:,2),'Color',"r",'LineWidth',2);
    hold on;

    viscircles(pole_2,0.1,'Color','r','LineWidth',8);

    %vertices = [pole_1(1) pole_1(2);
    %    pole_1(1)+0.25 pole_1(2)-0.25;
    %    pole_1(1)+0.5 pole_1(2);
    %    pole_1(1)+1.5 pole_1(2);
    %    pole_1(1)+1.5 pole_1(2)-1;
    %    pole_1(1)-1.5 pole_1(2)-1;
    %    pole_1(1)-1.5 pole_1(2);
    %    pole_1(1)-0.5 pole_1(2);
    %    pole_1(1)-0.25 pole_1(2)-0.25;
    %    pole_1(1) pole_1(2)];
    %plot(vertices(:,1),vertices(:,2),'Color',[0 0 0], 'MarkerFaceColor','b');
    %hold on;

    %radius = 0.1;
    %centres = [pole_1(1)+0.75 pole_1(2)-1-radius;
    %    pole_1(1)-0.75 pole_1(2)-1-radius];
    %viscircles(centres,radius,'Color',[0 0 0]);

    axis([-5 13 -2 10]);
    hold off;

end
end