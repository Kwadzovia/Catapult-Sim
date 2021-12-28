clear
clc
close all

%REFERENCES
%https://www.mathworks.com/help/symbolic/solve.html
%https://www.mathworks.com/matlabcentral/answers/308927-how-to-round-a-result-to-two-decimal-places
%Motor: https://www.mcmaster.com/2709K17/

%VARIABLES
a_c = -9.8; %m/s^2
timeElapsed = [0 0 0];
totalTime = 0;

%SIMULINK VARIABLES
%positionList = 1.5*rand([1,3]);
%markerList = [50*rand([1,3])];

%Speed PI variables, different for long distance throws
speed_P = 2.6;
speed_I = 1.2;

forcedArmAngle = 90;
initialAngleOffset = 0;
ballPos = [4.75 -23.75 0]; %90 degrees: [4.75 -23.75 0], %45 degrees: [20.25 -12.5 0]

maxHeightFlag = true;
maxDistanceFlag = false;
%Insert Targets Here
XtargetList = [0];

%mechanical system
L = 0.28; %m
Lprime = 0.2435; %m
theta_ret = 0.785398; %rad
theta_0 = -0.785398; %rad --> LMAGEL: should these angles be absolute or relative?
r = 0.0315; %m
phi = 1.5708 - theta_ret; %rad
y_pivot = 0; %m

J_Garm = 0.00072567; %kg*m^2
m_arm = 0.1188; %kg
d_arm = 0.135; %m
J_Gball = 5.75505*10^-5; %kg*m^2
m_ball = 0.145; %kg
d_ball = 0.24839; %m
J_Gbackplate = 1.221*10^-5; %kg*m^2
m_backplate = 0.0396; %kg
d_backplate = 0.245397; %m
J_Gendplate = 1.20313*10^-6; %kg*m^2
m_endplate = 0.01155; %kg
d_endplate = 0.28009; %m

%J_load = 0.001918; %kg*m^2

%motor
n = 19.7;
J_motor = 2.6*10^-5; %kg*m^2
%Tau_rated_kgcm = 2.4; %kg*cm
Tau_rated_Nm = 20.7; % Nm %Tau_rated_kgcm*0.0980665;

Impact_Tracking = struct();
Debug_Tracking = struct();

%BACKGROUND CALCULATIONS
J_arm = J_Garm + m_arm*(d_arm^2);
J_backplate = J_Gbackplate + m_backplate*(d_backplate^2);
J_endplate = J_Gendplate + m_endplate*(d_endplate^2);
J_ball = J_Gball + m_ball*(d_ball^2);
J_load = J_arm + J_backplate + J_endplate + J_ball;
J_total = (n^2)*J_motor + J_load; %kg*m^2
alpha_rampmax = Tau_rated_Nm/J_total; %calculate max allowable acceleratio

for ballNumber = 1:length(XtargetList)
    tic
    ballPos = [20.25 -12.5 0];
    forcedArmAngle = 45;
    initialAngleOffset = 45;
    
    
    X_TARGET = XtargetList(ballNumber);
    
    markerPosition = X_TARGET + L; %(50*(ballNumber-1))+markerList(ballNumber)+L*100;
    %     X_TARGET = markerPosition/100
    
    %choose alpha_ramp
    acc_fraction = 0.9; % %we choose this, or can just insert our own alpha_ramp
    if(X_TARGET > 1.39)
        acc_fraction = 0.3; % Needs more time to ramp up
    end
    alpha_ramp = acc_fraction*alpha_rampmax; %choose our acceleration within the allowable range
    
    %CALCULATIONS
    X_total = X_TARGET + L + (Lprime*cos(theta_ret) - r*cos(phi)); %total distance ball most travel upon leaving the catapult
    
    c = y_pivot + Lprime*sin(phi) + r*(sin(phi) - 1);
    a = 0.5*a_c;
    syms V %phi y_pivot Lprime r a_c
    eqn1 = V*cos(phi)*[(-1*V*sin(phi) - sqrt(((V*sin(phi))^2) - 4*a*c))/(2*a)] == X_total; %a*x^2 + b*x + c == 0
    eqn2 = V > 0;
    eqns = [eqn1 eqn2]; %solving for initial velocity (positive)
    S = solve(eqns, V, 'ReturnConditions', true);
    S.V;
    
    W_cruise_raw = (S.V)/Lprime;
    calculated_W_cruise = round(W_cruise_raw, 4); %rounds W_cruise to 4 decimal places
    
    calculated_T_ramp = calculated_W_cruise/alpha_ramp;
    
    calculated_T_ret = (theta_ret - theta_0 - (0.5*alpha_ramp*(calculated_T_ramp^2)) + calculated_W_cruise*calculated_T_ramp)/calculated_W_cruise;
    
    %OUTPUT
    calculated_W_cruise = double(calculated_W_cruise);
    calculated_T_ramp = double(calculated_T_ramp);
    calculated_T_ret = double(calculated_T_ret);
    
    fprintf("\nTARGET #%d: %.2fm    W_cruise: %.2frad/s  T_ramp: %.3fs    T_ret: %.3fs",...
    ballNumber,X_TARGET,calculated_W_cruise,calculated_T_ramp,calculated_T_ret); %NOTE: Reported target is offset by L to account for origin at end of arm
    
    %RUN MODEL WITH CALCULATED VALUES
    if(maxHeightFlag)
        calculated_W_cruise = 17.0;
        calculated_T_ramp = 0.012;
        calculated_T_ret = 0.07;
    end
    
    w=warning('off','all');
    sim('controller_8_9_2021_final');
    warning(w);
    totalTime = totalTime + toc;
    timeElapsed(ballNumber) = toc;
    fprintf("\nThrow simulated in %.6f seconds\n",timeElapsed(ballNumber));
    
    ballx= ans.ballX;
    bally = ans.ballY;
    ballt = ans.tout;
    
    %power calculation & output
    integratedPower = ans.motorEnergy; %copying array so it doesn't get deleted
    simTime = ballt(end); %total simulated time
    
    totalEnergy = integratedPower(end);
    avgPower = totalEnergy/simTime;

    fprintf("The total energy output is %.4f J; the average power is %.4f W. \n",totalEnergy,avgPower);
    
    X_pos = ballx *(-1) - ballPos(1)/100;
    Y_pos = bally + ballPos(2)/100- 0.0315 - 0.005;
    
    changesignindex1= find(diff(Y_pos>=00.0000001),1);
    changesignindex2 = round(changesignindex1*1.03);
    changesignindex3= find(Y_pos(changesignindex2:end)<=00.00000001,1);
    
    ballt(changesignindex2+changesignindex3);
    
    % Position debugging
    if ballNumber == 1
       
        fprintf("The first ball landed %.4f cm away from the target in %.4f seconds. \n",abs(markerPosition - X_pos(changesignindex2+changesignindex3))*100, ballt(changesignindex2+changesignindex3));

    elseif ballNumber == 2
        fprintf("The second ball landed %.4f cm away from the target in %.4f seconds. \n",abs(markerPosition - X_pos(changesignindex2+changesignindex3))*100, ballt(changesignindex2+changesignindex3));
    else
        fprintf("The third ball landed %.4f cm away from the target in %.4f seconds. \n",abs(markerPosition - X_pos(changesignindex2+changesignindex3))*100, ballt(changesignindex2+changesignindex3));
    end
    
    
end


%Final throw prints Max Distance
if(maxDistanceFlag)
    fprintf("\nThe ball was launched %.4f metres.\n",X_pos(changesignindex2+changesignindex3)-L)
end
%Final throw prints Max Height
if(maxHeightFlag)
    fprintf("\nThe max height was %.4f metres\n", max(Y_pos))
end


fprintf("\n\n Total Elapsed time is %.6f seconds\n",totalTime);
