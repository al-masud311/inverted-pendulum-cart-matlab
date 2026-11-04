function invertedPendulumGUI(varargin)
%INVERTEDPENDULUMGUI  Cart-pendulum: swing-up from ANY angle + discrete PID balance.
%
%   invertedPendulumGUI
%       Opens the GUI. Set the initial angle (-180..+180 deg, 180 = hanging
%       straight down), optionally edit the PID gains, then press START.
%       The cart first pumps energy into the pendulum (swing-up) and, once the
%       bob is close to upright, hands over to the discrete PID controllers
%       which balance it above the cart and return the cart to x = 0.
%
%   invertedPendulumGUI('Name',Value,...)  (optional, mainly for batch use)
%       'InitialAngle'  initial pendulum angle in degrees (default 180)
%       'AutoStart'     true -> start the simulation immediately
%       'SnapshotTimes' simulation times [s] at which GUI PNG snapshots are saved
%       'SnapshotDir'   folder for the snapshots (default: folder of this file)
%       'Speed'         animation speed multiplier, 1 = real time
%       'CloseWhenDone' true -> close the figure when the run ends
%
%   SYSTEM
%       Cart  : M = 3 kg, slides horizontally only (force F applied to it)
%       Rod   : massless, L = 0.5 m, hinged on the cart
%       Bob   : point mass m = 1.5 kg at the end of the rod
%       theta : rod angle from the UPRIGHT vertical (positive = leaning to +x)
%
%   CONTROL (sample time Ts = 5 ms, zero-order hold on the force)
%       Mode 1  SWING-UP : energy-shaping law  a = k (Ep-Ed) sign(thd cos th)
%                          - kx x - kv xd, turned into a force through the
%                          nonlinear model (partial feedback linearisation).
%       Mode 2  BALANCE  : F = PID_angle(theta) + PID_position(x)
%                          u = Kp e + Ki Ts sum(e) + Kd (e[k]-e[k-1])/Ts
%       Hand-over when |theta| < 20.5 deg, |thdot| < 8 rad/s and the pendulum
%       energy is within 50 % of the upright value; falls back to swing-up if
%       |theta| ever exceeds 45 deg.
%
%   See InvertedPendulum_README.txt for details.

% ------------------------------------------------------------------ options
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
ip = inputParser;
addParameter(ip,'InitialAngle',180,@isnumeric);
addParameter(ip,'AutoStart',false,@islogical);
addParameter(ip,'SnapshotTimes',[],@isnumeric);
addParameter(ip,'SnapshotDir',here,@ischar);
addParameter(ip,'Speed',1,@isnumeric);
addParameter(ip,'CloseWhenDone',false,@islogical);
parse(ip,varargin{:});
opt = ip.Results;

% --------------------------------------------------------- physical constants
M    = 3.0;      % cart mass            [kg]
m    = 1.5;      % pendulum bob mass    [kg]
L    = 0.5;      % rod length           [m]
g    = 9.81;     % gravity              [m/s^2]
Ts   = 0.005;    % controller sample time [s]
Tsim = 8.0;      % simulated duration   [s]
Fmax = 1000;     % actuator saturation  [N]
THMAX = 180;     % largest allowed initial angle [deg]
BAND_TH = 0.5;   % "balanced" band      [deg]
BAND_X  = 0.02;  % "back at origin" band [m]

% swing-up controller (energy shaping) and hand-over conditions
SW.k    = 2;            % energy gain          [m/s^2 per J]
SW.amax = 10;           % cart acceleration limit during swing-up [m/s^2]
SW.kx   = 15;           % cart centring stiffness  [1/s^2]
SW.kv   = 2*sqrt(15);   % cart centring damping    [1/s]
SW.thIn = 20.5;         % hand-over angle      [deg]
SW.wIn  = 8;            % hand-over |thetadot| [rad/s]
SW.EIn  = 0.5;          % hand-over energy window (fraction of m g L)
SW.thOut = 45;          % fall back to swing-up beyond this angle [deg]

% default PID gains (balance mode)
def.KpT = 1941; def.KiT = 20; def.KdT = 413;    % angle PID
def.KpX = 1817; def.KiX = 50; def.KdX = 700;    % cart-position PID

% ------------------------------------------------------------- shared state
running = false;  stopFlag = false;  closePending = false;
simData = [];     curK = 1;
opt.InitialAngle = max(-THMAX,min(THMAX,opt.InitialAngle));

% drawing geometry
cartW = 0.34; cartH = 0.12; rw = 0.03; yb = 0.06; yp = yb + cartH; rb = 0.05;
colSwing = [0.85 0.25 0.15];  colBal = [0.20 0.70 0.30];

% ----------------------------------------------------------------------- GUI
fig = figure('Name','Inverted Pendulum on a Cart - Swing-up + Discrete PID', ...
    'NumberTitle','off','MenuBar','none','ToolBar','none','Resize','off', ...
    'Color',[0.97 0.97 0.98],'Units','pixels','Position',[60 40 1260 740], ...
    'CloseRequestFcn',@onClose);
movegui(fig,'center');

% --- animation axes
axA = axes('Parent',fig,'Units','normalized','Position',[0.04 0.32 0.53 0.64]);
hold(axA,'on'); box(axA,'on');
axis(axA,[-1.2 1.2 -0.75 0.85]); daspect(axA,[1 1 1]);
set(axA,'XTick',-1.2:0.4:1.2,'FontSize',10);
xlabel(axA,'Cart position x (m)'); title(axA,'Inverted pendulum on a cart');
plot(axA,[-1.2 1.2],[0 0],'-','Color',[0.25 0.25 0.25],'LineWidth',3);
plot(axA,[0 0],[-0.75 0.85],':','Color',[0.6 0.6 0.6]);              % x = 0 marker
hCart = rectangle('Parent',axA,'Position',[-cartW/2 yb cartW cartH], ...
    'FaceColor',[0.20 0.45 0.80],'EdgeColor','k','LineWidth',1.5);
hW1 = rectangle('Parent',axA,'Position',[-0.09-rw 0 2*rw 2*rw],'Curvature',[1 1], ...
    'FaceColor',[0.3 0.3 0.3],'EdgeColor','k');
hW2 = rectangle('Parent',axA,'Position',[ 0.09-rw 0 2*rw 2*rw],'Curvature',[1 1], ...
    'FaceColor',[0.3 0.3 0.3],'EdgeColor','k');
hRod = plot(axA,[0 0],[yp yp+L],'-','Color',[0.15 0.15 0.15],'LineWidth',3);
hBob = rectangle('Parent',axA,'Position',[-rb yp+L-rb 2*rb 2*rb],'Curvature',[1 1], ...
    'FaceColor',colSwing,'EdgeColor','k','LineWidth',1.5);
hPiv = plot(axA,0,yp,'ko','MarkerFaceColor','w','MarkerSize',6);
hTxt = text(axA,-1.15,0.80,'','FontSize',10,'VerticalAlignment','top', ...
    'BackgroundColor',[1 1 1],'EdgeColor',[0.7 0.7 0.7],'Margin',4);

% --- energy axes (KE, PE, total in ONE graph)
axE = axes('Parent',fig,'Units','normalized','Position',[0.65 0.66 0.33 0.29]);
hold(axE,'on'); grid(axE,'on'); box(axE,'on');
hKE = plot(axE,nan,nan,'-','Color',[0.85 0.15 0.15],'LineWidth',1.8);
hPE = plot(axE,nan,nan,'-','Color',[0.10 0.35 0.85],'LineWidth',1.8);
hET = plot(axE,nan,nan,'-','Color',[0 0 0],'LineWidth',2.2);
xlim(axE,[0 Tsim]); set(axE,'FontSize',10);
xlabel(axE,'Time (s)'); ylabel(axE,'Energy (J)');
title(axE,'Kinetic, potential and total energy');
legend(axE,[hKE hPE hET],{'Kinetic','Potential','Total'},'Location','northeast');

% --- state axes (angle and cart position)
axS = axes('Parent',fig,'Units','normalized','Position',[0.65 0.36 0.33 0.20]);
grid(axS,'on'); box(axS,'on'); set(axS,'FontSize',10);
yyaxis(axS,'left');
hTh = plot(axS,nan,nan,'-','Color',[0.85 0.33 0.10],'LineWidth',1.8);
ylabel(axS,'\theta (deg)'); axS.YAxis(1).Color = [0.85 0.33 0.10];
yyaxis(axS,'right');
hXc = plot(axS,nan,nan,'-','Color',[0 0.45 0.74],'LineWidth',1.8);
ylabel(axS,'x (cm)'); axS.YAxis(2).Color = [0 0.45 0.74];
xlim(axS,[0 Tsim]); xlabel(axS,'Time (s)');
title(axS,'Pendulum angle and cart displacement');

% --- control panel
pnl = uipanel('Parent',fig,'Title','Controls','FontSize',10,'FontWeight','bold', ...
    'Units','normalized','Position',[0.01 0.01 0.98 0.28]);
bg = get(pnl,'BackgroundColor');

mkText(pnl,'Initial angle (deg)  [+ = leans right]',[0.01 0.62 0.20 0.16],bg,'bold');
eAng = uicontrol(pnl,'Style','edit','Units','normalized','Position',[0.01 0.40 0.06 0.20], ...
    'String',sprintf('%.1f',opt.InitialAngle),'FontSize',11,'Callback',@onAngleEdit, ...
    'UserData',opt.InitialAngle);
sld = uicontrol(pnl,'Style','slider','Units','normalized','Position',[0.08 0.42 0.12 0.16], ...
    'Min',-THMAX,'Max',THMAX,'Value',opt.InitialAngle,'SliderStep',[1/360 10/360], ...
    'Callback',@onSlider);
mkText(pnl,sprintf('range: +/-%d deg  (180 = hanging down)',THMAX),[0.01 0.24 0.22 0.13],bg,'normal');

mkText(pnl,'ANGLE PID  (balance mode)',[0.24 0.80 0.28 0.16],bg,'bold');
eKpT = gainEdit(pnl,'Kp',[0.24 0.42 0.08 0.20],def.KpT);
eKiT = gainEdit(pnl,'Ki',[0.33 0.42 0.08 0.20],def.KiT);
eKdT = gainEdit(pnl,'Kd',[0.42 0.42 0.08 0.20],def.KdT);

mkText(pnl,'POSITION PID  (balance mode)',[0.53 0.80 0.30 0.16],bg,'bold');
eKpX = gainEdit(pnl,'Kp',[0.53 0.42 0.08 0.20],def.KpX);
eKiX = gainEdit(pnl,'Ki',[0.62 0.42 0.08 0.20],def.KiX);
eKdX = gainEdit(pnl,'Kd',[0.71 0.42 0.08 0.20],def.KdX);

btnStart = uicontrol(pnl,'Style','pushbutton','String','START','Units','normalized', ...
    'Position',[0.86 0.70 0.12 0.24],'FontSize',12,'FontWeight','bold', ...
    'BackgroundColor',[0.30 0.75 0.35],'Callback',@onStart);
btnStop = uicontrol(pnl,'Style','pushbutton','String','STOP','Units','normalized', ...
    'Position',[0.86 0.43 0.12 0.22],'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[0.90 0.40 0.35],'Enable','off','Callback',@onStop);
btnReset = uicontrol(pnl,'Style','pushbutton','String','Reset','Units','normalized', ...
    'Position',[0.86 0.22 0.055 0.18],'FontSize',10,'Callback',@onReset);
btnSnap = uicontrol(pnl,'Style','pushbutton','String','Snapshot','Units','normalized', ...
    'Position',[0.92 0.22 0.06 0.18],'FontSize',9,'Callback',@onSnapshot);
btnDef = uicontrol(pnl,'Style','pushbutton','String','Defaults','Units','normalized', ...
    'Position',[0.86 0.03 0.12 0.16],'FontSize',9,'Callback',@onDefaults);

hStatus = uicontrol(pnl,'Style','text','Units','normalized','Position',[0.01 0.02 0.83 0.20], ...
    'BackgroundColor',bg,'HorizontalAlignment','left','FontSize',10, ...
    'String','Ready. Set the initial angle and press START.');

editHandles = [eAng sld eKpT eKiT eKdT eKpX eKiX eKdX btnStart btnReset btnSnap btnDef];

resetView();
if opt.AutoStart
    drawnow;
    onStart([],[]);
    if opt.CloseWhenDone && ishandle(fig), delete(fig); end
end

% ============================================================ nested callbacks
    function onAngleEdit(src,~)
        v = str2double(get(src,'String'));
        if ~isfinite(v), v = get(src,'UserData'); end
        v = max(-THMAX,min(THMAX,v));
        set(src,'String',sprintf('%.1f',v),'UserData',v);
        set(sld,'Value',v);
        resetView();
    end

    function onSlider(src,~)
        v = round(get(src,'Value'));
        set(eAng,'String',sprintf('%.1f',v),'UserData',v);
        resetView();
    end

    function onDefaults(~,~)
        if running, return; end
        set(eKpT,'String',num2str(def.KpT),'UserData',def.KpT);
        set(eKiT,'String',num2str(def.KiT),'UserData',def.KiT);
        set(eKdT,'String',num2str(def.KdT),'UserData',def.KdT);
        set(eKpX,'String',num2str(def.KpX),'UserData',def.KpX);
        set(eKiX,'String',num2str(def.KiX),'UserData',def.KiX);
        set(eKdX,'String',num2str(def.KdX),'UserData',def.KdX);
    end

    function onStart(~,~)
        if running, return; end
        running = true; stopFlag = false;
        set(editHandles,'Enable','off'); set(btnStop,'Enable','on');
        try
            th0 = get(eAng,'UserData');
            G.KpT = get(eKpT,'UserData'); G.KiT = get(eKiT,'UserData'); G.KdT = get(eKdT,'UserData');
            G.KpX = get(eKpX,'UserData'); G.KiX = get(eKiX,'UserData'); G.KdX = get(eKdX,'UserData');
            simData = simulate(th0,G);          % discrete closed-loop simulation
            prepareAxes();
            set(hStatus,'String','Running ...');
            animate();
            if ~stopFlag && ishandle(fig)
                set(hStatus,'String',summaryText());
            elseif ishandle(fig)
                set(hStatus,'String',sprintf('Stopped at t = %.2f s.',simData.t(curK)));
            end
        catch ME
            running = false;
            if ishandle(fig)
                set(editHandles,'Enable','on'); set(btnStop,'Enable','off');
            end
            rethrow(ME);
        end
        running = false;
        if closePending && ishandle(fig), delete(fig); return; end
        if ishandle(fig)
            set(editHandles,'Enable','on'); set(btnStop,'Enable','off');
        end
    end

    function onStop(~,~)
        stopFlag = true;
    end

    function onReset(~,~)
        if running, stopFlag = true; return; end
        resetView();
        set(hStatus,'String','Ready. Set the initial angle and press START.');
    end

    function onSnapshot(~,~)
        if isempty(simData), tt = 0; else, tt = simData.t(curK); end
        saveSnap(sprintf('snapshot_manual_t%.2fs.png',tt));
    end

    function onClose(~,~)
        stopFlag = true;
        if running
            closePending = true;
        else
            delete(fig);
        end
    end

% =================================================================== helpers
    function resetView()
        simData = []; curK = 1;
        set([hKE hPE hET hTh hXc],'XData',nan,'YData',nan);
        th0 = deg2rad(get(eAng,'UserData'));
        drawPose(0,th0);
        set(hBob,'FaceColor',colSwing);
        set(hTxt,'String',sprintf('t = 0.000 s\n\\theta = %.1f deg\nx = 0.000 m\nmode: ready', rad2deg(th0)));
    end

    function prepareAxes()
        set([hKE hPE hET hTh hXc],'XData',nan,'YData',nan);
        allE = [simData.KE simData.PE simData.E];
        pad = 0.08*max(eps,max(allE)-min(allE));
        ylim(axE,[min(allE)-pad max(allE)+pad]);
        yyaxis(axS,'left');
        a = max(1,1.1*max(abs(simData.thdeg(~isnan(simData.thdeg)))));  ylim(axS,[-a a]);
        yyaxis(axS,'right');
        b = max(1,1.1*max(abs(100*simData.Z(1,:))));       ylim(axS,[-b b]);
    end

    function drawPose(x,th)
        set(hCart,'Position',[x-cartW/2 yb cartW cartH]);
        set(hW1,'Position',[x-0.09-rw 0 2*rw 2*rw]);
        set(hW2,'Position',[x+0.09-rw 0 2*rw 2*rw]);
        bx = x + L*sin(th);  by = yp + L*cos(th);
        set(hRod,'XData',[x bx],'YData',[yp by]);
        set(hBob,'Position',[bx-rb by-rb 2*rb 2*rb]);
        set(hPiv,'XData',x,'YData',yp);
    end

    function showFrame(k)
        curK = k;
        drawPose(simData.Z(1,k),simData.Z(3,k));
        if simData.mode(k) == 1
            ms = 'BALANCE (PID)';  set(hBob,'FaceColor',colBal);
        else
            ms = 'SWING-UP (energy)';  set(hBob,'FaceColor',colSwing);
        end
        set(hTxt,'String',sprintf('t = %.3f s\n\\theta = %.2f deg\nx = %.3f m\nF = %.1f N\nmode: %s', ...
            simData.t(k),simData.thdeg(k),simData.Z(1,k),simData.F(k),ms));
        idx = 1:k;
        set(hKE,'XData',simData.t(idx),'YData',simData.KE(idx));
        set(hPE,'XData',simData.t(idx),'YData',simData.PE(idx));
        set(hET,'XData',simData.t(idx),'YData',simData.E(idx));
        set(hTh,'XData',simData.t(idx),'YData',simData.thdeg(idx));
        set(hXc,'XData',simData.t(idx),'YData',100*simData.Z(1,idx));
    end

    function animate()
        N = numel(simData.t);
        pending = sort(opt.SnapshotTimes(:)');
        clk = tic;
        k = 1;
        while ishandle(fig) && ~stopFlag
            te = toc(clk)*opt.Speed;
            k = min(floor(te/Ts + 1e-9) + 1, N);
            showFrame(k);
            while ~isempty(pending) && te >= pending(1)
                kk = min(round(pending(1)/Ts) + 1, N);
                showFrame(kk);
                saveSnap(sprintf('snapshot_t%.2fs.png',pending(1)));
                pending(1) = [];
            end
            drawnow limitrate;
            if k >= N, break; end
        end
        if ishandle(fig) && ~stopFlag, showFrame(N); drawnow; end
    end

    function saveSnap(fname)
        drawnow;
        fr = getframe(fig);
        if ~exist(opt.SnapshotDir,'dir'), mkdir(opt.SnapshotDir); end
        fn = fullfile(opt.SnapshotDir,fname);
        imwrite(fr.cdata,fn);
        fprintf('Snapshot saved: %s\n',fn);
    end

    function S = simulate(th0deg,G)
        % Discrete closed loop: controller runs every Ts, force held constant
        % (zero-order hold) while the nonlinear plant is integrated with RK4.
        N = round(Tsim/Ts);
        z = [0; 0; deg2rad(th0deg); 0];            % [x; xdot; theta; thetadot]
        Z = zeros(4,N+1); Z(:,1) = z;
        F = zeros(1,N+1);  modeLog = zeros(1,N+1);
        Ed = m*g*L;                                % energy of upright, at rest
        mode = 0;                                  % 0 = swing-up, 1 = balance
        iT = 0; iX = 0;                            % integrator states
        eTp = wrapPi(z(3)); eXp = z(1);            % previous errors
        nsub = 4; h = Ts/nsub;
        for k = 1:N
            thw = wrapPi(z(3));  w = z(4);  x = z(1);  xd = z(2);
            Ep = 0.5*m*L^2*w^2 + m*g*L*cos(thw);   % pendulum energy (cart frame)

            % ---- mode supervisor
            if mode == 0
                if abs(rad2deg(thw)) < SW.thIn && abs(w) < SW.wIn && abs(Ep-Ed) < SW.EIn*Ed
                    mode = 1; iT = 0; iX = 0;      % hand over to PID, reset integrators
                end
            elseif abs(rad2deg(thw)) > SW.thOut
                mode = 0;
            end
            modeLog(k) = mode;

            % ---- discrete PID signals (errors = measurement - reference 0)
            eT = thw;  eX = x;
            dT = wrapPi(eT-eTp)/Ts;  dX = (eX-eXp)/Ts;   % discrete derivative
            eTp = eT;  eXp = eX;

            if mode == 1
                iT = iT + eT*Ts;  iX = iX + eX*Ts;       % discrete integral
                if G.KiT ~= 0, lim = 0.5*Fmax/abs(G.KiT); iT = max(-lim,min(lim,iT)); end
                if G.KiX ~= 0, lim = 0.5*Fmax/abs(G.KiX); iX = max(-lim,min(lim,iX)); end
                u = G.KpT*eT + G.KiT*iT + G.KdT*dT + G.KpX*eX + G.KiX*iX + G.KdX*dX;
            else
                sg = sign(w*cos(thw));  if sg == 0, sg = 1; end   % kick if at rest
                a = SW.k*(Ep-Ed)*sg - SW.kx*x - SW.kv*xd;         % desired cart accel.
                a = max(-SW.amax,min(SW.amax,a));
                u = (M+m*sin(thw)^2)*a - m*L*w^2*sin(thw) + m*g*sin(thw)*cos(thw);
            end
            u = max(-Fmax,min(Fmax,u));            % actuator limit
            F(k) = u;
            for s = 1:nsub
                k1 = plant(z,u);
                k2 = plant(z+h/2*k1,u);
                k3 = plant(z+h/2*k2,u);
                k4 = plant(z+h*k3,u);
                z = z + h/6*(k1+2*k2+2*k3+k4);
            end
            Z(:,k+1) = z;
        end
        F(N+1) = F(N);  modeLog(N+1) = mode;
        xd = Z(2,:); th = Z(3,:); w = Z(4,:);
        vbx = xd + L*cos(th).*w;                   % bob velocity components
        vby = -L*sin(th).*w;
        S.t  = (0:N)*Ts;  S.Z = Z;  S.F = F;  S.mode = modeLog;
        S.thw = wrapPi(th);
        S.thdeg = rad2deg(S.thw);                  % wrapped angle for plotting
        jmp = find(abs(diff(S.thdeg)) > 180);      % break the line at +/-180 wrap
        S.thdeg(jmp+1) = NaN;
        S.KE = 0.5*M*xd.^2 + 0.5*m*(vbx.^2 + vby.^2);
        S.PE = m*g*L*cos(th);                      % reference: hinge height
        S.E  = S.KE + S.PE;
    end

    function dz = plant(z,F)
        % Nonlinear cart-pendulum (massless rod, point-mass bob)
        th = z(3); w = z(4);
        xdd  = (F + m*L*w^2*sin(th) - m*g*sin(th)*cos(th)) / (M + m*sin(th)^2);
        thdd = (g*sin(th) - xdd*cos(th)) / L;
        dz = [z(2); xdd; w; thdd];
    end

    function s = summaryText()
        t  = simData.t;
        iH = find(simData.mode == 1,1);
        if isempty(iH), sH = 'PID never took over'; else, sH = sprintf('PID took over at t = %.2f s',t(iH)); end
        tT = settleTime(rad2deg(simData.thw),BAND_TH,t);
        tX = settleTime(simData.Z(1,:),BAND_X,t);
        s = sprintf(['%s;  pendulum within +/-%.1f deg for good after %.2f s;  cart within +/-%.0f cm of x = 0 for good after %.2f s.\n' ...
            'Peak |x| = %.1f cm,  peak |F| = %.0f N,  final theta = %.4f deg, final x = %.4f cm.'], ...
            sH,BAND_TH,tT,100*BAND_X,tX,100*max(abs(simData.Z(1,:))),max(abs(simData.F)), ...
            rad2deg(simData.thw(end)),100*simData.Z(1,end));
    end
end

% ================================================================ local funcs
function y = wrapPi(a)
y = atan2(sin(a),cos(a));
end

function tS = settleTime(sig,band,t)
i = find(abs(sig) > band,1,'last');
if isempty(i), tS = 0; elseif i >= numel(t), tS = NaN; else, tS = t(i+1); end
end

function h = mkText(par,str,pos,bg,weight)
h = uicontrol(par,'Style','text','String',str,'Units','normalized','Position',pos, ...
    'BackgroundColor',bg,'HorizontalAlignment','left','FontSize',10,'FontWeight',weight);
end

function h = gainEdit(par,label,pos,val)
bg = get(par,'BackgroundColor');
uicontrol(par,'Style','text','String',label,'Units','normalized', ...
    'Position',[pos(1) pos(2)+pos(4)+0.01 pos(3) 0.14],'BackgroundColor',bg,'FontSize',10);
h = uicontrol(par,'Style','edit','Units','normalized','Position',pos,'String',num2str(val), ...
    'FontSize',11,'UserData',val,'Callback',@gainCallback);
end

function gainCallback(src,~)
v = str2double(get(src,'String'));
if ~isfinite(v), v = get(src,'UserData'); end
set(src,'String',num2str(v,'%g'),'UserData',v);
end
