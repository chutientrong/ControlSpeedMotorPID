function varargout = test(varargin)
tic
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @test_OpeningFcn, ...
                   'gui_OutputFcn',  @test_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

function test_OpeningFcn(hObject, eventdata, handles, varargin)

% Choose default command line output for test
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = test_OutputFcn(hObject, eventdata, handles)


varargout{1} = handles.output;

function btnStart_Callback(hObject, eventdata, handles)

button_state = get(hObject,'Value'); % trang thai buttom start
if button_state == get(hObject,'Max')
    
    cla(handles.pictureBox1);
    clearvars time pPV time_out ;  
	set(handles.btnStart,'string','Stop')
    set(handles.btnStart,'BackgroundColor','red');

    PV=0;
    MV=0;
    error=0;
    time = now;
    startTime = clock;    
    pSetpoint = 0;
    pPV = 0;  % actual possition (Process Value)
    pInterval = 0;
    pError = 0;   % how much SP and PV are diff (SP - PV)
    pIntegral = 0; % curIntegral + (error * Delta Time)
    pDerivative = 0;  %error - prev error) / Delta time
    preError=0; % error from last time (previous Error)
    pKp = 1;      pKi = 1;    pKd = 1; % PID constant multipliers
    pDt = 0; % delta time
    pMV = 0; % the drive amount that effects the PV.
    preMVv=0;
    pNoisePercent = 0; % amount of the full range to randomly alter the PV
    pNoise = 0;  % random noise added to PV
    Vmotor=0;
    preVmotor=0;
    prepreVmotor=0;
    noisePercent=0;
    noise=0;
    lastderivative=0;
    preIntegral=0;
    preMV = 0;
    prePV = 0;
    minn = -1000;
    maxx=1000;
    Dt=0.1;
    lastError=0;
    timeInterval = 0.005; %Pause
    filter = 0.05;
    lastTime = now;
    ppPV=0;
    ppMV=0;
    ppSetpoint=0;
    outSYS=0;
    ppoutSYS=0;
    ppPVV=0;
    PVV=0;
    kp=0;
    ki=0;
    kd=0;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot khoi tao %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
%     plotHandle = plot(handles.pictureBox1,time,ppPV,'Marker','.','LineWidth',.25,'Color',[0 1 0]);
%     hold on;
% %     grid on;
%     plotHandle2 = plot(handles.pictureBox1,time,ppSetpoint,'Marker','.','LineWidth',.25,'Color','red');
%     plotHandle3 = plot(handles.pictureBox1,time,ppMV,'Marker','.','LineWidth',.22,'Color',[0 0 1]);
%     xlim(handles.pictureBox1,[max(time-0.005) max(time+0.0005)]);
%     
    plotHandle = plot(handles.pictureBox1,time,ppPV,'Marker','.','LineWidth',.25,'Color',[0 1 0]);
    hold on;
%     grid on;
    plotHandle2 = plot(handles.pictureBox1,time,ppSetpoint,'Marker','.','LineWidth',.25,'Color','red');
    plotHandle3 = plot(handles.pictureBox1,time,ppMV,'Marker','.','LineWidth',.22,'Color',[0 0 1]);
    xlim(handles.pictureBox1,[max(time-0.005) max(time+0.0005)]);
    ylim(handles.pictureBox1,[-10,1000]);    
    leg = legend('(PV)','(SP)','(OP)');
    legtxt = findobj(leg,'type','text');    
    lastKi=pKi;
    count = 1;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Khoi tao bo Motor  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     J=0.01; b=0.1; K=0.01; R=1; L=0.5;
    J = 0.01;
    b = 0.1;
    K = 0.01;Ke = K;Kt= K;
    R = 1;
    L = 0.5;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Vong lap %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
     while (1) %button_state == get(hObject,'Max')   
%     for i=1:1:10000
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Lay gia tri Setpoint %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

        pSetpoint =get(handles.trackBarSP,'value');   
        if(get(handles.trackBarSP,'Max') > maxx)
            pSetpoint = maxx;
            set(handles.lblSP,'String',num2str(pSetpoint));
        elseif(get(handles.trackBarSP,'Min') < minn)
            pSetpoint = minn;
            set(handles.lblSP,'String',num2str(pSetpoint));
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Lay gia tri Ts %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        pDt = get(handles.trackBarInterval,'value');
        if(get(handles.trackBarInterval,'value') > maxx)
            pDt = maxx;
            set(handles.lblInterval,'String',num2str(pDt));
        elseif(get(handles.trackBarInterval,'value') < minn)
            pDt = minn;
            set(handles.lblInterval,'String',num2str(pDt));
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Lay gia tri PID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

        pKp = str2double(get(handles.tbKp,'string'));
        pKi = str2double(get(handles.tbKi,'string'));  
        pKd = str2double(get(handles.tbKd,'string'));
        
        if (pKi~=lastKi)
            % Reset integrator
            pIntegral=0; %errSum = 0;  
            preMV = MV - pKp * error; %error=pError
        end        
        
%%%%%%%%%%%%%%%%%%5%%%%%%%%%%%%%% Tinh noise  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if get(handles.cbNoise,'Value') == 1
            pNoise= get(handles.slideNoisePersent,'value');
            set(handles.nudNoisePercent,'String',num2str(pNoise));
            r = rand;    
            pNoisePercent = (str2double(get(handles.nudNoisePercent,'string'))) / 100.0;
            noise = (100 * pNoisePercent) * (r - 0.5);
        else
            noise = 0;
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%% time to display %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        time(count) = datenum(clock);
        Dtt(count)=Dt;
        time_out(count) = etime(clock,startTime);

%         previous_error = 0 
%         integral = 0 
%         start: 
%         error = setpoint - actual_position 
%         integral = integral + (error*dt) 
%         derivative = (error - previous_error)/dt 
%         output = (Kp*error) + (Ki*integral) + (Kd*derivative) 
%         previous_error = error 
%         wait(dt) 
%         goto start

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Tinh He thong %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Dt = (now - lastTime)*3600*24;  
        pDt=pDt/100;
        lastTime = now;

        pIntegral = preIntegral + (error * pDt);
%         dErr = (error - preError) / Dt;
        derivative_raw = (error - preError) / pDt;
%         pDerivative = (error - preError) / pDt;
        pDerivative = filter * derivative_raw + (1 - filter) * lastderivative;

%             PV = PV + (output * pDt) -(PV * pDt)+ noise;
%%xxxxxxxxx%%% preV

        error = pSetpoint - Vmotor;
        
        A=((2*J*L)/pDt^2)+((J*L+b*L)/pDt);
        B=(J*L/pDt^2)+((J*L+b*L)/pDt)+b*R+K^2;
        
        Vmotor=((K*error)+preVmotor*A-prepreVmotor*(J*L/(pDt^2)))/B;
        MV = preMV + (pKp * error) + (pKi * pIntegral) + (pKd * pDerivative);   
        PV = MV + Vmotor;%prePV + MV + Vmotor;
        disp([
            ' (MV): ' num2str(MV,'%3.0f') ...
            ' (PV): ' num2str(PV,'%6.2f') ...
            ' Sp: ' num2str(pSetpoint,'%6.2f')]);
        
                 disp([' Motor: ' num2str(Vmotor,'%6.2f')]);
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         if isnan(MV)
%             MV = 0;
%         end
        if MV > maxx
            MV = maxx;
        end
        if MV < minn
            MV = minn;
        end
        
        if isnan(PV)
            PV = 0;
        end
        if PV > maxx
            PV = maxx;
        end
        if PV < minn
            PV = minn;
        end
        
%         if PV > maxx
%             PV = maxx;
%         end
%         if PV < minn
%             PV = minn;
%         end
        % Anti-reset Wind-up
        if (MV<=minn),
            pIntegral = pIntegral - (pKi*error * pDt);
        end
        if (MV>=maxx),
            pIntegral = pIntegral - (pKi*error * pDt);              
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Lam` tron` %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        MV = round(MV);
        PV=round(PV);
        pSetpoint=round(pSetpoint);
%         outSYS=round(outSYS);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Dem de Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        preError = error;
        prepreVmotor=preVmotor;
        preVmotor=Vmotor;
        prePV=PV;
        preMVv=MV;
        lastderivative = pDerivative;  
        preIntegral=pIntegral;
        ppMV(count) = MV;   
        ppPV(count)=PV;
        ppSetpoint(count)= pSetpoint;

%         ppoutSYS(count)=outSYS;


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Show gia tri %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         disp([
%             ' (OP): ' num2str(MV,'%3.0f') ...
%             ' (PV): ' num2str(PV,'%6.2f') ...
%             ' Sp: ' num2str(pSetpoint,'%6.2f')]);
%           disp([' DT: ' pDt]);
          
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Truyen gia tri vao GUI %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         set(plotHandle,'YData',ppPV,'XData',time);
        set(plotHandle,'YData',ppPV,'XData',time);
        set(plotHandle2,'YData',ppSetpoint,'XData',time);
        set(plotHandle3,'YData',ppMV,'XData',time);
        set(handles.pictureBox1,'xlim',[max(time)-.00005 max(time)+.00001]);
        
        pNoise= get(handles.slideNoisePersent,'value');
        set(handles.nudNoisePercent,'String',num2str(pNoise));
        set(handles.lblError,'string',num2str(error))
        set(handles.lblIntegral,'string',num2str(pIntegral))
        set(handles.lblDeriv,'string',num2str(pDerivative))    
        set(handles.lblOutput,'string',num2str(round(MV)))
        set(handles.lblPV,'string',num2str(round(PV)))

        pPV = str2double(get(handles.lblPV,'string')); 
        set(handles.progBarPV,'value',round(pPV))
        pMV = str2double(get(handles.lblOutput,'string'));
        set(handles.progBarOut,'value',round(pMV))
        pSetpoint = num2str(get(handles.trackBarSP,'value')); 
        set(handles.lblSP,'string',pSetpoint);

        kp = get(handles.tbKp,'string');
        ki = get(handles.tbKi,'string');            
        kd = get(handles.tbKd,'string');
        set(handles.tbKp,'string',kp);
        set(handles.tbKi,'string',ki);
        set(handles.tbKd,'string',kd);       

        datetick('x','HH:MM:SS','keeplimits');        
        pause(timeInterval);
%         DT
        count = count +1;
        % save values
        lastKi = pKi; 
        drawnow;

%         if get(handles.btnStart,'Userdata')
%              clc            % close if the EXIT button has been pressed
% %              close all
%              clear all
%              % or use break
%             continue
%         end
%       end
    end 
        set(handles.pictureBox1,'xlim',[min(time)-.01 max(time)+.01]);
        pause(.01);
        
elseif button_state == get(hObject,'Min')
	set(handles.btnStart,'string','Start')
    set(handles.btnStart,'BackgroundColor','green');
    uiwait;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function btnStart_CreateFcn(hObject, eventdata, handles)

function btnStop_Callback(hObject, eventdata, handles)

if ~strcmp(get(handles.btnStart,'String'),'Stop')
    set(handles.btnStart, 'String', 'Start');
    
   clc            % close if the plotting has been stop in advance
else
   set(handles.btnStart,'Userdata',1)
end
 cla(handles.axes1);

function tbKp_Callback(hObject, eventdata, handles)
% hObject    handle to tbKp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

kp = get(handles.tbKp,'string');
set(handles.tbKp,'string',kp)
% --- Executes during object creation, after setting all properties.
function tbKp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbKp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tbKi_Callback(hObject, eventdata, handles)
% hObject    handle to tbKi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%pKi=0.01;
ki = get(handles.tbKi,'string');
set(handles.tbKi,'string',ki)
% --- Executes during object creation, after setting all properties.
function tbKi_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbKi (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function tbKd_Callback(hObject, eventdata, handles)
% pKd=1;
kd = get(handles.tbKd,'string');
set(handles.tbKd,'string',kd)

function tbKd_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tbKd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function cbNoise_Callback(hObject, eventdata, handles)
%
% --- Executes on slider movement.
function trackBarSP_Callback(hObject, eventdata, handles)
% hObject    handle to trackBarSP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pSetpoint = get(handles.trackBarSP,'Value');
set(handles.lblSP,'string',num2str(round(pSetpoint)))

function trackBarSP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to trackBarSP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function trackBarInterval_Callback(hObject, eventdata, handles)
% hObject    handle to trackBarInterval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pDt = get(handles.trackBarInterval,'Value');
set(handles.lblInterval,'string',num2str(round(pDt)))

function trackBarInterval_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function progBarPV_Callback(hObject, eventdata, handles)
% hObject    handle to progBarPV (see GCBO)

pPV = round(get(handles.progBarPV,'value'));
set(handles.lblPV,'string',num2str(round(pPV)))
% set(handles.pPV,'string',num2str(round(lblPV)))
% --- Executes during object creation, after setting all properties.
function progBarPV_CreateFcn(hObject, eventdata, handles)
% hObject    handle to progBarPV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% --- Executes on slider movement.
function progBarOut_Callback(hObject, eventdata, handles)
% hObject    handle to progBarOut (see GCBO)
pOutput = get(handles.progBarOut,'value');
set(handles.lblOutput,'string',num2str(round(pOutput)))
% set(handles.pPV,'string',num2str(round(lblPV)))
% --- Executes during object creation, after setting all properties.
function progBarOut_CreateFcn(hObject, eventdata, handles)
% hObject    handle to progBarOut (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function slideNoisePersent_Callback(hObject, eventdata, handles)
% hObject    handle to slideNoisePersent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
            pNoise= get(handles.slideNoisePersent,'value');
            set(handles.nudNoisePercent,'String',num2str(pNoise));
% --- Executes during object creation, after setting all properties.
function slideNoisePersent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slideNoisePersent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function nudNoisePercent_Callback(hObject, eventdata, handles)
% hObject    handle to nudNoisePercent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
            pNoise= get(handles.slideNoisePersent,'value');
            set(handles.nudNoisePercent,'String',num2str(pNoise));
% --- Executes during object creation, after setting all properties.
function nudNoisePercent_CreateFcn(hObject, eventdata, handles)
% hObject    handle to nudNoisePercent (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% --- Executes during object deletion, before destroying properties.
function pictureBox1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to pictureBox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes during object creation, after setting all properties.
function lblSP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblSP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes on slider movement
% --- Executes during object creation, after setting all properties.
function lblError_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblError (see GCBO)
% even all CreateFcns called
% --- Executes during object creation, after setting all properties.
function lblIntegral_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblIntegral (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes during object creation, after setting all properties.
function lblDeriv_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblDeriv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes during object creation, after setting all properties.
function lblPV_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblPV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns calle
% --- Executes during object creation, after setting all properties.
function lblOutput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblOutput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% --- Executes during object creation, after setting all properties.
function lblInterval_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lblInterval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


