function varargout = untitled1(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled1_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

function untitled1_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = untitled1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function edit1_Callback(hObject, eventdata, handles)
edit1 = str2double(get(hObject, 'String'));
w = edit1;
guidata(hObject,handles)
handles.edit1=w;

function edit2_Callback(hObject, eventdata, handles)
edit2 = str2double(get(hObject, 'String'));
z = edit2;
guidata(hObject,handles)

function edit3_Callback(hObject, eventdata, handles)
edit3 = str2double(get(hObject, 'String'));
k = edit3;
guidata(hObject,handles)

%--- Executes on button press in Pc.
function Pc_Callback(hObject, eventdata, handles)
w=str2double(get(handles.edit1,'String'));
z=str2double(get(handles.edit2,'String'));
k=str2double(get(handles.edit3,'String'));

k=1;
syms t;
s=tf('s');
plant=w^2/(2*z*w*s+s^2);

int=log(-1+2*z^2+2*z*sqrt(z^2-1))/(2*sqrt(z^2-1)*w);
inx=k+(-1+2*z^2+2*z*sqrt(z^2-1))^((-z-sqrt(z^2-1))/(2*sqrt(z^2-1)))/(2*sqrt(z^2-1)*(z+sqrt(z^2-1)))-(-1+2*z^2+2*z*sqrt(z^2-1))^((-z+sqrt(z^2-1))/(2*sqrt(z^2-1)))/(2*sqrt(z^2-1)*(z-sqrt(z^2-1)));
ins=(-z-sqrt(-1+z^2))*(-1+2*z^2+2*z*sqrt(-1+z^2))^((-z-sqrt(-1+z^2))/(2*sqrt(-1+z^2)))*w/(2*sqrt(-1+z^2)*(z+sqrt(-1+z^2)))-(-z+sqrt(-1+z^2))*(-1+2*z^2+2*z*sqrt(-1+z^2))^((-z+sqrt(-1+z^2))/(2*sqrt(-1+z^2)))*w/(2*sqrt(-1+z^2)*(z-sqrt(-1+z^2)));

L=double(solve(inx+ins*(t-int)==0,t));
T=double(solve(inx+ins*(t-int)==k,t))-L;


Ti=Inf;
Td=0;

Kp=T/L;
Ki=Kp/Ti;
Kd=Kp*Td;

cont=Kp+Ki/s+Kd*s;
cp=cont*plant;

original_system=feedback(w^2/(2*z*w*s+s^2),1);
controlled_system=feedback(cp,1);


info=stepinfo(k*controlled_system);

cla reset
plot(handles.axes1,step(k*controlled_system));
grid on
hold on
plot(handles.axes1,step(k*original_system));

set(handles.ki,'String',num2str(Ki));
set(handles.kp,'String',num2str(Kp));
set(handles.kd,'String',num2str(Kd));

set(handles.settime,'String',num2str(info.SettlingTime));
set(handles.overshoot,'String',num2str(info.Overshoot));
set(handles.raisetime,'String',num2str(info.RiseTime));
% hObject    handle to Pc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in PIc.
function PIc_Callback(hObject, eventdata, handles)
w=str2double(get(handles.edit1,'String'));
z=str2double(get(handles.edit2,'String'));
k=str2double(get(handles.edit3,'String'));
k=1;

syms t;
s=tf('s');
plant=w^2/(2*z*w*s+s^2);

int=log(-1+2*z^2+2*z*sqrt(z^2-1))/(2*sqrt(z^2-1)*w);
inx=k+(-1+2*z^2+2*z*sqrt(z^2-1))^((-z-sqrt(z^2-1))/(2*sqrt(z^2-1)))/(2*sqrt(z^2-1)*(z+sqrt(z^2-1)))-(-1+2*z^2+2*z*sqrt(z^2-1))^((-z+sqrt(z^2-1))/(2*sqrt(z^2-1)))/(2*sqrt(z^2-1)*(z-sqrt(z^2-1)));
ins=(-z-sqrt(-1+z^2))*(-1+2*z^2+2*z*sqrt(-1+z^2))^((-z-sqrt(-1+z^2))/(2*sqrt(-1+z^2)))*w/(2*sqrt(-1+z^2)*(z+sqrt(-1+z^2)))-(-z+sqrt(-1+z^2))*(-1+2*z^2+2*z*sqrt(-1+z^2))^((-z+sqrt(-1+z^2))/(2*sqrt(-1+z^2)))*w/(2*sqrt(-1+z^2)*(z-sqrt(-1+z^2)));

L=double(solve(inx+ins*(t-int)==0,t));
T=double(solve(inx+ins*(t-int)==k,t))-L;



Ti=L/0.3;
Td=0;

Kp=0.9*T/L;
Ki=Kp/Ti;
Kd=Kp*Td;

cont=Kp+Ki/s+Kd*s;
cp=cont*plant;

original_system=feedback(w^2/(2*z*w*s+s^2),1);
controlled_system=feedback(cp,1);

original_response=step(k*original_system);
controlled_response=step(k*controlled_system);

info=stepinfo(k*controlled_system);

cla reset
plot(handles.axes1,step(k*controlled_system));
grid on
hold on
plot(handles.axes1,step(k*original_system));

set(handles.ki,'String',num2str(Ki));
set(handles.kp,'String',num2str(Kp));
set(handles.kd,'String',num2str(Kd));

set(handles.settime,'String',num2str(info.SettlingTime));
set(handles.overshoot,'String',num2str(info.Overshoot));
set(handles.raisetime,'String',num2str(info.RiseTime));


% --- Executes on button press in PIDc.
function PIDc_Callback(hObject, eventdata, handles)
w=str2double(get(handles.edit1,'String'));
z=str2double(get(handles.edit2,'String'));
k=str2double(get(handles.edit3,'String'));
k=1;
syms t;
s=tf('s');
plant=w^2/(2*z*w*s+s^2);


int=log(-1+2*z^2+2*z*sqrt(z^2-1))/(2*sqrt(z^2-1)*w);
inx=k+(-1+2*z^2+2*z*sqrt(z^2-1))^((-z-sqrt(z^2-1))/(2*sqrt(z^2-1)))/(2*sqrt(z^2-1)*(z+sqrt(z^2-1)))-(-1+2*z^2+2*z*sqrt(z^2-1))^((-z+sqrt(z^2-1))/(2*sqrt(z^2-1)))/(2*sqrt(z^2-1)*(z-sqrt(z^2-1)));
ins=(-z-sqrt(-1+z^2))*(-1+2*z^2+2*z*sqrt(-1+z^2))^((-z-sqrt(-1+z^2))/(2*sqrt(-1+z^2)))*w/(2*sqrt(-1+z^2)*(z+sqrt(-1+z^2)))-(-z+sqrt(-1+z^2))*(-1+2*z^2+2*z*sqrt(-1+z^2))^((-z+sqrt(-1+z^2))/(2*sqrt(-1+z^2)))*w/(2*sqrt(-1+z^2)*(z-sqrt(-1+z^2)));

L=double(solve(inx+ins*(t-int)==0,t));
T=double(solve(inx+ins*(t-int)==k,t))-L;



Ti=2*L;
Td=0.5*L;

Kp=1.2*T/L;
Ki=Kp/Ti;
Kd=Kp*Td;

cont=Kp+Ki/s+Kd*s;
cp=cont*plant;

original_system=feedback(w^2/(2*z*w*s+s^2),1);
controlled_system=feedback(cp,1);

original_response=step(k*original_system);
controlled_response=step(k*controlled_system);

info=stepinfo(k*controlled_system);

cla reset
plot(handles.axes1,step(k*controlled_system));
grid on
hold on
plot(handles.axes1,step(k*original_system));

set(handles.ki,'String',num2str(Ki));
set(handles.kp,'String',num2str(Kp));
set(handles.kd,'String',num2str(Kd));

set(handles.settime,'String',num2str(info.SettlingTime));
set(handles.overshoot,'String',num2str(info.Overshoot));
set(handles.raisetime,'String',num2str(info.RiseTime));
display(L)
display(T)




% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function noc_Callback(hObject, eventdata, handles)

w=str2double(get(handles.edit1,'String'));
z=str2double(get(handles.edit2,'String'));
k=str2double(get(handles.edit3,'String'));
k=1;
syms t;
s=tf('s');
plant=w^2/(2*z*w*s+s^2);


Kp=0;
Ki=0;
Kd=0;


original_system=feedback(w^2/(2*z*w*s+s^2),1);

original_response=step(k*original_system);

info=stepinfo(k*original_system);

cla reset
plot(handles.axes1,step(k*original_system));
grid on

set(handles.ki,'String',num2str(Ki));
set(handles.kp,'String',num2str(Kp));
set(handles.kd,'String',num2str(Kd));

set(handles.settime,'String',num2str(info.SettlingTime));
set(handles.overshoot,'String',num2str(info.Overshoot));
set(handles.raisetime,'String',num2str(info.RiseTime));
