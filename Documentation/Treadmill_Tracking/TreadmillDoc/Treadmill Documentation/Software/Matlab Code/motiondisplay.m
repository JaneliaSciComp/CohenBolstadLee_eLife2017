function motiondisplay(varargin)
% Fly Treadmill Motion Acquisition Demo Program - Virtual COM port 
%
% MOTIONDISPLAY controls the data stream from the Lott/Jayaraman treadmill
% system under the assumption that the cameras are 45 degrees off the back
% of the target ball and centered at the equator.  It will either show raw
% independent traces of the x/y motion from each camera or convert the x/y
% motion to rotational translation of the ball through vector combinations.
%
% This "COM" port code uses the virtual com port driver interface provided
% by FTDI (www.ftdichip.com).  In our WindowsXP applications, we found that
% these drivers were not reliable over long periods in that windows seemed
% to be dropping data (treating it as a low priority process).  We
% subsequently went with a solution that uses a custom IMAQ toolbox adaptor
% for Matlab, but this solution requires extra toolbox purchase whereas
% serial is a base matlab function.
%
% The IMAQ adaptor also greatly simplifies the data structure. It 
% parses it into a 2D array where each row is a 12-byte packet (a single
% sample)
% 
% Set the COM port in the first line of code of this function
%
% May 10, 2010
% Gus K. Lott III, PhD (lottg@janelia.hhmi.org)
% Developed at HHMI - Janelia Farm Research Campus
% 
%
% 3D sphere generation and rotation code taken from Bruce Land's web site
% http://www.nbb.cornell.edu/neurobio/land/PROJECTS/Hierarchy/
%
COMport = 'COM3';

%Cleanup any open serial interfaces or instances of this program
delete(findobj('tag','gmot'))
delete(instrfind)

%Create GUI and plots
gui.fig=figure('tag','gmot','numbertitle','off','menubar','none','name','Fly Treadmill Position Track - Gus K Lott III, PhD - HHMI JFRC 2008');
gui.pl0=plot(0,0);
hold on
gui.head0=plot(0,0,'r*');
gui.pl1=plot(0,0);
hold on
gui.head1=plot(0,0,'g*');

%Axis for independent camera traces
gui.ax=get(gui.pl0,'parent');
set(gui.ax,'xtick',[0],'ytick',[0],'xticklabel',[],'yticklabel',[],'position',[.1 .1 .8 .85])
grid on
axis off

%Axis for Sphere
gui.sphereax=axes('position',get(gui.ax,'position'));
axis off
gui.sphere=UnitSphere(10);
gui.a=patch(gui.sphere);
set(gui.a,'facecolor','w')
daspect([1 1 1])

%Set the viewing angle on the ball.  First argument is Azimuth (around the equator) and the second is elevation angle
view(180,0)

%Specify UI for Quality and Shutter Speed bar graphs
gui.q0=axes('xtick',[],'ytick',[],'position',[.01 .15 .04 .8],'xlim',[0 1],'ylim',[0 1],'box','on');
gui.p0=patch([0 1 1 0],[0 0 0 0],'r');
title('Q0')
gui.shut0=uicontrol('style','text','units','normalized','string','','position',[.01 .1 .08 .04]);
gui.s0=axes('xtick',[],'ytick',[],'position',[.05 .15 .04 .8],'xlim',[0 1],'ylim',[0 1],'box','on');
title('S0')
gui.sp0=patch([0 1 1 0],[0 0 0 0],[.8 0 0]);
line([0 1],[1 1]*200/834,'color','k','linewidth',2)

gui.q1=axes('xtick',[],'ytick',[],'position',[.91 .15 .04 .8],'xlim',[0 1],'ylim',[0 1],'box','on');
gui.p1=patch([0 1 1 0],[0 0 0 0],'g');
title('Q1')
gui.shut1=uicontrol('style','text','units','normalized','string','','position',[.91 .1 .08 .04]);
gui.s1=axes('xtick',[],'ytick',[],'position',[.95 .15 .04 .8],'xlim',[0 1],'ylim',[0 1],'box','on');
title('S1')
gui.sp1=patch([0 1 1 0],[0 0 0 0],[0 .8 0]);
line([0 1],[1 1]*200/834,'color','k','linewidth',2)

%Open serial port interface
%__________________________________________________________________
s=serial(COMport);%<------------------------------------------------COM PORT DEFINITION
%------------------------------------------------------------------
s.baudrate=1250000;
s.bytesavailablefcn={@baf,gui.fig}; %function called whenever data is available
s.bytesavailablefcncount=2400; %12 bytes per sample, 4kSamples/sec, 48000 bytes per second.  20Hz=2400
s.bytesavailablefcnmode='byte';
s.inputbuffersize=50000;
fopen(s);

%Default to Vf,Vs,Om mode
fwrite(s,[246,0])

%Create start/stop button
gui.go=uicontrol('style','toggle','backgroundcolor','r','value',0,'string','Start Motion Tracking',...
    'units','normalized','position',[0.01 0.01 0.25 0.05],'callback',{@startstop,gui.fig,s});
gui.angle=uicontrol('style','toggle','value',1,'string','Rotational','units','normalized','position',[0.60 0.01 0.15 0.05],'callback',{@gAngle,gui.fig,s});
gui.inde=uicontrol('style','toggle','value',0,'string','Independent','units','normalized','position',[0.76 0.01 0.15 0.05],'callback',{@gAngle,gui.fig,s});

%File logging
gui.fname = uicontrol('style','edit','backgroundcolor','w','string','data.txt','horizontalalignment','left',...
    'units','normalized','position',[0.35 0.01 0.18 0.05],'callback','');
gui.log = uicontrol('style','checkbox','backgroundcolor',get(gui.fig,'color'),'string','Log',...
    'units','normalized','position',[0.5 0.01 0.1 0.05],'callback','');
gui.fid = [];

set(gui.fig,'deletefcn',{@fdel,s})

set(gui.fig,'userdata',gui)


%Handle switching between rotational and independent data streams
function gAngle(obj,event,fig,s)
gui=get(fig,'userdata');

switch obj
    case gui.angle
        fwrite(s,[246,0]) %command to set to angular mode
        set(gui.inde,'value',0)
        set(gui.angle,'value',1)
        set(gui.a,'visible','on')
        set(gui.ax,'visible','off')
    case gui.inde 
        fwrite(s,[246,1]) %Command to set to independent mode
        set(gui.angle,'value',0)
        set(gui.inde,'value',1)
        set(gui.a,'visible','off')
        set(gui.ax,'visible','on')
end

%Handle pushing the "Start Motion Tracking" button
function startstop(obj,event,fig,s)
gui=get(fig,'userdata');
switch get(obj,'value')
    case 0 %Stop Acquisition
        set(obj,'backgroundcolor','r','string','Start Motion Tracking');
        fwrite(s,[254,0]);
        set([gui.pl0 gui.pl1],'xdata',0,'ydata',0);
        set([gui.head0 gui.head1],'xdata',0,'ydata',0)
        if s.bytesavailable>0
            fread(s,s.bytesavailable); 
        end
        if get(gui.log,'value')
            fclose(gui.fid);
            gui.fid = [];
        end
        set([gui.fname gui.log],'enable','on')
    case 1 %start acquisition
        set(obj,'backgroundcolor','g','string','Stop Motion Tracking');
        set([gui.pl0 gui.pl1],'xdata',0,'ydata',0);
        set([gui.head0 gui.head1],'xdata',0,'ydata',0)
        if s.bytesavailable>0
            fread(s,s.bytesavailable); 
        end
        if get(gui.log,'value')
            gui.fid = fopen(get(gui.fname,'string'),'w');
        end
        set([gui.fname gui.log],'enable','off')
        fwrite(s,[255,0]);
end
set(gui.fig,'userdata',gui);

function fdel(obj,event,s)
fwrite(s,[254,0]);
fclose(s);
fclose('all');
delete(s);
delete(obj);


%Bytes Available Function
function baf(obj,event,fig)
% disp(datestr(now))
gui=get(fig,'userdata');
%Sometimes this event gets called with no data available... for whatever reason, if so, return
if obj.bytesavailable<obj.bytesavailablefcncount
    disp('returned')
    return
end

% pull the data out of the buffer
raw=fread(obj,obj.bytesavailablefcncount);

%12 bytes per sample, only the first byte of the packet is ever zero
zinds=find(raw==0);

%Check the input stream for appropriate packets.  If the stream contains malformed packets, then restart the interface
if sum(diff(zinds)~=12)>1|(obj.bytesavailablefcncount-zinds(end)+1)~=12
    disp('Packets Dropped, Resetting 0')
    fwrite(obj,[254,0])
    if obj.bytesavailable>0
            fread(obj,obj.bytesavailable); 
    end
    fwrite(obj,[255,0]);
    return
end

%Extract the data relative to the packet header indices
ind=raw(zinds+1);
% ind'
md=min(diff(ind));

%Respond to errors in the packet index.  If the packet count jumps more than 1 (255 to 1 counts as 1) then restart interface (missing packets)
if(max(diff(ind))>1)|~(md==1|md==-254)
    disp('Packets Dropped, Resetting 1')
    fwrite(obj,[254,0])
    if obj.bytesavailable>0
            fread(obj,obj.bytesavailable); 
    end
    fwrite(obj,[255,0]);
    return
end



%Raw Data, make motion data signed around zero
raw(zinds+2) = raw(zinds+2)-128;
raw(zinds+3) = raw(zinds+3)-128;
raw(zinds+4) = raw(zinds+4)-128;
raw(zinds+5) = raw(zinds+5)-128;

%if logging, write data to file
if get(gui.log,'value')
    out = reshape(raw,[12,200]);
    fprintf(gui.fid,'%d %d %d %d %d %d %d %d %d %d %d %d\n',out);
end

%Extract Motion Data from stream
x0=raw(zinds+2);
y0=raw(zinds+3);
x1=raw(zinds+4);
y1=raw(zinds+5);

%Camera Status Parameters
squal0=(raw(zinds+6)-1)/169;
squal1=(raw(zinds+7)-1)/169;
shut0=((raw(zinds+8)-1)*256+raw(zinds+9))/24; %Shutter Period microseconds
shut1=((raw(zinds+10)-1)*256+raw(zinds+11))/24; %Shutter Period

%Camera State Information (surface quality and shutter speed) update GUI
f0=round(mean(shut0));
f1=round(mean(shut1));
set(gui.shut0,'string',[num2str(f0),' us'])
set(gui.shut1,'string',[num2str(f1),' us'])
set(gui.p0,'ydata',[0 0 1 1]*mean(squal0))
set(gui.p1,'ydata',[0 0 1 1]*mean(squal1))
set(gui.sp0,'ydata',[0 0 1 1]*f0/834)
set(gui.sp1,'ydata',[0 0 1 1]*f1/834)

%Depending on Data Stream Content, rotate ball or draw traces for motion
switch get(gui.angle,'value')
    case 1 %update Ball orientation
        
        %******************************************************************
        %At this point, scale the vectors by sin(45) and 1/D respectively to get actual numbers
        %Also add scale transform due to camera field of view
        
        Vfwd=sum(x0); %integration rotation for the entire 1/20th of a second
        Vss=sum(y0);
        Omega=sum(x1);

        gui.sphere=rotateX(gui.sphere,Vfwd); %Carry out a coordinate transform about the axes where Y points in front of the fly, X points to the right of the fly, and Z points up above the fly
        gui.sphere=rotateY(gui.sphere,Vss);
        gui.sphere=rotateZ(gui.sphere,Omega);

        set(gui.a,'Vertices',gui.sphere.vertices,'Faces',gui.sphere.faces) %update 3D sphere

        set(gui.fig,'userdata',gui) %Store new vertices

    case 0
        %Grab old data
        xdat0=get(gui.pl0,'xdata');
        ydat0=get(gui.pl0,'ydata');
        newx0=zeros(size(x0))';
        newy0=zeros(size(y0))';

        xdat1=get(gui.pl1,'xdata');
        ydat1=get(gui.pl1,'ydata');
        newx1=zeros(size(x1))';
        newy1=zeros(size(y1))';

        %Integrate Velocity to get Position
        for i=1:length(x0)
            newx0(i)=xdat0(end)+sum(x0(1:i));
            newy0(i)=ydat0(end)+sum(y0(1:i));
        end

        for i=1:length(x1)
            newx1(i)=xdat1(end)+sum(x1(1:i));
            newy1(i)=ydat1(end)+sum(y1(1:i));
        end
        
        %Create new position vectors
        xdat0=[xdat0, newx0];
        ydat0=[ydat0, newy0];

        xdat1=[xdat1, newx1];
        ydat1=[ydat1, newy1];

        %Trim data to produce a tail that indicates velocity
        N=500;
        if length(ydat0)>N
            ydat0=ydat0(end-N:end);
            xdat0=xdat0(end-N:end);
        end
        if length(ydat1)>N
            ydat1=ydat1(end-N:end);
            xdat1=xdat1(end-N:end);
        end

        %Wrap motion around screen edges
        W=100;
        if xdat0(end)>W; xdat0(end)=nan; xdat0(end+1)=-W; ydat0(end+1)=ydat0(end); end
        if xdat0(end)<-W; xdat0(end)=nan; xdat0(end+1)=W; ydat0(end+1)=ydat0(end); end
        if ydat0(end)>W; ydat0(end)=nan; ydat0(end+1)=-W; xdat0(end+1)=xdat0(end); end
        if ydat0(end)<-W; ydat0(end)=nan; ydat0(end+1)=W; xdat0(end+1)=xdat0(end); end

        if xdat1(end)>W; xdat1(end)=nan; xdat1(end+1)=-W; ydat1(end+1)=ydat1(end); end
        if xdat1(end)<-W; xdat1(end)=nan; xdat1(end+1)=W; ydat1(end+1)=ydat1(end); end
        if ydat1(end)>W; ydat1(end)=nan; ydat1(end+1)=-W; xdat1(end+1)=xdat1(end); end
        if ydat1(end)<-W; ydat1(end)=nan; ydat1(end+1)=W; xdat1(end+1)=xdat1(end); end

        %Update Graphics
        set(gui.pl0,'xdata',xdat0,'ydata',ydat0,'linewidth',2)
        set(gui.pl1,'xdata',xdat1,'ydata',ydat1,'linewidth',2)
        set(gui.ax,'xlim',[-W W],'ylim',[-W W])
        set(gui.head0,'xdata',xdat0(end),'ydata',ydat0(end))
        set(gui.head1,'xdata',xdat1(end),'ydata',ydat1(end))
end

% drawnow


function sphere=UnitSphere(res)
    %Function by Bruce Land (Cornell University)
%unit sphere in a format consistent with hieracrhical 
%modeler
%The input paramenter is related to the sphere resolution.
%Range 1-10. Higher number is better approximation
%1=>octahedron
%1.5=> 44 faces
%2=> 100 faces
%2.5 => 188 faces
%3=> 296 faces
%5=> 900 faces
%10=>3600 faces

%range check
if (res>10)
   res=10;
elseif (res<1)
   res=1;
end

res=1/res;
[x,y,z]=meshgrid(-1-res:res:1+res, ...
   -1-res:res:1+res, -1-res:res:1+res);
w=sqrt(x.^2+y.^2+z.^2);
sphere=isosurface(x,y,z,w,1);


function objOut = rotateX(objIn,a)
    %Function by Bruce Land (Cornell University)
%hierarchical rotate function for structs and cell arrays
a=a/57.29;  %degrees to radians
if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;
      V=[V(:,1), ...
            cos(a)*V(:,2)-sin(a)*V(:,3), ...
            sin(a)*V(:,2)+cos(a)*V(:,3)];
      objOut{i}.vertices=V;   
   end      
 elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    V=[V(:,1), ...
            cos(a)*V(:,2)-sin(a)*V(:,3), ...
            sin(a)*V(:,2)+cos(a)*V(:,3)];
    objOut=objIn;
    objOut.vertices=V; 
 else
    error('input must be s struct or cell array')
 end %if  
 
 function objOut = rotateY(objIn,a)
    %Function by Bruce Land (Cornell University)
%hierarchical rotate function for structs and cell arrays
a=a/57.29;  %degrees to radians
if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;
      
      V=[cos(a)*V(:,1)+sin(a)*V(:,3), ...
            V(:,2), ...
            -sin(a)*V(:,1)+cos(a)*V(:,3)];
      
      objOut{i}.vertices=V;   
   end      
 elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    
    V=[cos(a)*V(:,1)+sin(a)*V(:,3), ...
            V(:,2), ...
            -sin(a)*V(:,1)+cos(a)*V(:,3)];

    objOut=objIn;
    objOut.vertices=V; 
 else
    error('input must be s struct or cell array')
 end %if   
 
 function objOut = rotateZ(objIn,a)
    %Function by Bruce Land (Cornell University)
%hierarchical rotate function for structs and cell arrays
a=a/57.29;  %degrees to radians
if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;
      
      V=[cos(a)*V(:,1)-sin(a)*V(:,2), ...
            sin(a)*V(:,1)+cos(a)*V(:,2), ...
            V(:,3)];
      
      objOut{i}.vertices=V;   
   end      
 elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    
    V=[cos(a)*V(:,1)-sin(a)*V(:,2), ...
            sin(a)*V(:,1)+cos(a)*V(:,2), ...
            V(:,3)];
      
    objOut=objIn;
    objOut.vertices=V; 
 else
    error('input must be s struct or cell array')
 end %if   

