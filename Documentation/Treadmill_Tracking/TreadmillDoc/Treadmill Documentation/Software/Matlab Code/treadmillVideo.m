function treadmillVideo(varargin)
%Fly Treadmill Video Acquisition Example
%
% TREADMILLVIDEO Simplified version of treadmill camera visualization tool
% using serial virtual com port interface.
%
% This software tool allows a user to see the 30x30 pixel field of view
% from each optical flow camera to assist in calibration, focusing,
% illumination, and alignment.  The update rate is 20Hz and is
% approximately as fast as the chips will offload their image data.  This
% is for experimental setup only.
%
% May 10, 2010
% Gus K. Lott III, PhD (lottg@janelia.hhmi.org)
% Developed at HHMI - Janelia Farm Research Campus
%
%
COMport = 'COM3';

%Cleanup from previously opened instances
delete(findobj('tag','gvid'))
delete(instrfind)
tic

%Create UI and image to dump values into
gui.fig=figure('tag','gvid','numbertitle','off','menubar','none','name','Fly Treadmill Video Monitor - Gus K Lott III, PhD - HHMI JFRC 2008');
centerfig(gui.fig)
gui.im=image(zeros(30,61)); 
colormap gray
gui.ax=get(gui.im,'parent');
axis equal
axis fill
set(gca,'position',[0 0 1 1])
set(gui.ax,'visible','off')

%Connect to serial object
s=serial(COMport);
s.baudrate=1250000;
s.inputbuffersize=50000;
fopen(s);

%Create control button
gui.vidgo=uicontrol('style','toggle','backgroundcolor','r','value',0,'string','Start Video','units','normalized','position',[0.01 0.01 0.25 0.05],'callback',{@vidstartstop,gui.fig,s});
set(gui.fig,'deletefcn',{@fdel,s})

set(gui.fig,'userdata',gui)

function vidstartstop(obj,event,fig,s)
gui=get(fig,'userdata');
switch get(obj,'value')
    case 0 %Stop Video
        set(obj,'backgroundcolor','r','string','Start Video');
        fwrite(s,[250,0]);
        pause(0.1);
        if s.bytesavailable>0
            fread(s,s.bytesavailable); 
        end
    case 1 %start Video
        set(obj,'backgroundcolor','g','string','Stop Video');
        fclose(s)
        s.bytesavailablefcn={@baf,fig};
        s.bytesavailablefcncount=900*2; %one frame of pixels from each camera
        s.bytesavailablefcnmode='byte';
        fopen(s)
        set(gui.im,'visible','on')
        fwrite(s,[251,0]);
end

function fdel(obj,event,s)
fwrite(s,[250,0]);
fclose(s);
delete(s);
delete(obj);

%Fires every 1800 bytes (one frame from each camera).  MCU sends frames at 20Hz
function baf(obj,event,fig)
gui=get(fig,'userdata');
if obj.bytesavailable<obj.bytesavailablefcncount
    disp('returned')  %error checking
    return
end
%Grab a frame
raw=fread(obj,obj.bytesavailablefcncount);
if obj.bytesavailable>5000
    return;
end
raw0=raw(1:2:end);
raw1=raw(2:2:end);

%remove bit 7 from all bytes.  Remove bit 6 from first pixel
raw0=raw0-128; raw0(1)=raw0(1)-64;
raw1=raw1-128; raw1(1)=raw1(1)-64;
b0=reshape(raw0,[30,30]); %Map linear vector into 30 colums by 30 rows
b1=reshape(raw1,[30,30]); %Map linear vector into 30 colums by 30 rows
set(gui.im,'cdata',[b0,ones(30,1)*64*.8,b1]) %Update graphics


% toc
% drawnow




