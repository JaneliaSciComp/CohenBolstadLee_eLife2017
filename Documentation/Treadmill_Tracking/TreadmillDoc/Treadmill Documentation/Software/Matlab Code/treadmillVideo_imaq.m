function treadmillVideo_imaq(varargin)
%Fly Treadmill Video Acquisition Example using IMAQ adaptor interface
% The imaq adaptor seems to be more reliable than the Virtual COM port
% driver interface
%
% TREADMILLVIDEO_IMAQ comprehensive version of treadmill camera visualization tool
% using our custom IMAQ adaptor.
%
% This software tool allows a user to see the 30x30 pixel field of view
% from each optical flow camera to assist in calibration, focusing,
% illumination, and alignment.  The update rate is 20Hz and is
% approximately as fast as the chips will offload their image data.  This
% is for experimental setup only.
%
% The user can use the draggable horizontal and vertical line overlay to align their
% cameras to a target, and the algorithm will implement sub-pixel
% interpolation for a higher resolution image through the camera if
% desired.  The software also provides a histogram of the 900 pixels for
% lighting assistance.
% 
% May 19, 2010
% Gus K. Lott III, PhD (lottg@janelia.hhmi.org)
% Developed at HHMI - Janelia Farm Research Campus
%
devID = 0; %FTDI Device ID

% uncomment the next line to initially register the treadmill IMAQ interface
% imaqregister(fullfile(pwd,'treadmill.dll'))

%Cleanup from previously opened instances
imaqreset
delete(findobj('tag','gvid'))

%Create UI and image to dump values into
gui.fig=figure('tag','gvid','numbertitle','off','menubar','none','name','Fly Treadmill Video Monitor - Gus K Lott III, PhD - HHMI JFRC 2008');
centerfig(gui.fig)
gui.im=image(zeros(30,61)); 
set(gui.im,'xdata',[0 60],'ydata',[0 29])
colormap gray
gui.ax=get(gui.im,'parent');
axis equal
axis fill
set(gca,'position',[0 0 1 1])
set(gui.ax,'visible','off')

%connect to the camera
vi = videoinput('treadmill',devID);
vi.framespertrigger = inf;
vi.framesacquiredfcn = {@baf,gui.fig};
vi.framesacquiredfcncount = 1;
set(vi.Source,'motionVideo',1);

%Create Overlay alignments
gui.h0Line = line([0-.5 29+.5],[15 15]-.5,'color','r','linestyle','-','marker','.');
gui.h1Line = line([31-.5 60+.5],[15 15]-.5,'color','b','linestyle','-','marker','.');
gui.v0Line = line([15 15]-.5,[-0.5 29.5],'color','r','linestyle','-','marker','.');
gui.v1Line = line([15 15]+31-.5,[-0.5 29.5],'color','b','linestyle','-','marker','.');
set([gui.h0Line,gui.h1Line,gui.v0Line,gui.v1Line],'buttondownfcn',{@dragAlignment,gui.fig})

gui.hist0ax = axes('position',[.05 .82 .4 .15],'xtick',[],'ytick',[],'box','on','xlim',[-.5 63.5],'ylim',[0 1],'color','none');
for i = 0:63
    gui.bar0(i+1) = patch([-.5 .5 .5 -.5]+i, [0 0 0 0],'r','edgecolor','none','parent',gui.hist0ax);
end
gui.hist1ax = axes('position',[.55 .82 .4 .15],'xtick',[],'ytick',[],'box','on','xlim',[-.5 63.5],'ylim',[0 1],'color','none');
for i = 0:63
    gui.bar1(i+1) = patch([-.5 .5 .5 -.5]+i, [0 0 0 0],'b','edgecolor','none','parent',gui.hist1ax);
end

%Create control button
gui.vidgo=uicontrol('style','toggle','backgroundcolor','r','value',0,'string','Start Video','units','normalized','position',[0.01 0.01 0.25 0.05],'callback',{@vidstartstop,gui.fig,vi});
gui.vidInterp=uicontrol('style','checkbox','backgroundcolor',get(gcf,'color'),'value',0,'string','Interpolate Pixels','units','normalized','position',[0.27 0.01 0.25 0.05]);
gui.alignXhairs=uicontrol('style','checkbox','backgroundcolor',get(gcf,'color'),'value',0,'string','Show Alignment Crosshairs','units','normalized','position',[0.27 0.07 0.3 0.05],...
    'value',1,'callback',{@switchAlignView,gui.fig});
gui.statBox = uicontrol('style','text','backgroundcolor',[.7 .7 .7],'units','normalized','position',[0.6 0.01 0.38 0.15],'horizontalalignment','left');
set(gui.fig,'deletefcn',{@fdel,vi})




set(gui.fig,'userdata',gui)

function vidstartstop(obj,event,fig,vi)
gui=get(fig,'userdata');
switch get(obj,'value')
    case 0 %Stop Video
        stop(vi)
        set(obj,'backgroundcolor','r','string','Start Video');
        flushdata(vi);
    case 1 %start Video
        set(obj,'backgroundcolor','g','string','Stop Video');
        flushdata(vi)
        set(gui.im,'visible','on')
        start(vi)
end

function fdel(obj,event,vi)
try stop(vi); end
delete(vi)
imaqreset
delete(obj);

function switchAlignView(obj,event,fig)
gui = get(fig,'userdata');
switch get(obj,'value')
    case 0
        set([gui.h0Line,gui.h1Line,gui.v0Line,gui.v1Line],'visible','off')
    case 1
        set([gui.h0Line,gui.h1Line,gui.v0Line,gui.v1Line],'visible','on')
        set(gui.h0Line,'xdata',[0-.5 29+.5],'ydata',[15 15]-.5)
        set(gui.h1Line,'xdata',[31-.5 60+.5],'ydata',[15 15]-.5)
        set(gui.v0Line,'xdata',[15 15]-.5,'ydata',[-0.5 29.5])
        set(gui.v1Line,'xdata',[15 15]+31-.5,'ydata',[-0.5 29.5])
end


function dragAlignment(obj,event,fig)
set(fig,'windowbuttonmotionfcn',{@moveLine,fig,obj})
set(fig,'windowbuttonupfcn','set(gcbf,''windowbuttonmotionfcn'','''')');

function moveLine(obj,event,fig,h)
gui = get(fig,'userdata');
pos = get(gui.ax,'currentpoint');



switch h
    case gui.h0Line
        if (pos(1,2) < -.5) | (pos(1,2)>29.5); return; end
        set(gui.h0Line,'ydata',[1 1]*pos(1,2))
    case gui.h1Line
        if (pos(1,2) < -.5) | (pos(1,2)>29.5); return; end
        set(gui.h1Line,'ydata',[1 1]*pos(1,2))
    case gui.v0Line
        if (pos(1,1) < -.5) | (pos(1,1)>29.5); return; end
        set(gui.v0Line,'xdata',[1 1]*pos(1,1))
    case gui.v1Line
        if (pos(1,1) < -.5+31) | (pos(1,1)>29.5+31); return; end
        set(gui.v1Line,'xdata',[1 1]*pos(1,1))
end

%Fires every 1800 bytes (one frame from each camera).  MCU sends frames at 20Hz
function baf(obj,event,fig)
gui=get(fig,'userdata');

%Details of camera data stream are in the ADNS-6090 datasheet.  The raw
%pixel dump data streams are interleaved pixel for pixel when returned to
%Matlab.

%Grab a frame
raw=double(getdata(obj,1));

%remove bit 7 from all bytes.  Remove bit 6 from first pixel
raw0=raw(:,1:2:end)-128;
raw1=raw(:,2:2:end)-128;

raw0(1)=raw0(1)-64;
raw1(1)=raw1(1)-64;

raw0 = fliplr(raw0);
raw1 = fliplr(raw1);
raw0 = rot90(raw0,1);
raw1 = rot90(raw1,1);

n0 = hist(raw0(:),0:63)/900;
n0 = n0/max(n0);
n1 = hist(raw1(:),0:63)/900;
n1 = n1/max(n1);
for i = 1:64
    set(gui.bar0(i),'ydata',[0 0 1 1]*n0(i))
    set(gui.bar1(i),'ydata',[0 0 1 1]*n1(i))
end

if get(gui.vidInterp,'value')
    raw0 = interp2(raw0,2,'spline');
    raw1 = interp2(raw1,2,'spline');
end
set(gui.im,'cdata',[raw0, ones(size(raw0,1),1+3*get(gui.vidInterp,'Value'))*64*.8,raw1]) %Update graphics
set(gui.im,'xdata',[0 60],'ydata',[0 29])





