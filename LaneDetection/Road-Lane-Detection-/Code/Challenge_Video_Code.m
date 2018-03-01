
%%Create Output file for Challenge video output
writerObj = VideoWriter('Challenge_outputv19','MPEG-4');
open(writerObj);
%using below reference for color detection of yellow and white using HSV
%REference : https://www.mathworks.com/matlabcentral/fileexchange/28512-simplecolordetectionbyhue--
v = VideoReader('challenge_video.mp4');
 while hasFrame(v) 
%  thisFrame = read(v,426);

%%read eachframe
thisFrame = readFrame(v);
rgbImage = thisFrame;
%%covert each frame into hsv
hsvImage= rgb2hsv(thisFrame);
  
  hImage = hsvImage(:,:,1);
  sImage = hsvImage(:,:,2);
  vImage = hsvImage(:,:,3);
  
%% creat mask for yellow image 
  hueMasky = (hImage >= 0) & (hImage <= 0.15);
  saturationMasky = (sImage >= 0.25) & (sImage <= 1);
  valueMasky = (vImage >= 0.4) & (vImage <= 1);
  smallestAcceptableArea = 100;
  coloredObjectsMasky = uint8(hueMasky & saturationMasky & valueMasky);
  coloredObjectsMasky = uint8(bwareaopen(coloredObjectsMasky, smallestAcceptableArea));
%   imshow(coloredObjectsMasky,[]);
  coloredObjectsMasky = cast(coloredObjectsMasky, 'like', rgbImage);
  maskedImageR_y = coloredObjectsMasky .* rgbImage(:,:,1);
	maskedImageG_y = coloredObjectsMasky .* rgbImage(:,:,2);
	maskedImageB_y = coloredObjectsMasky .* rgbImage(:,:,3);
    maskedRGBImage_y = cat(3,maskedImageR_y, maskedImageG_y, maskedImageB_y);
    maskedRGBImage_y = im2bw(maskedRGBImage_y);


  %% Create Mask for white lane 
  hueMaskw = (hImage >= 0) & (hImage <= 1);
  saturationMaskw = (sImage >= 0) & (sImage <= 0.29);
  valueMaskw = (vImage >= 0.69) & (vImage <= 1);
  
  coloredObjectsMaskw = uint8(hueMaskw & saturationMaskw & valueMaskw);
  coloredObjectsMaskw = uint8(bwareaopen(coloredObjectsMaskw, smallestAcceptableArea));
%   imshow(coloredObjectsMaskw,[]);
  coloredObjectsMaskw = cast(coloredObjectsMaskw, 'like', rgbImage);
  maskedImageR_w= coloredObjectsMaskw .* rgbImage(:,:,1);
	maskedImageG_w = coloredObjectsMaskw .* rgbImage(:,:,2);
	maskedImageB_w = coloredObjectsMaskw .* rgbImage(:,:,3);
    maskedRGBImage_w = cat(3, maskedImageR_w, maskedImageG_w, maskedImageB_w);
    maskedRGBImage_w = im2bw(maskedRGBImage_w);

    %% Combine eachwhite and yellow masked images 
  maskedbinaryimage = maskedRGBImage_y + maskedRGBImage_w;

  %% Create your region of interest 
  
c = 1000*[0.5422;    0.2453 ;   0.7132;   1.1527 ;  0.7612 ;   0.5422];
r = [ 460.2500  ;701.7500;  713.7500  ;707.7500 ; 449.7500 ; 460.2500];
BW1 = roipoly(maskedbinaryimage,c,r);
roimage= BW1.*maskedbinaryimage;

% imshow(roimage);
%% HOugh lines and slope calculation 

[H,theta,rho] = hough(roimage); 
P = houghpeaks(H,30,'threshold',ceil(0.1*max(H(:)))) ; 
lines = houghlines(roimage,theta,rho,P,'FillGap',30,'MinLength',26); 

imshow(thisFrame), hold on
% imshow(roimage), hold on
x=1;y=1;rightpoints=[];leftpoints=[];
 t=1;u=1;
for m = 1:length(lines) 
    
    xy = [lines(m).point1;lines(m).point2];
%     plot(xy(:,1),(xy(:,2)),'LineWidth',3,'Color','yellow'); 
   
    line_slope(m) = (xy(1,2) - xy(2,2))/ (xy(1,1) - xy(2,1));

    if line_slope(m) > 0.2 
          if xy(1,1) > 724 && xy(2,1) > 724
%                plot(xy(:,1),(xy(:,2)),'LineWidth',3,'Color','blue');
                rightpoints(x,:)=lines(m).point1;
                rightpoints(x+1,:)=lines(m).point2;
                x=x+2;
                rightslope(t)=line_slope(m);t=t+1;
          end
    end
        
    if line_slope(m) < -0.2
         if xy(1,1) < 635 && xy(2,1) < 635
%               plot(xy(:,1),(xy(:,2)),'LineWidth',3,'Color','red'); 
             leftpoints(y,:)=lines(m).point1;
             leftpoints(y+1,:)=lines(m).point2;
             y=y+2;
             leftslope(u) = line_slope(m);u=u+1;
        end
    end

 
end
 
%% Curve fitting for right lane
if size(rightpoints) > 0
a=rightpoints(:,1);
b=rightpoints(:,2);
c= polyfit(a,b,1);
d = linspace(min(a),max(a));
e = polyval(c,d);
rightslopem = mean(rightslope);


       if (max(d) < 1006) && (min(d) > 658 ) && (min(e)>447)
       dq = linspace(700,1006);
       eq = interp1(d,e,dq,'linear','extrap');
       drawnow
       plot(dq,eq,'LineWidth',5,'Color','blue');
       else
       drawnow
       plot(d,e,'LineWidth',5,'Color','blue');
   end
   
    
rightlineslope = (e(end) - e(1))/ ( d(end) - d(1));
 
end 

%% Curve fitting for left lane
if size(leftpoints) > 0
a1=leftpoints(:,1);
b1=leftpoints(:,2);
c1= polyfit(a1,b1,1);
d1 = linspace(min(a1),max(a1));
e1 = polyval(c1,d1);
 
if (max(d1) < 670) && (min(d1) > 240)&& (min(e1)>455)
    
    dq1 = linspace(286,572);
    eq1 = interp1(d1,e1,dq1,'linear','extrap');
    drawnow
    plot(dq1,eq1,'LineWidth',5,'Color','red');
else 
    drawnow
    plot(d1,e1,'LineWidth',5,'Color','red');
end
 
 leftslopem= mean(leftslope);
 leftlineslope = (e1(end) - e1(1))/ ( d1(end) - d1(1));
end

%% Showing turns based on slopes , also print out if there are more number of lines, so disturbances. 
% Depending on noise present we take deciding factor as slopes of polyfit
% line or the slope from mean of slopes of all slopes of detected hough lines.

if length(lines) > 38
    str = ['Lots of disturbances, prediction might not be correct']
    text(200,200,str,'Color','red','FontSize',20)
    if (rightlineslope + leftlineslope)  < -0.2
       strL = ['Left Turn Ahead'];
       text(500,600,strL,'Color','red','FontSize',14);
   elseif (rightlineslope + leftlineslope)  > 0.2
       strR = ['Right Turn Ahead'];
       text(500,600,strR,'Color','blue','FontSize',14);
   else 
        strS = ['Straight Road'];
        text(500,600,strS,'Color','yellow','FontSize',14)
    end
    
else
    
    if (leftslopem < -0.9) && (rightslopem > 0.48) && (rightslopem < 0.599)
       strL = ['Left Turn Ahead'];
       text(500,600,strL,'Color','red','FontSize',14);
   elseif (leftslopem > -0.76) &&  (leftslopem < -0.65) && (rightslopem > 0.53)
       strR = ['Right Turn Ahead'];
       text(500,600,strR,'Color','blue','FontSize',14);
   else 
        strS = ['Straight Road'];
        text(500,600,strS,'Color','yellow','FontSize',14)
    end
end
    
%% Patch
% if all points and lines are in region of interest then create a patch on
% road
    if size(leftpoints) > 0 
         if size(rightpoints) > 0
            if (max(d1) < 670) && (min(d1) > 240) && (max(e1)<700) && (min(e1)>455) ...
              && (max(d) < 1006) && (min(d) > 700 ) && (max(e)<700) && (min(e)>455)
                   
                    Xp = [286 572 700 1006];
                    Yp = [ max(eq1) min(eq1) min(eq) max(eq) ];  
                    v1 = [286 max(e1) ;572 min(e1);700 min(eq);1006 max(eq)];
                    f1= [1 2 3 4];
                    drawnow
                    patch('Faces',f1,'Vertices',v1,'FaceColor','blue','FaceAlpha',0.1);
                          
            else
                    Xp = [min(d1) max(d1) min(d) max(d)];
                    Yp = [ max(e1) min(e1) min(e) max(e) ];  
                    v1 = [min(d1) max(e1) ;max(d1) min(e1);min(d) min(e);max(d) max(e)];
                    f1= [1 2 3 4];
                    drawnow
                    patch('Faces',f1,'Vertices',v1,'FaceColor','blue','FaceAlpha',0.1);
            end
         end
    end
    
    %% save image into frame of the outut video
F = getframe(gcf);
img232 = frame2im(F);
writeVideo(writerObj, img232);

%% Clear major values for fast execution of values
    clear rightpoints;
   clear leftpoints;
   clear rightslope;
   clear leftslope;
   clearvars lines i j k1 d d1 e1 e dq1 BW BW1 a1 a b b1 c c1 H o P rho;
 
 hold off
    
end
close(writerObj);
