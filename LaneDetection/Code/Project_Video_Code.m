%%Initiate Output video 

writerObj = VideoWriter('outv10Project8','MPEG-4');
open(writerObj);
%% start input video 

v = VideoReader('project_video.mp4');
while hasFrame(v) 
%  thisFrame = read(v,95);
    
%%Reading Frames and converting to greyscale
    thisFrame = readFrame(v);
    j= rgb2gray(thisFrame);
%% Filtering the image 
j = j > 180;
k1 = medfilt2(j);
 

BW = edge(k1,'canny');

h = fspecial('sobel'); 
h = h'; 
I = imfilter(BW,h); 
%% Applying Roi to image
c = 1000*[0.5782 ; 0.2423 ;0.2723 ;0.4673 ;0.8647 ;1.0207;1.1527;0.7747;0.5782];
r = [449.75 ; 680.75 ; 704.75 ; 605.75 ; 587.75 ; 683.75 ; 662.75 ; 446.75 ; 449.75 ];

BW1 = roipoly(I,c,r);
o= BW1.*I; 
 imshow(o);
%% hough lines and slope calculation
[H,theta,rho] = hough(o); 
P = houghpeaks(H,30,'threshold',ceil(0.1*max(H(:)))) ; 
lines = houghlines(o,theta,rho,P,'FillGap',30,'MinLength',26); 

imshow(thisFrame), hold on 
x=1;y=1;rightpoints=[];leftpoints=[];
 t=1;u=1;
for m = 1:length(lines) 
    
    xy = [lines(m).point1;lines(m).point2];
%     plot(xy(:,1),(xy(:,2)),'LineWidth',3,'Color','yellow'); 
   
    line_slope(m) = (xy(1,2) - xy(2,2))/ (xy(1,1) - xy(2,1));

    if line_slope(m) > 0.2 
          if xy(1,1) > 665 && xy(2,1) > 665
%                plot(xy(:,1),(xy(:,2)),'LineWidth',3,'Color','blue');
                rightpoints(x,:)=lines(m).point1;
                rightpoints(x+1,:)=lines(m).point2;
                x=x+2;
                rightslope(t)=line_slope(m);t=t+1;
          end
    end
        
    if line_slope(m) < -0.2
         if xy(1,1) < 665 && xy(2,1) < 665
%               plot(xy(:,1),(xy(:,2)),'LineWidth',3,'Color','red'); 
             leftpoints(y,:)=lines(m).point1;
             leftpoints(y+1,:)=lines(m).point2;
             y=y+2;
             leftslope(u) = line_slope(m);u=u+1;
        end
    end

 
end
 %% find a line which fits the points (right and left line both)
if size(rightpoints) > 0
a=rightpoints(:,1);
b=rightpoints(:,2);
c= polyfit(a,b,1);
d = linspace(min(a),max(a));
e = polyval(c,d);
rightslopem = mean(rightslope);

%% plot the right line as per limits 
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
%% plot the left line as per limits 
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

%% Predict if image has lot of disturbances depending on number for lines
if length(lines) > 38
    str = ['Lots of disturbances, prediction might not be correct'];
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
    
%% Plot the patch for lines inside limits 
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
    
%% write your image to output video
F = getframe(gcf);
img232 = frame2im(F);
writeVideo(writerObj, img232);

%% clear variables 
    clear rightpoints;
   clear leftpoints;
   clear rightslope;
   clear leftslope;
   clearvars lines i j k1 d d1 e1 e dq1 BW BW1 a1 a b b1 c c1 H o P rho;
 %% 
 hold off
    
end
close(writerObj);