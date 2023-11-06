clear
global wait_confirm

% use ipconfig in terminal to find your Local IP address if needed
t2_client = tcpclient("192.168.1.12", 6060);
configureCallback(t2_client, "byte", 2, @call_back_client)

cam = webcam;
cam.Resolution = '640x480';

wait_confirm = 1;
tic
counter = 0;
while(toc<10)
    if t2_client.NumBytesAvailable
        pause(0.01) % pause to give t2_client time to run async fun
    end
    
    if wait_confirm
        img = snapshot(cam);
        w = 640;
        h = 480;
        img2 = reshape(img, 1, w*h*3);
        t2_client.write(img2,"uint8")
        counter = counter+1
        wait_confirm = 0;
    else
        continue
    end
end

clear('cam')

