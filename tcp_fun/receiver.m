clear

figure(999)

w = 640;
h = 480;
t2 = tcpserver("192.168.1.12", 6060);
configureCallback(t2, "byte", w*h*3, @call_back_fun)
