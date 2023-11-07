clear

figure(999)

w = 640;
h = 480;
t2 = tcpserver("127.0.0.1", 6060);
configureCallback(t2, "byte", w*h*3, @call_back_fun)
