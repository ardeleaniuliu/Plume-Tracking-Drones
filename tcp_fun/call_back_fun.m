function call_back_fun(t2,~)
    w = 640;
    h = 480;
    figure(999)
    bytes_read = t2.read(w*h*3, "uint8");
    img = reshape(bytes_read, h, w, 3);
    img = uint8(img);
    imshow(img)
    title(t2.NumBytesAvailable/(w*h*3))
    drawnow
    t2.write('ok',"char")

end


