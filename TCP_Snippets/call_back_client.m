function call_back_client(t, ~)

    global wait_confirm

    %disp(wait_confirm)
    wait_confirm = 1;
    %disp(t.read(t.NumBytesAvailable,"char"))
    
end
