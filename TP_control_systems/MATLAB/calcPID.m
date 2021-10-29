function u = calcPID(y,r)
    uPtr = libpointer('doublePtr',0); % "C" array pointer
    yPtr = libpointer('doublePtr',[y,r]); % "C" array pointer
    calllib('pid','calc',yPtr,uPtr);
    u = uPtr.value;
end



