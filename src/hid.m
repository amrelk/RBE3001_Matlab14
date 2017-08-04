javaaddpath('../lib/hid4java-0.5.1.jar');

import org.hid4java.*;
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.lang.*;


pp = PacketProcessor(7);

sum = 0;
min = 100000;
max = 0;
values = zeros(15, 1, 'single');
sinWaveInc = 2000;
seconds = 0.01;
range = 400;

%         for k=1:size(sinWaveInc)
%             for j=1:4
%                 values((j * 3) + 0) = (sin((k / sinWaveInc * pi *2) * range) -range);
%                 values((j * 3) + 1) = 0;
%                 values((j * 3) + 2) = 3;
%             end
returnValues = pp.command(37, values)
%             timeit(returnValues)
%         end

clear java;