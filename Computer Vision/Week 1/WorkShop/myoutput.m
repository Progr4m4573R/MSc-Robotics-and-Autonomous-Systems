function myoutput = myoutput()
% H1 line
% help text
x = rand();

if x < 0.7
    disp('Hello world!');
else
    disp('Goodbye');
end
disp(x)
myoutput = x;
