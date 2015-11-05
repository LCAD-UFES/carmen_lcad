
F = [1 0 1 0; 0 1 0 1; 0 0 1 0; 0 0 0 1]
H = [1 0 0 0; 0 1 0 0]

initx = [1.5 6.5 1 0.1]';

X = initx;
for i=1:100
  X = F*X;
  plot(X(1), X(2), 'x');
  hold on
  pause
end
