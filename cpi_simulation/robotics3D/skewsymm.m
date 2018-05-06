function C = skewsymm(X1)

% generates skew symmetric matrix

C = [0      , -X1(3) ,  X1(2)
    X1(3) , 0      , -X1(1)
    -X1(2) , X1(1)  ,     0];
