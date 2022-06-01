%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Cinematica inversa para robot Phantom X
Por: Maria Alejandra Arias Frontanilla
Entradas:
T: Matriz 4x4 con posición y orientación de la herramienta
l: Longitud de eslabones
Salida:
q: Matriz 2x4 con valores de las 4 articulaciones en grados. 
    Row 1: codo abajo
    Row 2: codo arriba
    [q1d q2d q3d q4d]
    [q1u q2u q3u q4u]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%}

function q = invKinPhantom(T, l)
    %Cálculo de la primera articulación en radianes
    q(1,1) = atan2(T(2,4), T(1,4));
    q(2,1) = q(1,1);
    
    %Desacople de muñeca:
    %Cálculo de la posición de la muñeca W
    Pos_w = T(1:3, 4) - l(4)*T(1:3, 3)

    %Solución de mecanismo 2R para q2 y q3
    h = Pos_w(3) - l(1)
    r = sqrt(Pos_w(1)^2 + Pos_w(2)^2)

    D = (r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3))

    % Codo abajo:

    %Tercera articulación en radianes    
    q(1,3) = acos((r^2+h^2-l(2)^2-l(3)^2)/(2*l(2)*l(3)));
    
    %Segunda articulación en radianes sin offset
    q(1,2) = atan2(h,r) - atan2(l(3)*sin(q(1,3)), l(2)+l(3)*cos(q(1,3)));
    %Teniendo en cuenta offset en la segunda articulación
    q(1,2) = q(1,2) - pi/2;

    %Codo arriba: 

    % %Tercera articulación en radianes: negativo de codo abajo para q3
    q(2,3) = -q(1,3);
    %Segunda articulación en radianes sin offset
    q(2,2) = atan2(h,r) + atan2(l(3)*sin(q(1,3)), l(2)+l(3)*cos(q(1,3)));
    %Teniendo en cuenta offset en la segunda articulación
    q(2,2) = q(2,2) - pi/2;    


    %Obtención de valor de cuarta articulación usando vector |a| de T
    phi = atan2(T(3,3), sqrt(T(1,3)^2 +T(2,3)^2)) - pi/2;
    q(1,4) = phi - q(1,2) -q(1,3);
    q(2,4) = phi - q(2,2) -q(2,3);


    if ~isreal(q(1,3))
        q(:,:) = NaN;
    end

end