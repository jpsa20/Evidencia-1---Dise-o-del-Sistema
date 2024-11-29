function deployToRaspberry() %#codegen
    % Conexión con Raspberry Pi
    r = raspi('172.20.10.2', 'davidios', '123456');  
    
    % Conexión a la cámara de la Raspberry Pi
    w = webcam(r);

    % Parámetros del entorno
    h = 3.30;  % Altura de la cámara en metros
    f = 0.26;  % Distancia focal en metros
    theta = deg2rad(83.07); 
    fps = 30;  % Obtener el frame rate de la cámara
  % Cuadros por segundo
    time_per_frame = 1 / fps;

    % Relación píxeles a metros (calcular según tu configuración)
    pixels_height = 480;  % Resolución vertical
    D = 3;  % Distancia horizontal en metros
    v = 24;  % Dimensión vertical en mm
    theta3 = 2 * atand(v / (2 * f * 1000));  % Ángulo de campo de visión
    P = 2 * D * tan(deg2rad(theta3) / 2);  % Distancia perpendicular
    meters_per_pixel = P / pixels_height;  % Relación píxeles a metros

    % Parámetros de suavizado
    N = 2;  % Tamaño de la ventana para suavizado
    centroidBuffer = zeros(N, 2);  % Buffer de posiciones suavizadas
    bufferIndex = 1;
    roi = [95 50 110 240];

    foregroundDetector = vision.ForegroundDetector('NumGaussians', 3, ...
    'NumTrainingFrames', 250, 'MinimumBackgroundRatio', 0.3, ...
    'InitialVariance', 110*110);

    % Inicialización de variables
    prevDistance = NaN;  % Inicialización de prevDistance
    velocities = [];  % Vector para guardar las velocidades en y (vacío inicialmente)
    prevCentroid = [NaN, NaN];
    ytPositions = NaN;  % Inicialización del arreglo para las posiciones en y

    % Estructuras morfológicas
    seOpen = strel('square', 3);  % Estructura para operación de apertura
    seClose = strel('square', 5); % Estructura para operación de cierre
    se = strel('disk', 7);         % Estructura para dilatación

    % Proceso de captura y análisis
    for k = 1:90  % Ejecutar un número fijo de iteraciones (modificar según sea necesario)
        % Capturar imagen de la cámara
        frame = snapshot(w);
        croppedFrame = imcrop(frame, roi);
        grayFrame = rgb2gray(croppedFrame);
        filteredFrame = imgaussfilt(grayFrame, 2);
        filteredFrame = imopen(filteredFrame, seOpen);
        foreground = step(foregroundDetector, filteredFrame);
        filteredForeground = bwareaopen(foreground, 500);
        filteredForeground = imfill(filteredForeground, 'holes');
        filteredForeground = imclose(filteredForeground, seClose);
        filteredForeground = imdilate(filteredForeground, se);
        
        stats = regionprops(filteredForeground, 'Centroid', 'BoundingBox', 'Area');
        minArea = 1000;
        stats = stats([stats.Area] >= minArea);

        for i = 1:numel(stats)
            centroid = stats(i).Centroid;
            
            % Si el buffer aún no ha alcanzado el tamaño máximo, agregar el centroide
            if bufferIndex <= N
                centroidBuffer(bufferIndex, :) = centroid;
                bufferIndex = bufferIndex + 1;
            else
                % Si el buffer ya está lleno, hacer un "desplazamiento" para agregar el nuevo centroide
                centroidBuffer = circshift(centroidBuffer, [-1, 0]);
                centroidBuffer(N, :) = centroid;
            end

            % Suavizar la posición del centroide mediante un promedio de las posiciones en el buffer
            smoothedCentroid = mean(centroidBuffer, 1);

            % Guardar la posición en y para análisis (si lo necesitas)
            ytPositions = [ytPositions; smoothedCentroid(2)];  % Esto guarda la posición en y de cada centroide suavizado

            % Calcular el ángulo phi basado en la posición y del centroide suavizado
            phi = atan((roi(4) / 2) / f) - atan((smoothedCentroid(2) - roi(4) / 2) / f);

            % Calcular la distancia al objeto
            dO = h * tan(phi + theta);

            % Calcular la velocidad en y si existe una distancia previa
            if ~isnan(prevDistance)
                % Desplazamiento en metros entre mediciones
                dy = dO - prevDistance;  % Diferencia en la dirección y
                velocity_y = dy * fps * 3.6;  % Velocidad en km/h en la dirección y

                % Guardar la velocidad en el vector de velocidades (solo agregar si es válida)
                if ~isnan(velocity_y) && isfinite(velocity_y)
                    velocities = [velocities; velocity_y];
                end
            end

            % Actualizar la distancia previa
            prevDistance = dO;

            % Mostrar el centroide suavizado
            croppedFrame = insertMarker(croppedFrame, smoothedCentroid, 'o', 'Color', [255 0 0], 'Size', 5);

            % Mostrar la bounding box (opcional)
            bbox = stats(i).BoundingBox;
            croppedFrame = insertShape(croppedFrame, 'Rectangle', bbox, 'Color', [0 255 0], 'LineWidth', 2);
        end

        % Mostrar el número de objetos detectados
        numCars = numel(stats);
        croppedFrame = insertText(croppedFrame, [10 10], numCars, 'BoxOpacity', 1, 'FontSize', 14);

        % Mostrar la imagen procesada
        displayImage(r, croppedFrame);
    end

    % Calcular la velocidad promedio en la dirección y
    avgVelocity_y = mean(velocities);
    avgVelocity_y =avgVelocity_y *2;% Promedio de las velocidades en y
    fprintf('La velocidad promedio en y es: %.2f km/h\n', avgVelocity_y);
end
