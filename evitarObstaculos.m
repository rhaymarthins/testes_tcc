% Define o sensor LIDAR
% A função LidarSensor é criada para representar o sensor LIDAR do robô.
% O deslocamento do sensor em relação ao centro do robô é definido em lidar.sensorOffset.
% O número de leituras de laser e seus ângulos são definidos em lidar.scanAngles.
% O alcance máximo do sensor é definido em lidar.maxRange.
lidar = LidarSensor;
lidar.sensorOffset = [0,0]; % Deslocamento em [m] a partir do centro do robô
lidar.scanAngles = linspace(-pi/2,pi/2,51); % Define quantas leituras de laser e o ângulo de cada uma
lidar.maxRange = 0.5; % Alcance máximo [m]


% Carrega uma imagem como mapa de ocupação. 
% O limiar de binarização é definido como 0.1.
treshold = 0.1;
% A escala do mapa é definida como 10 pixels por metro.
scale = 10;
% Lê a imagem do arquivo especificado no caminho
image = imread('img/lab_test3.png');           
% Converte a imagem para tons de cinza
grayimage = rgb2gray(image);
% Aplica o limiar à imagem para obter uma imagem binária.
bwimage = grayimage < treshold;
% Converte a imagem binária em um mapa de ocupação utilizando a escala (pixels/metro).
map = binaryOccupancyMap(bwimage,scale);


% Cria um visualizador e atribui o mapa ao binaryOccupancyMap. 
robotRadius = 0.085;
viz = Visualizer2D;
viz.mapName = 'map';
viz.robotRadius = robotRadius;

% Anexa sensores ao robô 
attachLidarSensor(viz,lidar);

% Define pose inicial e um robô diferencial
initPose = [ 1.6; 1.5; 0]; % [x ; y ; theta]
R = 0.7; % Raio da roda do robo
L = 1.6; % Comprimento do eixo 
mobileRobot = DifferentialDrive(R,L);

% Definição de velocidade
v = 0.2; % [m/s]
%w = pi/4; % [rad/s]
w = 0;
bodyV = [v; 0; w]; % Velocidades do corpo [vx; vy; w], vy = 0 porque o robô diferencial não pode se mover no eixo y.

% OBS:pesquisar sobre como funciona faixas de distancias e obstaculos para um robo
% Histograma de Campo Vetorial (VFH) para evitar obstáculos 
% O código não implementa a lógica para evitar obstáculos com base nas leituras do sensor LIDAR.
% A implementação atual do VFH calcula apenas a direção de controle wRef, mas não a velocidade linear v.
% É importante implementar uma lógica completa para lidar com obstáculos e garantir a segurança do robô durante a simulação.

vfh = controllerVFH;
vfh.DistanceLimits = [0.1 1]; % Limites de distância
vfh.RobotRadius = robotRadius; % Raio do robô
vfh.SafetyDistance = 0.25; % Distância de segurança
vfh.MinTurningRadius = 0.1; % Raio mínimo de viragem


% Tempo de amostragem da simulação (0.1 segundos)
sampleTime = 0.1;
% Número total de iterações (30).
executeTime = 30;
% O retorno dessa função inclui o tempo de amostragem (sampleTime), um vetor de tempo (tVec) e um objeto de controle de taxa (r)
tVec = 0:sampleTime:executeTime;     % Cria o vetor de tempo de 0 a tempoExecucao com intervalos de tempoAmostra
r = rateControl(1/sampleTime);       % Ajusta o controle de taxa de acordo com o inverso do tempoAmostra


% Loop é responsável por simular o movimento do robô ao longo do tempo, tomando medidas do ambiente através do sensor LIDAR, 
% calculando a direção desejada com base nessas medidas e atualizando a pose do robô de acordo com as velocidades calculadas.
% Execução da simulação
currentPose = initPose;

for idx = 2:numel(tVec) % Loop é iniciado para iterar sobre cada intervalo de tempo da simulação, a partir do segundo elemento do vetor de tempo
    % Obter leituras do sensor a partir da pose atual
    ranges = lidar(currentPose);
    targetDir = 0.5707; % direção desejada para evitar obstáculos
    %steerDir = vfh(ranges,lidar.scanAngles,targetDir);  

    % Calcula apenas a direção de controle para evitar obstáculos, não a velocidade linear
    wRef = vfh(ranges, lidar.scanAngles, targetDir); % Sensor e os ângulos de varredura do sensor para calcular uma direção de controle desejada 
    
    %  Garante que haja uma direção de controle válida se o valor de wRef for NaN.
    if isnan(wRef)
        wRef = 0.5;
    end
    
    % Velocidades linear e angular do corpo do robô 
    bodyV = [v; 0; wRef];
    
    % Converter do corpo do robô para o mundo
    vel = bodyToWorld(bodyV, currentPose);  
    
    % Executar um passo de integração discreta para a frente
    currentPose = currentPose + vel * sampleTime; 
    % Atualizar visualização
    viz(currentPose, ranges);
    
    % Aguardar a taxa de visualização
    waitfor(r);
end

% Verificar saida SensorLidar




