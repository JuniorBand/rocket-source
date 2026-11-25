function F41()


    % ------------------------ Estado da aplicacao ------------------------
    portaSerial = [];
    rx = uint8([]);
    txSeq = uint8(0);
    pacotesOK = 0;
    errosCRC = 0;
    rxBytes = 0;
    stmPacotes = 0;
    gpsPacotesRecebidos = 0;
    tipoDesconhecido = 0;
    errosProcessamento = 0;
    ultimoRxEm = NaN;
    ultimoSTMEm = NaN; ultimoGPSEm = NaN;
    ultimoGPSIdade = Inf;
    ultimoDadoRFEm=NaN; ultimoAvancoRFEm=NaN; idadeAvancoRF=Inf;
    temTokenRF=false; tokenBootRF=0; tokenContadorRF=0; tokenTempoRF=0; tokenRespostasRF=0;
    stmRemotoRecente = true;
    stmDadosValidos = false; bmpDadosValidos = false; gpsDadosValidos = false;
    ultimoGPSFixValido=false; ultimoGPSSats=0;
    stmPareamentoPendente = false;
    watchdogTimer = [];
    errosCRCInicial = 0;
    ultimaSequencia = NaN;
    ultimoEstado = "SEM DADOS";
    ultPlot = tic;
    plotT = [];
    plotAlt = [];
    plotAcc = [];
    plotVz = []; plotPress = []; plotTemp = []; plotGiro = [];
    gpsCSV = -1; gpsArquivo = '';
    lastAlt=NaN;lastVz=NaN;lastPress=NaN;lastTemp=NaN;lastTempo=NaN;
    gpsLat0=NaN; gpsLon0=NaN; estLat=NaN;estLon=NaN;
    gpsEast=[]; gpsNorth=[]; gpsHeights=[];
    gpsLatitudes=[];gpsLongitudes=[];
    gpsPacotes=0;gpsFixOK=0; gpsUltimoValido='';
    ultimoDesenhoGPS=tic;
    anteriorRecebSTM=NaN; taxaSTM=NaN;
    linhasLog = strings(0,1);
    maxLinhasLog = 140;

    logRows = containers.Map('KeyType', 'double', 'ValueType', 'any');
    logEsperados = 0;
    logVersao = 0;
    logTamanho = 0;
    gyroBias = [0 0 0];
    gyroCalibrado = false;

    % ATTITUDE: eixos do MPU6050 devem coincidir com o eixo do foguete.
    % Altere aqui se o sensor foi montado com sinais invertidos.
    SINAL_ACEL = [1, 1, 1];
    SINAL_GIRO = [1, 1, 1];
    attRoll=NaN; attPitch=NaN; attYaw=0;
    attUltTempo=NaN; attDesenho=tic; attValida=false;
    attConfianca='AGUARDANDO';

    corFundo = [0.946 0.960 0.977];
    corNavy = [0.10 0.16 0.26];
    corAzul = [0.15 0.35 0.70];
    corVerde = [0.11 0.53 0.34];
    corLaranja = [0.75 0.41 0.13];


    % --------------------------- Interface ------------------------------
    fig = uifigure('Name', 'Estacao V4.0.4 | STM32 + GPS + Atitude 3D | Telemetria e Telecomandos', ...
        'Position', [40 35 1460 910], 'Color', corFundo, ...
        'CloseRequestFcn', @aoFechar);
    principal = uigridlayout(fig, [5 1]);
    principal.RowHeight = {44, 60, 194, '1x', 148};
    principal.Padding = [12 10 12 10];
    principal.RowSpacing = 8;

    cab = uigridlayout(principal, [1 2]);
    cab.Layout.Row = 1;
    cab.ColumnWidth = {'1x', 320};
    cab.Padding = [2 0 2 0];
    titulo = uilabel(cab, 'Text', 'ESTACAO V4.0.4  /  STM32 + GPS + ATITUDE 3D', ...
        'FontSize', 19, 'FontWeight', 'bold', 'FontColor', corNavy);
    titulo.Layout.Column = 1;
    subtitulo = uilabel(cab, 'Text', 'ESP32 LoRa / USB 115200  |  foco: voo, GPS e telecomandos', ...
        'HorizontalAlignment', 'right', 'FontColor', [0.37 0.43 0.51]);
    subtitulo.Layout.Column = 2;

    barra = uigridlayout(principal, [1 10]);
    barra.Layout.Row = 2;
    barra.ColumnWidth = {48, 98, 95, 102, 115, 120, '1x', 160, 120, 118};
    barra.Padding = [10 8 10 8];
    barra.BackgroundColor = [1 1 1];
    uiCellLabel(barra, 1, 'Porta:', 12, corNavy);
    ddPorta = uidropdown(barra, 'Items', {'COM6'}, 'Value', 'COM6');
    ddPorta.Layout.Column = 2;
    btnAtualizar = uibutton(barra, 'Text', 'Atualizar', 'ButtonPushedFcn', @atualizarPortas);
    btnAtualizar.Layout.Column = 3;
    btnConectar = uibutton(barra, 'push', 'Text', 'Conectar', ...
        'BackgroundColor', corVerde, 'FontColor', [1 1 1], ...
        'ButtonPushedFcn', @conectar);
    btnConectar.Layout.Column = 4;
    btnDesconectar = uibutton(barra, 'push', 'Text', 'Desconectar', ...
        'Enable', 'off', 'ButtonPushedFcn', @desconectar);
    btnDesconectar.Layout.Column = 5;
    lbConexao = uilabel(barra, 'Text', 'DESCONECTADO', 'FontWeight', 'bold', ...
        'FontColor', corLaranja, 'HorizontalAlignment', 'center');
    lbConexao.Layout.Column = 6;
    lbRadio=uilabel(barra,'Text','RF: aguardando dados', ...
        'FontSize',11,'FontColor',corLaranja);
    lbRadio.Layout.Column=7;
    lbPacotes = uilabel(barra, 'Text', 'RX 0 B | 0 quadros', ...
        'HorizontalAlignment', 'right', 'FontWeight', 'bold');
    lbPacotes.Layout.Column = 8;
    lbErros = uilabel(barra, 'Text', 'CRC: 0', ...
        'HorizontalAlignment', 'right', 'FontWeight', 'bold');
    lbErros.Layout.Column = 9;
    btnTestePainel = uibutton(barra, 'Text', 'Testar painel', ...
        'ButtonPushedFcn', @testarPainelSemRadio);
    btnTestePainel.Layout.Column = 10;

    resumo = uigridlayout(principal, [1 4]);
    resumo.Layout.Row = 3;
    resumo.ColumnWidth = {'1.05x', '1.15x', '1.15x', '1x'};
    resumo.ColumnSpacing = 8;
    resumo.Padding = [0 0 0 0];

    bmpPainel = uipanel(resumo, 'Title', 'Altitude / BMP280', ...
        'FontWeight', 'bold', 'ForegroundColor', corNavy);
    bmpPainel.Layout.Column = 1;
    gbmp = uigridlayout(bmpPainel, [4 2]);
    gbmp.RowHeight = {28, 28, 28, 28};
    gbmp.ColumnWidth = {'1x', 145};
    gbmp.Padding = [12 8 12 8];
    lbAlt = parDeMedida(gbmp, 1, 'Altitude relativa', '-- m');
    lbVz = parDeMedida(gbmp, 2, 'Velocidade vertical', '-- m/s');
    lbPress = parDeMedida(gbmp, 3, 'Pressao', '-- Pa');
    lbTemp = parDeMedida(gbmp, 4, 'Temperatura', '-- C');

    imuPainel = uipanel(resumo, 'Title', 'Atitude / IMU (sempre visivel)', ...
        'FontWeight', 'bold', 'ForegroundColor', corNavy);
    imuPainel.Layout.Column = 2;
    gimu = uigridlayout(imuPainel, [6 3]);
    gimu.RowHeight = {20, 23, 23, 23, 25, 18};
    gimu.ColumnWidth = {'1x', '1x', '1x'};
    gimu.Padding = [10 7 10 7];
    cabImu(gimu, 1, 1, 'EIXO');
    cabImu(gimu, 1, 2, 'ACC [m/s2]');
    cabImu(gimu, 1, 3, 'GYR [graus/s]');
    nomes = {'X', 'Y', 'Z'};
    lbAcc = cell(1,3);
    lbGyr = cell(1,3);
    for j = 1:3
        cabImu(gimu, j+1, 1, nomes{j});
        lbAcc{j} = uilabel(gimu, 'Text', '--', ...
            'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold');
        lbAcc{j}.Layout.Row = j+1; lbAcc{j}.Layout.Column = 2;
        lbGyr{j} = uilabel(gimu, 'Text', '--', ...
            'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold');
        lbGyr{j}.Layout.Row = j+1; lbGyr{j}.Layout.Column = 3;
    end
    lbRoll = uilabel(gimu,'Text','Roll: --','FontWeight','bold', 'FontSize',11);
    lbRoll.Layout.Row=5;lbRoll.Layout.Column=1;
    lbPitch = uilabel(gimu,'Text','Pitch: --','FontWeight','bold', 'FontSize',11);
    lbPitch.Layout.Row=5;lbPitch.Layout.Column=2;
    lbYaw = uilabel(gimu,'Text','Yaw: --','FontWeight','bold', 'FontSize',11);
    lbYaw.Layout.Row=5;lbYaw.Layout.Column=3;
    lbImu = uilabel(gimu, 'Text', 'IMU: aguardando', 'FontColor', corLaranja,'FontSize',10);
    lbImu.Layout.Row = 6; lbImu.Layout.Column = [1 3];

    gpsPainel = uipanel(resumo, 'Title', 'GPS (sempre visivel)', ...
        'FontWeight', 'bold', 'ForegroundColor', corNavy);
    gpsPainel.Layout.Column = 3;
    ggps = uigridlayout(gpsPainel, [4 1]);
    ggps.RowHeight = {24, 24, 24, 24};
    ggps.Padding = [12 9 12 9];
    lbGPSFix = uilabel(ggps, 'Text', 'GPS: aguardando', 'FontWeight', 'bold', 'FontColor', corLaranja);
    lbGPSCoord = uilabel(ggps, 'Text', 'Lat: --   |   Lon: --');
    lbGPSSatelites = uilabel(ggps, 'Text', 'Satelites: --   |   Idade: -- ms');
    lbDist = uilabel(ggps, 'Text', 'Distancia: --   |   Azimute: --');

    estadoPainel = uipanel(resumo, 'Title', 'Estado do voo / computador de bordo', ...
        'FontWeight', 'bold', 'ForegroundColor', corNavy);
    estadoPainel.Layout.Column = 4;
    gest = uigridlayout(estadoPainel, [6 1]);
    gest.RowHeight = {28,18,18,18,18,18};
    gest.RowSpacing=4;
    gest.Padding = [14 9 14 9];
    lbEstado = uilabel(gest, 'Text', 'SEM DADOS', 'FontWeight', 'bold', ...
        'FontSize', 20, 'FontColor', corLaranja);
    lbEstado.Layout.Row = 1;
    lbTempo = uilabel(gest, 'Text', 'Tempo desde boot: -- s');
    lbTempo.Layout.Row = 2;
    lbSeq = uilabel(gest, 'Text', 'Ultima sequencia: --');
    lbSeq.Layout.Row = 3;
    lbReg = uilabel(gest, 'Text', 'Registros na Flash: --');
    lbReg.Layout.Row = 4;
    lbUlt = uilabel(gest, 'Text', 'Resposta: aguardando');
    lbUlt.Layout.Row = 5;
    lbAtualidadeSTM=uilabel(gest,'Text','Aguardando primeira leitura STM32', ...
        'FontSize',10,'FontColor',corLaranja);
    lbAtualidadeSTM.Layout.Row=6;

    abas = uitabgroup(principal);
    abas.Layout.Row = 4;
    abaGraficos = uitab(abas,'Title','Graficos limpos');
    abaTrajetoria = uitab(abas,'Title','Trajetoria 3D');
    abaGPS = uitab(abas,'Title','Rastreamento GPS');
    abaAtitude = uitab(abas,'Title','Atitude 3D / Horizonte');

    graficos = uigridlayout(abaGraficos, [2 4]);
    graficos.RowHeight = {28, '1x'};
    graficos.ColumnWidth = {'1x', 110, '1x', 110};
    graficos.ColumnSpacing = 9;
    graficos.RowSpacing = 6;
    graficos.Padding = [4 4 4 4];
    opcoesGraficos = {'Altitude (m)','Velocidade vertical (m/s)', ...
        'Aceleracao (m/s2)','Pressao (Pa)','Temperatura (C)','Giro (graus/s)'};
    ddGraf1 = uidropdown(graficos,'Items',opcoesGraficos,'Value','Altitude (m)',...
        'ValueChangedFcn',@alterarGraficos);
    ddGraf1.Layout.Row=1;ddGraf1.Layout.Column=1;
    ddGraf2 = uidropdown(graficos,'Items',opcoesGraficos,'Value','Aceleracao (m/s2)',...
        'ValueChangedFcn',@alterarGraficos);
    ddGraf2.Layout.Row=1;ddGraf2.Layout.Column=3;
    btnLimparGraf=uibutton(graficos,'Text','Limpar graficos', 'ButtonPushedFcn',@limparGraficos);
    btnLimparGraf.Layout.Row=1;btnLimparGraf.Layout.Column=4;
    lbJanela=uilabel(graficos,'Text','Ultimos 60 s','HorizontalAlignment','center', 'FontColor',corNavy);
    lbJanela.Layout.Row=1;lbJanela.Layout.Column=2;
    axAlt = uiaxes(graficos);
    axAlt.Layout.Row=2;axAlt.Layout.Column = [1 2];
    axAlt.BackgroundColor = [1 1 1];
    axAlt.Box = 'on';
    xlabel(axAlt,'Tempo STM32 (s)'); ylabel(axAlt,''); grid(axAlt,'on');
    linhaAlt = plot(axAlt, NaN, NaN, '-', 'LineWidth', 2.0, 'Color', corAzul);
    title(axAlt,'Altitude (m)');
    axAcc = uiaxes(graficos);
    axAcc.Layout.Row=2;axAcc.Layout.Column = [3 4];
    axAcc.BackgroundColor = [1 1 1];
    axAcc.Box = 'on';
    xlabel(axAcc,'Tempo STM32 (s)'); ylabel(axAcc,''); grid(axAcc,'on');
    linhaAcc = plot(axAcc, NaN, NaN, '-', 'LineWidth', 2.0, 'Color', corVerde);
    title(axAcc,'Aceleracao (m/s2)');

    g3d = uigridlayout(abaTrajetoria,[1 1]);
    g3d.Padding=[4 4 4 4];
    ax3D = uiaxes(g3d);
    title(ax3D,'Trajetoria 3D do foguete (origem marcada)');
    xlabel(ax3D,'Leste (m)'); ylabel(ax3D,'Norte (m)'); zlabel(ax3D,'Altitude relativa (m)');
    ax3D.BackgroundColor = [1 1 1];
    grid(ax3D,'on'); view(ax3D,45,25);
    hold(ax3D,'on');
    linha3D = plot3(ax3D,NaN,NaN,NaN,'-','LineWidth',2.0,'Color',corAzul);
    marcadorInicio3D = plot3(ax3D,NaN,NaN,NaN,'^','MarkerSize',9,'MarkerFaceColor',corVerde,'Color',corVerde);
    ponto3D = plot3(ax3D,NaN,NaN,NaN,'o','MarkerSize',8,'MarkerFaceColor',corLaranja,'Color',corLaranja);
    legend(ax3D, [linha3D marcadorInicio3D ponto3D], {'Trajetoria','Ponto inicial','Posicao atual'}, ...
        'Location','northeast');

    gGPS = uigridlayout(abaGPS,[2 1]);
    gGPS.RowHeight={44,'1x'};
    gGPS.RowSpacing=5;
    gGPS.Padding=[4 4 4 4];
    barraGPS = uigridlayout(gGPS,[1 8]);
    barraGPS.Layout.Row = 1;
    barraGPS.ColumnWidth = {52, 100, 52, 100, 120, '1x', 150, 130};
    barraGPS.Padding = [4 4 4 4];
    uilabel(barraGPS,'Text','LAT:','FontWeight','bold');
    campoEstLat = uieditfield(barraGPS,'numeric','Value',0);
    campoEstLat.Layout.Column = 2;
    lbLon = uilabel(barraGPS,'Text','LON:','FontWeight','bold');
    lbLon.Layout.Column = 3;
    campoEstLon = uieditfield(barraGPS,'numeric','Value',0);
    campoEstLon.Layout.Column = 4;
    btnEst = uibutton(barraGPS,'Text','Definir estacao','ButtonPushedFcn',@definirEstacao);
    btnEst.Layout.Column = 5;
    btnMarcarInicio = uibutton(barraGPS,'Text','Marcar ponto inicial', ...
        'ButtonPushedFcn',@marcarPontoInicial);
    btnMarcarInicio.Layout.Column = 6;
    lbGPSFixTab = uilabel(barraGPS,'Text','Origem: primeiro FIX valido', 'HorizontalAlignment','right');
    lbGPSFixTab.Layout.Column = 7;
    lbGPSMapa = uilabel(barraGPS,'Text','Sem mapa online', 'HorizontalAlignment','right', 'FontColor', corLaranja);
    lbGPSMapa.Layout.Column = 8;

    axMapa = uiaxes(gGPS);
    axMapa.Layout.Row = 2;
    title(axMapa,'Rastreamento local do foguete');
    xlabel(axMapa,'Leste relativo ao ponto inicial (m)');
    ylabel(axMapa,'Norte relativo ao ponto inicial (m)');
    axMapa.BackgroundColor = [1 1 1];
    grid(axMapa,'on'); axis(axMapa,'equal'); hold(axMapa,'on');
    linhaRastro = plot(axMapa,NaN,NaN,'-','LineWidth',2.0,'Color',corAzul);
    marcadorInicio2D = plot(axMapa,NaN,NaN,'^','MarkerSize',9,'MarkerFaceColor',corVerde,'Color',corVerde);
    marcadorFoguete = plot(axMapa,NaN,NaN,'o','MarkerSize',8,'MarkerFaceColor',corLaranja,'Color',corLaranja);
    marcadorEstacao = plot(axMapa,NaN,NaN,'s','MarkerSize',8,'MarkerFaceColor',corVerde,'Color',corVerde);
    linhaEstacaoFoguete = plot(axMapa,NaN,NaN,'--','LineWidth',1.2,'Color',[0.25 0.25 0.25]);
    legend(axMapa, [linhaRastro marcadorInicio2D marcadorFoguete marcadorEstacao linhaEstacaoFoguete], ...
        {'Trajetoria','Ponto inicial','Posicao atual','Estacao terrestre','Linha de visada'}, ...
        'Location','northeast');

    % Atitude em tempo real: horizonte artificial e foguete 3D.
    % Estas figuras usam a estimativa do giroscopio + gravidade, NAO uma
    % orientacao absoluta: o yaw deriva, aceleracoes de voo afetam tilt.
    painelAt = uigridlayout(abaAtitude,[2 2]);
    painelAt.RowHeight = {43, '1x'};
    painelAt.ColumnWidth = {'1x','1x'};
    painelAt.Padding = [4 4 4 4];
    painelAt.ColumnSpacing = 12;
    barraAt = uigridlayout(painelAt,[1 3]);
    barraAt.Layout.Row=1; barraAt.Layout.Column=[1 2];
    barraAt.ColumnWidth = {170,'1x',210};
    btnReiniciarAt = uibutton(barraAt,'Text','Reiniciar estimativa', ...
        'ButtonPushedFcn',@reiniciarAtitude);
    btnReiniciarAt.Layout.Column=1;
    lbAtInfo = uilabel(barraAt, 'Text','Aguardando IMU | Yaw relativo (sem magnetometro)', ...
        'FontWeight','bold','HorizontalAlignment','center');
    lbAtInfo.Layout.Column=2;
    lbAtAviso = uilabel(barraAt, 'Text','Estimativa: nao usar para controle', ...
        'FontColor',corLaranja,'HorizontalAlignment','right');
    lbAtAviso.Layout.Column=3;

    axHoriz=uiaxes(painelAt);
    axHoriz.Layout.Row=2; axHoriz.Layout.Column=1;
    axHoriz.XLim=[-1.2 1.2];axHoriz.YLim=[-1.2 1.2];
    axHoriz.XTick=[];axHoriz.YTick=[];axHoriz.Box='on';
    axHoriz.DataAspectRatio=[1 1 1];
    title(axHoriz,'Horizonte artificial | pitch e roll estimados');
    hold(axHoriz,'on');
    skyPatch=patch(axHoriz,[-3,3,3,-3],[0,0,3,3],[0.19,0.46,0.74],'EdgeColor','none');
    landPatch=patch(axHoriz,[-3,3,3,-3],[-3,-3,0,0],[0.55,0.36,0.21],'EdgeColor','none');
    linhaHorizonte=plot(axHoriz,[-3 3],[0 0],'-w','LineWidth',2);
    plot(axHoriz,[-0.42,-0.12],[0,0],'-w','LineWidth',3);
    plot(axHoriz,[0.12,0.42],[0,0],'-w','LineWidth',3);
    plot(axHoriz,[-0.12,0,0.12],[-0.03,0.08,-0.03],'-w','LineWidth',2);
    plot(axHoriz,0,0,'+','MarkerSize',9,'LineWidth',1.3,'Color',[1 1 1]);

    axFoguete=uiaxes(painelAt);
    axFoguete.Layout.Row=2;axFoguete.Layout.Column=2;
    title(axFoguete,'Foguete 3D | movimento estimado pela IMU');
    grid(axFoguete,'on');hold(axFoguete,'on');
    axFoguete.XLim=[-2.2 2.2];axFoguete.YLim=[-2.2 2.2];
    axFoguete.ZLim=[-2.2 2.2];
    axis(axFoguete,'equal');view(axFoguete,42,21);
    xlabel(axFoguete,'X');ylabel(axFoguete,'Y');zlabel(axFoguete,'Z');
    grupoFoguete=hgtransform(axFoguete);
    [xb,yb,zb]=cylinder(0.24,28);zb=zb*2.45-1.3;
    hCorpo=surf(axFoguete,xb,yb,zb,'FaceColor',[0.82 0.86 0.91], ...
        'EdgeColor','none','FaceLighting','gouraud');
    hCorpo.Parent=grupoFoguete;
    [xn,yn,zn]=cylinder([0.24,0],28);zn=zn*0.67+1.15;
    hCone=surf(axFoguete,xn,yn,zn,'FaceColor',[0.95 0.43 0.23], ...
        'EdgeColor','none');
    hCone.Parent=grupoFoguete;
    [xs,ys,zs]=cylinder(0.246,28);zs=zs*0.2-0.4;
    hFaixa=surf(axFoguete,xs,ys,zs,'FaceColor',[0.22 0.36 0.58], ...
        'EdgeColor','none');
    hFaixa.Parent=grupoFoguete;
    for kFin=0:3
        tFin=kFin*pi/2;
        % Aletas fixadas no corpo, girando em conjunto com o foguete.
        vFin=[0.22,-0.05,-1.23;0.70,-0.05,-1.30;0.26,-0.05,-0.54; ...
              0.22,0.05,-1.23;0.70,0.05,-1.30;0.26,0.05,-0.54];
        rf=[cos(tFin) -sin(tFin);sin(tFin) cos(tFin)];
        vFin(:,1:2)=(rf*vFin(:,1:2).').';
        hFin=patch(axFoguete,'Vertices',vFin,'Faces',[1 2 3 NaN;4 6 5 NaN;1 4 5 2;2 5 6 3], ...
            'FaceColor',[0.26 0.42 0.64],'EdgeColor','none');
        hFin.Parent=grupoFoguete;
    end
    % Nao criar novos objetos graficos dentro do callback serial: apenas atualizar Matrix.
    grupoFoguete.Matrix=eye(4);
    abas.SelectionChangedFcn=@selecionarAba;

    rodape = uigridlayout(principal, [1 2]);
    rodape.Layout.Row = 5;
    rodape.ColumnWidth = {390, '1x'};
    rodape.ColumnSpacing = 9;
    rodape.Padding = [0 0 0 0];

    cmdPainel = uipanel(rodape, 'Title', 'Telecomandos STM32', ...
        'FontWeight', 'bold', 'ForegroundColor', corNavy);
    cmdPainel.Layout.Column = 1;
    gcmd = uigridlayout(cmdPainel, [5 3]);
    gcmd.RowHeight = {36, 36, 34, 22, '1x'};
    gcmd.ColumnWidth = {'1x', '1x', '1x'};
    gcmd.Padding = [10 8 10 8];
    gcmd.ColumnSpacing = 7;

    btnPing = uibutton(gcmd, 'push', 'Text', 'PING / STATUS', 'Enable', 'off', ...
        'ButtonPushedFcn', @(~,~) enviarComando(hex2dec('10')));
    btnPing.Layout.Row = 1; btnPing.Layout.Column = 1;
    btnV = uibutton(gcmd, 'push', 'Text', 'V  |  Ler Flash', 'Enable', 'off', ...
        'ButtonPushedFcn', @(~,~) enviarComando(1));
    btnV.Layout.Row = 1; btnV.Layout.Column = 2;
    btnS = uibutton(gcmd, 'push', 'Text', 'S  |  Parar / Desarmar', 'Enable', 'off', ...
        'ButtonPushedFcn', @(~,~) enviarComando(2));
    btnS.Layout.Row = 1; btnS.Layout.Column = 3;

    btnA = uibutton(gcmd, 'push', 'Text', 'A  |  Apagar + Armar', 'Enable', 'off', ...
        'ButtonPushedFcn', @(~,~) enviarComando(3));
    btnA.Layout.Row = 2; btnA.Layout.Column = [1 2];
    lbA = uilabel(gcmd, 'Text', 'A apaga a Flash', ...
        'FontSize', 10, 'FontColor', corLaranja);
    lbA.Layout.Row = 2; lbA.Layout.Column = 3;

    lbHex = uilabel(gcmd, 'Text', 'CMD (HEX):', 'FontWeight', 'bold');
    lbHex.Layout.Row = 3; lbHex.Layout.Column = 1;
    campoHex = uieditfield(gcmd, 'text', 'Value', '10', ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    campoHex.Layout.Row = 3; campoHex.Layout.Column = 2;
    btnEnviarHex = uibutton(gcmd, 'push', 'Text', 'Enviar HEX', 'Enable', 'off', ...
        'ButtonPushedFcn', @enviarHex);
    btnEnviarHex.Layout.Row = 3; btnEnviarHex.Layout.Column = 3;

    lbTran = uilabel(gcmd, 'Text', 'Transferencia de log: inativa', ...
        'FontColor', [0.36 0.41 0.50]);
    lbTran.Layout.Row = 4; lbTran.Layout.Column = [1 3];
    lbInfoCmd = uilabel(gcmd, 'Text', ...
        'Foco no essencial: telecomandos, estado do voo e GPS.', ...
        'FontSize', 10, 'WordWrap', 'on', 'FontColor', [0.35 0.40 0.49]);
    lbInfoCmd.Layout.Row = 5; lbInfoCmd.Layout.Column = [1 3];

    painelLog = uipanel(rodape, 'Title', 'Eventos principais / monitor compacto', ...
        'FontWeight', 'bold', 'ForegroundColor', corNavy);
    painelLog.Layout.Column = 2;
    glog = uigridlayout(painelLog, [2 1]);
    glog.RowHeight = {24, '1x'};
    glog.Padding = [9 6 9 6];
    cbHex = uicheckbox(glog, 'Text', 'Modo debug HEX (opcional)', ...
        'Value', false);
    cbHex.Layout.Row = 1;
    areaLog = uitextarea(glog, 'Editable', 'off', ...
        'FontName', 'Consolas', 'FontSize', 10, 'Value', {'Interface pronta.'});
    areaLog.Layout.Row = 2;

    atualizarPortas();
    adicionarLog('Selecione a COM USB do ESP32 e clique em Conectar.');


    % ---------------------- Autoteste sem LoRa -------------------------
    function testarPainelSemRadio(~,~)
        % Valida parser, CRC, etiquetas, graficos, GPS e IMU sem modificar
        % nenhum transmissor. So permitido com a porta serial fechada.
        if ~isempty(portaSerial)
            uialert(fig,'Desconecte a COM antes do autoteste local.','Autoteste');
            return;
        end
        try
            p=zeros(1,41,'uint8');
            p(1:4)=typecast(uint32(250000),'uint8');
            p(5:8)=typecast(uint32(101325),'uint8');
            p(9:10)=typecast(int16(2650),'uint8');
            p(11:14)=typecast(int32(1234),'uint8');
            p(15:16)=typecast(int16(12),'uint8');
            p(17:22)=typecast(int16([0 0 1000]),'uint8');
            p(23:28)=typecast(int16([0 0 0]),'uint8');
            p(29)=uint8(28); % IMU, IMU calibrada e BMP validos
            p(30:33)=typecast(int32(-25300000),'uint8');
            p(34:37)=typecast(int32(-443000000),'uint8');
            p(38)=uint8(3); % FIX valido e atual
            p(39)=uint8(10); % 10 satelites
            p(40:41)=typecast(uint16(100),'uint8');
            rx=uint8([]);pacotesOK=0;rxBytes=0;
            ultimoRxEm=tic;
            reiniciarSupervisaoRF();
            stmPacotes=0;gpsPacotesRecebidos=0;
            frame=montarQuadro(uint8(9),uint8(201),p);
            rxBytes=numel(frame);rx=[rx frame];
            interpretarBuffer();
            if stmPacotes==1 && gpsPacotesRecebidos==1
                adicionarLog('AUTOTESTE OK: STM32, GPS, CRC e graficos processaram o quadro local.');
                uialert(fig,'Painel e parser funcionaram com um quadro GPS+STM32 simulado. Agora conecte a COM para verificar o enlace.','Autoteste OK');
            else
                adicionarLog('AUTOTESTE INCOMPLETO: verifique o monitor de eventos.');
                uialert(fig,'Quadro de teste nao foi processado: veja os eventos e a Command Window.','Autoteste incompleto');
            end
        catch ME
            adicionarLog(['ERRO AUTOTESTE: ' ME.message]);
            fprintf(2,'[V4.0.4] AUTOTESTE: %s\n',getReport(ME,'extended','hyperlinks','off'));
            uialert(fig,ME.message,'Falha no autoteste');
        end
    end

    % ------------------------- Acoes de conexao -------------------------
    function atualizarPortas(~, ~)
        if ~isempty(portaSerial), return; end
        atual = 'COM6';
        if isvalid(ddPorta), atual = char(ddPorta.Value); end
        try
            detectadas = cellstr(serialportlist('all'));
        catch ME
            adicionarLog(['Falha ao listar portas: ' ME.message]);
            detectadas = {};
        end
        if isempty(detectadas), detectadas = {'COM6'}; end
        if ~any(strcmp(detectadas, atual))
            if any(strcmp(detectadas, 'COM6'))
                atual = 'COM6';
            else
                atual = detectadas{1};
            end
        end
        ddPorta.Items = detectadas;
        ddPorta.Value = atual;
    end

    function conectar(~, ~)
        if ~isempty(portaSerial), return; end
        nomeCOM = char(ddPorta.Value);
        try
            s = serialport(nomeCOM, 115200, 'Timeout', 1);
            flush(s, 'input');
            portaSerial = s;
            rx = uint8([]);
            pacotesOK = 0; errosCRC = 0; ultimaSequencia = NaN;
            rxBytes = 0; stmPacotes = 0; gpsPacotesRecebidos = 0;
            tipoDesconhecido = 0; errosProcessamento = 0; ultimoRxEm = NaN;
            ultimoSTMEm=NaN; ultimoGPSEm=NaN;
            ultimoGPSIdade=Inf;
            reiniciarSupervisaoRF();
            stmRemotoRecente=true;
            stmDadosValidos=false; bmpDadosValidos=false; gpsDadosValidos=false;
            ultimoGPSFixValido=false; ultimoGPSSats=0;
            stmPareamentoPendente=false;
            ultimoEstado="SEM DADOS";
            lastAlt=NaN;lastVz=NaN;lastPress=NaN;lastTemp=NaN;lastTempo=NaN;
            limparLeiturasDaTela();
            lbPacotes.Text = 'RX 0 B | 0 quadros';
            lbErros.Text = 'CRC: 0';
            plotT = []; plotAlt = []; plotAcc = [];
            plotVz=[];plotPress=[];plotTemp=[];plotGiro=[];
            gpsLat0=NaN;gpsLon0=NaN; gpsEast=[];gpsNorth=[];gpsHeights=[];
            anteriorRecebSTM=NaN;taxaSTM=NaN;
            attUltTempo=NaN;attRoll=NaN;attPitch=NaN;attYaw=0;attValida=false;
            reiniciarVisualAtitude();
            gpsLatitudes=[];gpsLongitudes=[]; gpsPacotes=0;gpsFixOK=0;
            linha3D.XData=NaN;linha3D.YData=NaN;linha3D.ZData=NaN;
            linhaRastro.XData=NaN;linhaRastro.YData=NaN;
            marcadorInicio2D.XData=NaN;marcadorInicio2D.YData=NaN;
            marcadorInicio3D.XData=NaN;marcadorInicio3D.YData=NaN;marcadorInicio3D.ZData=NaN;
            ponto3D.XData=NaN;ponto3D.YData=NaN;ponto3D.ZData=NaN;
            linhaEstacaoFoguete.XData=NaN;linhaEstacaoFoguete.YData=NaN;
            marcadorFoguete.XData=NaN;marcadorFoguete.YData=NaN;
            estLat=NaN;estLon=NaN;
            lbGPSFix.Text='GPS: aguardando';lbGPSCoord.Text='Latitude: --  |  Longitude: --';
            lbGPSSatelites.Text='Satelites: -- | Idade: -- ms';
            lbDist.Text='Distancia: -- | Azimute: --';
            if gpsCSV>=0, fclose(gpsCSV);gpsCSV=-1;end
            gpsArquivo=fullfile(pwd,['gps_stm_ao_vivo_' datestr(now,'yyyymmdd_HHMMSS') '.csv']);
            gpsCSV=fopen(gpsArquivo,'w');
            if gpsCSV>=0
                fprintf(gpsCSV,'tempo_stm_s;altitude_bmp_m;vel_vertical_bmp_m_s;latitude;longitude;fix_valido;recente;satelites;idade_gps_ms;stm_valido\n');
            end
            linhaAlt.XData = NaN; linhaAlt.YData = NaN;
            linhaAcc.XData = NaN; linhaAcc.YData = NaN;
            configurarBotoes(true);
            configureCallback(s, 'byte', 1, @receberUART);
            iniciarWatchdog();
            lbConexao.Text = ['ON  ' nomeCOM];
            lbConexao.FontColor = corVerde;
            adicionarLog(['CONECTADO em ' nomeCOM ' | 115200 baud | 8N1']);
            if gpsCSV>=0, adicionarLog(['CSV ao vivo: ' gpsArquivo]); end
        catch ME
            pararWatchdog();
            if exist('s', 'var')
                try, delete(s); catch, end
            end
            portaSerial = [];
            configurarBotoes(false);
            if gpsCSV>=0,fclose(gpsCSV);gpsCSV=-1;end
            adicionarLog(['ERRO AO ABRIR PORTA: ' ME.message]);
            uialert(fig, ME.message, 'Falha na conexao COM');
        end
    end

    function desconectar(~, ~)
        liberarSerial();
        adicionarLog('Desconectado.');
    end

    function liberarSerial()
        pararWatchdog();
        if ~isempty(portaSerial)
            s = portaSerial;
            portaSerial = [];
            try, configureCallback(s, 'off'); catch, end
            try, delete(s); catch, end
        end
        configurarBotoes(false);
        if gpsCSV>=0
            fclose(gpsCSV);gpsCSV=-1;
        end
        lbConexao.Text = 'DESCONECTADO';
        lbConexao.FontColor = corLaranja;
        lbRadio.Text='RF: desconectado'; lbRadio.FontColor=corLaranja;
        invalidarSTM('DESCONECTADO');
        invalidarGPS('DESCONECTADO');
    end

    function iniciarWatchdog()
        pararWatchdog();
        watchdogTimer=timer('ExecutionMode','fixedSpacing','Period',0.5, ...
            'BusyMode','drop','TimerFcn',@verificarValidadeDados);
        start(watchdogTimer);
    end

    function pararWatchdog()
        if ~isempty(watchdogTimer)
            try, stop(watchdogTimer); catch, end
            try, delete(watchdogTimer); catch, end
            watchdogTimer=[];
        end
    end

    function segundos=idadeDesde(referencia)
        if isempty(referencia) || ~isfinite(referencia)
            segundos=Inf;
        else
            segundos=toc(referencia);
        end
    end

    function reiniciarSupervisaoRF()
        ultimoDadoRFEm=NaN; ultimoAvancoRFEm=NaN; idadeAvancoRF=Inf;
        temTokenRF=false; tokenBootRF=0; tokenContadorRF=0; tokenTempoRF=0; tokenRespostasRF=0;
        lbRadio.Text='RF: aguardando dados'; lbRadio.FontColor=corLaranja;
    end

    function registrarDadoRF()
        % With this ESP bridge, a valid STM/GPS data frame is emitted only
        % after a newly accepted radio downlink. USB bytes alone are no proof.
        ultimoDadoRFEm=tic;
    end

    function registrarDiagnosticoRF(bootRP,contadorRP,tempoRP,respostasRF,idadeRF)
        if bootRP==0 || respostasRF==0, return; end
        mudouBoot=~temTokenRF || bootRP~=tokenBootRF;
        deltaRespostas=mod(respostasRF-tokenRespostasRF,4294967296);
        deltaTempo=mod(tempoRP-tokenTempoRF,4294967296);
        avancouRespostas=deltaRespostas>0 && deltaRespostas<2147483648;
        avancouRP=deltaTempo>0 && deltaTempo<2147483648 && contadorRP~=tokenContadorRF;
        if mudouBoot || avancouRespostas || avancouRP
            ultimoAvancoRFEm=tic; idadeAvancoRF=idadeRF;
            temTokenRF=true; tokenBootRF=bootRP; tokenContadorRF=contadorRP;
            tokenTempoRF=tempoRP; tokenRespostasRF=respostasRF;
        end
    end

    function idadeMS=idadeEnlaceRFms()
        % A delayed LINK snapshot must not invalidate a newer data frame.
        % Repeated local LINK reports with unchanged counters do not renew RF.
        idadeMS=min(1000*idadeDesde(ultimoDadoRFEm), ...
            idadeAvancoRF+1000*idadeDesde(ultimoAvancoRFEm));
    end

    function vivo=enlaceAtual()
        cfg=limitesPainelTTC();
        vivo=idadeDesde(ultimoRxEm)<=cfg.usb_s && idadeEnlaceRFms()<=cfg.rf_ms;
    end

    function verificarValidadeDados(~,~)
        if ~isvalid(fig) || isempty(portaSerial), return; end
        cfg=limitesPainelTTC();
        if idadeDesde(ultimoRxEm)>cfg.usb_s
            lbConexao.Text='USB SEM DADOS'; lbConexao.FontColor=corLaranja;
            lbRadio.Text='RF: sem atualizacao USB'; lbRadio.FontColor=corLaranja;
            invalidarSTM('LEITURA DESATUALIZADA'); invalidarGPS('LEITURA DESATUALIZADA');
            return;
        end
        lbConexao.Text=['ON  ' char(ddPorta.Value)]; lbConexao.FontColor=corVerde;
        idadeRF=idadeEnlaceRFms();
        if idadeRF<=cfg.rf_ms
            lbRadio.Text=sprintf('RF OK | ultima recepcao %.0f ms',idadeRF);
            lbRadio.FontColor=corVerde;
        else
            lbRadio.Text='RF: aguardando resposta'; lbRadio.FontColor=corLaranja;
        end
        if ~enlaceAtual()
            invalidarSTM('LEITURA DESATUALIZADA'); invalidarGPS('LEITURA DESATUALIZADA');
            return;
        end
        if idadeDesde(ultimoSTMEm)>cfg.stm_s || ~stmRemotoRecente
            invalidarSTM('LEITURA DESATUALIZADA');
        else
            lbAtualidadeSTM.Text=sprintf('Ultima leitura ha %.1f s',idadeDesde(ultimoSTMEm));
            lbAtualidadeSTM.FontColor=corVerde;
        end
        if isfinite(ultimoGPSEm) && ultimoGPSIdade+1000*idadeDesde(ultimoGPSEm)>cfg.gps_ms
            invalidarGPS('LEITURA DESATUALIZADA');
        end
    end

    function invalidarSTM(motivo)
        stmDadosValidos=false; bmpDadosValidos=false;
        stmPareamentoPendente=false;
        % Expiry changes validity, not the last successfully decoded values.
        % Flight state is a sensor datum; link state belongs in lbRadio.
        if isfinite(ultimoSTMEm)
            lbEstado.Text=char(ultimoEstado);
            lbAtualidadeSTM.Text=sprintf('Ultima leitura ha %.1f s (antiga)',idadeDesde(ultimoSTMEm));
        else
            lbEstado.Text='AGUARDANDO STM32';
            lbAtualidadeSTM.Text='Nenhuma leitura STM32 recebida';
        end
        lbEstado.FontColor=corLaranja; lbAtualidadeSTM.FontColor=corLaranja;
        lbImu.Text=['IMU: ultima leitura | ' motivo]; lbImu.FontColor=corLaranja;
        attValida=false; attUltTempo=NaN;
        lbAtInfo.Text=['Ultima atitude recebida | ' motivo];
    end

    function invalidarGPS(motivo)
        gpsDadosValidos=false;
        if isfinite(ultimoGPSEm)
            if ultimoGPSFixValido
                lbGPSFix.Text='GPS: ULTIMO FIX (leitura antiga)';
            else
                lbGPSFix.Text='GPS: ULTIMA LEITURA SEM FIX (antiga)';
            end
            idadeGPS=ultimoGPSIdade+1000*idadeDesde(ultimoGPSEm);
            lbGPSSatelites.Text=sprintf('Satelites na ultima leitura: %u | Idade: %.1f s', ...
                uint8(ultimoGPSSats),idadeGPS/1000);
        else
            lbGPSFix.Text='GPS: aguardando primeira leitura';
        end
        lbGPSFix.FontColor=corLaranja;
        lbGPSMapa.Text=['GPS: ultima leitura | ' motivo];
        lbDist.Text='Distancia: ultima posicao nao atualizada';
    end

    function limparLeiturasDaTela()
        % Clear display only for an explicit new COM session, never on RF age.
        lbEstado.Text='AGUARDANDO STM32'; lbEstado.FontColor=corLaranja;
        lbAlt.Text='-- m'; lbVz.Text='-- m/s'; lbPress.Text='-- Pa'; lbTemp.Text='-- C';
        lbTempo.Text='Tempo desde boot: -- s'; lbSeq.Text='Ultima sequencia: --';
        lbImu.Text='IMU: aguardando'; lbImu.FontColor=corLaranja;
        for k=1:3, lbAcc{k}.Text='--'; lbGyr{k}.Text='--'; end
        lbAtualidadeSTM.Text='Aguardando primeira leitura STM32';
        lbAtualidadeSTM.FontColor=corLaranja;
    end

    function valido=gpsAtualParaNavegacao()
        cfg=limitesPainelTTC();
        valido=gpsDadosValidos && enlaceAtual() && ...
            ultimoGPSIdade+1000*idadeDesde(ultimoGPSEm)<=cfg.gps_ms;
    end

    function valido=stmParaGPSAtual()
        % O ESP32 envia a telemetria antes do GPS do mesmo downlink.
        % Sem nova amostra STM nesta janela, nao combinar altitude antiga com GPS novo.
        % Esta associacao por recepcao nao implica sincronismo exato entre sensores.
        cfg=limitesPainelTTC();
        valido=stmPareamentoPendente && stmDadosValidos && bmpDadosValidos && enlaceAtual() && ...
            stmRemotoRecente && idadeDesde(ultimoSTMEm)<=cfg.pareamento_s;
    end

    function configurarBotoes(ligado)
        if ligado, e='on'; f='off'; else, e='off'; f='on'; end
        btnConectar.Enable = f;
        btnAtualizar.Enable = f;
        ddPorta.Enable = f;
        btnDesconectar.Enable = e;
        btnPing.Enable = e;
        btnV.Enable = 'off'; % V bloqueado no protocolo TTC V4.
        btnS.Enable = e;
        btnEnviarHex.Enable = e;
        btnA.Enable = e; % Comando A liberado no firmware de teste.
    end

    function aoFechar(~, ~)
        liberarSerial();
        delete(fig);
    end

    % -------------------------- Telecomandos ----------------------------
    function enviarHex(~, ~)
        texto = upper(strtrim(char(campoHex.Value)));
        if strncmp(texto, '0X', 2), texto = texto(3:end); end
        if isempty(regexp(texto, '^[0-9A-F]{2}$', 'once'))
            uialert(fig, 'Digite exatamente um byte HEX: 10, 01, 02 etc.', ...
                'Comando HEX invalido');
            return;
        end
        enviarComando(hex2dec(texto));
    end

    function enviarComando(cmd)
        if isempty(portaSerial)
            uialert(fig, 'Conecte primeiro o ESP32 pela COM USB.', 'Sem conexao');
            return;
        end
        if cmd == 2
            escolha = uiconfirm(fig, ...
                ['S remoto somente desarma quando ARMADO; ' ...
                 'a STM32 rejeita S durante GRAVANDO. Enviar?'], ...
                'Confirmar telecomando S', ...
                'Options', {'Enviar S', 'Cancelar'}, ...
                'DefaultOption', 2, 'CancelOption', 2);
            if ~strcmp(escolha, 'Enviar S'), return; end
        end
        if cmd == 3
            escolha = uiconfirm(fig, ...
                'ATENCAO: A apaga a Flash e arma o sistema. Executar somente em bancada segura, sem ignicao/atuadores conectados?', ...
                'Confirmar A (apagar + armar)', ...
                'Options', {'Enviar A','Cancelar'}, ...
                'DefaultOption', 2, 'CancelOption', 2);
            if ~strcmp(escolha, 'Enviar A'), return; end
        end
        quadro = montarQuadro(uint8(2), txSeq, uint8(cmd));
        seqEnviada = txSeq;
        txSeq = uint8(mod(double(txSeq)+1, 256));
        try
            write(portaSerial, quadro, 'uint8');
            adicionarLog(sprintf('TX CMD=0x%02X SEQ=%u | %s', ...
                cmd, seqEnviada, strtrim(sprintf('%02X ', quadro))));
            lbUlt.Text = sprintf('Ultimo TX: 0x%02X / SEQ %u', cmd, seqEnviada);
        catch ME
            adicionarLog(['ERRO NO ENVIO: ' ME.message]);
            uialert(fig, ME.message, 'Erro ao enviar');
        end
    end

    % ----------------------- Recepcao e parser --------------------------
    function receberUART(src, ~)
        if isempty(portaSerial), return; end
        try
            n = src.NumBytesAvailable;
            if n < 1, return; end
            bytes = read(src, n, 'uint8');
            rxBytes = rxBytes + numel(bytes);
            ultimoRxEm = tic;
            rx = [rx uint8(bytes(:).')]; %#ok<AGROW>
            % O parser atualiza a interface somente ao completar um quadro.
            % Leituras pequenas drenam a UART sem redesenhar o painel por byte.
            if numel(rx) > 200000
                rx = rx(end-5000:end);
                adicionarLog('AVISO: buffer UART reduzido por excesso de dados.');
            end
            interpretarBuffer();
        catch ME
            errosProcessamento=errosProcessamento+1;
            adicionarLog(['ERRO NA RECEPCAO: ' ME.message]);
            fprintf(2,'[V4.0.4] ERRO UART: %s\n', getReport(ME,'extended','hyperlinks','off'));
        end
    end

    function interpretarBuffer()
        while true
            if numel(rx) < 2, break; end
            pos = find(rx(1:end-1) == uint8(165) & rx(2:end) == uint8(90), 1);
            if isempty(pos)
                if rx(end) == uint8(165), rx = rx(end); else, rx = uint8([]); end
                break;
            end
            if pos > 1, rx = rx(pos:end); end
            if numel(rx) < 6, break; end
            tam = double(rx(6));
            if tam > 48
                rx = rx(2:end);
                continue;
            end
            total = tam + 8;
            if numel(rx) < total, break; end
            frame = rx(1:total);
            crcLocal = crc16ccitt(frame(3:end-2));
            crcRemoto = u16le(frame, total-1);
            if frame(3) ~= 1 || crcLocal ~= crcRemoto
                errosCRC = errosCRC + 1;
                lbErros.Text = sprintf('CRC erros: %d', errosCRC);
                if mod(errosCRC, 10) == 1
                    adicionarLog(sprintf('Quadro CRC/versao invalido, erros=%d', errosCRC));
                end
                rx = rx(2:end); % Ressincronizar sem perder possivel proximo sync.
                continue;
            end
            rx = rx(total+1:end);
            pacotesOK = pacotesOK + 1;
            lbPacotes.Text = sprintf('RX %d B | %d quadros',rxBytes,pacotesOK);
            tipo = frame(4);
            seq = frame(5);
            payload = frame(7:end-2);
            if cbHex.Value
                adicionarLog(sprintf('RX HEX [%d]: %s', total, strtrim(sprintf('%02X ', frame))));
            end
            try
                interpretarPacote(tipo, seq, payload);
                if tipo==1 || tipo==9, stmPacotes=stmPacotes+1; end
                if tipo==9 || tipo==32, gpsPacotesRecebidos=gpsPacotesRecebidos+1; end
                if tipo~=1 && tipo~=9 && tipo~=3 && tipo~=4 && tipo~=5 && tipo~=6 && tipo~=7 && tipo~=8 && tipo~=32 && tipo~=33 && tipo~=34
                    tipoDesconhecido=tipoDesconhecido+1;
                end
                % Os contadores mostram a diferenca entre USB, quadro valido e GPS.
                lbPacotes.Text=sprintf('STM %d | GPS %d',stmPacotes,gpsPacotesRecebidos);
            catch ME
                errosProcessamento=errosProcessamento+1;
                adicionarLog(['ERRO DECODIFICACAO: ' ME.message]);
                fprintf(2,'[V4.0.4] ERRO DECODIFICACAO: %s\n',getReport(ME,'extended','hyperlinks','off'));
            end
        end
    end

    function interpretarPacote(tipo, seq, p)
        switch double(tipo)
            case 1 % Telemetria: 29 bytes, 37 bytes no total.
                if numel(p) ~= 29
                    adicionarLog('Telemetria com tamanho inesperado.');
                    return;
                end
                registrarDadoRF();
                tempo = double(u32le(p,1)) / 1000;
                if isfinite(anteriorRecebSTM) && tempo>anteriorRecebSTM
                    f=1/(tempo-anteriorRecebSTM);
                    if f>0 && f<30
                        if isnan(taxaSTM),taxaSTM=f;
                        else,taxaSTM=0.8*taxaSTM+0.2*f;end
                    end
                end
                anteriorRecebSTM=tempo;
                if isfinite(taxaSTM)
                    subtitulo.Text=sprintf('ESP32 USB 115200 | Telemetria recebida: %.2f Hz',taxaSTM);
                end
                press = double(u32le(p,5));
                temp = double(i16le(p,9)) / 100;
                alt = double(i32le(p,11)) / 100;
                vz = double(i16le(p,15)) / 10;
                a = double([i16le(p,17),i16le(p,19),i16le(p,21)]) * 9.80665 / 1000;
                g = double([i16le(p,23),i16le(p,25),i16le(p,27)]) / 10;
                flags = p(29);
                codEstado = double(bitand(flags,uint8(3)));
                estados = {'ARMADO', 'GRAVANDO', 'PARADO', 'DESCONHECIDO'};
                ultimoEstado = string(estados{codEstado+1});
                imuOK = bitand(flags,uint8(4)) ~= 0;
                imuCal = bitand(flags,uint8(8)) ~= 0;
                bmpOK = bitand(flags,uint8(16)) ~= 0;
                ultimoSTMEm=tic; stmRemotoRecente=true;
                stmDadosValidos=true; bmpDadosValidos=bmpOK;
                stmPareamentoPendente=true;
                ultimaSequencia = double(seq);

                lbEstado.Text = char(ultimoEstado);
                lbAtualidadeSTM.Text='Leitura STM32 recebida agora';
                lbAtualidadeSTM.FontColor=corVerde;
                if codEstado == 1
                    lbEstado.FontColor = corVerde;
                elseif codEstado == 0
                    lbEstado.FontColor = corAzul;
                else
                    lbEstado.FontColor = corLaranja;
                end
                if bmpOK
                    lbAlt.Text = sprintf('%.2f m', alt);
                    lbVz.Text = sprintf('%.1f m/s', vz);
                    lbPress.Text = sprintf('%.0f Pa', press);
                    lbTemp.Text = sprintf('%.2f C', temp);
                else
                    lbAlt.Text='INV'; lbVz.Text='INV'; lbPress.Text='INV'; lbTemp.Text='INV';
                end
                lbTempo.Text = sprintf('Tempo desde boot: %.2f s', tempo);
                lbSeq.Text = sprintf('Ultima sequencia: %u', seq);
                if imuOK
                    for j=1:3
                        lbAcc{j}.Text = sprintf('%.3f', a(j));
                        lbGyr{j}.Text = sprintf('%.1f', g(j));
                    end
                    if imuCal
                        lbImu.Text = 'IMU: OK  |  giro calibrado';
                    else
                        lbImu.Text = 'IMU: OK  |  giro nao calibrado';
                    end
                    lbImu.FontColor = corVerde;
                else
                    for j=1:3
                        lbAcc{j}.Text = 'INV';
                        lbGyr{j}.Text = 'INV';
                    end
                    lbImu.Text = 'IMU: leitura INVALIDA';
                    lbImu.FontColor = corLaranja;
                end
                if ~bmpOK, adicionarLog('AVISO: BMP sinalizado como invalido.'); end
                lastAlt=alt;lastVz=vz;lastPress=press;lastTemp=temp;lastTempo=tempo;
                if bmpOK
                    atualizarGraficos(tempo,alt,a,imuOK,vz,press,temp,g);
                else
                    atualizarGraficos(tempo,NaN,a,imuOK,NaN,NaN,NaN,g);
                end
                try
                    atualizarAtitude(tempo,a,g,imuOK,imuCal);
                catch ME
                    % Uma falha grafica/3D nunca deve impedir os dados de sensores.
                    errosProcessamento=errosProcessamento+1;
                    if errosProcessamento<=5 || mod(errosProcessamento,20)==0
                        adicionarLog(['AVISO GRAFICO 3D: ' ME.message]);
                    end
                end

            case 9 % Compatibilidade: telemetria combinada da antiga V3.8.
                if numel(p)~=41
                    adicionarLog('V3.8: telemetria combinada com tamanho invalido.'); return;
                end
                interpretarPacote(uint8(1),seq,p(1:29));
                lat=double(i32le(p,30))*1e-7;
                lon=double(i32le(p,34))*1e-7;
                gpsFlags=p(38);sats=double(p(39));idade=double(u16le(p,40));
                recente=bitand(gpsFlags,uint8(2))~=0;
                fixOK=bitand(gpsFlags,uint8(1))~=0 && recente;
                atualizarGPSRecebido(lat,lon,sats,idade,fixOK,recente);

            case 32 % V4: GPS independente, 18 bytes do RP2040.
                if numel(p)~=18
                    adicionarLog('TTC V4: GPS com tamanho invalido.'); return;
                end
                registrarDadoRF();
                lat=double(i32le(p,1))*1e-7;
                lon=double(i32le(p,5))*1e-7;
                altMSL=double(i32le(p,9)); % milimetros (INT32_MIN = indisponivel)
                speed=double(u16le(p,13))/100; % cm/s -> m/s
                sats=double(p(15));gpsFlags=p(16);idade=double(u16le(p,17));
                recente=bitand(gpsFlags,uint8(4))~=0;
                fixOK=bitand(gpsFlags,uint8(2))~=0 && recente;
                fixOK=atualizarGPSRecebido(lat,lon,sats,idade,fixOK,recente);
                if altMSL<=-2147483648
                    lbGPSMapa.Text=sprintf('GPS: %.1f m/s',speed);
                else
                    lbGPSMapa.Text=sprintf('MSL %.0f m | %.1f m/s',altMSL/1000,speed);
                end
                if ~fixOK, lbGPSMapa.Text='GPS: posicao sem FIX valido'; end

            case 33 % V4: diagnostico link, 32 bytes.
                if numel(p)~=32
                    adicionarLog('TTC V4: LINK com tamanho invalido.'); return;
                end
                ageRF=double(u16le(p,15));
                rpRx=double(u16le(p,17));rpCRC=double(u16le(p,19));
                flagsRP=p(22);flagsESP=p(24);
                stmRemotoRecente=bitand(flagsRP,uint8(4))~=0;
                pollsRF=double(u32le(p,25));downlinksRF=double(u32le(p,29));
                registrarDiagnosticoRF(double(u32le(p,1)),double(u16le(p,9)), ...
                    double(u32le(p,5)),downlinksRF,ageRF);
                lbInfoCmd.Text=sprintf('RF: %u/%u respostas | idade no relatorio %u ms | STM UART %u | CRC %u | RP 0x%02X | ESP 0x%02X', ...
                    uint32(downlinksRF),uint32(pollsRF),uint32(ageRF),uint32(rpRx),uint32(rpCRC),flagsRP,flagsESP);
                verificarValidadeDados([],[]);

            case 34 % V4: fases do telecomando (nao confundir ACK RF com ACK STM32).
                if numel(p)~=14
                    adicionarLog('TTC V4: evento de comando com tamanho invalido.'); return;
                end
                ident=double(u32le(p,1));comando=p(5);seqCMD=p(6);
                fase=double(p(7));motivo=double(p(8));
                decorrido=double(u32le(p,9));tentativas=double(u16le(p,13));
                fases={'NA FILA','RF TRANSMITIDO','RP ACEITOU','UART STM32 ENVIADA', ...
                       'RESPOSTA STM32','BLOQUEIO LOCAL','RP OCUPADO', ...
                       'ENLACE INDETERMINADO','STM32 SEM RESPOSTA','RP REINICIOU', ...
                       'ESTACAO OCUPADA','CONFLITO ID / SEQ','RESULTADO EXPIRADO'};
                if fase>=0 && fase<numel(fases)
                    nomeFase=fases{fase+1};
                else
                    nomeFase=sprintf('FASE %u',uint8(fase));
                end
                adicionarLog(sprintf('[TTC V4] ID=%u SEQ=%u CMD=0x%02X | %s | motivo=%u | %u ms | tent=%u', ...
                    uint32(ident),seqCMD,comando,nomeFase,uint8(motivo),uint32(decorrido),uint16(tentativas)));
                lbUlt.Text=sprintf('TC 0x%02X: %s',comando,nomeFase);

            case 3 % ACK
                if numel(p) ~= 2, return; end
                msg = sprintf('ACK  SEQ=%u CMD=0x%02X | aceito (codigo=%d)', ...
                    seq, p(1), p(2));
                lbUlt.Text = sprintf('Resposta: ACK 0x%02X', p(1));
                adicionarLog(msg);

            case 4 % NACK
                if numel(p) ~= 2, return; end
                motivos = {'tamanho invalido','condicao indisponivel/insegura', ...
                    'comando bloqueado','comando desconhecido'};
                motivo = 'erro desconhecido';
                if p(2)>=1 && p(2)<=4, motivo = motivos{p(2)}; end
                lbUlt.Text = sprintf('Resposta: NACK 0x%02X', p(1));
                adicionarLog(sprintf('NACK SEQ=%u CMD=0x%02X COD=%u: %s', ...
                    seq, p(1), p(2), motivo));

            case 5 % STATUS do PING
                if numel(p) ~= 10, return; end
                estado = double(p(1));
                regs = double(u32le(p,3));
                uptime = double(u32le(p,7)) / 1000;
                lbReg.Text = sprintf('Registros na Flash: %.0f', regs);
                lbUlt.Text = 'Resposta: STATUS / PING OK';
                adicionarLog(sprintf('STATUS SEQ=%u estado=%u IMU=0x%02X registros=%u uptime=%.2fs', ...
                    seq, estado, p(2), uint32(regs), uptime));

            case 7 % LOG_INFO
                if numel(p) ~= 38
                    adicionarLog('LOG_INFO com tamanho invalido.'); return;
                end
                logEsperados = double(u32le(p,1));
                logVersao = double(p(5));
                logTamanho = double(p(6));
                hdr = p(7:38);
                gyroBias = [0 0 0];
                gyroCalibrado = false;
                if logVersao == 3
                    gyroBias = double([i16le(hdr,25),i16le(hdr,27),i16le(hdr,29)]);
                    gyroCalibrado = hdr(31) == 1;
                end
                logRows = containers.Map('KeyType','double','ValueType','any');
                lbTran.Text = sprintf('Recebendo log: 0 / %.0f', logEsperados);
                adicionarLog(sprintf('LOG_INFO versao=%u regBytes=%u total=%.0f', ...
                    logVersao, logTamanho, logEsperados));

            case 6 % LOG_RECORD
                if numel(p) < 4, return; end
                indice = double(u32le(p,1));
                raw = uint8(p(5:end));
                if numel(raw) ~= logTamanho || logTamanho == 0
                    adicionarLog(sprintf('LOG_RECORD indice=%.0f tamanho invalido', indice));
                    return;
                end
                logRows(indice) = raw;
                nRec = logRows.Count;
                if mod(nRec, 50) == 0 || nRec == logEsperados
                    lbTran.Text = sprintf('Recebendo log: %u / %.0f', nRec, logEsperados);
                end
                if mod(nRec, 500) == 0
                    adicionarLog(sprintf('... LOG %d/%.0f registros recebidos', nRec, logEsperados));
                end

            case 8 % LOG_END
                if numel(p) ~= 5, return; end
                enviados = double(u32le(p,1));
                erro = double(p(5));
                adicionarLog(sprintf('LOG_END enviados=%.0f erro=%u / recebidos=%u', ...
                    enviados, erro, logRows.Count));
                lbTran.Text = sprintf('Log encerrado: %u / %.0f', logRows.Count, logEsperados);
                if erro ~= 0
                    adicionarLog('ATENCAO: STM32 informou falha de leitura da Flash.');
                end
                exportarLogCSV(erro);

            otherwise
                adicionarLog(sprintf('Quadro TIPO=0x%02X SEQ=%u LEN=%u CRC OK', ...
                    tipo, seq, numel(p)));
        end
    end

    % --------------------- GPS V3.8 / TTC V4 ----------------------------
    function fixOK=atualizarGPSRecebido(lat,lon,sats,idade,fixOK,recente)
        cfg=limitesPainelTTC();
        gpsPacotes=gpsPacotes+1;
        ultimoGPSEm=tic; ultimoGPSIdade=idade;
        posicaoPlausivel=isfinite(lat)&&isfinite(lon)&&abs(lat)<=90&&abs(lon)<=180&&~(lat==0&&lon==0);
        fixOK=logical(fixOK)&&logical(recente)&&idade<=cfg.gps_ms&&posicaoPlausivel;
        gpsDadosValidos=fixOK;
        ultimoGPSFixValido=fixOK; ultimoGPSSats=sats;
        stmPareado=stmParaGPSAtual();
        stmPareamentoPendente=false;
        tempoCSV=NaN; altCSV=NaN; vzCSV=NaN;
        if stmPareado, tempoCSV=lastTempo; altCSV=lastAlt; vzCSV=lastVz; end
        lbGPSSatelites.Text=sprintf('Satelites: %u   |   Idade: %u ms',uint8(sats),uint16(idade));
        % Coordinates can be received without a valid navigation solution.
        % Display the raw values, but keep plotting/distance gated by FIX.
        if posicaoPlausivel
            if fixOK
                lbGPSCoord.Text=sprintf('Lat: %.7f   |   Lon: %.7f',lat,lon);
            else
                lbGPSCoord.Text=sprintf('Lat RX: %.7f   |   Lon RX: %.7f',lat,lon);
            end
        else
            lbGPSCoord.Text='Latitude: --  |  Longitude: -- (sem coordenadas)';
        end
        if fixOK
            lbGPSFix.Text='GPS: FIX VALIDO';lbGPSFix.FontColor=corVerde;
            gpsFixOK=gpsFixOK+1;
            if isnan(gpsLat0)
                gpsLat0=lat;gpsLon0=lon;
                adicionarLog('Origem da trajetoria definida na primeira posicao GPS valida.');
            end
            [leste,norte]=coordenadasLocais(lat,lon,gpsLat0,gpsLon0);
            gpsEast(end+1)=leste;gpsNorth(end+1)=norte;gpsHeights(end+1)=altCSV;
            gpsLatitudes(end+1)=lat;gpsLongitudes(end+1)=lon;
            if numel(gpsEast)>5000
                gpsEast=gpsEast(end-4999:end);gpsNorth=gpsNorth(end-4999:end);
                gpsHeights=gpsHeights(end-4999:end);
                gpsLatitudes=gpsLatitudes(end-4999:end);
                gpsLongitudes=gpsLongitudes(end-4999:end);
            end
            if toc(ultimoDesenhoGPS)>0.18
                ultimoDesenhoGPS=tic;
                linha3D.XData=gpsEast;linha3D.YData=gpsNorth;linha3D.ZData=gpsHeights;
                marcadorInicio3D.XData=gpsEast(1);marcadorInicio3D.YData=gpsNorth(1);marcadorInicio3D.ZData=gpsHeights(1);
                ponto3D.XData=gpsEast(end);ponto3D.YData=gpsNorth(end);ponto3D.ZData=gpsHeights(end);
                linhaRastro.XData=gpsEast;linhaRastro.YData=gpsNorth;
                marcadorInicio2D.XData=gpsEast(1);marcadorInicio2D.YData=gpsNorth(1);
                marcadorFoguete.XData=gpsEast(end);marcadorFoguete.YData=gpsNorth(end);
            end
            if ~isnan(estLat)
                [esL,esN]=coordenadasLocais(estLat,estLon,gpsLat0,gpsLon0);
                marcadorEstacao.XData=esL;marcadorEstacao.YData=esN;
                linhaEstacaoFoguete.XData=[esL gpsEast(end)];
                linhaEstacaoFoguete.YData=[esN gpsNorth(end)];
                [dist,az]=distanciaAzimute(estLat,estLon,lat,lon);
                lbDist.Text=sprintf('Distancia: %.0f m   |   Azimute: %.0f°',dist,az);
            else
                lbDist.Text='Distancia: defina a estacao terrestre';
            end
        else
            if recente && idade<=cfg.gps_ms
                lbGPSFix.Text='GPS: SEM FIX (coordenadas nao validadas)';
            else
                lbGPSFix.Text='GPS: DADOS DESATUALIZADOS / SEM FIX';
            end
            lbGPSFix.FontColor=corLaranja;
            lbDist.Text='Distancia: sem posicao valida atual';
        end
        if gpsCSV>=0
            fprintf(gpsCSV,'%.3f;%.3f;%.3f;%.7f;%.7f;%d;%d;%u;%u;%d\n',...
                tempoCSV,altCSV,vzCSV,lat,lon,fixOK,recente,uint8(sats),uint16(idade),stmPareado);
        end
    end

    % -------------------- Estimador de atitude (complementar) --------------------
    function atualizarAtitude(t,a,g,imuOK,gyroCalOK)
        if ~imuOK || any(~isfinite(a)) || any(~isfinite(g))
            attValida=false;attUltTempo=NaN;
            lbRoll.Text='Roll: --';lbPitch.Text='Pitch: --';lbYaw.Text='Yaw: --';
            lbAtInfo.Text='IMU sem dados validos';
            return;
        end
        a=a.*SINAL_ACEL;
        g=g.*SINAL_GIRO; % graus por segundo enviados pelo STM32
        accelG=norm(a)/9.80665;
        % Gravidade observavel apenas quando o vetor de aceleracao fica perto de 1g.
        confiarAcel=abs(accelG-1)<0.18;
        roA=atan2d(a(2),a(3));
        piA=atan2d(-a(1),hypot(a(2),a(3)));
        dt=NaN;
        if isfinite(attUltTempo),dt=t-attUltTempo;end
        if ~attValida || ~isfinite(dt) || dt<=0 || dt>1.2
            if confiarAcel
                attRoll=roA;attPitch=piA;
            elseif ~isfinite(attRoll) || ~isfinite(attPitch)
                attRoll=0;attPitch=0;
            end
            % Quando o STM reinicia, a referencia do yaw volta a zero.
            if isfinite(dt) && dt<=0,attYaw=0;end
        else
            % Integrar todo o intervalo valido, incluindo a cadencia RF normal de 600 ms.
            attRoll=attRoll+g(1)*dt;
            attPitch=attPitch+g(2)*dt;
            attYaw=mod(attYaw+g(3)*dt+180,360)-180;
            if confiarAcel
                % Constante temporal de 3 s: corrige inclinacao lenta sem
                % tratar aceleracao do motor como gravidade.
                peso=exp(-dt/3);
                attRoll=peso*attRoll+(1-peso)*roA;
                attPitch=peso*attPitch+(1-peso)*piA;
            end
        end
        attUltTempo=t;
        attValida=true;
        attRoll=mod(attRoll+180,360)-180;
        attPitch=max(-89.9,min(89.9,attPitch));
        lbRoll.Text=sprintf('Roll: %+.1f°',attRoll);
        lbPitch.Text=sprintf('Pitch: %+.1f°',attPitch);
        lbYaw.Text=sprintf('Yaw: %+.1f°',attYaw);
        if confiarAcel
            attConfianca='TILT: GIRO + GRAVIDADE';
        else
            attConfianca='TILT: APENAS GIRO (ACELERACAO DINAMICA)';
        end
        if ~gyroCalOK
            lbAtInfo.Text=[attConfianca ' | GIRO NAO CALIBRADO'];
            lbAtAviso.Text='Yaw relativo com deriva';
        else
            lbAtInfo.Text=attConfianca;
            lbAtAviso.Text='Yaw relativo: sem norte absoluto';
        end
        desenharAtitude(false);
    end

    function reiniciarVisualAtitude()
        grupoFoguete.Matrix=eye(4);
        configurarHorizonte(0,0);
        lbAtInfo.Text='Aguardando IMU | yaw relativo';
        lbRoll.Text='Roll: --';lbPitch.Text='Pitch: --';lbYaw.Text='Yaw: --';
    end

    function reiniciarAtitude(~,~)
        % Redefine yaw e permite reestimar roll/pitch no proximo pacote.
        attValida=false;attUltTempo=NaN;
        attRoll=NaN;attPitch=NaN;attYaw=0;
        reiniciarVisualAtitude();
        adicionarLog('Estimativa de atitude reiniciada. Yaw relativo redefinido em 0 graus.');
    end

    function selecionarAba(~,~)
        if abas.SelectedTab==abaAtitude,desenharAtitude(true);end
    end

    function configurarHorizonte(roll,pitch)
        % Horizonte move na direcao oposta da arfagem; a inclinacao do
        % horizonte muda com o roll (convencao ilustrativa, ajuste por montagem).
        sh=-max(-1.2,min(1.2,pitch/42));
        ang=-roll*pi/180;
        rot=[cos(ang),-sin(ang);sin(ang),cos(ang)];
        pontosCeu=rot*[-3,3,3,-3; sh,sh,sh+3,sh+3];
        pontosSolo=rot*[-3,3,3,-3;sh-3,sh-3,sh,sh];
        linha=rot*[-3,3;sh,sh];
        skyPatch.XData=pontosCeu(1,:);skyPatch.YData=pontosCeu(2,:);
        landPatch.XData=pontosSolo(1,:);landPatch.YData=pontosSolo(2,:);
        linhaHorizonte.XData=linha(1,:);linhaHorizonte.YData=linha(2,:);
    end

    function desenharAtitude(forcar)
        if ~attValida,return;end
        if ~forcar && (abas.SelectedTab~=abaAtitude || toc(attDesenho)<0.16)
            return;
        end
        attDesenho=tic;
        configurarHorizonte(attRoll,attPitch);
        % Convencao da imagem: corpo do foguete apontando ao longo de +Z.
        r=attRoll*pi/180;p=attPitch*pi/180;y=attYaw*pi/180;
        rxm=[1,0,0;0,cos(r),-sin(r);0,sin(r),cos(r)];
        rym=[cos(p),0,sin(p);0,1,0;-sin(p),0,cos(p)];
        rzm=[cos(y),-sin(y),0;sin(y),cos(y),0;0,0,1];
        M=eye(4);M(1:3,1:3)=rzm*rym*rxm;
        grupoFoguete.Matrix=M;
    end

    function marcarPontoInicial(~,~)
        if isempty(gpsLatitudes) || isempty(gpsLongitudes) || ~gpsAtualParaNavegacao()
            uialert(fig,'Aguarde uma coordenada GPS com FIX valido antes de marcar a origem.','GPS sem FIX');
            return;
        end
        gpsLat0=gpsLatitudes(end);gpsLon0=gpsLongitudes(end);
        altInicio=gpsHeights(end);
        gpsEast=0;gpsNorth=0;gpsHeights=altInicio;
        gpsLatitudes=gpsLat0;gpsLongitudes=gpsLon0;
        linhaRastro.XData=0;linhaRastro.YData=0;
        marcadorInicio2D.XData=0;marcadorInicio2D.YData=0;
        marcadorFoguete.XData=0;marcadorFoguete.YData=0;
        linha3D.XData=0;linha3D.YData=0;linha3D.ZData=altInicio;
        marcadorInicio3D.XData=0;marcadorInicio3D.YData=0;marcadorInicio3D.ZData=altInicio;
        ponto3D.XData=0;ponto3D.YData=0;ponto3D.ZData=altInicio;
        if ~isnan(estLat)
            [e,n]=coordenadasLocais(estLat,estLon,gpsLat0,gpsLon0);
            marcadorEstacao.XData=e;marcadorEstacao.YData=n;
            linhaEstacaoFoguete.XData=[e,0];
            linhaEstacaoFoguete.YData=[n,0];
        end
        adicionarLog(sprintf('Ponto inicial do mapa redefinido: %.7f, %.7f. O CSV continua intacto.',gpsLat0,gpsLon0));
    end

    function limparGraficos(~,~)
        plotT=[];plotAlt=[];plotAcc=[];plotVz=[];plotPress=[];
        plotTemp=[];plotGiro=[];
        alterarGraficos([],[]);
        adicionarLog('Graficos limpos. O CSV gravado nao foi apagado.');
    end

    function atualizarGraficos(t,alt,a,imuOK,vz,press,temp,g)
        if ~isempty(plotT) && t<plotT(end)
            plotT=[];plotAlt=[];plotAcc=[];plotVz=[];plotPress=[];plotTemp=[];plotGiro=[];
        end
        plotT(end+1)=t;plotAlt(end+1)=alt;plotVz(end+1)=vz;
        plotPress(end+1)=press;plotTemp(end+1)=temp;
        if imuOK
            plotAcc(end+1)=norm(a);plotGiro(end+1)=norm(g);
        else
            plotAcc(end+1)=NaN;plotGiro(end+1)=NaN;
        end
        recente=plotT>=t-60;
        plotT=plotT(recente);plotAlt=plotAlt(recente);plotAcc=plotAcc(recente);
        plotVz=plotVz(recente);plotPress=plotPress(recente);
        plotTemp=plotTemp(recente);plotGiro=plotGiro(recente);
        if toc(ultPlot)<0.18,return;end
        ultPlot=tic;alterarGraficos([],[]);
    end
    function alterarGraficos(~,~)
        linhaAlt.XData=plotT;linhaAlt.YData=serieGrafico(ddGraf1.Value);
        linhaAcc.XData=plotT;linhaAcc.YData=serieGrafico(ddGraf2.Value);
        title(axAlt,char(ddGraf1.Value));title(axAcc,char(ddGraf2.Value));
    end
    function dados=serieGrafico(nome)
        switch char(nome)
            case 'Altitude (m)',dados=plotAlt;
            case 'Velocidade vertical (m/s)',dados=plotVz;
            case 'Aceleracao (m/s2)',dados=plotAcc;
            case 'Pressao (Pa)',dados=plotPress;
            case 'Temperatura (C)',dados=plotTemp;
            otherwise,dados=plotGiro;
        end
    end
    function definirEstacao(~,~)
        lat=campoEstLat.Value;lon=campoEstLon.Value;
        if ~isfinite(lat)||~isfinite(lon)||abs(lat)>90||abs(lon)>180
            uialert(fig,'Latitude deve ser -90 a 90 e longitude -180 a 180.','Coordenadas invalidas');
            return;
        end
        estLat=lat;estLon=lon;
        adicionarLog(sprintf('Estacao terrestre definida: %.7f, %.7f',estLat,estLon));
        if ~isnan(gpsLat0)
            [e,n]=coordenadasLocais(estLat,estLon,gpsLat0,gpsLon0);
            marcadorEstacao.XData=e;marcadorEstacao.YData=n;
            if ~isempty(gpsLatitudes) && gpsAtualParaNavegacao()
                [d,az]=distanciaAzimute(estLat,estLon,gpsLatitudes(end),gpsLongitudes(end));
                lbDist.Text=sprintf('Distancia: %.0f m   |   Azimute: %.0f°',d,az);
                linhaEstacaoFoguete.XData=[e gpsEast(end)];
                linhaEstacaoFoguete.YData=[n gpsNorth(end)];
            else
                lbDist.Text='Distancia: aguardando posicao GPS atual';
            end
        end
    end

    % ---------------------- Exportacao do log CSV -----------------------
    function exportarLogCSV(erroSTM)
        qtd = logRows.Count;
        if qtd == 0
            adicionarLog('Nao ha registros recebidos para exportar.');
            return;
        end
        if ~((logVersao==2 && logTamanho==16) || (logVersao==3 && logTamanho==32))
            adicionarLog(sprintf('Versao Flash nao suportada no CSV: v%d / %d bytes', ...
                logVersao, logTamanho));
            return;
        end
        faltantes = max(0, logEsperados-qtd);
        incompleto = (faltantes~=0 || erroSTM~=0);
        sufixo = '';
        if incompleto, sufixo = '_INCOMPLETO'; end
        caminho = fullfile(pwd, ['voo_ttl_' datestr(now,'yyyymmdd_HHMMSS') sufixo '.csv']);
        fid = fopen(caminho, 'w');
        if fid < 0
            adicionarLog(['Nao foi possivel criar CSV: ' caminho]);
            return;
        end
        fechamento = onCleanup(@() fclose(fid)); %#ok<NASGU>
        fprintf(fid, ['registro;tempo_voo_ms;altitude_m;pressao_Pa;temperatura_C;' ...
            'accX_m_s2;accY_m_s2;accZ_m_s2;gyroX_dps;gyroY_dps;gyroZ_dps;' ...
            'flags;imu_status;crc8_ok\n']);
        indices = sort(cell2mat(keys(logRows)));
        crcRuins = 0;
        for k = 1:numel(indices)
            idx = indices(k);
            raw = logRows(idx);
            bom = double(crc8atm(raw(1:end-1)) == raw(end));
            if ~bom, crcRuins=crcRuins+1; end
            if logVersao == 2
                ms = i32le(raw,1);
                press = float32le(raw,5);
                alt = float32le(raw,9);
                tc = double(i16le(raw,13))/100;
                flags = double(raw(15));
                imuStat = NaN;
                a = [NaN NaN NaN]; g = [NaN NaN NaN];
            else
                ms = i32le(raw,1);
                press = float32le(raw,5);
                alt = float32le(raw,9);
                tc = double(i16le(raw,13))/100;
                acelRaw = double([i16le(raw,15), i16le(raw,17), i16le(raw,19)]);
                giroRaw = double([i16le(raw,21), i16le(raw,23), i16le(raw,25)]);
                flags = double(raw(27));
                imuStat = double(raw(28));
                a = [NaN NaN NaN]; g = [NaN NaN NaN];
                if bitand(uint8(imuStat),uint8(1)) ~= 0
                    a = acelRaw * (9.80665/2048);
                    bias = [0 0 0];
                    if bitand(uint8(imuStat),uint8(2)) ~= 0 && gyroCalibrado
                        bias = gyroBias;
                    end
                    g = (giroRaw-bias)/16.4;
                end
            end
            fprintf(fid, ['%u;%d;%.4f;%.2f;%.2f;' ...
                '%.4f;%.4f;%.4f;%.3f;%.3f;%.3f;%.0f;%.0f;%d\n'], ...
                idx+1, ms, alt, press, tc, a(1),a(2),a(3),g(1),g(2),g(3), ...
                flags, imuStat, bom);
        end
        if incompleto
            adicionarLog(sprintf('ATENCAO: log INCOMPLETO! %d registros ausentes.', faltantes));
        end
        if crcRuins > 0
            adicionarLog(sprintf('ATENCAO: %u registros com CRC8 incorreto.', crcRuins));
        end
        adicionarLog(sprintf('CSV salvo: %s | registros=%d', caminho, qtd));
        lbTran.Text = sprintf('CSV salvo (%d registros; faltantes=%d)', qtd, faltantes);
    end

    function adicionarLog(msg)
        if ~isvalid(fig), return; end
        linha = string(sprintf('[%s] %s', datestr(now,'HH:MM:SS'), char(msg)));
        linhasLog = [linha; linhasLog]; %#ok<AGROW>
        if numel(linhasLog) > maxLinhasLog
            linhasLog = linhasLog(1:maxLinhasLog);
        end
        if isvalid(areaLog)
            areaLog.Value = cellstr(linhasLog);
        end
    end
end

% ------------------ Funcoes locais auxiliares ---------------------------
function cfg=limitesPainelTTC()
    % Local helper: callbacks do not capture timeout constants from the GUI
    % workspace. Close and reopen the panel after replacing this source file.
    cfg=struct('usb_s',2.5,'rf_ms',2000,'stm_s',1.5, ...
        'gps_ms',1200,'pareamento_s',0.35);
end

function lab = parDeMedida(grid, linha, nome, inicial)
    x = uilabel(grid, 'Text', nome, 'FontSize', 12);
    x.Layout.Row = linha;
    x.Layout.Column = 1;
    lab = uilabel(grid, 'Text', inicial, 'FontWeight', 'bold', ...
        'FontSize', 13, 'HorizontalAlignment', 'right');
    lab.Layout.Row = linha;
    lab.Layout.Column = 2;
end

function lab = cabImu(grid, linha, coluna, str)
    lab = uilabel(grid, 'Text', str, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', 'FontSize', 11);
    lab.Layout.Row = linha;
    lab.Layout.Column = coluna;
end

function lab = uiCellLabel(grid, coluna, str, tam, cor)
    lab = uilabel(grid, 'Text', str, 'FontSize', tam, 'FontColor', cor);
    lab.Layout.Column = coluna;
end

function val = u16le(bytes, i)
    val = uint16(bytes(i)) + bitshift(uint16(bytes(i+1)),8);
end

function val = i16le(bytes,i)
    val = typecast(u16le(bytes,i),'int16');
end

function val = u32le(bytes,i)
    val = uint32(bytes(i)) + bitshift(uint32(bytes(i+1)),8) + ...
        bitshift(uint32(bytes(i+2)),16) + bitshift(uint32(bytes(i+3)),24);
end

function val = i32le(bytes,i)
    val = typecast(u32le(bytes,i),'int32');
end

function val = float32le(bytes,i)
    val = double(typecast(uint8(bytes(i:i+3)),'single'));
end

function crc = crc16ccitt(bytes)
    crc = uint16(65535);
    for b = uint8(bytes)
        crc = bitxor(crc,bitshift(uint16(b),8));
        for k = 1:8
            if bitand(crc,uint16(32768)) ~= 0
                crc = bitxor(bitshift(crc,1),uint16(hex2dec('1021')));
            else
                crc = bitshift(crc,1);
            end
        end
    end
end

function quadro = montarQuadro(tipo, sequencia, payload)
    payload = uint8(payload(:).');
    corpo = uint8([1 double(tipo) double(sequencia) double(numel(payload)) double(payload)]);
    crc = crc16ccitt(corpo);
    crcLo = uint8(bitand(crc,uint16(255)));
    crcHi = uint8(bitshift(crc,-8));
    quadro = [uint8([165 90]) corpo crcLo crcHi];
end

function crc = crc8atm(bytes)
    crc = uint8(0);
    for b = uint8(bytes)
        crc = bitxor(crc,b);
        for k=1:8
            if bitand(crc,uint8(128)) ~= 0
                crc=bitxor(bitshift(crc,1),uint8(7));
            else
                crc=bitshift(crc,1);
            end
        end
    end
end

% Aproximacao ENU local adequada para rastreamento de curta distancia.
function [este,norte]=coordenadasLocais(lat,lon,lat0,lon0)
    R=6371000;
    norte=R*(lat-lat0)*pi/180;
    este=R*cos((lat+lat0)*pi/360)*(lon-lon0)*pi/180;
end

function [dist,az]=distanciaAzimute(lat1,lon1,lat2,lon2)
    R=6371000;
    p1=lat1*pi/180;p2=lat2*pi/180;
    dp=(lat2-lat1)*pi/180;dl=(lon2-lon1)*pi/180;
    h=sin(dp/2)^2+cos(p1)*cos(p2)*sin(dl/2)^2;
    dist=2*R*atan2(sqrt(h),sqrt(max(0,1-h)));
    az=mod(atan2(sin(dl)*cos(p2),cos(p1)*sin(p2)-sin(p1)*cos(p2)*cos(dl))*180/pi+360,360);
end
