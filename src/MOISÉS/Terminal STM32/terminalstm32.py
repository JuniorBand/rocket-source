import os
import sys
import requests
import zipfile
import shutil
import subprocess
from pathlib import Path
import json

# ==============================================================================
# FUNÇÃO CRÍTICA PARA DETERMINAR O CAMINHO REAL DO EXECUTÁVEL
# ==============================================================================
def get_exe_directory():
    """Garante que o caminho da pasta onde o .exe está rodando seja detectado corretamente,
    não importa como ele foi chamado (terminal, IDE ou atalho)."""
    if getattr(sys, 'frozen', False):
        # Se estiver rodando como um executável .exe compilado (pelo PyInstaller)
        return os.path.dirname(sys.executable)
    else:
        # Se estiver rodando como um script .py normal (terminal, IDE)
        return os.path.dirname(os.path.realpath(__file__))

# Define os caminhos de arquivo baseados na pasta onde o executável está
EXE_DIR = get_exe_directory()
# Garante as aspas no caminho para o JSON caso haja espaços
ARQUIVO_CONFIG = os.path.join(EXE_DIR, "config_terminal.json")
VERSION_FILE = os.path.join(EXE_DIR, "versao_atual.txt")
ICONE_FILE_NAME = "icone.ico" # O ícone deve estar na pasta do .exe com este nome

# Configurações do repositório
REPO_OWNER = "JuniorBand"
REPO_NAME = "rocket-source"

# ==============================================================================
# SISTEMA DE CONFIGURAÇÃO (JSON)
# ==============================================================================
def salvar_config(caminho_teraterm, pasta_download):
    """Salva a descoberta no arquivo JSON para as próximas vezes."""
    dados = {
        "caminho_teraterm": caminho_teraterm,
        "pasta_download": pasta_download
    }
    with open(ARQUIVO_CONFIG, "w") as f:
        json.dump(dados, f, indent=4)

def carregar_config():
    """Carrega as configurações salvas se o arquivo existir."""
    if os.path.exists(ARQUIVO_CONFIG):
        with open(ARQUIVO_CONFIG, "r") as f:
            return json.load(f)
    return {}

def obter_versao_local():
    """Lê a versão atual salva no computador."""
    if os.path.exists(VERSION_FILE):
        with open(VERSION_FILE, "r") as f:
            return f.read().strip()
    return None

def salvar_versao_local(versao):
    """Salva a nova versão no computador."""
    with open(VERSION_FILE, "w") as f:
        f.write(versao)

# ==============================================================================
# FUNÇÃO PARA DESCOBRIR A PASTA DE DOWNLOAD
# ==============================================================================
def obter_pasta_download(config):
    """Descobre a pasta de download, salvando em JSON para execuções futuras."""
    # 1. Verifica se já existe uma configuração salva
    pasta_salva = config.get("pasta_download", "")
    if pasta_salva and os.path.exists(pasta_salva):
        return pasta_salva

    # 2. Se não existir, pede para o usuário
    print("\n--- Configuração da Pasta de Download ---")
    caminho_input = input("Onde você deseja salvar a pasta RAD atualizada?\nExemplo: C:\\Users\\usuario\\STM32\\Projeto_X\n> ").strip().strip('"').strip("'")
    
    os.makedirs(caminho_input, exist_ok=True)
    return caminho_input

# ==============================================================================
# FUNÇÃO PARA ENCONTRAR O TERA TERM
# ==============================================================================
def obter_caminho_teraterm(config):
    """Descobre o caminho do Tera Term, salvando em JSON para execuções futuras."""
    caminho_salvo = config.get("caminho_teraterm", "")
    if caminho_salvo and os.path.exists(caminho_salvo):
        return caminho_salvo

    locais_padrao = [
        r"C:\Program Files\teraterm\ttermpro.exe",
        r"C:\Program Files (x86)\teraterm\ttermpro.exe",
        r"C:\teraterm\ttermpro.exe"
    ]

    caminho_encontrado = None
    print("\nProcurando o Tera Term no sistema...")
    for caminho in locais_padrao:
        if os.path.exists(caminho):
            caminho_encontrado = caminho
            print(f"Tera Term encontrado automaticamente em: {caminho}")
            break

    if not caminho_encontrado:
        print("\nO Tera Term não foi encontrado nos locais padrão.")
        caminho_input = input("Por favor, arraste o executável 'ttermpro.exe' para esta janela ou cole o caminho completo aqui e dê Enter:\n> ").strip()
        caminho_encontrado = caminho_input.strip('"').strip("'")

    if os.path.exists(caminho_encontrado):
        return caminho_encontrado
    else:
        print("Erro: O caminho fornecido não é válido ou o arquivo não existe.")
        return None

# ==============================================================================
# ORGANIZAÇÃO DE ARQUIVOS (MUDADO PARA USAR O CAMINHO DE DOWNLOAD CORRETO)
# ==============================================================================
def organizar_arquivos(caminho_zip, pasta_download):
    """Extrai o ZIP para uma pasta temporária, localiza a pasta RAD, move para o destino e limpa o lixo."""
    print(f"Extraindo {caminho_zip}...")
    
    pasta_temp = os.path.join(pasta_download, "temp_extracao")
    if os.path.exists(pasta_temp):
        shutil.rmtree(pasta_temp)
    os.makedirs(pasta_temp)
    
    with zipfile.ZipFile(caminho_zip, 'r') as zip_ref:
        zip_ref.extractall(pasta_temp)

    caminho_rad_encontrado = None
    for root, dirs, files in os.walk(pasta_temp):
        if "RAD" in dirs:
            caminho_rad_encontrado = os.path.join(root, "RAD")
            break

    if caminho_rad_encontrado:
        caminho_pasta_rad_destino = os.path.join(pasta_download, "RAD")
        
        if os.path.exists(caminho_pasta_rad_destino):
            print("Removendo a versão anterior da pasta RAD...")
            shutil.rmtree(caminho_pasta_rad_destino)
            
        print("Movendo a nova pasta RAD para o diretório principal...")
        shutil.move(caminho_rad_encontrado, caminho_pasta_rad_destino)
        
        print("Limpando arquivos temporários e o .zip...")
        shutil.rmtree(pasta_temp)  
        os.remove(caminho_zip)     
        
        caminho_absoluto = os.path.abspath(caminho_pasta_rad_destino)
        print("\n=====================================================")
        print("✅ Atualização concluída com sucesso!")
        print(f"A pasta foi salva EXATAMENTE neste local do Windows:")
        print(f"-> {caminho_absoluto}")
        print("=====================================================\n")
        print("Dica: Se estiver usando STM32CubeIDE, aperte F5 no projeto para atualizar a visualização.")
    else:
        print("Erro: A pasta 'RAD' não foi encontrada dentro do arquivo .zip.")
        shutil.rmtree(pasta_temp)

# ==============================================================================
# VERIFICAÇÃO E DOWNLOAD NO GITHUB (MUDADO FILTRO DE RELEASE)
# ==============================================================================
def verificar_e_baixar_atualizacao(pasta_download):
    # Mudança para pegar todos os releases em vez de apenas o latest
    url_api = f"https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/releases"
    
    print(f"Verificando atualizações em {REPO_OWNER}/{REPO_NAME}...")
    resposta = requests.get(url_api)

    if resposta.status_code != 200:
        print("Erro: Não foi possível acessar a API.")
        return

    lista_releases = resposta.json()
    dados_release = None

    # Procura o primeiro release que contenha "RAD_v." ou "RAD-v." (ignorando maiúsculas)
    for release in lista_releases:
        tag = release.get("tag_name", "").upper()
        # Mudei para aceitar tanto traço quanto underline, sem o ponto depois do v
        if "RAD_V" in tag or "RAD-V" in tag:
            dados_release = release
            break

    if not dados_release:
        print("Erro: Nenhum release contendo 'RAD_v' ou 'RAD-v' foi encontrado neste repositório.")
        return

    versao_mais_recente = dados_release.get("tag_name")
    versao_local = obter_versao_local()

    if versao_local == versao_mais_recente:
        print(f"Nenhuma atualização necessária. A versão local ({versao_local}) já é a mais recente válida.")
        return

    print(f"Nova versão válida encontrada: {versao_mais_recente}")

    if dados_release.get("assets"):
        asset = dados_release["assets"][0]
        url_download = asset["browser_download_url"]
        nome_arquivo = asset["name"]
    else:
        url_download = dados_release.get("zipball_url")
        nome_arquivo = f"{REPO_NAME}-{versao_mais_recente}.zip"

    if not url_download:
        print("Erro: Nenhum arquivo disponível para download neste release.")
        return

    os.makedirs(pasta_download, exist_ok=True)
    caminho_arquivo = os.path.join(pasta_download, nome_arquivo)

    print(f"Baixando {nome_arquivo}...")
    
    resposta_download = requests.get(url_download, stream=True)
    with open(caminho_arquivo, "wb") as f:
        for chunk in resposta_download.iter_content(chunk_size=8192):
            if chunk:
                f.write(chunk)

    organizar_arquivos(caminho_arquivo, pasta_download)
    salvar_versao_local(versao_mais_recente)

# ==============================================================================
# FUNÇÃO PROFISSONAL PARA CRIAR ATALHO NO DESKTOP (A CORREÇÃO ESTÁ AQUI)
# ==============================================================================
def criar_atalho_desktop():
    """Cria um atalho na Área de Trabalho com ícone e pasta de início correta."""
    if getattr(sys, 'frozen', False):
        caminho_exe = sys.executable
        # Pega a pasta real onde o .exe está rodando
        diretorio_exe = get_exe_directory()
        
        # O arquivo .ico deve estar na pasta do .exe com o nome 'icone.ico'
        caminho_icone = os.path.join(diretorio_exe, ICONE_FILE_NAME)
        
        desktop = os.path.join(os.environ["USERPROFILE"], "Desktop")
        caminho_atalho = os.path.join(desktop, "Terminal STM32.lnk") # Nome do atalho
        
        # Só cria se o atalho ainda não existir
        if not os.path.exists(caminho_atalho):
            print("\nCriando atalho na Área de Trabalho...")
            vbs_path = os.path.join(os.environ["TEMP"], "criar_atalho.vbs")
            
            # Escreve o script VBScript para criar o atalho
            with open(vbs_path, "w") as f:
                f.write('Set oWS = WScript.CreateObject("WScript.Shell")\n')
                f.write(f'sLinkFile = "{caminho_atalho}"\n')
                f.write('Set oLink = oWS.CreateShortcut(sLinkFile)\n')
                f.write(f'oLink.TargetPath = "{caminho_exe}"\n')
                
                # ==========================================================
                # LINHA DA CORREÇÃO MÁGICA: Define o 'Iniciar em' do atalho
                # ==========================================================
                f.write(f'oLink.WorkingDirectory = "{diretorio_exe}"\n')
                
                # Adiciona o ícone ao atalho caso o arquivo .ico exista na pasta
                if os.path.exists(caminho_icone):
                    f.write(f'oLink.IconLocation = "{caminho_icone}, 0"\n')
                
                f.write('oLink.Save\n')
            
            # Executa o script silenciosamente e apaga o temporário
            subprocess.call(['cscript.exe', '//Nologo', vbs_path])
            if os.path.exists(vbs_path):
                os.remove(vbs_path)
            print("✅ Atalho criado automaticamente!")

# ==============================================================================
# BLOCO PRINCIPAL
# ==============================================================================
if __name__ == "__main__":
    print(f"Diretório Base Detectado: {EXE_DIR}\n")
    
    # 0. Tenta criar o atalho na Área de Trabalho (só funciona no .exe)
    criar_atalho_desktop()
    
    # 1. Carrega as configurações existentes
    config = carregar_config()
    
    # 2. Pede/Lê a pasta de download desejada
    pasta_download = obter_pasta_download(config)
    
    # 3. Descobre (ou lê do JSON) o caminho do terminal
    caminho_teraterm = obter_caminho_teraterm(config)
    
    # 4. Salva as novas configurações, mantendo as antigas intactas
    salvar_config(caminho_teraterm, pasta_download)
    
    # 5. Faz a lógica de baixar a atualização enviando a pasta correta
    verificar_e_baixar_atualizacao(pasta_download)
    
    # 6. Abre o terminal
    if caminho_teraterm:
        print("\nAbrindo o terminal...")
        subprocess.Popen([caminho_teraterm])
    else:
        print("\nNão foi possível iniciar o terminal.")
        input("Pressione Enter para fechar...")