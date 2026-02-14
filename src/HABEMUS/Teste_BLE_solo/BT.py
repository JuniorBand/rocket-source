import asyncio
import csv
from datetime import datetime
from bleak import BleakClient, BleakScanner, BleakError

# --- CONFIGURAÇÕES ---
DEVICE_NAME = "HABEMUS_S3_ROCKET"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# Nome do arquivo único por execução do script
NOME_ARQUIVO = f"telemetria_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
INTERVALO_SALVAMENTO = 5  # Segundos

# Variáveis Globais
data_buffer = []
stop_event = asyncio.Event() # Para controlar o encerramento limpo (Ctrl+C)

def notification_handler(sender, data):
    """Recebe dados e joga no buffer"""
    try:
        texto_recebido = data.decode('utf-8').strip()
        timestamp_pc = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        
        print(f"[{timestamp_pc}] RX: {texto_recebido}")
        data_buffer.append([timestamp_pc, texto_recebido])
    except Exception as e:
        print(f"Erro de decode: {e}")

def on_disconnect(client):
    """Callback chamado automaticamente quando o bluetooth cai"""
    print("\n!!! DISPOSITIVO DESCONECTADO !!!")
    print("Tentando reconectar automaticamente...")

async def salvador_de_arquivo():
    """Roda em paralelo pra sempre, salvando o buffer no disco"""
    print(f"--- Iniciando gravação em: {NOME_ARQUIVO} ---")
    
    # Cria arquivo com cabeçalho
    with open(NOME_ARQUIVO, "w", newline='', encoding='utf-8') as f:
        csv.writer(f).writerow(["Hora_PC", "Dados"])
    
    while not stop_event.is_set():
        await asyncio.sleep(INTERVALO_SALVAMENTO)
        if data_buffer:
            qtd = len(data_buffer)
            try:
                with open(NOME_ARQUIVO, "a", newline='', encoding='utf-8') as f:
                    csv.writer(f).writerows(data_buffer)
                data_buffer.clear()
                print(f"-> Salvo no disco: {qtd} registros.")
            except Exception as e:
                print(f"Erro ao salvar arquivo: {e}")

async def main():
    # Inicia o salvador de arquivo em segundo plano (ele não para nunca)
    asyncio.create_task(salvador_de_arquivo())

    print("--- INICIANDO SISTEMA DE TELEMETRIA ---")
    print("Pressione Ctrl+C para encerrar o programa.")

    # LOOP DE RECONEXÃO INFINITA
    while not stop_event.is_set():
        device = None
        try:
            print(f"\nProcurando {DEVICE_NAME}...")
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: d.name == DEVICE_NAME,
                timeout=5.0
            )

            if device:
                print(f"Encontrado! Conectando em {device.address}...")
                
                # O disconnected_callback avisa a gente quando cair
                async with BleakClient(device, disconnected_callback=on_disconnect) as client:
                    print("CONECTADO! Aguardando dados...")
                    
                    await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
                    
                    # Mantém a conexão viva enquanto estiver conectado
                    while client.is_connected:
                        await asyncio.sleep(1)
            else:
                print("Dispositivo não encontrado. Tentando novamente em 2s...")
                await asyncio.sleep(2)

        except BleakError as e:
            print(f"Erro de Bluetooth: {e}")
            await asyncio.sleep(1) # Espera um pouco antes de tentar de novo
        except Exception as e:
            print(f"Erro genérico: {e}")
            await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n--- Encerrando programa... ---")
        stop_event.set()
        # Salva o que sobrou no buffer antes de sair
        if data_buffer:
            with open(NOME_ARQUIVO, "a", newline='', encoding='utf-8') as f:
                csv.writer(f).writerows(data_buffer)
            print("Dados remanescentes salvos.")