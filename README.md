# MPU6050 com Modbus RTU

O PIC18LF4685 lê os três eixos do acelerômetro MPU6050 e disponibiliza as leituras como *Holding Registers* Modbus RTU, função `0x03`. O Scada-LTS deve atuar como mestre Modbus RTU serial e consultar o escravo de endereço 1.

## Ligação

- MPU6050 SCL: RC3/SCL; SDA: RC4/SDA; AD0 em nível baixo (endereço I2C `0x68`).
- UART do PIC: RC6/TX e RC7/RX.
- Transceptor RS-485: DI em RC6/TX, RO em RC7/RX, DE em RD0 e `/RE` em GND. Conecte A/B ao barramento e compartilhe o GND de referência.
- PIC, transceptor e módulo MPU6050 devem usar níveis de tensão compatíveis; as linhas I2C precisam de pull-ups adequados.

O RD0 é configurado como controle de transmissão do RS-485 e fica em nível baixo enquanto o PIC recebe. Se sua placa usa outro pino para DE, altere `UART_Init`, `UART_TransmitBegin` e `UART_TransmitEnd` em `uart.c`.

## Configuração serial

- Modbus RTU: 115200 baud, 8 bits, sem paridade, 1 stop bit (8N1).
- Endereço do escravo: `1`.
- Interface física: RS-485 half-duplex.
- Aquisição do acelerômetro: escala ±2 g, aproximadamente 250 amostras/s.
- I²C do MPU6050: 100 kHz (modo padrão), endereço `0x68`.

Configure no Scada-LTS um datasource serial Modbus RTU com esses parâmetros e crie pontos de leitura da função `03`.

## Ponte Modbus RTU para Modbus TCP no PC

Quando o Scada-LTS precisar acessar o sensor por Modbus TCP/IP, execute a ponte Python no PC conectado ao adaptador USB-RS-485. A ponte consulta o PIC como mestre RTU e publica os valores como servidor Modbus TCP.

Instale os requisitos e inicie a ponte (troque `COM4` pela porta do adaptador):

```powershell
python -m pip install -r pc/requirements.txt
python pc/modbus_tcp_bridge.py COM4
```

Por padrão, a ponte consulta o escravo RTU `1` em 115200 8N1 a cada 4 ms, lê doze registradores e calcula a FFT com uma janela de 512 amostras. A aquisição do PIC é nominalmente 250 Hz; o PC pode observar amostras repetidas ou perdidas conforme o adaptador e o escalonamento do Windows. A ponte escuta Modbus TCP na porta `5020`. Opções como `--tcp-port`, `--host`, `--poll-ms`, `--fft-window` e `--slave-id` podem ser consultadas com `python pc/modbus_tcp_bridge.py --help`.

No Scada-LTS, crie um datasource Modbus TCP com o IP do PC que executa a ponte, porta `5020` e unit ID `1`. Leia os holding registers pela função `03`, usando estes endereços zero-based:

| Endereço | Conteúdo | Escala / interpretação |
|---:|---|---|
| 0 | Acelerômetro X | `int16`, contagem bruta |
| 1 | Acelerômetro Y | `int16`, contagem bruta |
| 2 | Acelerômetro Z | `int16`, contagem bruta |
| 3 | Estado do MPU | `0` = leitura válida; `1` = erro I²C |
| 4 | Comunicação PC↔PIC | `0` = consulta RTU válida; `1` = falha de comunicação |
| 5–7 | Frequência do pico FFT X/Y/Z | inteiro sem sinal em centi-hertz; multiplicador `0,01` para Hz |
| 8–10 | Amplitude do pico FFT X/Y/Z | inteiro sem sinal em mili-g; multiplicador `0,001` para g |
| 11 | Estado da FFT | `0` = janela completa; `1` = coletando ou reiniciada |
| 12 | Sequência da amostra no PIC | contador de 16 bits, volta a zero após 65535 |
| 13 | Ticks de aquisição perdidos no PIC | contador saturado de eventos do Timer2 não atendidos a tempo |
| 14 | Amostras perdidas entre PC e PIC | contador saturado de saltos observados na sequência |
| 15 | Taxa de amostragem medida | centi-hertz; multiplicador `0,01` para Hz |
| 16–18 | RMS de vibração X/Y/Z | mili-g, calculado no PIC em janelas de 64 amostras após remover a média |
| 19–21 | Vibração pico a pico X/Y/Z | mili-g, calculado no PIC em janelas de 64 amostras |

Os valores de FFT são o pico dominante diferente de zero em cada eixo, calculado após remover a média e aplicar janela Hann. A janela padrão de 512 amostras leva cerca de 2,05 s à taxa nominal de 250 Hz. O limite de Nyquist depende da taxa de amostragem efetivamente medida e corresponde à metade dela. O PC só calcula a FFT com amostras de sequência diferentes; o registrador 14 conta lacunas observadas, enquanto o 13 mostra os ticks perdidos no firmware. O registrador 15 publica a taxa medida em centi-hertz.

Em falha de comunicação, X/Y/Z e os últimos resultados de FFT permanecem armazenados; o endereço 4 indica que os dados estão desatualizados e o endereço 11 volta a `1`. Se o Scada-LTS estiver em outro computador, permita conexões TCP de entrada na porta 5020 do firewall do PC.

O Virtual Terminal do Proteus não substitui o adaptador USB-RS-485 nem o mestre Modbus RTU: a ponte precisa de uma conexão serial real com o barramento do PIC.

## Mapa de registradores

Os endereços do pedido Modbus são zero-based. Alguns clientes exibem o endereço como referência `4xxxx`; nesse caso, o registro de endereço 0 costuma aparecer como `40001`.

| Endereço Modbus | Referência usual | Dado | Tipo / escala |
|---:|---:|---|---|
| 0 | 40001 | Acelerômetro X | int16 com sinal, valor bruto |
| 1 | 40002 | Acelerômetro Y | int16 com sinal, valor bruto |
| 2 | 40003 | Acelerômetro Z | int16 com sinal, valor bruto |
| 3 | 40004 | Estado | 0 = leitura válida; 1 = falha de I2C |
| 4–21 | 40005–40022 | Estado da ponte, FFT, diagnósticos e métricas de vibração | conforme tabela acima |

Os eixos estão em contagens do sensor. Para converter para g, divida o valor assinado por `16384`. Em cada ponto do Scada-LTS, configure interpretação de inteiro de 16 bits com sinal e escala `1/16384`, se o datasource oferecer essa opção. Alguns clientes esperam digitar `0` para o primeiro endereço; outros mostram `40001`.

O PIC agenda aquisições a cada 4 ms e responde somente quando consultado pelo mestre Modbus. Leia doze registradores a partir do endereço 0 para obter as leituras instantâneas, estado, sequência, contador de ticks perdidos, RMS e pico a pico. O RMS e o pico a pico são atualizados a cada 64 amostras válidas consecutivas. Se houver perda de ticks ou falha I2C, a janela parcial é descartada; os últimos indicadores completos permanecem nos registradores até a próxima janela válida.

## Visualização no Virtual Terminal do Proteus

O Modbus usa a UART de hardware em RC6/RC7, 115200 baud. O Virtual Terminal pode monitorar RC6/TX em paralelo com a entrada DI do transceptor RS-485; ligue RC6 ao RXD do terminal e compartilhe o GND. Configure o terminal para 115200 baud, 8N1. O firmware envia `BOOT` e o estado do MPU durante a inicialização; as leituras contínuas são consultadas pelo Modbus para não atrasar a aquisição.

Se aparecer `MPU ERRO #n`, o código identifica a etapa que falhou: `#1` sem resposta I²C ao ler `WHO_AM_I` nos endereços `0x68` e `0x69`; `#2` ID diferente de `0x68`; `#3` falha ao acordar o sensor; `#4` falha ao configurar o filtro; `#5` falha ao configurar a taxa de amostragem; `#6` falha ao configurar a escala do acelerômetro. O firmware tenta liberar o barramento com nove pulsos de SCL antes de repetir a detecção. Para `#1`, também são impressos os níveis de repouso medidos com o MSSP liberado: `SCL` e `SDA` (`1` = alto, `0` = baixo). O endereço I²C é detectado automaticamente conforme o nível de AD0; o `WHO_AM_I` padrão é `0x68`.
