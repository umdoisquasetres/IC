# MPU6050 com Modbus RTU

O PIC18LF4685 lê os três eixos do acelerômetro MPU6050 e responde a consultas Modbus RTU pela UART. No PC, `pc/modbus_tcp_bridge.py` consulta o PIC, calcula a FFT e disponibiliza os indicadores por Modbus TCP para o Scada-LTS.

## Ligação

- MPU6050 SCL: RC3/SCL; SDA: RC4/SDA; AD0 em nível baixo (endereço I2C `0x68`).
- UART do PIC: RC6/TX e RC7/RX.
- CH340 USB–UART TTL: RXD do CH340 em RC6/TX do PIC, TXD do CH340 em RC7/RX e GND comum. Não conecte o CH340 TTL aos terminais A/B de um barramento RS-485.
- Confirme a tensão lógica do CH340 e do PIC antes de ligar TX/RX; não presuma que o seletor de alimentação do módulo altera os níveis lógicos. As linhas I²C também precisam de pull-ups compatíveis.

O firmware ainda configura RD0 como DE para permitir um transceptor RS-485 opcional; com CH340 TTL, deixe RD0 sem conexão. O fluxo ativo de aquisição envia somente quadros Modbus RTU, sem mensagens de inicialização ou diagnóstico em texto. As rotinas legadas de leitura da EEPROM podem emitir texto se chamadas, mas não são chamadas pelo fluxo ativo.

## Configuração serial

- Modbus RTU: 115200 baud, 8 bits, sem paridade, 1 stop bit (8N1).
- Endereço do escravo: `1`.
- Interface física desta montagem: CH340 USB–UART TTL, TX/RX cruzados e GND comum. Para cabos longos ou ambiente com ruído de inversor, prefira uma interface RS-485 apropriada.
- Aquisição do acelerômetro: escala ±2 g, aproximadamente 250 amostras/s.
- I²C do MPU6050: 100 kHz (modo padrão), endereço `0x68`.

Configure o PC para a porta serial do CH340 e o Scada-LTS para consultar por Modbus TCP, conforme a seção seguinte. Deve haver somente um mestre consultando o PIC pela UART.

## Ponte Modbus RTU para Modbus TCP no PC

Execute a ponte Python no PC conectado ao CH340. Ela consulta o PIC como mestre RTU, calcula a FFT no PC e publica os indicadores como servidor Modbus TCP.

Instale os requisitos e inicie a ponte (troque `COM4` pela porta do adaptador):

```powershell
python -m pip install -r pc/requirements.txt
python pc/modbus_tcp_bridge.py COM4
```

Para guardar o espectro completo em CSV (uma linha por frequência e janela, com amplitudes X/Y/Z em g), inicie com `--spectrum-csv dados/espectros.csv`:

```powershell
python pc/modbus_tcp_bridge.py COM4 --spectrum-csv dados/espectros.csv
```

O arquivo é anexado a cada inicialização; cada janela exportada contém os bins diferentes de zero, a frequência medida e um timestamp UTC. Proteja o armazenamento e faça rotação/backup se a aquisição ficar ativa por longos períodos.

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
| 11 | Estado da FFT | `0` = janela completa; `1` = coletando, reiniciada ou sem amostras novas |
| 12 | Sequência da amostra no PIC | contador de 16 bits, volta a zero após 65535 |
| 13 | Ticks de aquisição perdidos no PIC | contador saturado de eventos do Timer2 não atendidos a tempo |
| 14 | Amostras perdidas entre PC e PIC | contador saturado de saltos observados na sequência |
| 15 | Taxa de amostragem medida | centi-hertz; multiplicador `0,01` para Hz |
| 16–18 | RMS de vibração X/Y/Z | mili-g, calculado no PIC em janelas de 64 amostras após remover a média |
| 19–21 | Vibração pico a pico X/Y/Z | mili-g, calculado no PIC em janelas de 64 amostras |

Os valores de FFT são o pico dominante diferente de zero em cada eixo, calculado após remover a média e aplicar janela Hann. A janela padrão de 512 amostras leva cerca de 2,05 s à taxa nominal de 250 Hz. O limite de Nyquist depende da taxa de amostragem efetivamente medida e corresponde à metade dela. O PC só calcula a FFT com amostras de sequência diferentes; o registrador 14 conta lacunas observadas, enquanto o 13 mostra os ticks perdidos no firmware. O registrador 15 publica a taxa medida em centi-hertz.

Em falha de comunicação, X/Y/Z e os últimos resultados de FFT permanecem armazenados; o endereço 4 indica falha e o endereço 11 volta a `1`. Se as consultas RTU continuam válidas mas a sequência do PIC não avança por mais de 50 ms, a ponte limpa a janela FFT e marca o endereço 11 como `1`; isso diferencia uma resposta serial válida de aquisição nova. Se o Scada-LTS estiver em outro computador, permita conexões TCP de entrada na porta 5020 do firewall do PC.

O Virtual Terminal do Proteus pode monitorar RC6/TX em simulação, mas o firmware não envia texto de inicialização. Para a ponte, use uma conexão serial real com o CH340; o Virtual Terminal não substitui o PC nem o mestre RTU.

## Mapas de registradores

Os endereços nos pedidos Modbus são zero-based. Alguns clientes exibem o endereço como referência `4xxxx`; nesse caso, o endereço 0 costuma aparecer como `40001`.

### PIC: Modbus RTU

O PIC implementa somente a função `03` e disponibiliza 12 holding registers:

| Endereço | Referência usual | Dado | Tipo / escala |
|---:|---:|---|---|
| 0–2 | 40001–40003 | Acelerômetro X/Y/Z | `int16` com sinal, contagens brutas; dividir por 16384 para obter g |
| 3 | 40004 | Estado do MPU | 0 = leitura válida; 1 = falha de I2C |
| 4 | 40005 | Sequência da aquisição | contador de 16 bits, volta a zero após 65535 |
| 5 | 40006 | Ticks de aquisição perdidos | contador saturado do Timer2 |
| 6–8 | 40007–40009 | RMS X/Y/Z | mg, janela de 64 amostras após remover a média |
| 9–11 | 40010–40012 | Pico a pico X/Y/Z | mg, janela de 64 amostras |

### Ponte: Modbus TCP

A ponte publica 22 holding registers. Os endereços 0–3 espelham os registradores do PIC; os demais são produzidos ou mantidos pela ponte:

| Endereço | Referência usual | Dado | Tipo / escala |
|---:|---:|---|---|
| 0–3 | 40001–40004 | Acelerômetro X/Y/Z e estado do MPU | conforme mapa RTU |
| 4 | 40005 | Estado da comunicação PC↔PIC | 0 = consulta RTU válida; 1 = falha |
| 5–7 | 40006–40008 | Frequência do pico FFT X/Y/Z | centi-Hz; multiplicador 0,01 para Hz |
| 8–10 | 40009–40011 | Amplitude do pico FFT X/Y/Z | mili-g; multiplicador 0,001 para g |
| 11 | 40012 | Estado da FFT | 0 = janela completa; 1 = coletando, reiniciada ou sem amostras novas |
| 12 | 40013 | Sequência da amostra no PIC | contador de 16 bits |
| 13 | 40014 | Ticks de aquisição perdidos no PIC | contador saturado |
| 14 | 40015 | Amostras perdidas entre PC e PIC | contador saturado de lacunas observadas |
| 15 | 40016 | Taxa de amostragem medida | centi-Hz; multiplicador 0,01 para Hz |
| 16–18 | 40017–40019 | RMS X/Y/Z | mg, calculado no PIC |
| 19–21 | 40020–40022 | Pico a pico X/Y/Z | mg, calculado no PIC |

Use o mapa RTU quando o Scada-LTS consultar o PIC diretamente. Use o mapa TCP quando consultar a ponte; os registradores da ponte não existem no PIC. Não conecte dois mestres RTU ao mesmo barramento simultaneamente.

Os eixos estão em contagens do sensor. Para converter para g, divida o valor assinado por `16384`. Em cada ponto do Scada-LTS, configure interpretação de inteiro de 16 bits com sinal e escala `1/16384`, se o datasource oferecer essa opção. Alguns clientes esperam digitar `0` para o primeiro endereço; outros mostram `40001`.

O PIC agenda aquisições a cada 4 ms e responde somente quando consultado pelo mestre Modbus. Leia doze registradores a partir do endereço 0 para obter as leituras instantâneas, estado, sequência, contador de ticks perdidos, RMS e pico a pico. O RMS e o pico a pico são atualizados a cada 64 amostras válidas consecutivas. Após três falhas I2C consecutivas, o firmware tenta reinicializar o MPU a cada 250 ticks nominais (cerca de 1 s); após falha I2C ou perda de ticks, a janela parcial é descartada e os últimos indicadores completos permanecem nos registradores até a próxima janela válida.

## Diagnóstico do sensor

O firmware não envia mensagens textuais pela UART. O estado do MPU está no registrador RTU 3 (espelhado no TCP 3); `0` indica leitura válida e `1` indica falha I²C. Os códigos de etapa de inicialização continuam disponíveis internamente no driver, mas não são transmitidos pelo protocolo. O endereço I²C é detectado em `0x68` ou `0x69` conforme AD0; o `WHO_AM_I` esperado é `0x68`.
