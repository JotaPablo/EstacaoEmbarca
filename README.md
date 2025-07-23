# EmbarcaEstacao - Estação Meteorológica com Interface Web via FreeRTOS

Uma estação meteorológica embarcada com controle web via **RP2040 (Raspberry Pi Pico)**, que monitora temperatura, umidade, pressão atmosférica e altitude, exibindo os dados em tempo real e alertando sobre limites definidos.

---

## Funcionalidades Principais

-   **Monitoramento Climático em Tempo Real**
    -   Coleta contínua de dados de temperatura e umidade (AHT20), pressão atmosférica (BMP280) e cálculo da altitude.
    -   Exibição dos dados em **display OLED** e em uma **interface web responsiva**.

-   **Alertas Visuais e Sonoros**
    -   **LEDs RGB** e **Buzzer** são acionados automaticamente quando os dados ultrapassam limites configurados.
    -   Cores específicas nos LEDs indicam qual parâmetro está fora do normal (ex: vermelho para temperatura, azul para umidade, verde para pressão).

-   **Interface Web Dinâmica e Configurável**
    -   Acesse a estação de qualquer navegador (PC ou celular) para visualizar os dados atuais.
    -   **Defina limites** para temperatura, umidade e pressão, e adicione um **offset de temperatura** diretamente pela interface web (atualização via AJAX).

-   **Modo de Simulação Integrado**
    -   Alterne para o modo de simulação usando o **Joystick** para testar o sistema com valores fictícios. Isso é ótimo para demonstrações e depuração.

-   **Matriz de LEDs para Visualização Gráfica**
    -   Uma **Matriz de LEDs** mostra visualmente os níveis de temperatura, umidade e pressão através de barras coloridas.

---

## Componentes Utilizados

| Componente           | GPIO/Pino             | Função                                                                             |
| :------------------- | :-------------------- | :--------------------------------------------------------------------------------- |
| Sensores I2C         | 0 (SDA), 1 (SCL)      | **AHT20**: Mede temperatura e umidade. **BMP280**: Mede pressão.     |
| Display OLED SSD1306 | 14 (SDA), 15 (SCL)    | Exibe dados climáticos em tempo real e status da conexão Wi-Fi.                    |
| Joystick             | 26 (VRX), 27 (VRY)    | Utilizado para gerar valores simulados no modo de teste.                           |
| Botões               | 5 (SW), 6 (A), 22 (B) | **SW (Joystick)**: Alterna entre modo normal e simulado. **A**: Reseta os limites climáticos. **B**: Permite entrada no modo BOOTSEL. |
| LEDs RGB             | 11, 12, 13            | Indicam visualmente se algum parâmetro está fora dos limites (cores variam conforme o tipo de alerta). |
| Matriz de LEDs       | 7                     | Representa visualmente os dados de temperatura, umidade e pressão com barras coloridas. |
| Buzzer               | 21                    | Emite alertas sonoros quando um ou mais dados climáticos ultrapassam os limites definidos. |

---

## Estrutura do Código

O software da EmbarcaEstação é construído sobre o FreeRTOS, com cada funcionalidade implementada como uma tarefa separada para garantir paralelismo e modularização:

-   **`main.c`**
    -   Inicializa todos os periféricos e as tarefas do FreeRTOS.
    -   Cria e gerencia os mecanismos de sincronização (mutexes e semáforos) para acesso seguro aos dados e periféricos compartilhados.

-   **Tarefas Essenciais (FreeRTOS Tasks):**
    -   `vSensorTask()`: Responsável por ler os dados dos sensores AHT20 e BMP280, ou gerar valores simulados quando no modo de teste.
    -   `vDisplayTask()`: Controla a exibição das informações no display OLED.
    -   `vWebServerTask()`: Gerencia o servidor HTTP e a interface web, processando requisições de dados e configurações.
    -   `vLedsBuzzerTask()`: Monitora os limites configurados e ativa os LEDs RGB e o buzzer para alertas.
    -   `vMatrizLedsTask()`: Atualiza a matriz de LEDs para uma representação visual dos dados.

-   **Sincronização e Concorrência:**
    -   `xMutexSensorData` e `xMutexDisplay`: Usados para proteger o acesso simultâneo a dados dos sensores e ao display OLED, respectivamente.
    -   `xWifiReadySemaphore`: Sinaliza quando a conexão Wi-Fi está pronta, permitindo que as tarefas dependentes iniciem sua operação.

---

## Endpoints de Controle Web

A interface web da EmbarcaEstação interage com o sistema através dos seguintes endpoints HTTP:

---

## Endpoints de Controle Web

A interface web da EmbarcaEstação interage com o sistema através dos seguintes endpoints HTTP:

| URL          | Ação                                                                    |
| :----------- | :---------------------------------------------------------------------- |
| `/`          | Serve a página HTML principal da estação.                               |
| `/estado`    | Fornece os dados climáticos atuais (temperatura, umidade, pressão, altitude) e os limites configurados em formato **JSON**. |
| `/limites`   | **Atualiza os limites** de temperatura, umidade e pressão, e o offset de temperatura. |

### Detalhes dos Parâmetros para `POST /limites`

Ao enviar uma requisição `POST` para o endpoint `/limites`, o corpo da requisição deve ser um objeto JSON contendo os seguintes parâmetros:

```json
{
  "min_temperature": <float>,
  "max_temperature": <float>,
  "offset_temperature": <float>,
  "min_humidity": <float>,
  "max_humidity": <float>,
  "min_pressure": <float>,
  "max_pressure": <float>
}
```
## ⚙️ Instalação e Uso

1. **Pré-requisitos**
   - Clonar o repositório:
     ```bash
     git clone https://github.com/JotaPablo/RoboVigiaV2.git
     cd RoboVigia
     ```
   - Instalar o **Visual Studio Code** com as extensões:
     - **C/C++**
     - **Pico SDK Helper** ou extensão da Raspberry Pi Pico
     - **Compilador ARM GCC**
     - **CMake Tools**

2. **Configure o Wi-Fi:**
    - Renomeie o arquivo `wifi_config_example.h` na pasta config para `wifi_config.h`.
    - Edite o arquivo `wifi_config.h` e adicione suas credenciais Wi-Fi:

    ```c
    #define WIFI_SSID "SeuSSID"
    #define WIFI_PASSWORD "SuaSenha"
    ```
3. **Ajuste do caminho do FreeRTOS**
   - Abra o arquivo `CMakeLists.txt` na raiz do projeto e ajuste a variável `FREERTOS_KERNEL_PATH` para o diretório onde você instalou o FreeRTOS Kernel.  
     Exemplo:
     ```cmake
     set(FREERTOS_KERNEL_PATH "C:/FreeRTOS-Kernel")
     ```
     Substitua `"C:/FreeRTOS-Kernel"` pelo caminho correto na sua máquina.


4. **Compilação**
   - Compile o projeto manualmente via terminal:
     ```bash
     mkdir build
     cd build
     cmake ..
     make
     ```
   - Ou utilize a opção **Build** da extensão da Raspberry Pi Pico no VS Code.

5. **Execução**
   - Conecte o Raspberry Pi Pico no modo BOOTSEL
   - Copie o arquivo `.uf2` para o dispositivo `RPI-RP2`
