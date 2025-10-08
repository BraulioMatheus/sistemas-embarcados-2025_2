# Atividade 05 - Contador Binário com ESP32-S3

## Descrição
Implementação de um contador binário de 4 bits controlado por botões utilizando ESP32-S3 e framework ESP-IDF no simulador Wokwi.

## Funcionalidades
-  Contador binário de 4 bits (0-15)
-  Dois botões para controle (Incremento e Mudar Incremento)
-  Incremento configurável (+1/+2)
-  Overflow circular
-  Debounce por software
-  Display visual em 4 LEDs

## Hardware
- **Microcontrolador**: ESP32-S3 DevKitC-1
- **LEDs**: 4 (Vermelho, Verde, Azul, Amarelo)
- **Botões**: 2
- **Resistores**: 4x 220Ω
- **Protoboard**

## Diagrama de Blocos
![Diagrama de Blocos]([docs/diagrama-bloco.png](https://raw.githubusercontent.com/BraulioMatheus/sistemas-embarcados-2025_2/469c2da6550587b81401eb931d6046295f9166e3/atividade-05/Diagrama%20de%20blocos.png))

## Esquemático
![Esquemático do Circuito](docs/esquematico.png)

## Código Fonte

### Estrutura do Projeto
