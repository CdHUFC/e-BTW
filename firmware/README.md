# e-BTW
## Low-cost Braille Printer using Bluepill and recycled materials

### Contributors
    @MinhaTu        Ailson Alexandre da Silva Morais
    @Esteves4       Lucas Esteves Rocha
    @studart301d    Gabriel Studart 
    @RussellFox     Ítalo Aguiar
### Check the prototype in the link below
https://www.youtube.com/watch?v=vpWazr3z5D8
### Description
This project aims to build a functional Braille Printer that can help in the process of writting and reading in that system. Briefly speaking, our printer works with a CNC-like mechanism that moves in the axes X,Y in which there is a needle coupled to a DC motor. Controlled by the Cortex-M0 processor by means of an PS2 Interface, the needle is reponsible for making the Braille writting itself, the dots.    

### Descrição
Este projeto visa construir uma impressora de Braille funcional que pode ajudar no processo de escrita e leitura nesse sistema. Resumidamente, nossa impressora trabalha com um mecanismo tipo CNC que se move nos eixos X, Y, nos quais há uma agulha acoplada a um motor DC que representa o eixo Z. Controlado pelo processador Cortex-M0 por meio de uma interface PS2, a agulha é responsável por fazer o Braille "escrevendo" os pontos.

### Material List:
    -Microcontrolador STM32F1030C8T6;
    -Wooden base (50cm x 40cm);
    -Wooden base (31cm x 25cm);
    -3x DC motors (12V);
    -Structure similar to a CNC for the X, Y axes;
    -Headless bore of 53cm wide and 8mm in diameter;
    -2x Headless screw 26cm wide and 8mm in diameter;
    -12x Nuts of 8 mm;
    -4x Aluminum structures;
    -6x Bearings;
    -2x Aluminum structures (40cm x 0.4cm);
    -20x Screws;
    -2x Gears;
    -2x Optical Encoder;
    -3x H Bridges;
    
### Lista de materiais
    -Microcontrolador STM32F1030C8T6;
    -Base de madeira (50cm x 40cm);
    -Base de madeira (31cm x 25cm);
    -3x Motores DC (12V);
    -Estrutura parecida com um CNC para eixos X e Y;
    -Parafuso sem cabeça de 53cm de largura e 8 mm de diâmetro;
    -2x Parafuso sem cabeça de 26cm de largura e 8 mm de diâmetro;
    -12x Porcas de 8 mm;
    -4x Estruturas de alumínio;
    -6x Rolamentos;
    -2x Estrutura de alumínio (40cm x 0,4cm);
    -20x Parafusos;
    -2x Engrenagens;
    -2x Encoder ópticos;
    -3x Pontes H;

## Mounting
### Structure of the Y Axis:
    1. On the base of wood (50cm x 40cm), it was fixed at a distance of 19cm the two aluminum structures (40cm x 0.4cm);
    2. Thus, we fixed the four aluminum structures close to the edges of the wooden base (31cm x 25cm) by sticking two of them, which were arranged parallel, at their center and two others near their edges;
    3. Next, we passed the headless screw (53cm) through the two aluminum structures that only have one hole connecting them and before passing bolt we couple a nut inside the aluminum structure.
    4. We passed the two headless screws (26cm) through the other two aluminum frames connecting them. Thus, we put a bearing on each side of the headless screws (26cm) and fix with a nut.
    In this way, we obtained a movable base with a central screw.
    5. The bearings placed on the outside should fit perfectly on the outside of the two aluminum structures (40cm x 0.4cm) fixed to the base.
    6. We create a fastening and rotation structure from the association of the DC motor with gears and the creation of a base to fix the motor.
    7. The encoder was fixed in an improvised wooden base so that the movement could be calibrated.
### Structure of the X Axis:
    Basically, we positioned the CNC that was extracted from an old printer that we've umounted and then we put the encoder to calibrate the axis moviment.
### Structure of the Z Axis:
    We attach a type of knitting needle to a DC motor and we sand its tip so that the degree of perforation is reduced and we have points closer to the actual Braille writing.
    
## Montagem
### Estrutura do Eixo Y:
    1. Sobre a base de madeira (50cm x 40cm) foram fixadas a uma distância de 19 cm as duas estruturas de alumínio (40cm x 0,4cm);
    2. Assim, fixamos próximo às extremidades da base de madeira (31cm x 25cm) as 4 estruturas de alumínio furando duas delas, que foram dispostas paralelamente, em seu centro e outras duas próximo de suas extremidades; 
    3. Em seguida, passamos o parafuso sem cabeça (53cm) pelas duas estruturas de alumínio que possuem somente um furo conectando-as e antes de passar parafuso acoplamos uma porca dentro da estrutura de alumínio.
    4. Passamos os dois parafusos sem cabeça (26cm) pelas outras duas estruturas de alumínio conectando-as. Assim, colocamos um rolamento em cada lado dos parafusos sem cabeça (26cm) e fixamos com uma porca. 
    Desse modo, obtivemos uma base móvel com um parafuso central.
    5. Os rolamentos colocados na parte externa devem se encaixar perfeitamente na parte exterior das duas estruturas de alumínio (40cm x 0,4cm) fixadas na base.
    6. Criamos uma estrutura de fixação e rotação a partir da associação do motor DC com engrenagens e da criação de uma base para fixar o motor.
    7. O encoder foi fixado numa base de madeira improvisada para que se pudesse calibrar a movimentação.
### Estrutura do Eixo X:
    Basicamente, posicionamos o CNC que foi extraído de uma impressora antiga que desmontamos e colocamos o encoder para calibrar o movimento do eixo.
### Estrutura do Eixo Z:
    Acoplamos um tipo de agulha de tricô a um motor DC e lixamos a sua ponta para que o grau de perfuração seja reduzido e tenhamos pontos mais próximos da escrita real de Braille.
    
## How does it work?
    The device operates by an input via the PS2 keyboard, which is stored in a buffer within the cortex-M0. In this, what was passed by the keyboard goes through an algorithm to count the number of characters that will be printed per line at the time of transcribing for braille. The algorithm inverts the phrase and uses an array that contains the braille letters to be able to print line by line, being coordinated by the code and calibrated via an encoder.

## Como funciona?
    O funcionamento do dispositivo se dá a partir de uma entrada, via teclado PS2, que é armazenada num buffer dentro do cortex-M0. Nisso, o que foi passado pelo teclado passa por um algoritmo para contar o número que caracteres que poderão ser imprimidos por linha na hora de transcrever para braille. O algoritmo inverte a frase e utiliza de uma matriz que contém as letras em braille para poder imprimir linha a linha, sendo coordenado pelo código e calibrado via encoder.

## Limitations
    -The printer works by printing line to line, so it is not possible to pass a full text uninterruptedly via the PS2 keyboard and it is necessary to pass line by line so that the printer can print correctly.
    -The lack of an appropriate voltage source can disturb the operation, because the motors require voltage close to 12V to operate well, given the weight of the structure and all frictions.
    -The structure is relatively fragile, as accidents can decalibrate the structure and this takes a while to be adjusted.


## Limitações
    -A impressora funciona imprimindo linha à linha, logo não é possível passar um texto completo ininterruptamente via teclado PS2 e é necessário passar linha por linha para que a impressora possa imprimir corretamente.
    -A falta de uma fonte de tensão apropriada pode atrapalhar o funcionamento, pois os motores exigem tensão próxima de 12V para operar bem, dado o peso da estrutura e todos os atritos.
    -A estrutura é relativamente frágil, pois acidentes podem descalibrar a estrutura e isso toma um tempo para ser ajustado.
