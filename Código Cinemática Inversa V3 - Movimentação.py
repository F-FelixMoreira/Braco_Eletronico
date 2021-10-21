
# Ang1 - Ângulo de abertura braço inferior a partir da direita (olhar desenho)
# Ang2 - Ângulo de abertura braço superior a partir da esquerda (olhar)
# Ang3 - Ângulo de abertura do motor-base a partir da direita

########### Bibliotecas de comandos usadas ##############

from  pyfirmata import Arduino, SERVO
import pyfirmata
from pyfirmata.util import Iterator
from scipy.optimize import least_squares
import math
import time
import numpy as np

def adquirirAngs(ang,xd,yd,zd):      #Criando função inversa para adquirir a1,a2 e a3

        [a1,a2,a3] = ang
        
        xb = bracoInf*math.cos(a1)*math.cos(a3)
        xa = xb + bracoSup*math.cos(a2)*math.cos(a3)
        yb = bracoInf*math.cos(a1)*math.sin(a3)
        ya = yb + bracoSup*math.cos(a2)*math.sin(a3)
        zb = bracoInf*math.sin(a1)
        za = zb - bracoSup*math.sin(a2)
        
        return xa-xd,za-zd,ya-yd

def giraServoBase(pinS1, angle):
    servoBase.write(angle)
    time.sleep(0.015)

def giraServoLateral(pinS2, angle):
    servoLateral.write(angle)
    time.sleep(0.015)

def giraServoCentral(pinS3, angle):
    servoCentral.write(angle)
    time.sleep(0.015)

def giraServoGarra(pinS4, angle):
    servoGarra.write(angle)
    time.sleep(0.015)

########################## Modos ##########################

inserirCoord = 1
calcVolTrabalho = 0
adquirirAng = 0
pontoAtual = 1
pontoUmCalculado = 0
calcTrajeto = 0
calcTrajetoFim = 0
repetir = 0

exibirMensagem = True
ativarRotina = False
fimDoPrograma = False

############### Valores e entradas motores/botões  ################

port = 'COM6'                          # Porta SUB em que o Arduino foi conectado
placa = Arduino(port)                  # Dando um nome para o nosso arduino

it = Iterator(placa)
it.start()

pinS1 = 8   #base                      # Indicando onde estão as entradas e o que são
pinS2 = 9   #lateral
pinS3 = 10  #central
pinS4 = 11  #garra

placa.digital[pinS1].mode = SERVO      # Indicando que essas entradas são servos
placa.digital[pinS2].mode = SERVO
placa.digital[pinS3].mode = SERVO
placa.digital[pinS4].mode = SERVO

botao1 = placa.digital[5]               #Indicando o botão
botao1.mode = pyfirmata.INPUT

servoBase = placa.digital[pinS1]       # Substituindo o nome das entradas pra simplificar
servoLateral = placa.digital[pinS2]
servoCentral = placa.digital[pinS3]
servoGarra = placa.digital[pinS4]

pos0baseImaginaria = 0                         # Definindo posições iniciais para cada motor
pos0centralImaginaria = 90
pos0lateralImaginaria = 40
pos0garraImaginaria = 90

pos0baseReal = 0                                # Base real = 0          # Definindo posições iniciais para cada motor
pos0centralReal = 140-pos0centralImaginaria     # Central real = 50
pos0lateralReal = 45-pos0lateralImaginaria+45   # Lateral real = 95
pos0garraReal = 90                              # Garra real = 90  |   Aberta

servoBase.write(pos0baseReal)                   # Enviando posições iniciais para os servos
servoCentral.write(pos0centralReal)        
servoLateral.write(pos0lateralReal)      
servoGarra.write(pos0garraReal)                

velAng = 2   #Velocidade angular de giro dos motores

########################## Valores do braço ###############

bracoInf = 80     #Tamanho braço inferior 80mm
bracoSup = 135    #Tamanho braço superior + centro da garra 126mm

limiteInf = 90    #Tamanho entre a ponta da garra e o eixo da base no mínimo
limiteSup = 185   #Tamanho entre a ponta da garra e o eixo da base no máximo

########################## Inserindo coordenadas ##########################


while calcTrajetoFim != 1:    #Enquanto o calculo do trajeto não chegar ao fim, ele continua rodando.

    if inserirCoord == 1:

        if pontoAtual == 1 and pontoUmCalculado == 1:  # Passa pro ponto2 se o ponto1 já estiver estabelecido

            pontoAtual = 2

        print("")
        print("")
        print("Digite as coordenadas desejadas para o ponto",pontoAtual,":")
        print("")

        xd = float(input("X: "))          # Valores dos ângulos desejados
        yd = float(input("Y: " ))         # serão utilizados tanto no cálculo de vol. de trabalho
        zd = float(input("Z: "))          # quanto na inversão de coordenadas

        calcVolTrabalho = 1
        inserirCoord = 0


    ########################## Cálculo volume de trabalho ##########################

    if calcVolTrabalho == 1 and inserirCoord == 0 :

        R = ((xd)**2 + (yd)**2 + (zd)**2)**(1/2)           #Função que descreve a soma vetorial

        time.sleep(0.3)

        if R <= (limiteSup) and R >= (limiteInf) and zd >= -80:   
            
                print(' ')
                print('Valor do raio: %.2f'%R)
                print(' ')
                print('Coordenadas inseridas fazem parte do volume de trabalho')
                print(' ')

                adquirirAng = 1
                calcVolTrabalho = 0
                
        else:
            
                print(' ')
                print('Valor do raio: %.2f'%R)
                print(' ')
                print('Coordenadas inseridas não fazem parte do volume de trabalho')
                time.sleep(0.5)
                print(' ')
                print('Insira outros valores')
                
                inserirCoord = 1
                calcVolTrabalho = 0
                
            
    ############# Cálculo cinemática inversa | Obtenção dos ângulos ###########

    if adquirirAng == 1:   
                
            time.sleep(0.3)

            (a1,a2,a3) = least_squares(adquirirAngs,[0,0,0], args=(xd, yd, zd), bounds= (-(3.14/4), 3.14)).x
                
            Ang1 = round((math.degrees(a1)))   # Angulo final que será armazenado no valor do motor
            Ang2 = round((math.degrees(a2)))   # Para mover o braço, bastará transferir esses resultados
            Ang3 = round((math.degrees(a3)))   # Para a IDE do arduino e aplicar os valores ao motor (FIRMATA)


            Ang1real = 140 - Ang1
            Ang2real = 45 - Ang2 + 45
            Ang3real = Ang3

            time.sleep(0.4)

            #   Exibição de resultados:
            
            print ("Ângulos obtidos para o ponto",pontoAtual, ": ")
            print(' ')
            print("A1 calc:", Ang1,"A2 calc:", Ang2,"A3 calc:", Ang3) 
            print("A1 real:", Ang1real,"A2 real:", Ang2real,"A3 real:", Ang3real) 

            print('________________________________')
            

            if pontoAtual == 1:

                p1 = [Ang1,Ang2,Ang3]
                p1real = [Ang1real, Ang2real ,Ang3real]

                adquirirAng = 0
                inserirCoord = 1
                pontoUmCalculado = 1                

            if pontoAtual == 2:

                p2 = [Ang1,Ang2,Ang3]
                p2real = [Ang1real ,Ang2real ,Ang3]

                adquirirAng = 0

                calcTrajeto = 1


    if calcTrajeto == 1:

        time.sleep(0.4)

        print(' ')
        print("Ponto 1: ", p1)
        print("Ponto 2: ", p2)
        print(' ')
        print("Ponto 1 real: ", p1real)
        print("Ponto 2 real:  ", p2real)

        print(' ')

        p1_a1 = p1real[0]          # Ang1 - Central
        p1_a2 = p1real[1]          # Ang2 - Lateral
        p1_a3 = p1real[2]          # Ang3 - Base

        p2_a1 = p2real[0]          # Ang1 - Central
        p2_a2 = p2real[1]          # Ang2 - Lateral
        p2_a3 = p2real[2]          # Ang3 - Base

        for pos0lateralReal in range(pos0lateralReal,-30,-(velAng) ):
            giraServoLateral(pinS2, pos0lateralReal)


        if pos0baseReal > p1_a3:                                           # Base vai

            for pos0baseReal in range (pos0baseReal, p1_a3, -(velAng)):       
                giraServoBase (pinS1, pos0baseReal)
                time.sleep(0.015)       
        
        else:                                           

            for pos0baseReal in range (pos0baseReal, p1_a3, velAng):       
                giraServoBase (pinS1, pos0baseReal)
                time.sleep(0.015)    


        if pos0lateralReal > p1_a2:                                             # Lateral para o p1

            for pos0lateralReal in range (pos0lateralReal, p1_a2, -(velAng)):   
                giraServoLateral (pinS2, pos0lateralReal)
                time.sleep(0.015)
              
        else:

            for pos0lateralReal in range (pos0lateralReal, p1_a2, (velAng)):   
                giraServoLateral (pinS2, pos0lateralReal)
                time.sleep(0.015)


        for pos0garraReal in range (pos0garraReal, 90, (velAng) ) :               # Abre a garra
            giraServoGarra (pinS4, pos0garraReal)
            time.sleep(0.015)


        if pos0centralReal > p1_a1:                                              # Central para o p1

            for pos0centralReal in range (pos0centralReal, p1_a1, -(velAng)):
                giraServoCentral (pinS3, pos0centralReal)
                time.sleep(0.015)

        else:

            for pos0centralReal in range (pos0centralReal,p1_a1, (velAng)):
                giraServoCentral (pinS3, pos0centralReal)
                time.sleep(0.015)
           
        for pos0garraReal in range (pos0garraReal, 0, -(velAng) ) :               # Fecha a garra
            giraServoGarra (pinS4, pos0garraReal)
            time.sleep(0.015)
     

 

        repetir = 1
        calcTrajeto = 0
        

    if calcTrajeto == 0 and repetir == 1:

            resetPos = input("Deseja voltar para a posição inicial? y/n: ")
            print("")

            if resetPos =="y":

                if pos0centralReal > 50:                                                 # Volta a central

                    for pos0centralReal in range (pos0centralReal, 50, -(velAng)):
                        giraServoCentral (pinS3, pos0centralReal)
                        time.sleep(0.015)

                else:

                    for pos0centralReal in range (pos0centralReal, 50, (velAng)):         
                        giraServoCentral(pinS4, pos0centralReal)
                        time.sleep(0.015)


                if pos0lateralReal > 95:                                                # Volta a lateral pra inicial

                    for pos0lateralReal in range (pos0lateralReal, 95, -(velAng)):   
                        giraServoLateral (pinS2, pos0lateralReal)
                        time.sleep(0.015)
              
                else:

                    for pos0lateralReal in range (pos0lateralReal, 95, (velAng)):   
                        giraServoLateral (pinS2, pos0lateralReal)
                        time.sleep(0.015)



                if pos0baseReal > 0:

                    for pos0baseReal in range (pos0baseReal, 0, -(velAng)):            # Manda a base para a p1
                        giraServoBase (pinS1, pos0baseReal)
                        time.sleep(0.015)

                else:

                    for pos0baseReal in range (pos0baseReal, 0, velAng):               # Manda a base para a p1
                        giraServoBase (pinS1, pos0baseReal)
                        time.sleep(0.015)


                for pos0lateralReal in range (pos0lateralReal, 40, -(velAng)):         # Desce lateral
                    giraServoLateral (pinS2, pos0lateralReal)
                    time.sleep(0.015)
                                            
                for pos0centralReal in range (pos0centralReal, 120, (velAng)):         # Central para a entrega
                    giraServoCentral (pinS3, pos0centralReal)
                    time.sleep(0.015)


                for pos0garraReal in range ( pos0garraReal, 50, (velAng)):             # abre garra
                    giraServoGarra (pinS1, pos0garraReal)
                    time.sleep(0.015)


                for pos0centralReal in range (pos0centralReal, 50, -(velAng)):
                    giraServoCentral (pinS3, pos0centralReal)
                    time.sleep(0.015)
            


                resetRotina = input("Recomeçar rotina? y/n: ")
                
                if resetRotina == "y":

                    calcTrajeto = 1
                    repetir = 0
                
                else:
                    print("")
                    print(" Adeus ;)")
                    print("")
                    calcTrajetoFim = 1
                

            else:

                print("")
                print(" Adeus ;)" )
                print("")
                calcTrajetoFim = 1



            