# Código Cinemática Inversa Final V4

########### Bibliotecas de comandos usadas quack ##############

from pyfirmata import Arduino, SERVO
import pyfirmata
from pyfirmata.util import Iterator
from scipy.optimize import least_squares
import math
import time
import numpy as np


def adquirirAngs(ang, xd, yd, zd):  # Criando função inversa para adquirir a1,a2 e a3

    [a1, a2, a3] = ang

    xb = bracoInf * math.cos(a1) * math.cos(a3)
    xa = xb + bracoSup * math.cos(a2) * math.cos(a3)
    yb = bracoInf * math.cos(a1) * math.sin(a3)
    ya = yb + bracoSup * math.cos(a2) * math.sin(a3)
    zb = bracoInf * math.sin(a1)
    za = zb - bracoSup * math.sin(a2)

    return xa - xd, za - zd, ya - yd


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
processoSoldagem = 0
indoSoldagem = 0

exibirMensagem = True
ativarRotina = False
fimDoPrograma = False

############### Valores e entradas motores/botões  ################

port = "COM6"  # Porta SUB em que o Arduino foi conectado
placa = Arduino(port)  # Dando um nome para o nosso arduino

it = Iterator(placa)
it.start()

pinS1 = 8  # base                      # Indicando onde estão as entradas e o que são
pinS2 = 9  # lateral
pinS3 = 10  # central
pinS4 = 11  # garra

placa.digital[pinS1].mode = SERVO  # Indicando que essas entradas são servos
placa.digital[pinS2].mode = SERVO
placa.digital[pinS3].mode = SERVO
placa.digital[pinS4].mode = SERVO

botao1 = placa.digital[5]  # Indicando o botão
botao1.mode = pyfirmata.INPUT

servoBase = placa.digital[pinS1]  # Substituindo o nome das entradas pra simplificar
servoLateral = placa.digital[pinS2]
servoCentral = placa.digital[pinS3]
servoGarra = placa.digital[pinS4]

pos0baseImaginaria = 170  # Definindo posições iniciais para cada motor
pos0centralImaginaria = 90
pos0lateralImaginaria = 40
pos0garraImaginaria = 0

pos0baseReal = (0 + pos0baseImaginaria)  # Base real = 0  
pos0centralReal = 140 - pos0centralImaginaria  # Central real = 50
pos0lateralReal = (45 - pos0lateralImaginaria + 45)  # Lateral real = 95, leste
pos0garraReal = 0  # Garra real = 90  |   Aberta

servoBase.write(pos0baseReal)  # Enviando posições iniciais para os servos
servoCentral.write(pos0centralReal)
servoLateral.write(pos0lateralReal)
servoGarra.write(pos0garraReal)

velAng = 2  # Velocidade angular de giro dos motores

########################## Valores do braço ###############

bracoInf = 80  # Tamanho braço inferior 80mm
bracoSup = 135  # Tamanho braço superior + centro da garra 135mm

limiteInf = 105  # Tamanho entre a ponta da garra e o eixo da base no mínimo
limiteSup = 205  # Tamanho entre a ponta da garra e o eixo da base no máximo

########################## Inserindo coordenadas ##########################


while (calcTrajetoFim != 1):  

    if inserirCoord == 1:

        if (
            pontoAtual == 1 and pontoUmCalculado == 1):  
            # Passa pro ponto2 se o ponto1 já estiver estabelecido

            pontoAtual = 2

        print("")
        print("")
        print("Digite as coordenadas desejadas para o ponto", pontoAtual, ":")
        print("")

        xd = float(input("X: "))  # Valores dos ângulos desejados
        yd = float(input("Y: ")) 
        zd = float(input("Z: ")) 

        calcVolTrabalho = 1
        inserirCoord = 0

    ########################## Cálculo volume de trabalho ##########################

    if calcVolTrabalho == 1 and inserirCoord == 0:

        R = ((xd) ** 2 + (yd) ** 2 + (zd) ** 2) ** (1 / 2)  # Soma vetorial

        time.sleep(0.3)

        if R <= (limiteSup) and R >= (limiteInf) and zd >= -80:

            print(" ")
            print("Valor do raio: %.2f" % R)
            print(" ")
            print("Coordenadas inseridas fazem parte do volume de trabalho")
            print(" ")

            adquirirAng = 1
            calcVolTrabalho = 0

        else:

            print(" ")
            print("Valor do raio: %.2f" % R)
            print(" ")
            print("Coordenadas inseridas não fazem parte do volume de trabalho")
            time.sleep(0.5)
            print(" ")
            print("Insira outros valores")

            inserirCoord = 1
            calcVolTrabalho = 0

    ############# Cálculo cinemática inversa | Obtenção dos ângulos ###########

    if adquirirAng == 1:

        time.sleep(0.3)

        (a1, a2, a3) = least_squares(adquirirAngs, [0, 0, 0], args=(xd, yd, zd), bounds=(-(3.14 / 4), 3.14)).x    #Método regressão linear

        Ang1 = round((math.degrees(a1)))       #Arredondamento dos resultados obtidos
        Ang2 = round((math.degrees(a2)))
        Ang3 = round((math.degrees(a3)))

        Ang1Interno = 140 - Ang1                  # Conversão dos ângulos calculados para o ângulo interno correspondente do motor.
        Ang2Interno = 45 - Ang2 + 45
        Ang3Interno = Ang3

        time.sleep(0.4)

        #   Exibição de resultados:

        print("Ângulos obtidos para o ponto", pontoAtual, ": ")
        print(" ")
        print("A1 calc:", Ang1, "A2 calc:", Ang2, "A3 calc:", Ang3)
        print("A1 interno:", Ang1Interno, " A2 real:", Ang2Interno, " A3 real:", Ang3Interno)

        print("________________________________")

        if pontoAtual == 1:

            p1 = [Ang1, Ang2, Ang3]
            p1Interno = [Ang1Interno, Ang2Interno, Ang3Interno]

            adquirirAng = 0
            inserirCoord = 1
            pontoUmCalculado = 1

        if pontoAtual == 2:

            p2 = [Ang1, Ang2, Ang3]
            p2Interno = [Ang1Interno, Ang2Interno, Ang3]

            adquirirAng = 0

            calcTrajeto = 1

    if calcTrajeto == 1:

        time.sleep(0.4)

        print(" ")
        print("Ponto 1: ", p1)
        print("Ponto 2: ", p2)
        print(" ")
        print("Ponto 1 interno: ", p1Interno)
        print("Ponto 2 interno:  ", p2Interno)

        print(" ")

        p1_a1 = p1Interno[0]  # Ang1 - Central
        p1_a2 = p1Interno[1]  # Ang2 - Lateral
        p1_a3 = p1Interno[2]  # Ang3 - Base

        p2_a1 = p2Interno[0]  # Ang1 - Central
        p2_a2 = p2Interno[1]  # Ang2 - Lateral
        p2_a3 = p2Interno[2]  # Ang3 - Base

        if pos0baseReal > p1_a3:  # Base vai

            for pos0baseReal in range(pos0baseReal, p1_a3, -(velAng)):
                giraServoBase(pinS1, pos0baseReal)
                time.sleep(0.015)

        else:

            for pos0baseReal in range(pos0baseReal, p1_a3, velAng):
                giraServoBase(pinS1, pos0baseReal)
                time.sleep(0.015)

        if pos0lateralReal > p1_a2:  # Lateral para o p1

            for pos0lateralReal in range(pos0lateralReal, p1_a2, -(velAng)):
                giraServoLateral(pinS2, pos0lateralReal)
                time.sleep(0.015)

        else:

            for pos0lateralReal in range(pos0lateralReal, p1_a2, (velAng)):
                giraServoLateral(pinS2, pos0lateralReal)
                time.sleep(0.015)

        for pos0garraReal in range(pos0garraReal, 90, (velAng)):  # Abre a garra
            giraServoGarra(pinS4, pos0garraReal)
            time.sleep(0.015)

        if pos0centralReal > p1_a1:  # Central para o p1

            for pos0centralReal in range(pos0centralReal, p1_a1, -(velAng)):
                giraServoCentral(pinS3, pos0centralReal)
                time.sleep(0.015)

        else:

            for pos0centralReal in range(pos0centralReal, p1_a1, (velAng)):
                giraServoCentral(pinS3, pos0centralReal)
                time.sleep(0.015)

        for pos0garraReal in range(pos0garraReal, 0, -(velAng)):  # Fecha a garra
            giraServoGarra(pinS4, pos0garraReal)
            time.sleep(0.015)

        indoSoldagem = 1
        print("modo indo pra Soldagem")
        calcTrajeto = 0

    if indoSoldagem == 1 and calcTrajeto == 0:  # Vai para a soldagem

        if pos0centralReal > 50:  # Volta a central

            for pos0centralReal in range(pos0centralReal, 50, -(velAng)):
                giraServoCentral(pinS3, pos0centralReal)
                time.sleep(0.015)

        else:

            for pos0centralReal in range(pos0centralReal, 50, (velAng)):
                giraServoCentral(pinS3, pos0centralReal)
                time.sleep(0.015)

        if pos0lateralReal > 135:  # Sobe a lateral para esquivar do objeto

            for pos0lateralReal in range(pos0lateralReal, 135, -(velAng)):
                giraServoLateral(pinS2, pos0lateralReal)
                time.sleep(0.015)

        else:

            for pos0lateralReal in range(pos0lateralReal, 135, (velAng)):
                giraServoLateral(pinS2, pos0lateralReal)
                time.sleep(0.015)

        if pos0baseReal > p2_a3:

            for pos0baseReal in range(
                pos0baseReal, p2_a3, -(velAng)):  # Manda a base para a p2
                giraServoBase(pinS1, pos0baseReal)
                time.sleep(0.015)

        else:

            for pos0baseReal in range(pos0baseReal, p2_a3, velAng):
                giraServoBase(pinS1, pos0baseReal)
                time.sleep(0.015)

        if pos0lateralReal > p2_a2:  # Desce a lateral após a esquiva

            for pos0lateralReal in range(pos0lateralReal, p2_a2, -(velAng)):
                giraServoLateral(pinS2, pos0lateralReal)
                time.sleep(0.015)

        else:

            for pos0lateralReal in range(pos0lateralReal, p2_a2, (velAng)):
                giraServoLateral(pinS2, pos0lateralReal)
                time.sleep(0.015)

        if pos0centralReal > p2_a1:  # Desce a central após a esquiva

            for pos0centralReal in range(pos0centralReal, p2_a1, -(velAng)):
                giraServoCentral(pinS3, pos0centralReal)
                time.sleep(0.015)

        else:

            for pos0centralReal in range(pos0centralReal, p2_a1, (velAng)):
                giraServoCentral(pinS3, pos0centralReal)
                time.sleep(0.015)

        processoSoldagem = 1
        indoSoldagem = 0

    if indoSoldagem == 0 and processoSoldagem == 1:

        print("")
        respostaSoldagem = input("Soldagem foi finalizada?? y/n: ")
        print("")

        if respostaSoldagem == "y":

            if pos0centralReal > 50:  # Volta a central

                for pos0centralReal in range(pos0centralReal, 50, -(velAng)):
                    giraServoCentral(pinS3, pos0centralReal)
                    time.sleep(0.015)

            else:

                for pos0centralReal in range(pos0centralReal, 50, (velAng)):
                    giraServoCentral(pinS4, pos0centralReal)
                    time.sleep(0.015)

            if pos0lateralReal > 95:  # Volta a lateral pra inicial

                for pos0lateralReal in range(pos0lateralReal, 95, -(velAng)):
                    giraServoLateral(pinS2, pos0lateralReal)
                    time.sleep(0.015)

            else:

                for pos0lateralReal in range(pos0lateralReal, 95, (velAng)):
                    giraServoLateral(pinS2, pos0lateralReal)
                    time.sleep(0.015)

            if pos0baseReal > 0:

                for pos0baseReal in range(
                    pos0baseReal, 0, -(velAng)):  # Manda a base para a posição de entrega
                    giraServoBase(pinS1, pos0baseReal)
                    time.sleep(0.015)

            else:

                for pos0baseReal in range(pos0baseReal, 0, velAng):
                    giraServoBase(pinS1, pos0baseReal)
                    time.sleep(0.015)

            for pos0lateralReal in range(
                pos0lateralReal, 40, -(velAng)):  # Desce a lateral para a entrega
                giraServoLateral(pinS2, pos0lateralReal)
                time.sleep(0.015)

            for pos0centralReal in range(
                pos0centralReal, 120, (velAng)):  # Desce a Central para a entrega
                giraServoCentral(pinS3, pos0centralReal)
                time.sleep(0.015)

            for pos0garraReal in range(pos0garraReal, 50, (velAng)):  # Abre a garra
                giraServoGarra(pinS1, pos0garraReal)
                time.sleep(0.015)

            for pos0centralReal in range(pos0centralReal, 50, -(velAng)):
                giraServoCentral(pinS3, pos0centralReal)
                time.sleep(0.015)

            if pos0lateralReal > 135:  # Sobe a lateral para esquivar do objeto

                for pos0lateralReal in range(pos0lateralReal, 135, -(velAng)):
                    giraServoLateral(pinS2, pos0lateralReal)
                    time.sleep(0.015)

            else:

                for pos0lateralReal in range(pos0lateralReal, 135, (velAng)):
                    giraServoLateral(pinS2, pos0lateralReal)
                    time.sleep(0.015)

            if pos0baseReal < 170:

                for pos0baseReal in range(
                    pos0baseReal, 170, (velAng)):  # Manda a base para a posição inicial
                    giraServoBase(pinS1, pos0baseReal)
                    time.sleep(0.015)

            else:

                for pos0baseReal in range(pos0baseReal, 170, -(velAng)):
                    giraServoBase(pinS1, pos0baseReal)
                    time.sleep(0.015)

            if pos0lateralReal > 40:  # Desce a lateral para a posição inicial

                for pos0lateralReal in range(pos0lateralReal, 40, -(velAng)):
                    giraServoLateral(pinS2, pos0lateralReal)
                    time.sleep(0.015)

            else:

                for pos0lateralReal in range(pos0lateralReal, 40, (velAng)):
                    giraServoLateral(pinS2, pos0lateralReal)
                    time.sleep(0.015)
                    
            print("")
            resetRotina = input("Recomeçar rotina? y/n: ")

            if resetRotina == "y":

                calcTrajeto = 1
                processoSoldagem = 0

            else:
                print("")
                print(" Adeus ;)")
                print("")
                calcTrajetoFim = 1

        else:

            print("")
            print(" Adeus ;)")
            print("")
            calcTrajetoFim = 1
