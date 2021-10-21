
########### Bibliotecas de comandos usadas ##############

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

########################## Modos ##########################

inserirCoord = 1
calcVolTrabalho = 0
adquirirAng = 0
pontoAtual = 1
pontoUmCalculado = 0
calcTrajeto = 0
calcTrajetoFim = 0

########################## Valores do braço ###############

bracoInf = 80     #Tamanho braço inferior 80mm
bracoSup = 135    #Tamanho braço superior + centro da garra 126mm

limiteInf = 105    #Tamanho entre a ponta da garra e o eixo da base no mínimo
limiteSup = 205   #Tamanho entre a ponta da garra e o eixo da base no máximo

########################## Inserindo coordenadas ##########################

while calcTrajetoFim != 1:    #Enquanto o calculo do trajeto não chegar ao fim, ele continua rodando.

    if inserirCoord == 1:

        if pontoAtual == 1 and pontoUmCalculado == 1:  # Passa pro ponto2 se o ponto1 já estiver estabelecido

            pontoAtual = 2

        print("Digite as coordenadas desejadas para o ponto",pontoAtual,":")

        xd = float(input("X: "))          # Valores dos ângulos desejados
        yd = float(input("Y: " ))         # serão utilizados tanto no cálculo de vol. de trabalho
        zd = float(input("Z: "))          # quanto na inversão de coordenadas

        calcVolTrabalho = 1
        inserirCoord = 0

    ########################## Cálculo volume de trabalho ##########################

    if calcVolTrabalho == 1 and inserirCoord == 0 :

        R = ((xd)**2 + (yd)**2 + (zd)**2)**(1/2)           #Função que descreve a soma vetorial

        time.sleep(0.3)

        if R <= (limiteSup) and R >= (0.1)*(limiteInf) and zd >= -75 :
            
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

            print("Iniciando cinemática inversa.")
            print(' ')

            time.sleep(0.3)

            (a1,a2,a3) = least_squares(adquirirAngs,[0,0,0], args=(xd, yd, zd), bounds= ((-3.14/4),3.14)).x
                
            Ang1 = round((math.degrees(a1)), 1)   # Angulo final que será armazenado no valor do motor
            Ang2 = round((math.degrees(a2)), 1)   # Para mover o braço, bastará transferir esses resultados
            Ang3 = round((math.degrees(a3)), 1)   # Para a IDE do arduino e aplicar os valores ao motor (FIRMATA)
                
            time.sleep(0.4)

            #   Exibição de resultados:
            
            print ("Ângulos obtidos para o ponto",pontoAtual, ": ")
            print(' ')
            print("A1: ", Ang1,"A2: ", Ang2,"A3: ", Ang3)   
            


            print("")

            if pontoAtual == 1:

                p1 = [Ang1,Ang2,Ang3]

                adquirirAng = 0
                inserirCoord = 1
                pontoUmCalculado = 1                

            if pontoAtual == 2:

                p2 = [Ang1,Ang2,Ang3]

                adquirirAng = 0

                calcTrajeto = 1


    if calcTrajeto == 1:

        time.sleep(0.4)
        
        print("Ponto 1: ", p1)
        print("Ponto 2: ", p2)
        
        calcTrajetoFim = 1