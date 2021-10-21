""" LEIA-ME: COMENTÁRIOS E DADOS DO PROJETO (Clicka na setinha à esquerda)":

Created on Tue Aug 31 20:03:49 2021

Este código realiza os cálculos de conversão de coordenadas da extremidade
de um braço robótico para ângulos de posicionamento de motores. 
(Cinemática inversa)

Após inserir o valor das coordenadas desejadas, o programa detecta os ângulos
necessários para que o braço alcance tal posição. Após isso, se o usuário
desejar, poderá fazer um teste, que primeiro transformará esses ângulos obtidos
nas coordenadas resultantes, para então testar novamente essas coordenadas e
observar se resulta nos mesmos ângulos e coordenadas que surgira50m durante o
programa.50


Problemas:
    
    - É necessário um código mais complexo para testar as possíveis saídas de
    ângulos, checando se possuem diferenças de no máximo 2 graus. (Se quisermos
    um teste perfeito, claro.)
    - Não há como controlar as saídas de ângulo para um posicionamento do motor
    mais otimizado as nossas necessidades.
    - Ainda há como programar uma forma automática de comparar os resultados
    
Próximos passos: 
    
    - Buscar a forma correta de linkar a programação Python com a IDE do
    arduino (provavelmente usando Firmata).
    - Necessário adquirir valores de limites de ângulos para testagem de volume de trabalho

"""
########### Bibliotecas de comandos usadas ##############

from scipy.optimize import least_squares
import math
import time
import numpy as np

##################### Firmata ##################
"""""
porta = 'COM6'    #Editar a entrada conforme o que está usando
placa = Arduino(porta)

pin1 = 8
pin2 = 9
pin3 = 10
pin4 = 11

placa.digital[pin1].mode = SERVO
placa.digital[pin2].mode = SERVO
placa.digital[pin3].mode = SERVO
placa.digital[pin4].mode = SERVO
"""""
                
############################# DEFININDO AS FUNÇÕES #################################

def posição(a1,a2,a3):                  #Criando função de análise cin. direta p/ xa,ya,za
            
            xb = B*math.cos(math.radians(a1))*math.cos(math.radians(a3))
            xa = xb + a*math.cos(math.radians(a2))*math.cos(math.radians(a3))
            yb = B*math.cos(math.radians(a1))*math.sin(math.radians(a3))
            ya = yb + a*math.cos(math.radians(a2))*math.sin(math.radians(a3))
            zb = B*math.sin(math.radians(a1))
            za = zb - a*math.sin(math.radians(a2))
        
            return xa,za,ya

def posição_reversa(ang,xd,yd,zd):      #Criando função inversa para adquirir a1,a2 e a3

        
        [a1,a2,a3] = ang
        
        xb = B*math.cos(a1)*math.cos(a3)
        xa = xb + a*math.cos(a2)*math.cos(a3)
        yb = B*math.cos(a1)*math.sin(a3)
        ya = yb + a*math.cos(a2)*math.sin(a3)
        zb = B*math.sin(a1)
        za = zb - a*math.sin(a2)
        
        return xa-xd,za-zd,ya-yd

########################## TESTE - VOLUME DE TRABALHO ##############################

print("")
print("")

B = float(input("Tamanho braço inferior (em mm): "))       # Valores do braço
a = float(input("Tamanho braço superior (em mm): ")) 

print("")      
print("Digite as coordenadas desejadas:")

xd = float(input("X: "))          # Valores dos ângulos desejados
yd = float(input("Y: " ))         # serão utilizados tanto no cálculo de vol. de trabalho
zd = float(input("Z: "))          # quanto na inversão de coordenadas

Dist = float(B+a)                                  #Distância total com o braço esticado sem limites de ângulos

R = ((xd)**2 + (yd)**2 + (zd)**2)**(1/2)           #Função que descreve a soma vetorial

time.sleep(0.5)

if R <= (0.8)*(Dist) and R >= (0.1)*(Dist) and zd > 0 :
    
        print(' ')
        print('Valor do raio: %.2f'%R)
        print(' ')
        print('Valores inseridos fazem parte do volume de trabalho')
        print('')
        vtrabalho = 1
        
else:
    
        print(' ')
        print('Valor do raio: %.2f'%R)
        print(' ')
        print('Valores inseridos não fazem parte do volume de trabalho')
        time.sleep(0.5)
        print(' ')
        print('Insira outros valores')
        vtrabalho = 0
        
####################### FIM DO VOLUME DE TRABALHO ####################################

if vtrabalho == 1:   
  
            
            time.sleep(0.5)
            print("Iniciando cinemática inversa...")
            print(' ')
            time.sleep(1)

            (a1,a2,a3) = least_squares(posição_reversa,[0,0,0], args=(xd, yd, zd), bounds= (0,2*3.14)).x
            
            Ang1 = (math.degrees(a1))   # Angulo final que será armazenado no valor do motor
            Ang2 = (math.degrees(a2))   # Para mover o braço, bastará transferir esses resultados
            Ang3 = (math.degrees(a3))   # Para a IDE do arduino e aplicar os valores ao motor (FIRMATA)
            
            #   Exibição de resultados:
                
            time.sleep(1)
            print ("Ângulos obtidos:")
            print(' ')
            print("A1: ", Ang1)
            print("A2: ", Ang2)
            print("A3: ", Ang3)
            
            print("")
            
            modoTeste = str(input("Deseja testar os valores obtidos? y/n:"))
            
            
            
            if modoTeste == "y":     # O teste de inversão é executado caso o usuário escolha.
            
                    time.sleep(0.5)
                    print(' ')
                    print("Iniciando teste: As coordenadas de saída e entrada serão comparadas.")
                    time.sleep(0.5) 
                    
                    a1 = Ang1
                    a2 = Ang2
                    a3 = Ang3
        
                    xa,za,ya = posição(a1,a2,a3)        #Chamando a posição e armazenando seus resultados
                
                    XA=xa
                    YA=ya
                    ZA=za
                    
                    time.sleep(0.5) 
                    print("")
                    print("- Coordenadas obtidas: X: %.2f"%XA,"Y: %.2f"%YA,"Z: %.2f"%ZA)
                    print("")
                    
                    Ang1, Ang2, Ang3 =  least_squares(posição_reversa,[0,0,0], args=(XA, YA, ZA), bounds= (0,2*3.14)).x
                    
                    Ang1 = (math.degrees(Ang1))
                    Ang2 = (math.degrees(Ang2))                    
                    Ang3 = (math.degrees(Ang3))
                     
                    time.sleep(0.5) 

                    print("")
                    print("- Ângulos obtidos:")
                    print(' ')
                    print("A1: ", Ang1)
                    print("A2: ", Ang2)
                    print("A3: ", Ang3)
                            
                    time.sleep(0.5)
                    print("")
                    print("Programa encerrado :D Gropo A é zika")
                    
            else:
                print("")
                print("Programa encerrado :D")

  #  np.linspace ( inicio, fim, pontos) #Cria um vetor com vários pontos entre o inicio e o fim (pos1, posx, npos)