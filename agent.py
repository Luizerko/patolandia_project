# MAC0318 Intro to Robotics
# Please fill-in the fields below with every team member info
#
# Name: Luis Vitor Pedreira Iten Zerkowski
# NUSP: 9837201
#
# Name: Ígor de Andrade Barberino
# NUSP: 11221689
#
# Any supplemental material for your agent to work (e.g. neural networks, data, etc.) should be
# uploaded elsewhere and listed down below together with a download link.
#
#
#
# ---
#
# Final Project - The Travelling Mailduck Problem
#
# Don't forget to run this file from the Duckievillage root directory path (example):
#   cd ~/MAC0318/duckievillage
#   conda activate duckietown
#   python3 assignments/mailduck/mailduck.py assignments/mailduck/examples/1.mail
#
# Submission instructions:
#  0. Add your names and USP numbers to the file header above.
#  1. Make sure that any last change hasn't broken your code. If the code crashes without running you'll get a 0.
#  2. Submit this file via e-disciplinas.

import pyglet
from pyglet.window import key
import numpy as np
import math
import random
from duckievillage import create_env, FRONT_VIEW_MODE
import cv2
import tensorflow

class Agent:
    def __init__(self, env):
        self.env = env
        self.radius = 0.0318
        self.baseline = env.unwrapped.wheel_dist/2
        self.motor_gain = 0.68*0.0784739898632288
        self.motor_trim = 0.0007500911693361842
        self.initial_pos = env.get_position()

        key_handler = key.KeyStateHandler()
        env.unwrapped.window.push_handlers(key_handler)
        self.key_handler = key_handler

        self.M = self.env.mailbox
        print(self.M.mail())
        self.count_entregas = 0
        self.conta_caminho = 0

        self.G = self.env.topo_graph
        print(self.G.path((self.M.mail()[0][1], self.M.mail()[0][2]), self.initial_pos))

        self.posicao_atual = self.G.closest_node(self.initial_pos)
        self.proxima_entrega = (self.M.mail()[self.count_entregas][1], self.M.mail()[self.count_entregas][2])
        
        if self.proxima_entrega != self.posicao_atual:
            self.path = self.G.path(self.proxima_entrega, self.posicao_atual)
            self.calcula_caminho = False
        else:
            self.M.deliver(0)
            self.count_entregas += 1
            self.calcula_caminho = True

        self.images = []
        self.vira = [0, 1, 2]
        self.labels = []

    def get_pwm_control(self, v: float, w: float)-> (float, float):
        ''' Takes velocity v and angle w and returns left and right power to motors.'''
        V_l = (self.motor_gain - self.motor_trim)*(v-w*self.baseline)/self.radius
        V_r = (self.motor_gain + self.motor_trim)*(v+w*self.baseline)/self.radius
        return V_l, V_r

    def preprocess(self):
        
        self.proxima_entrega = (self.M.mail()[self.count_entregas][1], self.M.mail()[self.count_entregas][2])
        self.posicao_atual = self.G.closest_node(self.env.get_position())

        self.path = self.G.path(self.proxima_entrega, self.posicao_atual)

        return
        

    def send_commands(self, dt: float):

        if self.calcula_caminho:
            self.preprocess()
            self.calcula_caminho = False
            self.conta_caminho = 0

        if len(self.path) > 0:
            destino = self.path[self.conta_caminho]
            posicao_auxiliar = self.env.get_position()
            angulo_auxiliar = self.env.cur_angle*180/np.pi
            print(posicao_auxiliar)
            print(destino)

            if self.conta_caminho == len(self.path) - 1:
                if abs(destino[0]-posicao_auxiliar[0]) > 0.13 or abs(destino[1]-posicao_auxiliar[1]) > 0.13:
                    
                    if abs(destino[0]-posicao_auxiliar[0]) > 0.13:
                        if (destino[0]-posicao_auxiliar[0]) > 0:
                            if(abs(angulo_auxiliar) > 3):
                                if (angulo_auxiliar > 0):
                                    print ("Direita!")
                                else:
                                    print('Esquerda!')
                            else:
                                print('ta ok!')

                        elif (destino[0]-posicao_auxiliar[0]) < 0:
                            if(abs(angulo_auxiliar) < 177):
                                if (angulo_auxiliar > 0):
                                    print ("Esquerda!")
                                else:
                                    print('Direita!')
                            else:
                                print('ta ok!')

                    else:
                        if (destino[1]-posicao_auxiliar[1]) > 0:
                            if(abs(angulo_auxiliar+90) > 3):
                                if (abs(angulo_auxiliar) < 90):
                                    print ("Direita!")
                                else:
                                    print('Esquerda!')
                            else:
                                print('ta ok!')

                        elif (destino[1]-posicao_auxiliar[1]) < 0:
                            if(abs(angulo_auxiliar-90) > 3):
                                if (abs(angulo_auxiliar) < 90):
                                    print ("Esquerda!")
                                else:
                                    print('Direita!')
                            else:
                                print('ta ok!')

                else:
                    print("cheguei no no final!")

                    self.M.deliver(self.count_entregas)
                    self.calcula_caminho = True

                    self.count_entregas += 1
                    if self.count_entregas >= len(self.M.mail()):
                        #MUDAR ESSE PEDAÇO DE CÓDIGO! NÃO DA PRA O MENINO RAPAZ TERMINAR DO NADA!
                        print('Patolandia salva! Safezada!')
                        return

            else:
                if abs(destino[0]-posicao_auxiliar[0]) > 0.3 or abs(destino[1]-posicao_auxiliar[1]) > 0.3:
                    
                    if abs(destino[0]-posicao_auxiliar[0]) > 0.3:
                        if (destino[0]-posicao_auxiliar[0]) > 0:
                            if(abs(angulo_auxiliar) > 3):
                                if (angulo_auxiliar > 0):
                                    print ("Direita!")
                                    self.vira.append(1)
                                else:
                                    print('Esquerda!')
                                    self.vira.append(2)
                            else:
                                print('ta ok!')
                                self.vira.append(0)

                        elif (destino[0]-posicao_auxiliar[0]) < 0:
                            if(abs(angulo_auxiliar) < 177):
                                if (angulo_auxiliar > 0):
                                    print ("Esquerda!")
                                    self.vira.append(2)
                                else:
                                    print('Direita!')
                                    self.vira.append(1)
                            else:
                                print('ta ok!')
                                self.vira.append(0)

                    else:
                        if (destino[1]-posicao_auxiliar[1]) > 0:
                            if(abs(angulo_auxiliar+90) > 3):
                                if (abs(angulo_auxiliar) < 90):
                                    print ("Direita!")
                                    self.vira.append(1)
                                else:
                                    print('Esquerda!')
                                    self.vira.append(2)
                            else:
                                print('ta ok!')
                                self.vira.append(0)

                        elif (destino[1]-posicao_auxiliar[1]) < 0:
                            if(abs(angulo_auxiliar-90) > 3):
                                if (abs(angulo_auxiliar) < 90):
                                    print ("Esquerda!")
                                    self.vira.append(2)
                                else:
                                    print('Direita!')
                                    self.vira.append(1)
                            else:
                                print('ta ok!')
                                self.vira.append(0)

                else:
                    print("cheguei num no")
                    self.conta_caminho += 1
        
        velocity = 0
        rotation = 0

        if self.key_handler[key.W]:
            velocity += 0.5
        if self.key_handler[key.A]:
            rotation += 1.5
        if self.key_handler[key.S]:
            velocity -= 0.5
        if self.key_handler[key.D]:
            rotation -= 1.5
        if self.key_handler[key.E]:
            success = self.M.deliver(0)
            print(success)
            

        pwm_left, pwm_right = self.get_pwm_control(velocity, rotation)
        self.env.step(pwm_left, pwm_right)

        self.images.append(cv2.resize(self.env.front(), (80, 60)))
        self.labels.append((velocity, rotation))