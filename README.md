# Littlebot

Projeto de robô móvel didático para estudo de robótica com ROS, sistemas embarcados, engenharia de controle... 


![Alt Text](https://user-images.githubusercontent.com/37759765/64899983-83557400-d664-11e9-9659-fe93a5824e9e.png)

## Testando o Littlebot

Estas instruções te ajudaram a fazer os primeiros testes com a plataforma Littlebot

### Prerequisitos

Para testar o Littlebot em seu sistema você precisa ter instalado o ROS (melodic recomendado)

```
Give examples
```

### Instalando

Para instalar o Littlebot é necessário criar um workspace ROS

```
mkdir -p ~/littlebot_ws/src
cd littlebot_ws
catkin_make
```

Após criar o workspace fazer o clone do repositório

```
cd src
git clone https://github.com/NestorDP/littlebot.git
cd ..
catkin_make
```

## Rodando a simulação 

Para rodar a simulção no Gazebo

```
roslaunch littlebot_gazebo gazebo.launch
```
