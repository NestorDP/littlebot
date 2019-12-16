# Littlebot

Projeto opensource para estudo de robótica móvel com Robot Operating System ROS. Além de áreas importantes no desemvolvimento de robôs, como: sistemas embarcados, engenharia de controle, processamento de sinais digitais...

![Alt Text](https://ap.imagensbrasil.org/images/2019/11/29/Screenshot-from-2019-11-29-14-00-38.png)

## Testando o Littlebot

Estas instruções te ajudaram a fazer os primeiros testes com a plataforma Littlebot

### Pré-requisitos

Para testar o Littlebot em seu sistema você precisa ter instalado o ROS ([melodic](http://wiki.ros.org/melodic/Installation) recomendado). Um erro pode acontecer ao tentar vizualizar o modelo no RVIZ caso isso ocorra pode ser necessário mudar uma variável de ambiente, como segue abaixo:

```bash
export LC_NUMERIC="en_US.UTF-8"
```

### Rodando a simulação

Para simular o Littlebot, o primeiro passo é criar um [workspace ROS](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

```bash
mkdir -p ~/littlebot_ws/src
cd littlebot_ws
catkin_make
```

Após criar o workspace fazer o clone do repositório

```bash
cd src
git clone https://github.com/NestorDP/littlebot.git
cd ..
catkin_make
source devel/setup.bash
```

Para rodar a simulção no Gazebo

```bash
roslaunch littlebot_gazebo gazebo.launch
```

## Wiki

Para mais informações acesse o [Littlebot wiki](https://github.com/NestorDP/littlebot/wiki)


a partir daqui a tradução para o inglês

