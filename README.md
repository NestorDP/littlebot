# ![Littlebot](https://ap.imagensbrasil.org/images/2019/12/13/littlebottitulo.png)

Projeto opensource para estudo de robótica móvel com Robot Operating System ROS. Além de áreas importantes no desenvolvimento de robôs, como: sistemas embarcados, engenharia de controle, processamento de sinais digitais...

![Alt Text](https://ap.imagensbrasil.org/images/2019/12/14/prints.png)

## Testando o Littlebot

Estas instruções te ajudarão a fazer os primeiros testes com a plataforma Littlebot

### Pré-requisitos

Para testar o Littlebot em seu sistema você precisa ter instalado o ROS ([melodic](http://wiki.ros.org/melodic) recomendado). Um erro pode acontecer ao tentar vizualizar o modelo no RVIZ no ROS-melodic, caso isso ocorra, pode ser necessário mudar uma variável de ambiente, como segue abaixo:

```bash
export LC_NUMERIC="en_US.UTF-8"
```

### Rodando a simulação

Para simular o Littlebot, o primeiro passo é criar um [workspace ROS](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

```bash
mkdir -p ~/littlebot_ws/src
cd littlebot_ws
catkin_make
source devel/setup.bash
```

Após criar o workspace fazer o clone do repositório

```bash
cd src
git clone https://github.com/NestorDP/littlebot.git
cd ..
catkin_make
```

Para rodar a simulção no Gazebo

```bash
roslaunch littlebot_gazebo gazebo.launch
```

## Wiki

Para mais informações acesse o [Littlebot wiki](https://github.com/NestorDP/littlebot/wiki)
