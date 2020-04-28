# Introdução de ROS e robôs móveis terrestres

Após clonar este repositório ao seu ROS workspace/src local (`$git clone https://bitbucket.org/grupomecatronica/gmr_intro.git`), não se esqueça de compilar:
```console
user@pc: ~/ros_ws$ catkin_make
```
Note que `catkin_make` sempre deve ser executada no diretório raiz do seu workspace. Depois, rode o master com `roscore` e finalmente,

```
user@pc: ~/ros_ws$ rosrun gmr_intro gmr_intro_node
```
para rodar o nó. 

*Dica 1:* use a capacidade do terminator de dividir tela (botão direito, split horizontal/split vertical ou Ctrl+Shift+o/Ctrl+Shift+e) para não precisar abrir vários terminais.

*Dica 2:* Já usou `rostopic list`?

## Escrevendo seu primeiro subscritor

Nós temos quatro tópicos ativos: */left_rpm*, */right_rpm*, */rosout* e */rosout_agg*. Os últimos dois estão sempre ativos quando há um ROS master rodando e eles são parte do mecanismo de log do console (e.g. útil para debug). Mas agora estamos interessados nos primeiros dois. Você pode acessar o valor deles utilizando a ferramenta de linha de comando `$ rostopic echo /left_rpm` ou `$ rostopic echo /right_rpm`. Como podemos ter essas informações dentro dos nossos códigos?

Se existe um tópico ao qual queremos *escutar*, a solução é criarmos um subscritor. Primeiro, vamos conhecer melhor o tópico ao qual queremos nos subscrever:

```console
user@pc: ~/$ rostopic info /left_rpm
Type: std_msgs/Float32

Publishers: 
 * /gmr_intro_node (http://ROS_IP:41291/)

Subscribers: None
```
Podemos ver que */left_rpm* carrega uma mensagem do tipo std_msgs/Float32 e que **/gmr_intro_node** é o publicador. E ninguém está subscrevendo àquele tópico. Agora vamos criar nosso pacote:

```console
user@pc:~/ros_ws$ cd src
user@pc:~/ros_ws/src$ catkin_create_pkg sub_rpm roscpp std_msgs 
Created file sub_rpm/package.xml
Created file sub_rpm/CMakeLists.txt
Created folder sub_rpm/include/sub_rpm
Created folder sub_rpm/src
Successfully created files in /home/dell/ros_ws/src/sub_rpm. Please adjust the values in package.xml.

```

em que `sub_rpm` é o nome do novo pacote e `roscpp`, `std_msgs`, e qualquer outro nome que aparecer depois são dependências do pacote. Quando queremos um nó em C++, utilizamos roscpp. Caso seja Python, rospy. Com este pacote, queremos nos subscrever a uma mensagem do tipo std_msgs/Float32, então o pacote tem uma dependência a ser colocada: std_msgs. Confira seu pacote criado:


```console
user@pc:~/ros_ws/src$ cd sub_rpm/
user@pc:~/ros_ws/src/sub_rpm$ ls
CMakeLists.txt  include  package.xml  src
```

Agora abra seu pacote no editor (sugestão de usar VS Code). Primeiro, iremos configurar  CMakeLists.txt. Descomente a linha 135:

```
add_executable(${PROJECT_NAME}_node src/sub_rpm_node.cpp)
```
e as linhas de 148 a 150:
```
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)
```
Na primeira linha, estamos adicionando um executável, i.e. um nó ROS, ao processo de compilação. Ele será chamado de ${PROJECT_NAME}_node e seu código fonte está localizado em src/sub_rpm_node.cpp. O segundo conjunto está fazendo os links entre o nó com as bibliotecas necessárias (pelo menos catkin libraries). Vamos então ao código. Crie um novo arquivo sub_rpm_node.cpp no diretório ~/ros_ws/src/sub_rpm/src. O código completo é:

```cpp

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>

void subCb(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO_STREAM(msg->data);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sub_node");
    ros::NodeHandle nh;

    std::string nome_topico = "/left_rpm";
    int tamanho_fila = 10;
    ros::Subscriber sub_rpm = nh.subscribe \
        (nome_topico, tamanho_fila, subCb);

    ros::spin();
}
```



Veremos agora cada parte. Comçamos pelos #includes. A ros.h está sempre presente, mas dê atenção especial a não se esquecer de incluir a biblioteca associada a cada mensagem que for utilizar:

```cpp
    #include <ros/ros.h>
    #include <std_msgs/Float32.h>
    #include <string>
```

Vamos iniciar o nó passando os argumentos da *main* e também conferindo o nome do nó:

```cpp
        ros::init(argc, argv, "sub_node");
```

Declararemos agora o gerenciador do nosso nó, o *NodeHandle*. Ele que fará esse link deste código com as funcionalidades de ROS:

```cpp
        ros::NodeHandle nh;
```

Um subscritor precisa saber o nome do tópico a quem se subscreverá, quantas mensagens queremos guardar na fila caso não consigamos processar e a função que ficará associada à chegada de novas mensagens. Iniciaremos agora o nosso subscritor:

```cpp
    std::string nome_topico = "/left_rpm";
    int tamanho_fila = 10;
    ros::Subscriber sub_rpm = nh.subscribe\
        (nome_topico, tamanho_fila, subCb);
```

em que `tamanho_fila = 10` indica que caso o nó não consiga processar as mensagens rápido o suficiente, manterá as mensagens que chegarem na fila. Porém, quando atingir 10, começará a excluir as mensagens mais antigas à medida que novas chegarem. E subCb é a função de callback, aquela que será chamada quando uma nova mensagem chegar:

```cpp
void subCb(const std_msgs::Float32::ConstPtr &msg)
{
  ROS_INFO_STREAM("Nova mensagem: " << msg->data);
}
```
Note que a mensagem *msg* é passada através de um *boost shared_ptr* então não precisa se preocupar de perder a informação porque chegou alguma mensagem nova enquanto dentro da função. Para acessar os campos da mensagem, utilize `->` e campo desejado. Por exemplo, para ver os campos de uma mensagem std_msgs/Float32, use `$ rosmsg info std_msgs/Float32`. Use ROS_INFO_STREAM para imprimir na tela. 

Finalmente, temos que criar o loop para manter o nó rodando. Conseguimos isso com:

```cpp
    ros::spin();
```

A função *spin()* constrói um loop interno e o nó fica à espera de atualizações do(s) tópico(s) subscrito(s). Salve o arquivo e compile:

```console
user@pc: ~/ros_ws$ catkin_make
```

e rode o nó criado com:
```console
user@pc: ~/ros_ws$ rosrun sub_rpm sub_rpm_node
```

Agora, você é capaz de usar o dados de um tópico dentro do seu código! 


Contato: Akihiro (akihirohh@gmail.com)

Apoio da Fundação de Amparo à Pesquisa do Estado de São Paulo (FAPESP) através do processo nº 2018/10894-2. 
As opiniões, hipóteses e conclusões ou recomendações expressas neste material são de responsabilidade do(s) autor(es) e não necessariamente refletem a visão da FAPESP.

