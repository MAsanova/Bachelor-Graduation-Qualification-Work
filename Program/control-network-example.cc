#include <stdio.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <math.h>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/propagation-loss-model.h"
#include <ns3/netanim-module.h>

using namespace std;
using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE ("control-network-example");

const uint32_t k_normalPacketSize = 512;
const uint32_t k_port = 80;

FILE *fileObject;
FILE *qualityIndicators;
FILE *delays;
AnimationInterface* anim;

//global vars and constants
double Alpha;
double delay;
double g_desiredX = 0;    //Желаемая точка
double const k_pi = 3.14159;  //Некоторые константы
double g_t=0, g_dt = 0.01; //Начальное время и его приращение
double const k_e = 0.050;  //Точность вычислений

double timeOfTransitionProcess = 0;
double overshoot = 0;
double staticError = 0;
double linearIntegral = 0;
double integralofModuleOfError = 0;
double integralOfSquareOfError = 0;
double integralOfWeightedModuleOfError = 0;
double integralOfWeightedSquareOfError = 0;


enum class Protocol
{
    OLSR, AODV, DSDV, DSR, QRouting
};

class RoutingExperiment
{
public:
    RoutingExperiment ();
    void Run (double txp);
    void CommandSetup (int argc, char **argv);
    void SetupObjectSocket (Ipv4Address addr, Ptr<Node> node);
    void SetupControllerSocket (Ipv4Address addr, Ptr<Node> node);
    void ObjectReceivesPacket (Ptr<Socket> socket);
    void ControllerReceivesPacket (Ptr<Socket> socket);

    double m_totalTime;
    double m_timeStep;
    double m_distStep;
    uint32_t m_nWifis;
    string m_protocolName;
    Protocol m_protocolId;
    uint32_t m_packetSize;
    double m_txp;
    bool m_traceMobility;
    uint32_t m_protocol;

    NodeContainer m_nodes;
    uint32_t m_objectNodeId;
    uint32_t m_controllerNodeId;
    Ptr<Socket> m_objectSocket;
    Ptr<Socket> m_controllerSocket;
};

class ObjectBall {
private:
  double m_x;
  double m_V;
  double m_a;
  double m_alpha;
  double m_aGravitation = 0, m_aFriction = 0, m_aAirResistance = 0;
  double m_xMax = g_desiredX;

public:
  double const k_g = 9.80665;

  ObjectBall(double x, double V, double a) {
    m_x = x;
    m_V = V;
    m_a = a;
  }

  ObjectBall() {
    m_x = -2;
    m_V = 0;
    m_a = 0;
  }

  void calculateParametersOfTheBall () {
    double const k_k = 0.00001, k_R = 0.05;   //Параметры для силы трения
    double const k_C = 0.5, k_densityAir = 1.226, k_densitySteel = 7800;    //Параметры для силы сопротивления воздуха
    int velocitySign;

    m_aGravitation = k_g * sin(m_alpha * k_pi / 180);
    m_aFriction = k_g * (k_k/k_R) * cos(m_alpha * k_pi / 180);
    m_aAirResistance = ((3 * k_C * k_densityAir * pow(m_V, 2))/(2 * k_densitySteel * k_R)) * cos(m_alpha * k_pi / 180);

    if (m_V > 0.0001) { //Расчет ускорения в зависимости от направления скорости
        velocitySign = -1;
    } else if (m_V < -0.0001) {
        velocitySign = 1;
    } else velocitySign = 0;

    m_a = m_aGravitation + velocitySign * m_aFriction + velocitySign * m_aAirResistance;
    m_V += m_a * g_dt;
    m_x += m_V * g_dt;
  }

  double getX() { return m_x;}
  double getV() { return m_V;}
  double getA() { return m_a;}
  double getAlpha() { return m_alpha;}

  void setAlpha(double alpha) {  m_alpha = alpha;}

  void calculateQualityIndicators () {
      if ((fabs(m_x - g_desiredX)) >= k_e) {
          timeOfTransitionProcess = Simulator::Now().GetSeconds();
      }

      if (m_x > m_xMax) {
          m_xMax = m_x;
      }
      if (fabs(m_x) > k_e) {
        overshoot = fabs(m_xMax - m_x) * 100 / fabs(m_x);
      } else {
        overshoot = m_xMax * 100;
      }

      staticError = g_desiredX - m_x;

      linearIntegral += (m_x - g_desiredX) * g_dt;
      integralofModuleOfError += fabs(m_x - g_desiredX) * g_dt;
      integralOfSquareOfError += pow((m_x - g_desiredX), 2) * g_dt;
      integralOfWeightedModuleOfError += Simulator::Now().GetSeconds() * fabs(m_x - g_desiredX) * g_dt;
      integralOfWeightedSquareOfError += Simulator::Now().GetSeconds() * pow((m_x - g_desiredX), 2) * g_dt;
  }

};

ObjectBall ball { -2, 0, 0 };

double hardCalculateANewAngle (double x, double V, double a, double alpha) {
  if (fabs(x - g_desiredX) > 0.5) {   //Если далеко
      if (fabs(V) > k_e) {  //Если движется
          if (x < g_desiredX && V < 0 && alpha < 0) {    //Если в другую сторону и планка наклонена не как нужно (Если в нужную сторону, то ничего не меняем)
              alpha = - alpha;
          } else if (x > g_desiredX && V > 0 && alpha > 0) {
              alpha = - alpha;
          }
      } else {    //Если остановился
          if (x < g_desiredX) { //Если левей нужной точки
              alpha = 2;  //То наклоняем по часовой стрелке
          } else {    //Если правей нужной точки
              alpha = -2; //То наклоняем против часовой стрелки
          }
      }
  } else {    //Если близко
      if (fabs(V) > k_e) {  //Если движется
          if ((x < g_desiredX && V > 0) || (x > g_desiredX && V < 0)) {   //Если в нужную сторону
              if (fabs(V) > k_e) {   //Если быстро
                  if (x < g_desiredX && V > 0 && alpha > 0) {    //Если не тот наклон
                      alpha = - alpha;
                  } else if (x > g_desiredX && V < 0 && alpha < 0) {
                      alpha = - alpha;
                  }
              } else {    //Если медленно
                  if (x < g_desiredX && V > 0 && alpha > 0) {
                      alpha = 0;
                  } else if (x > g_desiredX && V < 0 && alpha < 0) {
                      alpha = 0;
                  }
              }
          } else {    //Если в другую сторону
              if (fabs(V) > k_e) {   //Если быстро
                  if (x < g_desiredX && V < 0 && alpha < 0) {    //Если наклон не в ту сторону
                      alpha = - alpha;
                  } else if (x > g_desiredX && V > 0 && alpha > 0) {
                      alpha = - alpha;
                  }
              } else {    //Если медленно
                  if (x < g_desiredX && V < 0 && fabs(a) >= k_e && alpha < 0) {   //Если ускоряется или с постоянной скоростью
                      alpha = -alpha; //- 0.5 * alpha;    //Меняется на противоположный
                  } else if (x < g_desiredX && V < 0 && fabs(a) >= k_e && alpha > 0) {
                      alpha = alpha; //0.5 * alpha;
                  } else if (x > g_desiredX && V > 0 && fabs(a) >= k_e && alpha > 0) {  //Если замедляется
                      alpha = -alpha; //-0.5 * alpha;   //Меняем на маленький противоположный
                  } else if (x > g_desiredX && V > 0 && fabs(a) >= k_e && alpha < 0) {
                      alpha = alpha; //0.5 * alpha;    //Меняем на маленький в ту же сторону
                  } else if (x < g_desiredX && V < 0 && fabs(a) < k_e && alpha < 0) {
                      alpha = - alpha;
                  } else if (x > g_desiredX && V > 0 && fabs(a) < k_e && alpha > 0) {
                      alpha = - alpha;
                  }
              }
          }
      } else {    //Если остановился
          if (fabs(x - g_desiredX) < k_e) {
            alpha = 0;
          } else if (x < g_desiredX) { //Если левей нужной точки
              alpha = 0.1;  //То немного наклоняем по часовой стрелке
          } else if (x > g_desiredX) {    //Если правей нужной точки
              alpha = -0.1; //То немного наклоняем против часовой стрелки
          }
      }
  }

  double const k_viscosity = 0.001;  //Коэффициент для искусственного торможения при большой скорости
  if (fabs(V) > 0.3) {
      alpha += - k_viscosity * V;
  }
  return alpha;
}

double lowerCascade (double x) {
    double k = 5;
    double desiredV = (g_desiredX - x) * k;
    return desiredV;
}
double upperCascade (double desiredV, double V) {
    double k = 6;
    Alpha = (desiredV - V) * k;
    return Alpha;
}
double cascadeCalculateANewAngle (double x, double V) {
  return Alpha = upperCascade(lowerCascade(x), V);
}

RoutingExperiment::RoutingExperiment ()
    : m_totalTime (100),
      m_distStep (10.0),
      m_nWifis (2),
      m_packetSize (k_normalPacketSize),
      m_traceMobility (false),
      m_protocol (2) // AODV
{
}

static Ipv4Address
GetLocalAddress (Ptr<Node> node)
{
    return node->GetObject <Ipv4> ()->GetAddress (1, 0).GetLocal ();
}

static inline string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
    ostringstream oss;

    oss << setw(7) << Simulator::Now ().GetSeconds () << "s: " << GetLocalAddress (socket->GetNode ());

    if (InetSocketAddress::IsMatchingType (senderAddress))
    {
        InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
        oss << " <- " << addr.GetIpv4 () << " pkt [" << packet->GetUid () << "] ";
    }
    else
    {
        oss << " received one packet!";
    }
    return oss.str ();
}

void
RoutingExperiment::CommandSetup (int argc, char **argv)
{
    CommandLine cmd;
    cmd.AddValue ("totalTime", "The name of the CSV output file name", m_totalTime);
    cmd.AddValue ("timeStep", "The time step: at each step several packets are created according to lambda", m_timeStep);
    cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
    cmd.AddValue ("distStep", "Step for making distances", m_distStep);
    cmd.AddValue ("nWifis", "Number of wifi nodes", m_nWifis);
    cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
    cmd.AddValue ("packetSize", "Packet size", m_packetSize);
    cmd.Parse (argc, argv);
}


void
RoutingExperiment::SetupObjectSocket (Ipv4Address addr, Ptr<Node> node)     //Создается сокет объекта
{
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> socket = Socket::CreateSocket (node, tid);
    InetSocketAddress local = InetSocketAddress (addr, k_port);
    socket->Bind (local);   //привязка сокета к узлу
    socket->SetRecvCallback (MakeCallback (&RoutingExperiment::ObjectReceivesPacket, this));    //Когда придет пакет вызовется колбэк
    m_objectSocket = socket;
}

void
RoutingExperiment::SetupControllerSocket (Ipv4Address addr, Ptr<Node> node)     //Сокет для контроллера
{
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> socket = Socket::CreateSocket (node, tid);
    InetSocketAddress local = InetSocketAddress (addr, k_port);
    socket->Bind (local);
    socket->SetRecvCallback (MakeCallback (&RoutingExperiment::ControllerReceivesPacket, this));
    m_controllerSocket = socket;
}

static void
SendPacketToController(RoutingExperiment* e)        //Отправка пакета контроллеру
{
    Ptr<Socket> socket = e->m_objectSocket;
    Ptr<Socket> sink = e->m_controllerSocket;
    socket->Connect (InetSocketAddress (GetLocalAddress (sink->GetNode ()), k_port));
    std::ostringstream oss;     //Создается строка с значением переменных ОУ
    oss << Simulator::Now ().GetSeconds () << '\t'
        << ball.getX() << '\t'
        << ball.getV() << '\t'
        << ball.getA() << '\t'
        << ball.getAlpha()<< '\0';    //Какую строку передаем в пакете

    fprintf(fileObject, "%.2lf\t\t%.3lf\t\t%.3lf\t\t%.3lf\t\t%.2lf\n",
            Simulator::Now().GetSeconds(), ball.getX(), ball.getV(), ball.getA(), ball.getAlpha());

    Ptr<Packet> packet = Create<Packet> ((uint8_t*) oss.str().c_str(), oss.str().length() + 1);
    socket->Send (packet);
    NS_LOG_UNCOND ("Node Ball " << GetLocalAddress (socket->GetNode ())
                   << " sent to Controller: Packet " << packet->GetUid ()
                   << " at " << Simulator::Now ().GetSeconds () << " sec"
                   << " with string: " << oss.str().c_str());

}

static void
SendPacketToObject(RoutingExperiment* e)
{
    Ptr<Socket> socket = e->m_controllerSocket;
    Ptr<Socket> sink = e->m_objectSocket;
    socket->Connect (InetSocketAddress (GetLocalAddress (sink->GetNode ()), k_port));
    std::ostringstream oss;
    oss << (Simulator::Now ().GetSeconds () - delay) << '\t' << Alpha << '\0';   //Что отправляется объекту - угол (e, потому что принимаем его в скобках при вызове функции)
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) oss.str().c_str(), oss.str().length() + 1);
    socket->Send (packet);
    NS_LOG_UNCOND ("Node Controller " << GetLocalAddress (socket->GetNode ())
                   << " sent to Object: Packet " << packet->GetUid ()
                   << " at " << Simulator::Now ().GetSeconds () << " sec"
                   << " with string: " << oss.str().c_str());

}

static void
ObjectNextStep(RoutingExperiment* e)        //Подключается через скедьюл вызывается в 0 секунду
{
    ball.calculateParametersOfTheBall();        //Расчет параметров ОУ

    NS_LOG_UNCOND("Ball now in: " << Simulator::Now().GetSeconds() << '\t'
                  << ball.getX() << '\t'
                  << ball.getV() << '\t'
                  << ball.getA() << '\t'
                  << ball.getAlpha()<< '\0');

    Simulator::Schedule (Seconds (e->m_timeStep), &ObjectNextStep, e);
    SendPacketToController(e);

    ball.calculateQualityIndicators ();
}

void
RoutingExperiment::ControllerReceivesPacket (Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address senderAddress;
    while ((packet = socket->RecvFrom (senderAddress)))     //Определяется адрес отправителя
    {
        uint8_t *buf = new uint8_t[packet->GetSize ()];     //Расшифровка пакета
        packet->CopyData(buf, packet->GetSize ());
        string str = string(buf, buf+packet->GetSize());
        delete buf;
        NS_LOG_UNCOND ("Controller recieved string: " << PrintReceivedPacket (socket, packet, senderAddress) <<
                       "with string: " << str);     //Запись в лог

        //расшифровка принятой строки
        string strParameters [5]; //0-t, 1-X, 2-V, 3-a, 4-alpha
        strParameters[0]="";
        for (int i=0, j=0; str[i]!='\0'; i++) {
            if(str[i]!='\t') {
                strParameters[j] +=str[i];
            } else {
                j++;
                strParameters[j]="";
            }
        }

        delay = Simulator::Now().GetSeconds() - atof(strParameters[0].c_str());
        double X = atof(strParameters[1].c_str());
        double V = atof(strParameters[2].c_str());
        double A = atof(strParameters[3].c_str());
        Alpha = atof(strParameters[4].c_str());

        NS_LOG_UNCOND ("Controller recieved in " << Simulator::Now().GetSeconds() << " parameters: \t"
                       << X << '\t'
                       << V << '\t'
                       << A << '\t'
                       << Alpha << '\0');   //Запись принятых параметров в лог

        //Alpha = hardCalculateANewAngle(X, V, A, Alpha);
        Alpha = cascadeCalculateANewAngle(X, V);

        SendPacketToObject(this);       //Отправка пакета объекту

    }
}

void
RoutingExperiment::ObjectReceivesPacket (Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address senderAddress;
    while ((packet = socket->RecvFrom (senderAddress)))
    {
        uint8_t *buf = new uint8_t[packet->GetSize ()];
        packet->CopyData(buf, packet->GetSize ());
        string str = string(buf, buf+packet->GetSize());
        delete buf;
        NS_LOG_UNCOND ("Object recieved: " << PrintReceivedPacket (socket, packet, senderAddress)
                       << "with string: " << str);

        //Расшифровка принятой строки
        string strParameters [2]; //0-time + delay, 1-alpha
        strParameters[0]="";
        for (int i=0, j=0; str[i]!='\0'; i++) {
            if(str[i]!='\t') {
                strParameters[j] +=str[i];
            } else {
                j++;
                strParameters[j]="";
            }
        }

        delay = Simulator::Now().GetSeconds() - atof(strParameters[0].c_str());

        fprintf(delays, "%lf\t%f\n", Simulator::Now().GetSeconds(), delay);

        double alphaReceived =  atof(strParameters[1].c_str());      //Принятые данные от контроллера читаем и устанавливаем в текущую альфу
        NS_LOG_UNCOND ("Object: " << "received Alpha:" << alphaReceived);
        ball.setAlpha(alphaReceived);
    }
}


int
main (int argc, char *argv[])
{
    fileObject = fopen("/home/anrdey/practice/CC2.dat", "w");  //Подготавливаем файлы для записи HC1, HC2, CC1, CC2
    delays = fopen("/home/anrdey/practice/DCC2.dat", "w");  //DHC1, DHC2, DCC1, DCC2
    qualityIndicators = fopen("/home/anrdey/practice/QICC2.dat", "w");  //QIHC1 , QIHC2 , QICC1 , QICC2

    ball.setAlpha(3);

    fprintf(fileObject, "#Time,\t\tX,\t\tV,\t\ta,\t\talpha\n%.2lf\t\t%.3lf\t\t%.3lf\t\t%.3lf\t\t%.2lf\n", g_t, ball.getX(), ball.getV(), ball.getA(), ball.getAlpha());

    RoutingExperiment experiment;
    experiment.CommandSetup (argc, argv);
    double txp = 7.5;
    experiment.Run (txp);

    fprintf (qualityIndicators, "TimeOfTransitionProcess\t%.2lf\nOvershoot\t%.2lf\nStaticError\t%.2lf\nLinearIntegral\t%.2lf\nIntegralofModuleOfError\t%.2lf\nIntegralOfSquareOfError\t%.2lf\nIntegralOfWeightedModuleOfError\t%.2lf\nIntegralOfWeightedSquareOfError\t%.2lf\n",
             timeOfTransitionProcess, overshoot, staticError,
             linearIntegral, integralofModuleOfError, integralOfSquareOfError, integralOfWeightedModuleOfError, integralOfWeightedSquareOfError);

    return 0;
}

void
RoutingExperiment::Run (double txp)
{
    NS_LOG_UNCOND ("control-network-example is starting");  //Просто вывод сообщения

    Packet::EnablePrinting ();
    m_txp = txp;
    string phyMode ("DsssRate11Mbps");
    m_protocolName = "protocol";

    //Set Non-unicastMode rate to unicast mode
    Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

    m_nodes.Create (m_nWifis);  //Создание контейнера узлов
    MobilityHelper mobility;        //Для NetAnim


    //Без передвижения
    Ptr<ListPositionAllocator> listPositionAllocator = CreateObject<ListPositionAllocator> ();
    for (uint32_t i = 0; i < m_nodes.GetN (); i++)
    {
        listPositionAllocator->Add (Vector (i * m_distStep, 0, 1.2));
    }

    mobility.SetPositionAllocator (listPositionAllocator);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel"); //Постоянная позиция

    //С передвижением
//    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//                                   "MinX", DoubleValue (10.0),
//                                   "MinY", DoubleValue (10.0),
//                                   "DeltaX", DoubleValue (5.0),
//                                   "DeltaY", DoubleValue (2.0),
//                                   "GridWidth", UintegerValue (5),
//                                   "LayoutType", StringValue ("RowFirst"));
//    mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",    //Случайное перемещение
//                               "Bounds", RectangleValue (Rectangle (-50, 50, -25, 50)));   //Способы расположения и движения узлов

    //********конец*****
    mobility.Install (m_nodes);


    // setting up wifi phy and channel using helpers
    WifiHelper wifi;
    wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel (wifiChannel.Create ());

    // Add a mac and disable rate control
    WifiMacHelper wifiMac;
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode",StringValue (phyMode),
                                  "ControlMode",StringValue (phyMode));

    wifiPhy.Set ("TxPowerStart",DoubleValue (txp));     //Мощность вайфая
    wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));

    wifiMac.SetType ("ns3::AdhocWifiMac");
    NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, m_nodes);     //Создание контейнера ad hoc устройств

    AodvHelper aodv;
    aodv.Set ("ActiveRouteTimeout", TimeValue (Seconds (3))); //1 (Seconds (3))
    aodv.Set ("AllowedHelloLoss", UintegerValue (2));   //2 (2)
    aodv.Set ("NetDiameter", UintegerValue (35));  //10 (35)
    aodv.Set ("NodeTraversalTime", TimeValue (MilliSeconds (40)));  //13 (MilliSeconds (40))
    aodv.Set ("RreqRetries", UintegerValue (2));   //17 (2)
    aodv.Set ("RreqRateLimit", UintegerValue (10)); //18 (10)
    aodv.Set ("TtlStart", UintegerValue (1));   //20 (1)
    aodv.Set ("TtlIncrement", UintegerValue (2));   //21 (2)
    aodv.Set ("TtlThreshold", UintegerValue (20));   //22 (7)

    OlsrHelper olsr;
    DsdvHelper dsdv;
    DsrHelper dsr;
    DsrMainHelper dsrMain;
    Ipv4ListRoutingHelper list;
    InternetStackHelper internet;

    switch (m_protocol)
    {
    case 1:
        list.Add (olsr, 100);
        m_protocolName = "OLSR";
        m_protocolId = Protocol::OLSR;
        break;
    case 2:
        list.Add (aodv, 100);
        m_protocolName = "AODV";
        m_protocolId = Protocol::AODV;
        break;
    case 3:
        list.Add (dsdv, 100);
        m_protocolName = "DSDV";
        m_protocolId = Protocol::DSDV;
        break;
    case 4:
        m_protocolName = "DSR";
        m_protocolId = Protocol::DSR;
        break;
    default:
        NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

    if (m_protocol < 4)
    {
        internet.SetRoutingHelper (list);
        internet.Install (m_nodes);
    }
    else if (m_protocol == 4)
    {
        internet.Install (m_nodes);
        dsrMain.Install (dsr, m_nodes);
    }
    NS_LOG_INFO ("assigning ip address");

    Ipv4AddressHelper addressAdhoc;     //Задание IP адресации
    addressAdhoc.SetBase ("10.0.0.0", "255.255.255.0", "0.0.0.100");
    Ipv4InterfaceContainer adhocInterfaces;
    adhocInterfaces = addressAdhoc.Assign (adhocDevices);

    m_objectNodeId = 0;     //Номер первого узла
    m_controllerNodeId = m_nodes.GetN() - 1;    //Номер последнего узла
    SetupObjectSocket (adhocInterfaces.GetAddress (m_objectNodeId),     //Задание сокета для узла объекта и ниже контроллера
                       m_nodes.Get (m_objectNodeId));
    SetupControllerSocket (adhocInterfaces.GetAddress (m_controllerNodeId),
                       m_nodes.Get (m_controllerNodeId));
    Simulator::Schedule (Seconds (0.0), &ObjectNextStep, this);     //this указание на объект эксперимента, вызываем

    //Вывод в xml файл для NetAnim
    anim = new AnimationInterface ("GQW_project.xml");
    anim->EnablePacketMetadata (true); // Optional
    anim->EnableIpv4RouteTracking ("GQW_project_routing.xml", Seconds (0), Seconds (5), Seconds (0.25));
    anim->EnableWifiMacCounters (Seconds (0), Seconds (10)); //Optional
    anim->EnableWifiPhyCounters (Seconds (0), Seconds (10)); //Optional

    anim->SetMobilityPollInterval (Seconds (1)); //Интервал записи положения узлов
    for (uint32_t i = 0; i < m_nodes.GetN (); ++i) {
            if(i == 0) {
            anim->UpdateNodeDescription (m_nodes.Get(i), "Ball");
            } else if (i == m_nWifis-1) {
                anim->UpdateNodeDescription (m_nodes.Get(i), "Controller");
            } else {
                anim->UpdateNodeDescription (m_nodes.Get(i), "node");
            }
        }

    for (uint32_t i = 0; i < m_nodes.GetN (); i++)      //Создание файлов с таблицами маршрутизации
    {
        ostringstream oss;
        oss << "node" << setfill ('0') << setw (2) << i << ".routes";
        string str = oss.str ();
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> (str.c_str (), ios::out);
        aodv.PrintRoutingTableEvery (Seconds (1), m_nodes.Get (i), routingStream);
    }

    NS_LOG_UNCOND ("Run Simulation");

    Simulator::Stop (Seconds (m_totalTime));
    Simulator::Run ();

    Simulator::Destroy ();

    delete anim;
}

