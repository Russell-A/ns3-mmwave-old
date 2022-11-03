#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-propagation-loss-model.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/system-wall-clock-ms.h"
#include "ns3/traci-applications-module.h"
#include "ns3/traci-module.h"
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/core-module.h"
#include <fstream>
#include "ns3/applications-module.h"
#include "ns3/mmwave-sidelink-spectrum-phy.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/spectrum-helper.h"
#include "ns3/mmwave-spectrum-value-helper.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"

using namespace ns3;
using namespace millicar;

Ptr<TraciClient> sumoClient = CreateObject<TraciClient>();
Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>();
uint32_t packet_number = 0; // the identity of the packet
std::map<std::pair<int, int>, double> packet_velocity_map; 
// the map from the packet identity to the velocity of the client. 
//The first int stands for caridentity the second int stands for packet_number
NodeContainer n; // create a pool to contain the vehicles. 
std::map<int, int> clientAppIndex_to_vehicleIdentity;
uint32_t nodeCounter(0);

template<typename K, typename V>  // util function to monitor the map
void print_map(std::map<K, V> const &m)
{
    for (auto const &pair: m) {
        std::cout << "{" << pair.first << ": " << pair.second << "}\n";
    }
}


static void SetPacketNumberInUdp (uint32_t  clientApps_num, UdpEchoClientHelper * client, ApplicationContainer * clientApps, Ptr<const Packet> p)
{
  // set packet number
  int vechileidentity = clientAppIndex_to_vehicleIdentity[clientApps_num];
  std::cout << "call SetPacketNumberInUdp: " << clientApps_num<<";"<<packet_number<< std::endl;
  client->SetFill(clientApps->Get(clientApps_num), std::to_string(vechileidentity)+";"+std::to_string(packet_number));
  std::cout << "leave SetPacketNumberInUdp"<< std::endl;
}

static void SetVelocityMap (int vehicle_identity, uint32_t * packet_number, Ptr<const Packet> p)
{
  std::cout << "call SetVelocityMap: "<< vehicle_identity << std::endl;
  #include<string.h>
  // get velocity of the client
  double velocity_of_client;
  try{
   velocity_of_client = sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(n.Get(vehicle_identity)));
  }catch(...){
    std::cout << "Get the speed of vehicle " << vehicle_identity << " goes wrong"<< std::endl;
   velocity_of_client = 0;
  }
  // save the velocity in map
  packet_velocity_map[{vehicle_identity, std::stoi(std::to_string(*packet_number))}] = velocity_of_client;
  std::cout << "leave SetVelocityMap"<< std::endl;
}

static void Packetnumadd1 (uint32_t * packet_number, Ptr<const Packet> p)
{
  std::cout << "call Packetnumadd1"<< std::endl;
  *packet_number = *packet_number+1;
  std::cout << "leave Packetnumadd1"<< std::endl;

}


static void Tx (Ptr<OutputStreamWrapper> stream, uint32_t vehicle_identity, Ptr<const Packet> p)
{
  std::cout << "call Tx"<< std::endl;
  double velocity_of_client;
  try{
   velocity_of_client = sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(n.Get(vehicle_identity)));
  }catch(...){
    std::cout << "Get the speed of vehicle " << vehicle_identity << " goes wrong"<< std::endl;
   velocity_of_client = 0;
  }
  // save the sent content and the velocity to the steam (examples/sumo-millicar/circle-simple/communation-stream.txt)
  *stream->GetStream () << "Tx\t" << Simulator::Now ().GetSeconds () << "\t" << p->GetSize ()  <<"\t" ;
  p->CopyData(&(*stream->GetStream()), p->GetSize());
  *stream->GetStream () << "\t" <<  velocity_of_client << std::endl;
  std::cout << "leave Tx"<< std::endl;
}

static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  std::cout << "call Rx"<< std::endl;
 *stream->GetStream () << "Rx\t" << Simulator::Now ().GetSeconds () << "\t" << p->GetSize() << "\t" ;
 p->CopyData(&(*stream->GetStream()), p->GetSize());
 


//先临时输出packet_number, 再读入
std::ofstream ofs("examples/sumo-millicar/circle-simple/Rx_temp.txt");
p->CopyData(&ofs, p->GetSize());
ofs.close();

//读入
std::ifstream ifs("examples/sumo-millicar/circle-simple/Rx_temp.txt");
std::string vehicle_identity_and_packet_number_readin((std::istreambuf_iterator<char>(ifs)), // "vehicle_identity;packet_number"
                 std::istreambuf_iterator<char>());
std::string delimiter = ";";
std::string vehicle_identity_readin = vehicle_identity_and_packet_number_readin.substr(0, vehicle_identity_and_packet_number_readin.find(delimiter));
std::string packet_number_readin = vehicle_identity_and_packet_number_readin.erase(0, vehicle_identity_and_packet_number_readin.find(delimiter) + delimiter.length());



// 通过map找到对应的velocity
std:: cout << "key received:" << std::stoi(vehicle_identity_readin) << std::stoi(packet_number_readin) << std::endl;
print_map(packet_velocity_map);
 double velocity_received;
 velocity_received = packet_velocity_map[std::make_pair(std::stoi(vehicle_identity_readin), std::stoi(packet_number_readin))];
 std::cout << "velocity_received:\t" <<velocity_received << std::endl;

 *stream->GetStream () << '\t' << velocity_received <<std::endl;

 // 设置velocity的代码
 // sumoClient->TraCIAPI::vehicle.setSpeed (sumoClient->GetVehicleId (nodePool.Get (1)), velocity);

 std::cout << "leave Rx"<< std::endl;
}

static void test ( uint32_t nome, Ptr<const Packet> p){
  double v0, v1, v2, v3;
    if (nodeCounter < 4) return;
    std::cout << "test" << std::endl;
    v0 = sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(n.Get(0)));
    std::cout << "test 0 OK: v0 = " << v0<< std::endl;
    v1 = sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(n.Get(1)));
    std::cout << "test 1 OK: v1 = " << v1<< std::endl;
    v2 = sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(n.Get(2)));
    std::cout << "test 2 OK: v2 = " << v2<< std::endl;
    v3 = sumoClient->TraCIAPI::vehicle.getSpeed(sumoClient->GetVehicleId(n.Get(3)));
    std::cout << "test 3 OK: v3 = " << v3<< std::endl;
}


int main (int argc, char *argv[]){

  double frequency = 60e9;  // frequency in Hz
  double bandwidth = 2.5e8; // bandwidth in Hz
  // uint32_t simulationRun;   // number of simulation

  std::string channel_condition = "l";
  std::string scenario;

  ns3::Time simulationTime(ns3::Seconds(60));

  CommandLine cmd;
  cmd.Parse (argc, argv);

  Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq",
                     DoubleValue(frequency));
  Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::ChannelCondition", StringValue (channel_condition));
  Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::Scenario",
                     StringValue("V2V-Urban")); //change 10/23
  Config::SetDefault("ns3::MmWaveVehicularHelper::Bandwidth",
                     DoubleValue(bandwidth));
  Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::Shadowing",
                     BooleanValue(false));
  Config::SetDefault(
      "ns3::MmWaveVehicularSpectrumPropagationLossModel::UpdatePeriod",
      TimeValue(MilliSeconds(1)));

  Config::SetDefault ("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue (true));
  //Config::SetDefault ("ns3::MmWaveSidelinkMac::Mcs", UintegerValue (28));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (60.0e9));
  Config::SetDefault ("ns3::MmWaveVehicularNetDevice::RlcType", StringValue("LteRlcUm"));

  // SUMO Configuration
  sumoClient->SetAttribute ("SumoConfigPath", StringValue ("examples/sumo-millicar/circle-simple/circle.sumo.cfg"));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.01)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (true)); //10/17 change
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));  // portion of vehicles equipped with wifi
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (true));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue ("--verbose true"));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (5.0)));

  n.Create(8);



  // create the mobility nodes
 MobilityHelper mobility;
  Ptr<UniformDiscPositionAllocator> positionAlloc = CreateObject<UniformDiscPositionAllocator> ();
  // positionAlloc->SetX (320.0);
  // positionAlloc->SetY (320.0);
  // positionAlloc->SetRho (25.0);
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (n);

  n.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  n.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (10,0,0));
  n.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (20,0,0));
  n.Get (3)->GetObject<MobilityModel> ()->SetPosition (Vector (30,0,0));

  // Create and configure the helper
  Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>();
  helper->SetPropagationLossModelType(
      "ns3::MmWaveVehicularPropagationLossModel");
  helper->SetSpectrumPropagationLossModelType(
      "ns3::MmWaveVehicularSpectrumPropagationLossModel");
  helper->SetNumerology(3);   // 好像最大是3 车辆最大是2**3
  NetDeviceContainer devs = helper->InstallMmWaveVehicularNetDevices(n);

  InternetStackHelper internet;
  internet.Install(n);
//
// Explicitly create the channels required by the topology (shown above).
//
  // CsmaHelper csma;
  // csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate (5000000)));
  // csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (2)));
  // csma.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  // NetDeviceContainer d = csma.Install (n);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.0.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign(devs);

  helper->PairDevices(devs);

  //10/23 尝试修改成updecho

  uint32_t maxPacketCount = 800000;
  uint32_t packetSize = 512;
  Time interPacketInterval =  Seconds(1);

  //加入server并绑定车
  //车1
  UdpEchoServerHelper server (4001);
  ApplicationContainer apps = server.Install (n.Get(1));
  //车2
  UdpEchoServerHelper server2 (4002);
  apps.Add(server2.Install (n.Get(2)));
  // set application start time
  apps.Start (Seconds (1.0));

  // 加入client并绑定车
  //车0 -> 车1
  UdpEchoClientHelper client_0_1 (n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4001);
  client_0_1.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client_0_1.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client_0_1.SetAttribute ("PacketSize", UintegerValue (packetSize));
  ApplicationContainer clientApps = client_0_1.Install (n.Get (0));
  clientAppIndex_to_vehicleIdentity.insert(std::pair<int, int>(0,0));

  // 车0-> 车2
  UdpEchoClientHelper client_0_2 (n.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4002);
  client_0_2.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client_0_2.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client_0_2.SetAttribute ("PacketSize", UintegerValue (packetSize));
  clientApps.Add(client_0_2.Install(n.Get(0)));
  clientAppIndex_to_vehicleIdentity.insert(std::pair<int, int>(1,0));


  
  //车3 -> 车1
  UdpEchoClientHelper client_3_1 (n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4001);
  client_3_1.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client_3_1.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client_3_1.SetAttribute ("PacketSize", UintegerValue (packetSize));
  clientApps.Add ( client_3_1.Install (n.Get (3)));
  clientAppIndex_to_vehicleIdentity.insert(std::pair<int, int>(2,3));


  //车3 -> 车2
  UdpEchoClientHelper client_3_2 (n.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4001);
  client_3_2.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client_3_2.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client_3_2.SetAttribute ("PacketSize", UintegerValue (packetSize));
  clientApps.Add ( client_3_2.Install (n.Get (3)));
  clientAppIndex_to_vehicleIdentity.insert(std::pair<int, int>(3,3));

  //set client start time and stop time
  clientApps.Start (Seconds(1.0));
  clientApps.Stop (Seconds(100));
  
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();


  //set initial packet content

  client_0_1.SetFill(clientApps.Get(0), std::to_string(0) + ";" + std::to_string(packet_number));
  client_0_2.SetFill(clientApps.Get(1), std::to_string(0) + ";" + std::to_string(packet_number));
  client_3_1.SetFill(clientApps.Get(2), std::to_string(3) + ";" + std::to_string(packet_number));
  client_3_2.SetFill(clientApps.Get(3), std::to_string(3) + ";" + std::to_string(packet_number));

  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("examples/sumo-millicar/circle-simple/communation-stream.txt");
    *stream->GetStream () << "Func\t" << "Now" << "\t" << "Psize"  <<"\t" << "Pcontent" <<"\t" << "velocity" << std::endl;
  

  // clientApps.Get(0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&test, 0));

  clientApps.Get(0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&SetVelocityMap, 0,&packet_number));
  clientApps.Get(0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Packetnumadd1, &packet_number));
  clientApps.Get(0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&SetPacketNumberInUdp, 0, &client_0_1, &clientApps));
  clientApps.Get(1)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&SetPacketNumberInUdp, 1, &client_0_2, &clientApps));

  clientApps.Get(0)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx, stream, 0));
  clientApps.Get(1)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx,  stream, 0));

  apps.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream));
  apps.Get(1)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream));


  clientApps.Get(2)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&SetVelocityMap, 3, &packet_number));
  clientApps.Get(2)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&SetPacketNumberInUdp, 2, &client_3_1, &clientApps));
  clientApps.Get(3)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&SetPacketNumberInUdp, 3, &client_3_2, &clientApps));

  clientApps.Get(2)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx,  stream, 3));
  clientApps.Get(3)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx,  stream, 3));

  apps.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream));
  apps.Get(1)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream));

  // enable print into stream
  ns3::PacketMetadata::Enable ();


  VehicleSpeedControlHelper vehicleSpeedControlHelper(9);
  vehicleSpeedControlHelper.SetAttribute("Client", (PointerValue)sumoClient);

  // a callback function to setup the nodes
  std::function<Ptr<Node>()> setupNewWifiNode = [&]() -> Ptr<Node> {
    if (nodeCounter >= n.GetN())
      NS_FATAL_ERROR("Node Container empty!: " << nodeCounter
                                               << " nodes created.");

    // don't create and install the protocol stack of the node at simulation
    // time -> take from "node pool"
    Ptr<Node> includedNode = n.Get(nodeCounter);
    ++nodeCounter; // increment counter for next node
    std::cout << "nodeCounter: " << nodeCounter << std::endl;

    // Install Application
    ApplicationContainer vehicleSpeedControlApps =
        vehicleSpeedControlHelper.Install(includedNode);
    vehicleSpeedControlApps.Start(Seconds(0.0));
    vehicleSpeedControlApps.Stop(simulationTime);

    return includedNode;
  };

  // a callback function for node shutdown
  std::function<void(Ptr<Node>)> shutdownWifiNode = [](Ptr<Node> exNode) {
    // stop all applications
    Ptr<VehicleSpeedControl> vehicleSpeedControl =
        exNode->GetApplication(0)->GetObject<VehicleSpeedControl>();
    if (vehicleSpeedControl)
      vehicleSpeedControl->StopApplicationNow();
    

    // set position outside communication range in SUMO
    Ptr<ConstantPositionMobilityModel> mob =
        exNode->GetObject<ConstantPositionMobilityModel>();
    mob->SetPosition(
        Vector(-100.0 + (rand() % 25), 320.0 + (rand() % 25), 250.0));
    std::cout << "Node Shutdown" << std::endl;
  };

  // start traci client
  sumoClient->SumoSetup(setupNewWifiNode, shutdownWifiNode);

  AnimationInterface anim ("examples/sumo-millicar/circle-simple/ns3-sumo-coupling.xml"); // Mandatory


  Simulator::Stop(simulationTime);
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}


