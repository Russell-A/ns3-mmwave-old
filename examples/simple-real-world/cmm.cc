#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
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
#include "ns3/internet-module.h"
#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/mmwave-sidelink-spectrum-phy.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/spectrum-helper.h"
#include "ns3/mmwave-spectrum-value-helper.h"



/* This example creates a infrasture at (0,0,0) and send packet to each car every 1 ms. 
If the car receice a safety warning, it will stop*/

// namespace ns3, millicar
using namespace ns3;
using namespace millicar;

// set up TraciClient to communicate with sumo
Ptr<TraciClient> sumoClient = CreateObject<TraciClient>();

NodeContainer n; // create a pool to contain the vehicles. 

static void ChangeFillToStop(std::vector<UdpEchoClientHelper> * client, ApplicationContainer * clientApps, uint32_t index, Ptr<const Packet> p)
{
  if (index + 1 == 1 && Simulator::Now().GetSeconds() >= 4)
    {(*client)[index].SetFill(clientApps->Get(index), "Stop");}
  if (index + 1 == 2 && Simulator::Now().GetSeconds() >= 5)
    {(*client)[index].SetFill(clientApps->Get(index), "Stop");}
  if (index + 1 == 3 && Simulator::Now().GetSeconds() >= 6)
    {(*client)[index].SetFill(clientApps->Get(index), "Stop");}
  if (index + 1 == 4 && Simulator::Now().GetSeconds() >= 7)
    {(*client)[index].SetFill(clientApps->Get(index), "Stop");}
}

static void ChangeFillToNormal(std::vector<UdpEchoClientHelper> * client, ApplicationContainer * clientApps, uint32_t index, Ptr<const Packet> p)
{
  if (index + 1 == 1 && Simulator::Now().GetSeconds() >= 20)
    {(*client)[index].SetFill(clientApps->Get(index), "Normal");}
  if (index + 1 == 2 && Simulator::Now().GetSeconds() >= 20)
    {(*client)[index].SetFill(clientApps->Get(index), "Normal");}
  if (index + 1 == 3 && Simulator::Now().GetSeconds() >= 20)
    {(*client)[index].SetFill(clientApps->Get(index), "Normal");}
  if (index + 1 == 4 && Simulator::Now().GetSeconds() >= 20)
    {(*client)[index].SetFill(clientApps->Get(index), "Normal");}
}

static void Tx ( uint32_t node_counter, Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{   
    
    *stream->GetStream () << "Tx\t" << Simulator::Now ().GetSeconds () << "\t" << node_counter << "\t" << p->GetSize ()<< '\t';
    p->CopyData(&(*stream->GetStream()), p->GetSize());
    *stream->GetStream ()<< std::endl;
}

static void Rx (uint32_t node_counter, Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
    *stream->GetStream () << "Rx\t" << Simulator::Now ().GetSeconds () << "\t" << node_counter << "\t" << p->GetSize() << "\t";
    p->CopyData(&(*stream->GetStream()), p->GetSize());
    *stream->GetStream ()<< std::endl;

    //先临时输出packet_number, 再读入
    std::ofstream ofs("examples/simple-junction/Rx_temp.txt");
    p->CopyData(&ofs, p->GetSize());
    ofs.close();

  //读入    
    std::ifstream ifs("examples/simple-junction/Rx_temp.txt");
    std::string packet_readin((std::istreambuf_iterator<char>(ifs)),
                 std::istreambuf_iterator<char>());
    if (packet_readin.substr(0, 4) == "Stop"){
      sumoClient->TraCIAPI::vehicle.setSpeed (sumoClient->GetVehicleId (n.Get (node_counter)), 0);      
    }
    else if (packet_readin.substr(0, 6) == "Normal"){
      sumoClient->TraCIAPI::vehicle.setSpeed (sumoClient->GetVehicleId (n.Get (node_counter)), 10);      
    }
}



int main (int argc, char *argv[]){

    double frequency = 60e9;  // frequency in Hz
    double bandwidth = 2.5e8; // bandwidth in Hz
    uint32_t simulationtime = 30;
    // uint32_t simulationRun;   // number of simulation

    std::string channel_condition = "l";
    std::string scenario;

    ns3::Time simulationTime(ns3::Seconds(simulationtime));

    CommandLine cmd;
    cmd.Parse (argc, argv);

    // set mmwave config
    Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(frequency));
    Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::ChannelCondition", StringValue (channel_condition));
    Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::Scenario", StringValue("V2V-Urban")); //change 10/23
    Config::SetDefault("ns3::MmWaveVehicularHelper::Bandwidth", DoubleValue(bandwidth));
    Config::SetDefault("ns3::MmWaveVehicularPropagationLossModel::Shadowing", BooleanValue(false));
    Config::SetDefault("ns3::MmWaveVehicularSpectrumPropagationLossModel::UpdatePeriod", TimeValue(MilliSeconds(10)));
    Config::SetDefault ("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue (true));
    //Config::SetDefault ("ns3::MmWaveSidelinkMac::Mcs", UintegerValue (28));
    Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (60.0e9));
    Config::SetDefault ("ns3::MmWaveVehicularNetDevice::RlcType", StringValue("LteRlcUm"));

    // SUMO Configuration
    sumoClient->SetAttribute ("SumoConfigPath", StringValue("examples/simple-real-world/simple-real-world-data/00.net.xml"));
    sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
    sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.04)));
    sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
    sumoClient->SetAttribute ("SumoGUI", BooleanValue (true)); //10/17 change
    sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
    sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));  // portion of vehicles equipped with wifi
    sumoClient->SetAttribute ("SumoLogFile", BooleanValue (true));
    sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
    sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
    sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue ("--verbose true"));
    sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (3.0)));

    sumoClient->connect("localhost", 3400);
    sumoClient->setOrder(2);
    

    // create nodepool, large enough to hold all the simulated car
    n.Create(8);

    // create the mobility nodes
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (n);

    // initinalize all the postion, the modules cannot be initialize at the same point otherwise the propagation model goes wrong
    // the infracture is set at (0, 0, 0)
    for (uint32_t i = 0; i < n.GetN(); i++)  n.Get(i)->GetObject<MobilityModel>()->SetPosition (Vector (i,0,0));

    // Create and configure the helper
    Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>(); 
    helper->SetPropagationLossModelType("ns3::MmWaveVehicularPropagationLossModel");
    helper->SetSpectrumPropagationLossModelType("ns3::MmWaveVehicularSpectrumPropagationLossModel");
    helper->SetNumerology(3);   // 好像最大是3 车辆最大是2**3
    NetDeviceContainer devs = helper->InstallMmWaveVehicularNetDevices(n);

    InternetStackHelper internet;
    internet.Install(n);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devs);
    helper->PairDevices(devs);

    uint32_t maxPacketCount = 800000;
    uint32_t packetSize = 512;
    Time interPacketInterval =  MilliSeconds(10);


    // create server for each car
    UdpEchoServerHelper server1 (4001);
    ApplicationContainer apps = server1.Install (n.Get(1));

    UdpEchoServerHelper server2 (4002);
    apps.Add( server2.Install (n.Get(2)) );

    UdpEchoServerHelper server3 (4003);
    apps.Add( server3.Install (n.Get(3)) );

    UdpEchoServerHelper server4 (4004);
    apps.Add( server4.Install (n.Get(4)) );
    
    // create client for the infrasture
    UdpEchoClientHelper client_0_1 (n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4001);
    client_0_1.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
    client_0_1.SetAttribute ("Interval", TimeValue (interPacketInterval));
    client_0_1.SetAttribute ("PacketSize", UintegerValue (packetSize));
    ApplicationContainer clientApps = client_0_1.Install (n.Get (1));

    UdpEchoClientHelper client_0_2 (n.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4002);
    client_0_2.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
    client_0_2.SetAttribute ("Interval", TimeValue (interPacketInterval));
    client_0_2.SetAttribute ("PacketSize", UintegerValue (packetSize));
    clientApps.Add(client_0_2.Install (n.Get (2)));

    UdpEchoClientHelper client_0_3 (n.Get (3)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4003);
    client_0_3.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
    client_0_3.SetAttribute ("Interval", TimeValue (interPacketInterval));
    client_0_3.SetAttribute ("PacketSize", UintegerValue (packetSize));
    clientApps.Add(client_0_3.Install (n.Get (3)));

    UdpEchoClientHelper client_0_4 (n.Get (4)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4004);
    client_0_3.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
    client_0_3.SetAttribute ("Interval", TimeValue (interPacketInterval));
    client_0_3.SetAttribute ("PacketSize", UintegerValue (packetSize));
    clientApps.Add(client_0_4.Install (n.Get (4)));


    clientApps.Start (Seconds(3.0));
    clientApps.Stop (Seconds(simulationtime));
    //in this example, clinet index is car index - 1.

    //set the initial fill
    client_0_1.SetFill(clientApps.Get(0), "Normal");
    client_0_2.SetFill(clientApps.Get(1), "Normal");
    client_0_3.SetFill(clientApps.Get(2), "Normal");
    client_0_4.SetFill(clientApps.Get(3), "Normal");


    // help to output log of v2i communication 
    AsciiTraceHelper asciiTraceHelper;
    Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("examples/simple-junction/communation-stream.txt");

    std::vector<UdpEchoClientHelper> client_vector = {client_0_1, client_0_2, client_0_3, client_0_4};
    for (uint32_t i = 0; i < 4; i++){
      clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx , i+1,stream));
      clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&ChangeFillToStop, & client_vector, & clientApps, i ));
      clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&ChangeFillToNormal, & client_vector, & clientApps, i ));

      apps.Get(i)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, i+1, stream));
    }

    

  Simulator::Stop(simulationTime);
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}


