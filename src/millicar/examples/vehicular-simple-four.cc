/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2020 University of Padova, Dep. of Information Engineering,
*   SIGNET lab.
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

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

NS_LOG_COMPONENT_DEFINE ("VehicularSimpleFour");


using namespace ns3;
using namespace millicar;

/**
  This script creates two pairs of vehicles moving at 20 m/s. The two groups are virtually positioned in two different lanes.
  In the same group, the vehicles are positioned one in front of the other at 20 m distance, and they exchange packets through a UDP application.
  Communication is done via a wireless channel.
  The aim of this example is to check whether the interference was evaluated correctly when different groups
  of nodes are transmitting in the same slot, sharing the same cell.
*/

static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
 SeqTsHeader header;

 p->PeekHeader(header);

 *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << p->GetSize() << "\t" << header.GetSeq() << "\t" << header.GetTs().GetSeconds() << std::endl;
}

int main (int argc, char *argv[])
{

  Time endTime = Seconds (10.0);
  double bandwidth = 1e8;

  double speed = 20; // m/s
  CommandLine cmd;
  cmd.AddValue ("vehicleSpeed", "The speed of the vehicle", speed);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue (true));
  //Config::SetDefault ("ns3::MmWaveSidelinkMac::Mcs", UintegerValue (28));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (60.0e9));
  Config::SetDefault ("ns3::MmWaveVehicularNetDevice::RlcType", StringValue("LteRlcUm"));
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (50*1024));
  Config::SetDefault ("ns3::MmWaveVehicularPropagationLossModel::ChannelCondition", StringValue ("l"));
  Config::SetDefault ("ns3::MmWaveVehicularHelper::Bandwidth", DoubleValue (bandwidth));
  Config::SetDefault ("ns3::MmWaveVehicularHelper::SchedulingPatternOption", EnumValue(2)); // use 2 for SchedulingPatternOption=OPTIMIZED, 1 or SchedulingPatternOption=DEFAULT

   // SUMO Configuration
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient>();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue ("examples/sumo-millicar/circle-simple/circle.sumo.cfg"));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.1)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (true)); //10/17 change
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));  // portion of vehicles equipped with wifi
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (true));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue ("--verbose true"));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  // create the nodes
  NodeContainer group;
  group.Create (3);

  // create the mobility models
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (group);

  group.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0,0,0));
  group.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, -speed, 0));

  group.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (20,0,0));
  group.Get (1)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, speed, 0));

  group.Get (2)->GetObject<MobilityModel> ()->SetPosition (Vector (10,20,0));
  group.Get (2)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, speed, 0));

  // create and configure the helper
  Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper> ();
  helper->SetPropagationLossModelType ("ns3::MmWaveVehicularPropagationLossModel");
  helper->SetSpectrumPropagationLossModelType ("ns3::MmWaveVehicularSpectrumPropagationLossModel");
  helper->SetNumerology (3);
  NetDeviceContainer devs = helper->InstallMmWaveVehicularNetDevices (group);

  InternetStackHelper internet;
  internet.Install (group);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");

  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devs);

  helper->PairDevices(devs);

  // Ipv4StaticRoutingHelper ipv4RoutingHelper;
  //
  // Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (group.Get (0)->GetObject<Ipv4> ());
  // staticRouting->SetDefaultRoute (group.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );
  //
  // staticRouting = ipv4RoutingHelper.GetStaticRouting (group.Get (2)->GetObject<Ipv4> ());
  // staticRouting->SetDefaultRoute (group.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal () , 2 );

  NS_LOG_DEBUG("IPv4 Address node 0: " << group.Get (0)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 1: " << group.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());
  NS_LOG_DEBUG("IPv4 Address node 2: " << group.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ());

  Ptr<mmwave::MmWaveAmc> m_amc = CreateObject <mmwave::MmWaveAmc> (helper->GetConfigurationParameters());
  double availableRate = m_amc->CalculateTbSize (28, 14) * 8 / 0.001; // bps
  uint16_t port_1 = 4000;  // well-known echo port number
  uint16_t port_2 = 4001;

  UdpEchoServerHelper server (port_1);

  ApplicationContainer apps = server.Install (group.Get (2));
  apps.Start (Seconds (0.0));

  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream_1 = asciiTraceHelper.CreateFileStream ("user_1.txt");
  apps.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream_1));

  UdpEchoServerHelper server2 (port_2);

  ApplicationContainer apps2 = server2.Install (group.Get (2));
  apps2.Start (Seconds (0.0));

  Ptr<OutputStreamWrapper> stream_2 = asciiTraceHelper.CreateFileStream ("user_2.txt");
  apps2.Get(0)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, stream_2));

  uint32_t maxPacketCount = 800000;
  uint32_t packetSize = 512;
  NS_LOG_UNCOND(availableRate);
  Time interPacketInterval =  Seconds(double((packetSize * 8) / availableRate));

  UdpClientHelper client (group.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port_1);
  client.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client.SetAttribute ("PacketSize", UintegerValue (packetSize));
  ApplicationContainer clientApps = client.Install (group.Get (0));
  clientApps.Start (Seconds(1.0));
  clientApps.Stop (Seconds(100.0));

  UdpClientHelper client2 (group.Get (2)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), port_2);
  client2.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
  client2.SetAttribute ("Interval", TimeValue (interPacketInterval));
  client2.SetAttribute ("PacketSize", UintegerValue (packetSize));
  ApplicationContainer clientApps2 = client2.Install (group.Get (1));
  clientApps2.Start (Seconds(1.0));
  clientApps2.Stop (Seconds(100.0));

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  uint32_t nodeCounter = 0;
  VehicleSpeedControlHelper vehicleSpeedControlHelper(9);
  vehicleSpeedControlHelper.SetAttribute("Client", (PointerValue)sumoClient);

  // a callback function to setup the nodes
  std::function<Ptr<Node>()> setupNewWifiNode = [&]() -> Ptr<Node> {
    if (nodeCounter >= group.GetN())
      NS_FATAL_ERROR("Node Container empty!: " << nodeCounter
                                               << " nodes created.");

    // don't create and install the protocol stack of the node at simulation
    // time -> take from "node pool"
    Ptr<Node> includedNode = group.Get(nodeCounter);
    ++nodeCounter; // increment counter for next node

    // Install Application
    ApplicationContainer vehicleSpeedControlApps =
        vehicleSpeedControlHelper.Install(includedNode);
    vehicleSpeedControlApps.Start(Seconds(0.0));
    vehicleSpeedControlApps.Stop(Seconds(18.0));

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
  };

  // start traci client
  sumoClient->SumoSetup(setupNewWifiNode, shutdownWifiNode);

  AnimationInterface anim ("examples/sumo-millicar/circle-simple/ns3-sumo-coupling.xml"); // Mandatory



  Simulator::Stop (Seconds(18.0));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
