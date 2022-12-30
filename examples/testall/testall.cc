#include <iostream>
#include "ns3/traci-applications-module.h"
#include "ns3/traci-module.h"
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
#include <sys/stat.h>
#include "ns3/internet-module.h"
#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/mmwave-sidelink-spectrum-phy.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/spectrum-helper.h"
#include "ns3/mmwave-spectrum-value-helper.h"



using namespace ns3;
using namespace millicar;

// create a pool to contain the vehicles. 
std::vector<NodeContainer> nodecontainer1(100);
Ptr<TraciClient> sumoClient = CreateObject<TraciClient>();
std::map<std::string, int> VehDur;
std::map<int, std::string> NodetoVehID = {};
std::vector<std::string> mapped_IDlist = {};  //最多只能有7个元素


static void ChangeFilltoRED(std::vector<UdpEchoClientHelper> * client, ApplicationContainer * clientApps, uint32_t index, Ptr<const Packet> p)
{


    // std:: cout << "enter ChangeFilltoRED" << std::endl;
    if (NodetoVehID.find(index+1) != NodetoVehID.end() ){
        // std:: cout << "Find Related Veh" << std::endl;
        if (VehDur[NodetoVehID[index+1]] >= 5)
        {(*client)[index].SetFill(clientApps->Get(index), "RED");
        // std:: cout << "Change color" << std::endl;
        }
    }
}

static void ChangeFilltoYELLOW(std::vector<UdpEchoClientHelper> * client, ApplicationContainer * clientApps, uint32_t index, Ptr<const Packet> p)
{
    //First, find out whether the node is linked to any Veh
    if (NodetoVehID.find(index+1) == NodetoVehID.end() ){
        (*client)[index].SetFill(clientApps->Get(index), "YELLOW");
    }
}

static void ChangeFilltoBLUE(std::vector<UdpEchoClientHelper> * client, ApplicationContainer * clientApps, uint32_t index, Ptr<const Packet> p)
{   
    double nowtime = Simulator::Now ().GetSeconds ();
    if (index == 0 && nowtime > 10 && nowtime <= 20){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 0 && nowtime > 40 && nowtime <= 50){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 0 && nowtime > 65 && nowtime <= 70){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 0 && nowtime > 80 && nowtime <= 95){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 0 && nowtime > 110 && nowtime <= 125){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 0 && nowtime > 150 && nowtime <= 160){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 0 && nowtime > 180 && nowtime <= 190){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }

     if (index == 1 && nowtime > 20 && nowtime <= 30){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 1 && nowtime > 50 && nowtime <= 60){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 1 && nowtime > 80 && nowtime <= 87){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 1 && nowtime > 95 && nowtime <= 107){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 1 && nowtime > 119 && nowtime <= 125){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 1 && nowtime > 135 && nowtime <= 145){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 1 && nowtime > 160 && nowtime <= 170){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }

    if (index == 2 && nowtime > 13 && nowtime <= 26){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 2 && nowtime > 39 && nowtime <= 52){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 2 && nowtime > 70 && nowtime <= 83){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 2 && nowtime > 99 && nowtime <= 108){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 2 && nowtime > 118 && nowtime <= 121){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 2 && nowtime > 135 && nowtime <= 145){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 2 && nowtime > 162 && nowtime <= 168){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }

    if (index == 3 && nowtime > 5 && nowtime <= 15){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 3 && nowtime > 20 && nowtime <= 30){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 3 && nowtime > 35 && nowtime <= 45){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 3 && nowtime > 55 && nowtime <= 65){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 3 && nowtime > 79 && nowtime <= 100){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 3 && nowtime > 110 && nowtime <= 120){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 3 && nowtime > 135 && nowtime <= 150){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }

    if (index == 4 && nowtime > 11 && nowtime <= 22){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 4 && nowtime > 33 && nowtime <= 44){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 4 && nowtime > 55 && nowtime <= 66){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 4 && nowtime > 77 && nowtime <= 88){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 4 && nowtime > 99 && nowtime <= 110){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 4 && nowtime > 121 && nowtime <= 132){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 4 && nowtime > 143 && nowtime <= 154){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }

    if (index == 5 && nowtime > 8 && nowtime <= 16){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 5 && nowtime > 32 && nowtime <= 50){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 5 && nowtime > 56 && nowtime <= 64){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 5 && nowtime > 60 && nowtime <= 68){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 5 && nowtime > 84 && nowtime <= 92){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 5 && nowtime > 108 && nowtime <= 116){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }
    if (index == 5 && nowtime > 130 && nowtime <= 138){
        (*client)[index].SetFill(clientApps->Get(index), "BLUE");
    }

}

static void Tx ( uint32_t node_counter, Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{   
        std:: cout << "enter tx" << std::endl;

    *stream->GetStream () << "Tx\t" << Simulator::Now ().GetSeconds () << "\t" << node_counter << "\t" << p->GetSize ()<< '\t';
    p->CopyData(&(*stream->GetStream()), p->GetSize());
    *stream->GetStream ()<< std::endl;
    std:: cout << "leave tx" << std::endl;
}

static void Rx (uint32_t node_counter, Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
    *stream->GetStream () << "Rx\t" << Simulator::Now ().GetSeconds () << "\t" << node_counter << "\t" << p->GetSize() << "\t";
    p->CopyData(&(*stream->GetStream()), p->GetSize());
    *stream->GetStream ()<< std::endl;

    //先临时输出packet_number, 再读入
    std::ofstream ofs("examples/testall/temp.txt");
    p->CopyData(&ofs, p->GetSize());
    ofs.close();

  //读入    
    
    std::ifstream ifs("examples/testall/temp.txt");
    std::string packet_readin((std::istreambuf_iterator<char>(ifs)),
                 std::istreambuf_iterator<char>());


    libsumo::TraCIColor Red;
    Red.r = 255;
    Red.g = 0;
    Red.b = 0;
    Red.a = 255;
    libsumo::TraCIColor Yellow;
    Yellow.r = 255;
    Yellow.g = 255;
    Yellow.b = 0;
    Yellow.a = 255;
    libsumo::TraCIColor GREEN;
    GREEN.r = 0;
    GREEN.g = 255;
    GREEN.b = 0;
    GREEN.a = 255;
    libsumo::TraCIColor BLUE;
    BLUE.r = 0;
    BLUE.g = 0;
    BLUE.b = 255;
    BLUE.a = 255;

    if (NodetoVehID.find(node_counter) == NodetoVehID.end()){
        return;
    }
    
    if (packet_readin.substr(0, 3) == "RED"){
      sumoClient->TraCIAPI::vehicle.setColor(NodetoVehID[node_counter], Red);      
    }
    else if (packet_readin.substr(0, 6) == "YELLOW"){
      sumoClient->TraCIAPI::vehicle.setColor(NodetoVehID[node_counter], Yellow);      
    }
    else if (packet_readin.substr(0, 5) == "GREEN"){
      sumoClient->TraCIAPI::vehicle.setColor(NodetoVehID[node_counter], GREEN);      
    }
    else if (packet_readin.substr(0, 4) == "BLUE"){
      sumoClient->TraCIAPI::vehicle.setColor(NodetoVehID[node_counter], BLUE);      
    }
}



std::vector<std::string> intersection(std::vector<std::string> v1,
                                      std::vector<std::string> v2){
    std::vector<std::string> v3;

    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());

    std::set_intersection(v1.begin(),v1.end(),
                          v2.begin(),v2.end(),
                          back_inserter(v3));
    return v3;
}

//x-y
template <typename T>
std::vector<T> findDiff(std::vector<T> x, std::vector<T> y) {        // no-ref, no-const
    std::vector<T> diff;
    std::sort(x.begin(), x.end());
    std::sort(y.begin(), y.end());
    std::set_difference(x.begin(), x.end(), y.begin(), y.end(), std::back_inserter(diff));
    return diff;
}

void simulationStep(int step){

        std::cout << "enter Step" << std::endl;

        sumoClient->simulationStep();
        std::vector<std::string> IDlist = sumoClient-> TraCIAPI::vehicle.getIDList();
        //update VehDur
        for(auto item = IDlist.begin(); item != IDlist.end(); item++){
            std::string s1 = *item;
            if (VehDur.find(s1) != VehDur.end()){
                VehDur[s1] += 1;
            }
            else{
                VehDur[s1] = 1;
            }
        }



        std::vector<std::string> Add_list = findDiff(IDlist, mapped_IDlist);
        std::vector<std::string> Remove_list = findDiff(mapped_IDlist, IDlist);
        
         for(auto it = NodetoVehID.begin(); it != NodetoVehID.end();){  // 删除元素
            std::string temp_VehID = it->second;
            if(std::find(Remove_list.begin(), Remove_list.end(), temp_VehID) != Remove_list.end())
            {
                NodetoVehID.erase(it++);  
            }
            else
            {
                it++;
            }
        }
        mapped_IDlist = findDiff(mapped_IDlist, Remove_list);

        std::vector<int> keys;
        for(auto it = NodetoVehID.begin(); it != NodetoVehID.end(); ++it){
            keys.push_back(it->first);
        }

        std::vector<int> all_keys = {1, 2, 3, 4, 5, 6, 7};
        std::vector<int> add_keys = findDiff(all_keys, keys);
        int min_size = std::min(add_keys.size(), Add_list.size());
        for (int i = 0; i < min_size; i++){
            NodetoVehID.insert({add_keys[i], Add_list[i]});
            mapped_IDlist.push_back(Add_list[i]);
        }

        //
        std::cout << "step:" << "\t" << step << std::endl;
        for(auto it = NodetoVehID.begin(); it != NodetoVehID.end(); it++){
            std :: cout << it->first << "\t" << it -> second << std::endl;
        }

        std :: cout <<"VehDur:" << std::endl;
        for(auto it = VehDur.begin(); it != VehDur.end(); it++){
            std :: cout << it->first << "\t" << it -> second << std::endl;
        }


        // 更新每个车的ns3位置

        for (std::map<int, std::string>::iterator iter = NodetoVehID.begin(); iter != NodetoVehID.end(); iter++){
            int node = iter->first;
            const std::string VehId= iter->second;
            libsumo::TraCIPosition position = sumoClient->TraCIAPI::vehicle.getPosition(VehId);
            // n.Get(node)->GetObject<MobilityModel>()->SetPosition (Vector (position.x, position.y, position.z));
        }

    
    std:: cout << "leave step" << std::endl;
        

    }

int main()
{   
    double frequency = 60e9;  // frequency in Hz
    double bandwidth = 2.5e8; // bandwidth in Hz
    uint32_t simulationtime = 300;
    // uint32_t simulationRun;   // number of simulation

    std::string channel_condition = "l";
    std::string scenario;

    ns3::Time simulationTime(ns3::Seconds(simulationtime));

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


    // create nodepool, large enough to hold all the simulated car
    int i = 0;
    for (NodeContainer &n : nodecontainer1){
        n.Create(8);

            // create the mobility nodes
        MobilityHelper mobility;
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (n);

        // initinalize all the postion, the modules cannot be initialize at the same point otherwise the propagation model goes wrong
        // the infracture is set at (230,198,0)
        for (uint32_t i = 0; i < n.GetN(); i++)  n.Get(i)->GetObject<MobilityModel>()->SetPosition (Vector (0,0,0));
        n.Get(0)->GetObject<MobilityModel>()->SetPosition (Vector (230,198,0));

            // Create and configure the helper
        Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>(); 
        helper->SetPropagationLossModelType("ns3::MmWaveVehicularPropagationLossModel");
        helper->SetSpectrumPropagationLossModelType("ns3::MmWaveVehicularSpectrumPropagationLossModel");
        helper->SetNumerology(3);   // 好像最大是3 车辆最大是2**3
        NetDeviceContainer devs = helper->InstallMmWaveVehicularNetDevices(n);

         InternetStackHelper internet;
        internet.Install(n);

        Ipv4AddressHelper ipv4;
        std::string prefix = "10.1.";
        std::string suffix = ".0";
        const char * ip = (prefix+std::to_string(i)+suffix).c_str();
        ipv4.SetBase(ip , "255.255.255.0");
        i++;
        Ipv4InterfaceContainer i = ipv4.Assign(devs);
        helper->PairDevices(devs);

         uint32_t maxPacketCount = 800000;
        uint32_t packetSize = 512;
        Time interPacketInterval =  MilliSeconds(40);

        // create server for each car (7 in total)
        #pragma region
        UdpEchoServerHelper server1 (4001);
        ApplicationContainer apps = server1.Install (n.Get(1));

        UdpEchoServerHelper server2 (4002);
        apps.Add( server2.Install (n.Get(2)) );

        UdpEchoServerHelper server3 (4003);
        apps.Add( server3.Install (n.Get(3)) );

        UdpEchoServerHelper server4 (4004);
        apps.Add( server4.Install (n.Get(4)) );

        UdpEchoServerHelper server5 (4005);
        apps.Add( server5.Install (n.Get(5)) );

        UdpEchoServerHelper server6 (4006);
        apps.Add( server6.Install (n.Get(6)) );

        UdpEchoServerHelper server7 (4007);
        apps.Add( server7.Install (n.Get(7)) );

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
        client_0_4.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
        client_0_4.SetAttribute ("Interval", TimeValue (interPacketInterval));
        client_0_4.SetAttribute ("PacketSize", UintegerValue (packetSize));
        clientApps.Add(client_0_4.Install (n.Get (4)));

        UdpEchoClientHelper client_0_5 (n.Get (5)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4005);
        client_0_5.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
        client_0_5.SetAttribute ("Interval", TimeValue (interPacketInterval));
        client_0_5.SetAttribute ("PacketSize", UintegerValue (packetSize));
        clientApps.Add(client_0_5.Install (n.Get (5)));

        UdpEchoClientHelper client_0_6 (n.Get (6)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4006);
        client_0_6.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
        client_0_6.SetAttribute ("Interval", TimeValue (interPacketInterval));
        client_0_6.SetAttribute ("PacketSize", UintegerValue (packetSize));
        clientApps.Add(client_0_6.Install (n.Get (6)));

        UdpEchoClientHelper client_0_7 (n.Get (7)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4007);
        client_0_7.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
        client_0_7.SetAttribute ("Interval", TimeValue (interPacketInterval));
        client_0_7.SetAttribute ("PacketSize", UintegerValue (packetSize));
        clientApps.Add(client_0_7.Install (n.Get (7)));
        #pragma endregion

        clientApps.Start (Seconds(0));
        clientApps.Stop (Seconds(simulationtime));
        //in this example, client index is car index - 1.

        //set the initial fill
        client_0_1.SetFill(clientApps.Get(0), "YELLOW");
        client_0_2.SetFill(clientApps.Get(1), "YELLOW");
        client_0_3.SetFill(clientApps.Get(2), "YELLOW");
        client_0_4.SetFill(clientApps.Get(3), "YELLOW");
        client_0_5.SetFill(clientApps.Get(4), "YELLOW");
        client_0_6.SetFill(clientApps.Get(5), "YELLOW");
        client_0_7.SetFill(clientApps.Get(6), "YELLOW");

            // help to output log of v2i communication 
        AsciiTraceHelper asciiTraceHelper;
        Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream ("examples/testall/communation-stream.txt");

        std::cout << "error1" << std::endl;
        std::vector<UdpEchoClientHelper> client_vector = {client_0_1, client_0_2, client_0_3, client_0_4, client_0_5, client_0_6, client_0_7};
        for (uint32_t i = 0; i < 7; i++){
            clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx , i+1,stream));
            clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&ChangeFilltoRED, & client_vector, & clientApps, i ));
            clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&ChangeFilltoYELLOW, & client_vector, & clientApps, i ));
            clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&ChangeFilltoBLUE, & client_vector, & clientApps, i ));
            apps.Get(i)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, i+1, stream));
        }

        std::cout << "error2" << std::endl;

    }









   

   



    sumoClient -> connect("localhost", 3400);
    sumoClient -> setOrder(2);
    sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.1)));
    sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));


    



    for (int i = 0; i < (simulationtime*10);i++){
        
        Simulator::Schedule(Seconds(0.1*i), &simulationStep,  i);
    }
    
    Simulator::Run();
    // while(1){
    //     sumoClient -> simulationStep();
    //     std::vector<std::string> IDlist = sumoClient-> TraCIAPI::vehicle.getIDList();
    //     std::vector<std::string> Add_list = findDiff(IDlist, mapped_IDlist);
    //     std::vector<std::string> Remove_list = findDiff(mapped_IDlist, IDlist);
        
    //     for(auto it = NodetoVehID.begin(); it != NodetoVehID.end();){  // 删除元素
    //         std::string temp_VehID = it->second;
    //         if(std::find(Remove_list.begin(), Remove_list.end(), temp_VehID) != Remove_list.end())
    //         {
    //             NodetoVehID.erase(it++);  
    //         }
    //         else
    //         {
    //             it++;
    //         }
    //     }
    //     mapped_IDlist = findDiff(mapped_IDlist, Remove_list);

    //     std::vector<int> keys;
    //     for(auto it = NodetoVehID.begin(); it != NodetoVehID.end(); ++it){
    //         keys.push_back(it->first);
    //     }

    //     std::vector<int> all_keys = {1, 2, 3, 4, 5, 6, 7};
    //     std::vector<int> add_keys = findDiff(all_keys, keys);
    //     int min_size = std::min(add_keys.size(), Add_list.size());
    //     for (int i = 0; i < min_size; i++){
    //         NodetoVehID.insert({add_keys[i], Add_list[i]});
    //         mapped_IDlist.push_back(Add_list[i]);
    //     }

    //     //
    //     std :: cout << "step" << "\t" << step << std::endl;
    //     for(auto it = NodetoVehID.begin(); it != NodetoVehID.end(); it++){
    //         std :: cout << it->first << "\t" << it -> second << std::endl;
    //     }


    // step+=1;
    // }
    
    Simulator::Destroy();
    sumoClient->close();

    return 0;


}