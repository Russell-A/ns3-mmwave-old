# MMwave communication on ns3 and sumo

This repo is set up for simulating the vehicle-to-vehicle and vehicle-to-infrastrature communication on ns3-sumo-coupling platfrom.

The mmwave libraries are from:

* M. Mezzavilla, M. Zhang, M. Polese, R. Ford, S. Dutta, S. Rangan, M. Zorzi, "End-to-End Simulation of 5G mmWave Networks," in IEEE Communications Surveys & Tutorials, vol. 20, no. 3, pp. 2237-2263, thirdquarter 2018. 

* M. Drago, T. Zugno, M. Polese, M. Giordani, M. Zorzi, "Millicar - An ns-3 Module for MmWave NR V2X Networks," Proc. of the Workshop on ns-3 (WNS3), 2020.

and the ns3-sumo-coupling library is from [this repo](https://github.com/vodafone-chair/ns3-sumo-coupling).

## Example
The 'test' module in the folder 'example' implements a ns-3 and sumo coupling example. The example's senerio is set at the intersection of Amsterdam Ave and 120th street. To run the example, download [this repo](https://github.com/Russell-A/120-amsterdam-mqtt) first. Make sure that '--num-clients' is set as '2' in MQTTReceiver.py. Then run

```
python3 ./py/MQTTReceiver.py
./waf --run test
python3 ./py/MQTTSender.py
```
seperately. Be careful that the first and the last command should be run in the root of MQTT and the second command should be run in this ns3-mmwave repo.

The senerio implements the MMWAVE communication between a traffic infrastructure and vehicles and persons. The traffic infrastructure is implicitly placed in the center of the intersection. The linked vehicles and persons receive UDP packets from the traffic infrastructure in a constant rate. Occientially, the vehicles and persons receive warning messages. In this example, whether the vehicle is in communication is indicate by its color. YELLOW indicates the vehicles and persons not in communication. RED indicates the vehicles and persons in communication. BLUE indicates the vehicles and persons in communication and receive a warning message.

There is a video of this example on youtube. [https://youtu.be/uH4RUIp2U38](https://youtu.be/uH4RUIp2U38)

## Details
The code of the above example is 'examples/test/test.cc'. It includes several help functions. 

`static void ChangeFillto***()` sets the fills of the UDP packet. It's called every communication to indentify whether we need to change the fills of the UDP packet.

`Tx()` records the fills of the packet sent. It will record the fills into a local file 'examples/test/communation-stream.txt' to help inspect communication. It's called when a packet is sent.

`Rx()` records the fills of the packet received. It will record the fills into a local file 'examples/test/communation-stream.txt' to help inspect communication. It will also change the color of a vehicle according to the packet the vehicle receives. It's called when a packet is received. 

`Intersection()` returns the intersection of two vectors.

`finddiff(x, y)` returns the difference between x and y, i.e., x-y.

`simulationStep(step)` puts forward the sumo simulation by calling `sumoClient->simulationStep()`. Then we retrieve the IDlist of persons and vehicles to see which vehicles and persons are added or removed or needed to be updated in this frame. Then, we use these information to update the map from NS3 node to sumo persons and cars. Finally, if we update the NS3 nodes' positions with their corresponding sumo vehicles or persons' position.

Then, it the main function, the main function defines several variables first.
```c++
NodeContainer n; // create a pool to contain the vehicles. 
Ptr<TraciClient> sumoClient = CreateObject<TraciClient>(); // sumo link
std::map<std::string, int> VehDur; // Record the Duration sof Vehicles in the simulation
std::map<int, std::string> NodetoVehID = {}; // The map from NS3 node to sumo vehicles and persons
std::vector<std::string> mapped_IDlist = {};  // The mapped sumo vehicles and persons
```

Then, after setting the mmwave configures, we can create the ns3 scenerio.
```c++
// create nodepool, which can be not large enough to hold all the simulated car. 8 at most due to the feature of mmwave.
n.Create(8);

// create the mobility nodes
MobilityHelper mobility;
mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
mobility.Install (n);

// initinalize all the postion, the sender and receiver modules cannot be initialize at the same point otherwise the propagation model goes wrong
// the infracture is set at (320.80, 212.24 0)
for (uint32_t i = 0; i < n.GetN(); i++)  n.Get(i)->GetObject<MobilityModel>()->SetPosition (Vector (0,0,0)); 
n.Get(0)->GetObject<MobilityModel>()->SetPosition (Vector (230,198,0));


 // Create and configure the mmwavehelper. install mmwave on every ns3 node.
Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>(); 
helper->SetPropagationLossModelType("ns3::MmWaveVehicularPropagationLossModel");
helper->SetSpectrumPropagationLossModelType("ns3::MmWaveVehicularSpectrumPropagationLossModel");
helper->SetNumerology(3);
NetDeviceContainer devs = helper->InstallMmWaveVehicularNetDevices(n);

// Set the net address
InternetStackHelper internet;
internet.Install(n);

Ipv4AddressHelper ipv4;
ipv4.SetBase("10.1.0.0", "255.255.255.0");
Ipv4InterfaceContainer i = ipv4.Assign(devs);
helper->PairDevices(devs);

// create server for each vehs and persons (7 in total). (Only show 1 here)
UdpEchoServerHelper server1 (4001);
ApplicationContainer apps = server1.Install (n.Get(1));

 // create client for the traffic infrastructure (7 in total). (Only show 1 here)
UdpEchoClientHelper client_0_1 (n.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal (), 4001);
client_0_1.SetAttribute ("MaxPackets", UintegerValue (maxPacketCount));
client_0_1.SetAttribute ("Interval", TimeValue (interPacketInterval));
client_0_1.SetAttribute ("PacketSize", UintegerValue (packetSize));
ApplicationContainer clientApps = client_0_1.Install (n.Get (1));

// set the stop time
clientApps.Stop (Seconds(simulationtime));

//set the initial fill (7 in total). (Only show 1 here)
client_0_1.SetFill(clientApps.Get(0), "YELLOW");

// We use the callback function to call the function we define every time a packet is sent(Tx) or every time a packet is received(RX)
std::vector<UdpEchoClientHelper> client_vector = {client_0_1, client_0_2, client_0_3, client_0_4, client_0_5, client_0_6, client_0_7};
for (uint32_t i = 0; i < 7; i++){
    clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&Tx , i+1,stream));
    clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&ChangeFilltoRED, & client_vector, & clientApps, i ));
    clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&ChangeFilltoYELLOW, & client_vector, & clientApps, i ));
    clientApps.Get(i)->TraceConnectWithoutContext ("Tx", MakeBoundCallback (&ChangeFilltoBLUE, & client_vector, & clientApps, i ));
    apps.Get(i)->TraceConnectWithoutContext ("Rx", MakeBoundCallback (&Rx, i+1, stream));

    // Set the link to sumo
    sumoClient -> connect("localhost", 3400);
    sumoClient -> setOrder(2);
    sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.1)));
    sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));

    // Schedule the simulation step, call every synchinterval time
    float SynchInterval =  0.1;
    for (int i = 0; i < (simulationtime* 1/SynchInterval);i++){ 
        Simulator::Schedule(Seconds(SynchInterval*i), &simulationStep,  i);
    }

    // Run the simulation
    Simulator::Run();
}

```

The main function will run the simulation untill the stop time or the time the sumo-gui is shut down.