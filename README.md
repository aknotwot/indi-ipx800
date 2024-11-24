# indi-ipx800
Driver INDI for IPX800 domotic server
Tested only with V4
Requirements : 
- Needs M2M activated without header
- To use with "Universal ROR" Dome driver 
Limitations :
- Works only for IPX800 V4, V3 and V5 commands to implement
- no management of analogic input
- ROOF_ENGINE_POWERED, RASPBERRY_SUPPLIED, MAIN_PC_SUPPLIED logic's is reversed

To come : update of labels after function selection, additional check on mount park (using digital inputs), reversed logic selection

To Compile : 
cmake -DCMAKE_INSTALL_PREFIX=/usr [you folder with ipx800 sources files]
make -j4
sudo make install

First Use :  
- In Connection Tab, define IP and port (9870 by default) used by your IPX800 for M2M communication. It must be active in IPX800 setup page.  
- Select fonctions of each relay and digit input (Relays Outputs and Digital Inputs)
- Selection in "options" Tab if you want to manage roof power,
- Tab Status, and InputsOutputs show the same data.
- You can change Relay State on "InputsOutputs" Tab.