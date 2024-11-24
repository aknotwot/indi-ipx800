# indi-ipx800
Driver INDI for IPX800 domotic server
Tested only with V4
Requirements : 
<<<<<<< Updated upstream
- Needs M2M activated without header 
Todo :
- V3 / V5 commands to implement

=======
- Needs M2M activated without header
- To use with "Universal ROR" Dome driver 
Limitations :
- Works only for IPX800 V4, V3 and V5 commands to implement
- no management of analoagic input
- ROOF_ENGINE_POWERED, RASPBERRY_SUPPLIED, MAIN_PC_SUPPLIED logic's is reversed

To come : update of labels after function selection, additional check on mount park (using digital inputs), reversed logic selection...
>>>>>>> Stashed changes

cmake -DCMAKE_INSTALL_PREFIX=/usr [you folder with ipx800 sources files]
make -j4
sudo make install
