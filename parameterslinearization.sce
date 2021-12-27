/////////////////////////////////////////////////////////////////////////////
// Linearization with xcos
/////////////////////////////////////////////////////////////////////////////
loadXcosLibs(); loadScicos();
importXcosDiagram("C:\Users\Mrcs641\Desktop\2021 - II\SCA GA\Labs\Lab03\srcs-docs\srcs\xcoslinearizationA.xcos");
typeof(scs_m)
scs_m.props.context;
// specific context data
Ld = 0.230; //% d-axis Inductance
Lq = 0.110 ;//% q-axis Inductance
TLn = 14; //% Load Torque
B = 0.0045; //% Viscous damping coefficient 
dB = 0.25 ; //% Variation of B (%)
Rs = 2.95;//%
dRs = 0.25;//%
Jm = 0.04838; 
dJ = 0.25;
npp = 2;
a1 = 3*npp*(Ld-Lq)/2;


// looking for the Superblock to linearize
for i=1:length(scs_m.objs)
    if typeof(scs_m.objs(i))=="Block" & scs_m.objs(i).gui=="SUPER_f" then
        scs_m = scs_m.objs(i).model.rpar;
        break;
    end
end
sys = lincos(scs_m);
