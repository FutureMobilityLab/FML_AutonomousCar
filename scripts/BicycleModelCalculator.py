import numpy as np

def main():
    # CG Position | Lf | Lr --------------------------------------------------------------------------------------
    velocity    = 3.0               # m/s
    wheelbase   = .404              # m             - Wheelbase of Traxxas XO-1
    m_rl        = 1.365             # kg            - Rear Left Scale Mass
    m_rr        = 1.380             # kg            - Rear Right Scale Mass
    m_fl        = 1.419             # kg            - Front Left Scale Mass
    m_fr        = 1.419             # kg            - Front Right Scale Mass

    rearMass = (m_rl + m_rr)/2
    frontMass = (m_fl + m_fr)/2
    lf = wheelbase*frontMass/(rearMass+frontMass)
    lr = wheelbase-lf

    # MOI -------------------------------------------------------------------------------------------------------

    T_recorded = [0.819,0.820,0.814,0.832,0.815,0.820] # Recorded Trial Data using Bifalar Pendulum

    d       = 0.864                 # m             - Distance between Pendulum Strings
    m_block = 1.660                 # kg            - Mass of Test Block
    m_car   = 5.568                 # kg            - Mass of Car
    g       = 9.80                  # m/s**2        - Gravitational Constant
    T       = np.mean(T_recorded)   # seconds       - Oscillation Period (avg. of 10 cycles)
    print(T)
    L       = 0.80                  # m             - Length of Pendulum String
    I_Block = 0.1146                # kg*m**2       - MOI of Test Block (Observed) - Theoretical is 0.1158 kgmm


    I_tot = d**2 * (m_block + m_car) * g * T**2 / (16 * 3.14159**2 * L)
    I_car = I_tot - I_Block

    # Cornering Stiffnesses ------------------------------------------------------------------------------------
    
    #Data from https://www.researchgate.net/publication/4118976_Dimensionless_analysis_of_tire_characteristics_for_vehicle_dynamics_studies
    tireLoads   = [20,30,40,50,60,70]
    tire_B      = np.array([0.132,0.112,0.104,0.079,0.065,0.057])
    tire_C      = np.array([1.3,1.23,1.15,1.18,1.35,1.43])
    tire_D      = np.array([21.3,31.14,38.77,50.98,57.45,64.86])
    tire_Cf     = tire_B * tire_C * tire_D
    C_curve = np.polyfit(tireLoads,tire_Cf,1)   #Linear Interpolation
    Cf_each = C_curve[0]*g*frontMass/2+C_curve[1]
    Cr_each = C_curve[0]*g*rearMass/2+C_curve[1]
    Cf = Cf_each * 2
    Cr = Cr_each * 2   

    # Full Size Car --------------------------------------------------------------------------------------------
    # #LEAF https://www.sciencedirect.com/science/article/pii/S0016003214001641
    # m_car_ev = 1125 # kg
    # lf_ev = 1.1 # m
    # lr_ev = 1.3 # m
    # I_car_ev = 1519 #kgm^2
    # Cf_ev = 15325 #N/rad
    # Cr_ev = 16280 #N/rad
    # velocity_ev = 5.8 # m/s

    # # https://www.mdpi.com/1424-8220/21/11/3711
    # m_car_ev = 1600 # kg
    # lf_ev = 1.4 # m
    # lr_ev = 1.1  # m
    # I_car_ev = 2000 #kgm^2
    # Cf_ev = 66900 #N/rad
    # Cr_ev = 66900 #N/rad
    # velocity_ev = 10 # m/s

    # https://www.sciencedirect.com/science/article/pii/S0016003214002749
    m_car_ev = 900 # kg
    lf_ev = 1.0 # m
    lr_ev = 1.0  # m
    I_car_ev = 1200 #kgm^2
    Cf_ev = 22000 #N/rad
    Cr_ev = 25000 #N/rad
    velocity_ev = 20 # m/s

    # Printout -------------------------------------------------------------------------------------------------

    print('''
    Bicycle Model Scale Car Parameters:
    Mass: {0:.3f} kg
    Lf:   {1:.3f} m
    Lr:   {2:.3f} m
    Iz:   {3:.3f} kg*m^2
    Cf:   {4:.3f} N/rad
    Cr:   {5:.3f} N/rad
    '''.format(m_car,lf,lr,I_car,Cf,Cr))

    Pi1 = lf/wheelbase
    Pi2 = lr/wheelbase
    Pi3 = Cf*wheelbase/((frontMass+rearMass)*velocity**2)
    Pi4 = Cr*wheelbase/((frontMass+rearMass)*velocity**2)
    Pi5 = I_car/ ((frontMass+rearMass)*wheelbase**2)
    print('''
    Pi Groups:
    PI 1:   {0:.3f} 
    PI 2:   {1:.3f} 
    PI 3:   {2:.3f} 
    PI 4:   {3:.3f} 
    PI 5:   {4:.3f} 
    '''.format(Pi1,Pi2,Pi3,Pi4,Pi5))


    print('''
    Bicycle Model Full Size EV Parameters:
    Mass: {0:.3f} kg
    Lf:   {1:.3f} m
    Lr:   {2:.3f} m
    Iz:   {3:.3f} kg*m^2
    Cf:   {4:.3f} N/rad
    Cr:   {5:.3f} N/rad
    '''.format(m_car_ev,lf_ev,lr_ev,I_car_ev,Cf_ev,Cr_ev))

    Pi1_ev = lf_ev/(lf_ev+lr_ev)
    Pi2_ev = lr_ev/(lf_ev+lr_ev)
    Pi3_ev = Cf_ev*(lf_ev+lr_ev)/((m_car_ev)*velocity_ev**2)
    Pi4_ev = Cr_ev*(lf_ev+lr_ev)/((m_car_ev)*velocity_ev**2)
    Pi5_ev = I_car_ev/ ((m_car_ev)*(lf_ev+lr_ev)**2)
    print('''
    Pi Groups Full:
    PI 1:   {0:.3f} 
    PI 2:   {1:.3f} 
    PI 3:   {2:.3f} 
    PI 4:   {3:.3f} 
    PI 5:   {4:.3f} 
    '''.format(Pi1_ev,Pi2_ev,Pi3_ev,Pi4_ev,Pi5_ev))



if __name__ == "__main__":
   main()