#ifndef __CAR_PARA_DEF__
#define __CAR_PARA_DEF__

#define Degree_range 10
#define WIN_SIZE 800.0


#define Weel_plus_per_mm_contast ( 337.858 ) // step/mm
///#define Weel_plus_per_mm_contast ( 338.8047 ) // step/mm
#define Weel_plus_per_mm (  Weel_plus_per_mm_contast )
#define Weel_plus_per_meter (337858.0)

#define Weel_plus_per_deg_contast  ( 883.222 ) //  317962/360.0 => step/deg 883.222 
///#define Weel_plus_per_deg_contast  ( 891.364 ) //  320891/360.0 => step/deg 
//#define Weel_plus_per_deg_contast  ( 400 ) //  320891/360.0 => step/deg 

#define Weel_plus_per_deg (Weel_plus_per_deg_contast ) ///1043

#define Weel_plus_per_rad (50604.89297) /// Weel_plus_per_deg * 180 / PI

#define Weel_plus_per_circle 10000

#define CAR_WELL_to_Weel_L 298
#define OrientMotorCom "/dev/hl340"

#endif