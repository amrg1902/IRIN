#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

    void CalcPositionAndOrientation ( double *f_encoder );
    int  GoGoal                     ( double f_x, double f_y, double *prox);
    void TurnLeft                   ( double f_custom_speed );
    void TurnRight                  ( double f_custom_speed );
    void TurnAngle                  ( double f_custom_speed_left, double f_custom_speed_right  );
    void GoForwards                 ( double f_custom_speed );
    void GoBackwards                ( double f_custom_speed );
    void Stop                       ( void );

    void    PathPlanning            ( void );
    string  pathFind                ( const int &xStart, const int &yStart, const int &xFinish, const int &yFinish );
private:
    CEpuck* m_pcEpuck;
    
		CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CRealBlueLightSensor* m_seBlueLight;
		CRealRedLightSensor* m_seRedLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;  
		CBlueBatterySensor* m_seBlueBattery;  
		CRedBatterySensor* m_seRedBattery;  
		CEncoderSensor* m_seEncoder;  
		CCompassSensor* m_seCompass;  

    float m_fOrientation; 
    dVector2 m_vPosition;

		int m_nWriteToFile;
    int m_nState;

    dVector2 *m_vPositionsPlanning;
    int m_nPathPlanningStops;

    int m_nRobotActualGridX;
    int m_nRobotActualGridY;
};

#endif
