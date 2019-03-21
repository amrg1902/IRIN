/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

int     mapGridX          = 20;
int     mapGridY          = 20;
double  mapLengthX        = 3.0;
double  mapLengthY        = 3.0;
double  robotStartGridX   = 11; 
double  robotStartGridY   = 18;
double  robotEndGridX     = 15;
double  robotEndGridY     = 1;

#define ERROR_DIRECTION 0.05 
#define ERROR_POSITION 0.01

CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set blue battery Sensor */
	m_seBlueBattery = (CBlueBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BLUE_BATTERY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor (SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);
	/* Set compass Sensor */
	m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor (SENSOR_COMPASS);

  m_nState=0;

  PathPlanning();
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{


	/* FASE 1: LECTURA DE SENSORES */

	/* Leer Sensores de Suelo */
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);
	/* Leer Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Leer Compass */
	double* compass = m_seCompass->GetSensorReading(m_pcEpuck);
	
	/* FASE 2: CONTROLADOR */
	
	/* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */

  /* Remake Kinematic Equations */
  CalcPositionAndOrientation(encoder);
  
  /* DEBUG */
  printf("REAL: %2f,%2f,%2f  -- ODOM: %2f,%2f,%2f -- ENC: %2f,%2f \n", (m_pcEpuck->GetPosition()).x, (m_pcEpuck->GetPosition()).y, compass[0], m_vPosition.x,m_vPosition.y,m_fOrientation,encoder[0], encoder[1]);
  /* DEBUG */
 
  /* Finite State Machine */
  if (m_nState >= m_nPathPlanningStops)
    Stop();
  else if (GoGoal(m_vPositionsPlanning[m_nState].x, m_vPositionsPlanning[m_nState].y))
    m_nState++;


  /* If on Nest, signal it */
  if (ground[1] == 0.0)
    m_pcEpuck->SetAllColoredLeds( LED_COLOR_GREEN);

  printf("State: %d\n", m_nState);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::CalcPositionAndOrientation (double *f_encoder)
{
  /* Remake kinematic equations */
  double fIncU = (f_encoder[0]+ f_encoder[1] )/ 2;
  double fIncTetha = (f_encoder[1] - f_encoder[0])/ CEpuck::WHEELS_DISTANCE;

  /* Substitute arc by chord, take care of 0 division */
  if (fIncTetha != 0.0)
    fIncU = ((f_encoder[0]/fIncTetha)+(CEpuck::WHEELS_DISTANCE/2))* 2.0 * sin (fIncTetha/2.0);

  /* Update new Position */
  m_vPosition.x += fIncU * cos(m_fOrientation + fIncTetha/2);
  m_vPosition.y += fIncU * sin(m_fOrientation + fIncTetha/2);
  
  /* Update new Orientation */
  m_fOrientation += fIncTetha;

  /* Normalize Orientation */
  while(m_fOrientation < 0) m_fOrientation += 2*M_PI;
  while(m_fOrientation > 2*M_PI) m_fOrientation -= 2*M_PI;
}

/******************************************************************************/
/******************************************************************************/

int CIri1Controller::GoGoal (double f_x, double f_y)
{
	double fX = (f_x - m_vPosition.x);
	double fY = (f_y - m_vPosition.y);
	double fGoalDirection = 0;
 
  printf("fX: %2f, fY:%2f\n", fX, fY);
  if ( ( fabs(fX) <= ERROR_POSITION ) && ( fabs(fY) <= ERROR_POSITION ) )
    return 1; 
	else 
    fGoalDirection = atan2(fY, fX);


	while ( fGoalDirection < 0 ) fGoalDirection += 2 * M_PI;
	while ( fGoalDirection >= 2*M_PI) fGoalDirection-=2*M_PI;
	
  double fErrorDirection = fGoalDirection - m_fOrientation;

	while(fErrorDirection > M_PI) fErrorDirection -= 2*M_PI;
	while(fErrorDirection < -M_PI) fErrorDirection += 2*M_PI;

  if      ( fErrorDirection > ERROR_DIRECTION )
    TurnLeft(100);
  else if ( fErrorDirection < -ERROR_DIRECTION )
    TurnRight(100);
  else
    GoForwards(300);

  return 0;
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::TurnLeft( double f_custom_speed )
{
	m_acWheels->SetSpeed(-f_custom_speed, f_custom_speed);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::TurnRight( double f_custom_speed )
{
	m_acWheels->SetSpeed(f_custom_speed, -f_custom_speed);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoForwards( double f_custom_speed )
{
	m_acWheels->SetSpeed(f_custom_speed, f_custom_speed);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoBackwards( double f_custom_speed )
{
	m_acWheels->SetSpeed(-f_custom_speed, -f_custom_speed);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Stop( void )
{
	m_acWheels->SetSpeed(0, 0);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PathPlanning ( void )
{
  //Calc X movement of robot
  double fXmov = robotEndGridX - robotStartGridX;
  fXmov = mapLengthX/mapGridX * fXmov;

  //Calc Y movement of robot
  double fYmov = -(robotEndGridY - robotStartGridY);
  fYmov = mapLengthY/mapGridY * fYmov;

  m_nPathPlanningStops = 2;
  m_vPositionsPlanning = new dVector2[m_nPathPlanningStops]; 

  m_vPositionsPlanning[0].x = fXmov;
  m_vPositionsPlanning[0].y = 0.0;

  m_vPositionsPlanning[1].x = fXmov;
  m_vPositionsPlanning[1].y = fYmov;

  printf("Pos1: %2f, %2f\n", m_vPositionsPlanning[0].x, m_vPositionsPlanning[0].y);
  printf("Pos2: %2f, %2f\n", m_vPositionsPlanning[1].x, m_vPositionsPlanning[1].y);
}
