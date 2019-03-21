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
	/* Leer Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Leer Compass */
	double* compass = m_seCompass->GetSensorReading(m_pcEpuck);

	
	/* FASE 2: CONTROLADOR */
	
	/* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */
  printf("ENCODER: ");
	for ( int i = 0 ; i < m_seEncoder->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.5f ", encoder[i]);
	}
	printf("\n");
  
  printf("COMPASS: ");
	for ( int i = 0 ; i < m_seCompass->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.5f ", compass[i]);
	}
	printf("\n");

	/* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */

  /* Remake kinematic equations */
  double fIncU = (encoder[0]+ encoder[1] )/ 2;
  double fIncTetha = (encoder[1] - encoder[0])/ CEpuck::WHEELS_DISTANCE;

  /* Substitute arc by chord, take care of 0 division */
  if (fIncTetha != 0.0)
    fIncU = ((encoder[0]/fIncTetha)+(CEpuck::WHEELS_DISTANCE/2))* 2.0 * sin (fIncTetha/2.0);

  /* Update new Position */
  m_vPosition.x += fIncU * cos(m_fOrientation + fIncTetha/2);
  m_vPosition.y += fIncU * sin(m_fOrientation + fIncTetha/2);
  
  /* Update new Orientation */
  m_fOrientation += fIncTetha;

  /* Normalize Orientation */
  while(m_fOrientation < 0) m_fOrientation += 2*M_PI;
  while(m_fOrientation > 2*M_PI) m_fOrientation -= 2*M_PI;
  
  printf("REAL: %2f,%2f,%2f  -- ODOM: %2f,%2f,%2f -- ENC: %2f,%2f \n", (m_pcEpuck->GetPosition()).x, (m_pcEpuck->GetPosition()).y, compass[0], m_vPosition.x,m_vPosition.y,m_fOrientation,encoder[0], encoder[1]);

  if (m_nState == 0)
  {
    //Turn until Orien = pi/2
    if ((compass[0] >= M_PI/2 - 0.05) && (compass[0] <= M_PI/2 + 0.05 ))
      m_nState = 1;
    else
      m_acWheels->SetSpeed(-100,100);
  }
  else if (m_nState == 1)
  {
    if (  ( m_vPosition.x > -0.3) && (m_vPosition.x <= 0.3 ) &&
        ( m_vPosition.y > 0.9) && (m_vPosition.y < 0.91 ) )
      m_nState = 2;
    else
      m_acWheels->SetSpeed(300,300);
  }
  else if (m_nState == 2 )
  {
    if ((compass[0] >= - 0.05) && (compass[0] <= 0.05 ))
      m_nState = 3;
    else
      m_acWheels->SetSpeed(100,-100);
  }
  else if (m_nState == 3)
  {
    if (  ( m_vPosition.x > 0.60) && (m_vPosition.x <= 0.61 ) &&
        ( m_vPosition.y > 0.90) && (m_vPosition.y < 0.91 ) )
      m_nState = 4;
    else

      m_acWheels->SetSpeed(300,300);
  }
  else if (m_nState == 4)
  {
    //Turn until Orien = pi/2
    if ((compass[0] >= M_PI/2 - 0.05) && (compass[0] <= M_PI/2 + 0.05 ))
      m_nState = 5;
    else
      m_acWheels->SetSpeed(-100,100);
  }
  else if (m_nState == 5)
  {
    if (  ( m_vPosition.x > 0.60) && (m_vPosition.x <= 0.61 ) &&
        ( m_vPosition.y > 2.55) && (m_vPosition.y < 2.56 ) )
      m_nState = 6;
    else

      m_acWheels->SetSpeed(300,300);
  }
  else
  {
    m_acWheels->SetSpeed(0,0);
  }


	/* Fase 3: ACTUACIÓN */
  if ( ((m_pcEpuck->GetPosition()).x > 0.8 -0.05) && ((m_pcEpuck->GetPosition()).x < 0.8+0.05) &&
      ((m_pcEpuck->GetPosition()).y > 1.25 -0.05) && ((m_pcEpuck->GetPosition()).y < 1.25+0.05) )
  {
    m_pcEpuck->SetAllColoredLeds( LED_COLOR_GREEN); 
  }
}

/******************************************************************************/
/******************************************************************************/

